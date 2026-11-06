#include <Arduino.h>
#include <SPI.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>   // CCCD. arduino-esp32 3.x adds this descriptor on its own;
                       // the runtime check in setup() keeps both cores working.
#include "esp_log.h"

// ==========================================
// ===  BLE PERIPHERAL CONFIGURATION      ===
// ==========================================
// These MUST match Python_ble_LSL.py. The host scans for BLE_DEVICE_NAME /
// SERVICE_UUID, subscribes to DATA_CHAR_UUID and then writes CMD_START to
// CTRL_CHAR_UUID. That write is the BLE equivalent of the TCP connect that
// started the stream in the WiFi firmware.
const char* BLE_DEVICE_NAME = "CERELOG_EEG";
#define SERVICE_UUID    "7a1e8b00-9c3d-4b7e-8f21-6d0a5c2e91f3"
#define DATA_CHAR_UUID  "7a1e8b01-9c3d-4b7e-8f21-6d0a5c2e91f3"  // notify: EEG packets
#define CTRL_CHAR_UUID  "7a1e8b02-9c3d-4b7e-8f21-6d0a5c2e91f3"  // write:  start/stop

const uint8_t CMD_START = 0x01;   // payload: [0x01, mtu_hi, mtu_lo]
const uint8_t CMD_STOP  = 0x00;

// BLE moves far less data per second than TCP, so packets are coalesced into
// one notification instead of being written out one at a time. 13 * 37 = 481
// bytes, which fits the 517-byte ATT MTU that macOS/Windows/BlueZ negotiate.
const uint8_t  BLE_MAX_PACKETS_PER_NOTIFY = 13;
const uint16_t BLE_NOTIFY_FLUSH_MS = 25;   // bound on staging latency
const uint32_t BLE_STATS_PERIOD_MS = 5000; // how often drop counters are printed

// A refused notification is held and retried, not thrown away. The host's radio
// is shared: a Bluetooth mouse on the same adapter takes connection events away
// from this link for as long as it is moving, and the stack refuses notifies
// while its queue is full. BLE_RING_PACKETS is how much backlog that can leave
// before samples are lost: 2048 x 4 ms = ~8 s, ~74 KB of RAM.
const uint16_t BLE_RING_PACKETS = 2048;
const uint32_t BLE_RETRY_US     = 1000;    // pause after a refused notify

BLEServer         *bleServer = NULL;
BLECharacteristic *dataChar  = NULL;
BLECharacteristic *ctrlChar  = NULL;

volatile bool     bleConnected = false;
volatile bool     hostRequestedStream = false;  // set by the CMD_START write
volatile uint16_t ble_host_mtu = 23;            // ATT MTU as reported by the host
bool              ble_was_connected = false;

// --- Drop accounting -------------------------------------------------------
// The BLE link can refuse packets when the controller's buffers back up. The
// refused batch is retried and new samples wait in the ring. Only when the ring
// itself is full is the oldest sample discarded, which bounds the latency.
volatile uint32_t ble_dropped_samples  = 0;   // samples lost to a full ring
volatile uint32_t ble_notify_retries   = 0;   // notifies refused, then retried
volatile uint32_t adc_overrun_count    = 0;   // DRDY fired before loop() read the last sample
uint32_t          ble_sent_samples     = 0;
unsigned long     ble_stats_ms         = 0;   // last time counters were printed
volatile bool     ble_notify_failed    = false;  // set from the onStatus callback

// Set true only while a host is subscribed and streaming. Gates the DRDY
// interrupt so the ADS1299 is not free-running into nowhere before anyone
// connects.
volatile bool streaming_allowed = false;

void start_acquisition(void);
void stop_acquisition(void);

// --- Link tuning -----------------------------------------------------------
// Airtime is what a busy host radio runs short of, so ask for the 2M PHY and
// 251-byte link-layer packets: the same notification then needs far less of it.
// Both are requests the central may refuse; the outcome is printed as [LINK].
// The ESP32 core ships Bluedroid or NimBLE depending on its version, and they
// expose this differently, so each has its own branch below.
volatile bool     ble_peer_known   = false;
volatile bool     ble_link_changed = false;
volatile uint16_t link_interval    = 0;   // units of 1.25 ms
volatile uint16_t link_latency     = 0;
volatile uint16_t link_timeout     = 0;   // units of 10 ms
volatile uint8_t  link_tx_phy      = 0;   // 1 = 1M, 2 = 2M, 3 = coded
volatile uint8_t  link_rx_phy      = 0;
volatile uint16_t link_tx_octets   = 0;
volatile uint16_t link_rx_octets   = 0;

#if defined(CONFIG_BLUEDROID_ENABLED)
esp_bd_addr_t     ble_peer_bda;

static void ble_gap_observer(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  switch (event) {
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
      link_interval = param->update_conn_params.conn_int;
      link_latency  = param->update_conn_params.latency;
      link_timeout  = param->update_conn_params.timeout;
      ble_link_changed = true;
      break;
    case ESP_GAP_BLE_PHY_UPDATE_COMPLETE_EVT:
      link_tx_phy = param->phy_update.tx_phy;
      link_rx_phy = param->phy_update.rx_phy;
      ble_link_changed = true;
      break;
    case ESP_GAP_BLE_READ_PHY_COMPLETE_EVT:
      link_tx_phy = param->read_phy.tx_phy;
      link_rx_phy = param->read_phy.rx_phy;
      ble_link_changed = true;
      break;
    case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
      link_tx_octets = param->pkt_data_length_cmpl.params.tx_len;
      link_rx_octets = param->pkt_data_length_cmpl.params.rx_len;
      ble_link_changed = true;
      break;
    default:
      break;
  }
}
#elif defined(CONFIG_NIMBLE_ENABLED)
volatile uint16_t ble_peer_handle = 0;
int               link_data_len_rc = 0;   // 0 = the 251-byte request was sent
#endif

// --- BLE callbacks ---------------------------------------------------------
class CerelogServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
      bleConnected = true;
  }
  void onDisconnect(BLEServer *pServer) {
      bleConnected = false;
      hostRequestedStream = false;
      ble_peer_known = false;
  }
  // The ESP32 core ships the BLE library on either Bluedroid or NimBLE, and the
  // two spell the connection handle differently. Both call the plain onConnect
  // above as well, so this overload only has to do the parameter update: ask
  // for the fastest connection interval the central will grant (7.5-15 ms),
  // which at 250 Hz is what keeps the link ahead of the ADC.
#if defined(CONFIG_BLUEDROID_ENABLED)
  void onConnect(BLEServer *pServer, esp_ble_gatts_cb_param_t *param) {
      bleConnected = true;
      memcpy(ble_peer_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
      ble_peer_known = true;
      pServer->updateConnParams(param->connect.remote_bda, 6, 12, 0, 200);
  }
#elif defined(CONFIG_NIMBLE_ENABLED)
  void onConnect(BLEServer *pServer, ble_gap_conn_desc *desc) {
      bleConnected = true;
      ble_peer_handle = desc->conn_handle;
      ble_peer_known = true;
      pServer->updateConnParams(desc->conn_handle, 6, 12, 0, 200);
  }
#endif
};

class CerelogCtrlCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) {
      uint8_t *d = pChar->getData();
      size_t   n = pChar->getLength();
      if (d == NULL || n < 1) return;
      if (d[0] == CMD_START) {
          uint16_t mtu = 23;
          if (n >= 3) mtu = ((uint16_t)d[1] << 8) | (uint16_t)d[2];
          if (mtu < 23)  mtu = 23;
          if (mtu > 517) mtu = 517;
          ble_host_mtu = mtu;
          hostRequestedStream = true;
      } else if (d[0] == CMD_STOP) {
          hostRequestedStream = false;
      }
  }
};

// Fires for every notify(). Anything other than SUCCESS_NOTIFY means the
// stack would not take the packet, so ble_service() sends it again.
class CerelogDataCallbacks : public BLECharacteristicCallbacks {
  void onStatus(BLECharacteristic *pChar, Status s, uint32_t code) {
      if (s != BLECharacteristicCallbacks::SUCCESS_NOTIFY) ble_notify_failed = true;
  }
};
// ==========================================
















// #define DEBUG_ENABLED // Uncomment when debugging ONLY - will corrupt data otherwise
















#ifdef DEBUG_ENABLED
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__) // accepts variable arguments
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#endif
















// --- Packet/Protocol Global Variables ---
const uint8_t ADS1299_NUM_STATUS_BYTES = 3;
const uint8_t ADS1299_NUM_CHANNELS = 8;
const uint8_t ADS1299_BYTES_PER_CHANNEL = 3;
const uint8_t ADS1299_TOTAL_DATA_BYTES = ADS1299_NUM_STATUS_BYTES + (ADS1299_NUM_CHANNELS * ADS1299_BYTES_PER_CHANNEL);
const uint8_t PACKET_TIMESTAMP_BYTES = 4;
const uint8_t PACKET_START_MARKER_BYTES = 2;
const uint8_t PACKET_END_MARKER_BYTES = 2;
const uint8_t PACKET_LENGTH_FIELD_BYTES = 1;
const uint8_t PACKET_CHECKSUM_BYTES = 1;
const uint8_t PACKET_MSG_LENGTH = PACKET_TIMESTAMP_BYTES + ADS1299_TOTAL_DATA_BYTES;
const uint8_t PACKET_TOTAL_SIZE = PACKET_START_MARKER_BYTES + PACKET_LENGTH_FIELD_BYTES + PACKET_MSG_LENGTH + PACKET_CHECKSUM_BYTES + PACKET_END_MARKER_BYTES;
const uint8_t PACKET_IDX_START_MARKER = 0;
const uint8_t PACKET_IDX_LENGTH = PACKET_IDX_START_MARKER + PACKET_START_MARKER_BYTES;
const uint8_t PACKET_IDX_TIMESTAMP = PACKET_IDX_LENGTH + PACKET_LENGTH_FIELD_BYTES;
const uint8_t PACKET_IDX_ADS1299_DATA = PACKET_IDX_TIMESTAMP + PACKET_TIMESTAMP_BYTES;
const uint8_t PACKET_IDX_CHECKSUM = PACKET_IDX_ADS1299_DATA + ADS1299_TOTAL_DATA_BYTES;
const uint8_t PACKET_IDX_END_MARKER = PACKET_IDX_CHECKSUM + PACKET_CHECKSUM_BYTES;

// --- BLE notification staging ---
// Samples wait in the ring until the link can take them. ble_tx_buf holds the
// one batch being sent, which stays there until the stack has accepted all of
// it, so a discarded sample can never split a packet already partly sent.
// Everything here is touched from loop() only.
static uint8_t ble_ring[(uint32_t)BLE_RING_PACKETS * PACKET_TOTAL_SIZE];
uint16_t ble_ring_head = 0;         // next packet to write
uint16_t ble_ring_tail = 0;         // oldest packet waiting
uint16_t ble_ring_count = 0;        // packets waiting
uint16_t ble_ring_peak = 0;         // deepest backlog since the last [STATS]

uint8_t  ble_tx_buf[BLE_MAX_PACKETS_PER_NOTIFY * PACKET_TOTAL_SIZE];
uint16_t ble_tx_len = 0;            // bytes in the batch being sent
uint16_t ble_tx_off = 0;            // bytes of it the stack has accepted
uint32_t ble_retry_at_us = 0;       // no notify before this, after a refusal
bool     ble_retry_wait = false;
uint16_t ble_notify_bytes = PACKET_TOTAL_SIZE;  // flush threshold, from the MTU
uint16_t ble_chunk_max = PACKET_TOTAL_SIZE;     // max bytes per notification
unsigned long ble_tx_first_ms = 0;  // when the oldest staged packet arrived
















// --- Pin Mapping ---
static const uint8_t pin_BATT_MON = 1;     // ADC1_CH0, battery voltage monitor
static const uint8_t pin_TRIGGER_IN = 6;   // ADC1_CH5, reserved for future use
static const uint8_t pin_CS_NUM = 10;
static const uint8_t pin_MOSI_NUM = 11;
static const uint8_t pin_SCK_NUM = 12;
static const uint8_t pin_MISO_NUM = 13;
static const uint8_t pin_DRDY_NUM = 14;
static const uint8_t pin_PWDN_NUM = 15;
static const uint8_t pin_RST_NUM = 16;
static const uint8_t pin_START_NUM = 17;
static const uint8_t pin_LED_DEBUG = 18;

// --- SD Card Pin Mapping ---
static const uint8_t pin_SD_CLK = 35;   // SD_CLK / SCLK
static const uint8_t pin_SD_CMD = 36;   // SD_CMD / MOSI
static const uint8_t pin_SD_DAT0 = 37;  // SD_DAT0 / MISO
static const uint8_t pin_SD_CS = 38;
















// --- SPI instance ---
SPIClass *vspi = NULL;
















// Query Timing Setup
unsigned long _last_query_time = 0;
static const int SPI_FREQ = 4000000;
static const int SAMPLE_FREQ = 250;
static const float SAMPLE_PRD_us = (1.0 / SAMPLE_FREQ) * 1000000;
















// --- ADS1299 State Management ---
int _ADS1299_MODE = -2;
int ADS1299_MODE_SDATAC = 1;
int ADS1299_MODE_RDATAC = 2;
int _ADS1299_PREV_CMD = -1;
int _CMD_ADC_WREG = 3;
int _CMD_ADC_RREG = 4;
int _CMD_ADC_SDATAC = 17;
int _CMD_ADC_RDATAC = 16;
int _CMD_ADC_START = 8;
















// --- Interrupt Flag & Timestamp ---
volatile bool dataReady = false;
unsigned long _unix_timestamp_reference = 0;
unsigned long _millis_reference = 0;
bool _timestamp_initialized = false;
















// --- Function Prototypes ---
void ADS1299_WREG(uint8_t regAdd, uint8_t *values, uint8_t numRegs);
void ADS1299_RREG(uint8_t regAdd, uint8_t *buffer, uint8_t numRegs);
void ADS1299_SETUP(void);
void ADS1299_SDATAC(void);
void ADS1299_RDATAC(void);
void ADS1299_START(void);
byte SPI_SendByte(byte data_byte, bool cont);
void read_ADS1299_data(byte *buffer);
void IRAM_ATTR onDRDYFalling(void);
void print_all_ADS1299_registers_from_setup(void);
uint32_t get_baud_rate_from_config(uint8_t config_val);

// ============================================================
// === SD CARD LOGGING SECTION - START ========================
// ============================================================
#include "SD_MMC.h"
#include "SD.h"
#include "FS.h"

typedef struct {
    uint32_t timestamp_ms;
    uint8_t ads_data[27]; // ADS1299_TOTAL_DATA_BYTES = 3 status + 8*3 channels
} sd_sample_t;

typedef struct {
    bool sd_available;
    bool logging_active;
    File file;
    int file_number;
    uint32_t samples_written;
} sd_state_t;

sd_state_t sd_state = {false, false, File(), 0, 0};
fs::FS *sd_fs = NULL; // Points to whichever filesystem initialized successfully
QueueHandle_t sd_queue = NULL;
volatile uint32_t sd_dropped_count = 0;

bool sd_init() {
    // No GPIO-level card detect — both inserted and absent cards read HIGH on
    // DAT0 with a pull-up, making the check unreliable.  Instead just attempt
    // SD_MMC.begin() directly.  The ~3 s timeout when no card is present is
    // acceptable because sd_init() runs on core 0 via sd_setup_task and never
    // blocks the data-streaming loop on core 1.
    SD_MMC.setPins(pin_SD_CLK, pin_SD_CMD, pin_SD_DAT0);
    if (SD_MMC.begin("/sdcard", true)) {
        DEBUG_PRINTLN("SD: SDMMC 1-bit mode OK");
        sd_fs = &SD_MMC;
        return true;
    }
    DEBUG_PRINTLN("SD: SDMMC init failed, no card?");
    return false;
}

void sd_get_next_filename(char *buf, size_t len) {
    for (int i = 1; i <= 999; i++) {
        snprintf(buf, len, "/REC_%03d.csv", i);
        if (!sd_fs->exists(buf)) {
            sd_state.file_number = i;
            return;
        }
    }
    // All 999 used, overwrite last
    snprintf(buf, len, "/REC_999.csv");
    sd_state.file_number = 999;
}

bool sd_open_new_file() {
    char filename[20];
    sd_get_next_filename(filename, sizeof(filename));
    sd_state.file = sd_fs->open(filename, FILE_WRITE);
    if (!sd_state.file) {
        DEBUG_PRINT("SD: Failed to open "); DEBUG_PRINTLN(filename);
        sd_state.logging_active = false;
        return false;
    }
    sd_state.file.println("timestamp_ms,status,ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8");
    sd_state.logging_active = true;
    sd_state.samples_written = 0;
    DEBUG_PRINT("SD: Recording to "); DEBUG_PRINTLN(filename);
    return true;
}

static int32_t sd_convert_24bit(const uint8_t *b) {
    int32_t val = ((int32_t)b[0] << 16) | ((int32_t)b[1] << 8) | b[2];
    if (val & 0x800000) val |= 0xFF000000; // sign extend
    return val;
}

void sd_log_task(void *param) {
    sd_sample_t sample;
    unsigned long last_flush = millis();
    uint32_t last_logged_drops = 0;
    char line[160];

    while (true) {
        if (xQueueReceive(sd_queue, &sample, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (!sd_state.logging_active) continue;

            // Log any dropped samples as a comment
            uint32_t drops = sd_dropped_count;
            if (drops > last_logged_drops) {
                uint32_t new_drops = drops - last_logged_drops;
                last_logged_drops = drops;
                char drop_line[48];
                snprintf(drop_line, sizeof(drop_line), "# DROPPED %lu samples\n", (unsigned long)new_drops);
                sd_state.file.print(drop_line);
            }

            // Extract status (3 bytes) and 8 channels
            uint32_t status = ((uint32_t)sample.ads_data[0] << 16) |
                              ((uint32_t)sample.ads_data[1] << 8) |
                               (uint32_t)sample.ads_data[2];

            int32_t ch[8];
            for (int i = 0; i < 8; i++) {
                ch[i] = sd_convert_24bit(&sample.ads_data[3 + i * 3]);
            }

            snprintf(line, sizeof(line),
                     "%lu,0x%06lX,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\n",
                     (unsigned long)sample.timestamp_ms,
                     (unsigned long)status,
                     (long)ch[0], (long)ch[1], (long)ch[2], (long)ch[3],
                     (long)ch[4], (long)ch[5], (long)ch[6], (long)ch[7]);

            size_t written = sd_state.file.print(line);
            if (written == 0) {
                // Write failed — SD removed or full
                sd_state.logging_active = false;
                sd_state.file.close();
                DEBUG_PRINTLN("SD: Write failed, logging stopped");
                continue;
            }
            sd_state.samples_written++;
        }

        // Flush every 1 second
        if (sd_state.logging_active && (millis() - last_flush >= 1000)) {
            sd_state.file.flush();
            last_flush = millis();
        }
    }
}
// --- SD background init task: runs entirely on core 0 so it never blocks loop() ---
void sd_setup_task(void *param) {
    if (sd_init()) {
        sd_state.sd_available = true;
        sd_queue = xQueueCreate(256, sizeof(sd_sample_t));
        if (sd_queue != NULL) {
            sd_open_new_file();
            if (sd_state.logging_active) {
                xTaskCreatePinnedToCore(sd_log_task, "SD_Log", 8192, NULL, 1, NULL, 0);
            } else {
                vQueueDelete(sd_queue);
                sd_queue = NULL;
            }
        }
    }
    vTaskDelete(NULL); // self-delete when done
}
// ============================================================
// === SD CARD LOGGING SECTION - END ==========================
// ============================================================














// --- Register Setup ---
typedef struct Deez { int add; int reg_val; } regVal_pair;
const int size_reg_ls = 24;
static const regVal_pair ADS1299_REGISTER_LS[size_reg_ls] = {
  {0x01, 0b10110110}, {0x02, 0b11010000}, {0x03, 0b11101100}, {0x04, 0}, {-2, -2},
  {0x05, 0b01100000}, {0x06, 0b01100000}, {0x07, 0b01100000}, {0x08, 0b01100000},
  {0x09, 0b01100000}, {0x0A, 0b01100000}, {0x0B, 0b01100000}, {0x0C, 0b01100000},
  {0x0D, 0b00000001}, {0x0E, 0b00000001}, {0x0F, 0}, {0x10, 0}, {0x11, 0}, {-2, -2},
  {0x15, 0b00000000}, {0x16, 0}, {0x17, 0} // DO NOT EDIT 0x15 (MISC1) - use the montage switch on the PCB to select between SRB1 and differential mode. Modifying this value will stop the device from working.
};
















// --- Handshake from Computer ---
int handshake_packet_size = 12;
const uint8_t MSG_TYPE_TIMESTAMP = 0x02;
const uint8_t HANDSHAKE_START_MARKER_1 = 0xAA;
const uint8_t HANDSHAKE_START_MARKER_2 = 0xBB;
const uint8_t HANDSHAKE_END_MARKER_1 = 0xCC;
const uint8_t HANDSHAKE_END_MARKER_2 = 0xDD;
const uint8_t RING_BUFFER_SIZE = 24;
uint8_t ring_buffer[RING_BUFFER_SIZE];
uint8_t ring_head = 0, ring_tail = 0, ring_counter = 0;
















bool waitForTimestamp() {
  while (Serial.available() > 0 && ring_counter < RING_BUFFER_SIZE) {
      ring_buffer[ring_head] = Serial.read(); ring_head = (ring_head + 1) % RING_BUFFER_SIZE; ring_counter++;
  }
  if (ring_counter < handshake_packet_size) return false;
  for (uint8_t i = 0; i < handshake_packet_size + 3; i++) {
      uint8_t ring_index = (ring_tail + i) % RING_BUFFER_SIZE;
      if (ring_buffer[ring_index] == HANDSHAKE_START_MARKER_1 && ring_buffer[(ring_index + 1) % RING_BUFFER_SIZE] == HANDSHAKE_START_MARKER_2 && ring_buffer[(ring_index + 2) % RING_BUFFER_SIZE] == MSG_TYPE_TIMESTAMP) {
          uint32_t received_timestamp = (uint32_t)ring_buffer[(ring_index + 3) % RING_BUFFER_SIZE] << 24 | (uint32_t)ring_buffer[(ring_index + 4) % RING_BUFFER_SIZE] << 16 | (uint32_t)ring_buffer[(ring_index + 5) % RING_BUFFER_SIZE] << 8 | (uint32_t)ring_buffer[(ring_index + 6) % RING_BUFFER_SIZE];
          uint8_t reg_addr = ring_buffer[(ring_index + 7) % RING_BUFFER_SIZE];
          uint8_t reg_val = ring_buffer[(ring_index + 8) % RING_BUFFER_SIZE];
          if (reg_addr == 0x01) {
              uint32_t new_baud_rate = get_baud_rate_from_config(reg_val);
              if (new_baud_rate > 0) {
                  while (Serial.available() > 0) { Serial.read(); }
                  Serial.flush(); delay(100); Serial.begin(new_baud_rate); delay(100);
              }
          }
          _unix_timestamp_reference = received_timestamp; _millis_reference = millis(); _timestamp_initialized = true;
          delay(100); return true;
      }
  }
  return false;
}
















void IRAM_ATTR onDRDYFalling(void) {
  if (!streaming_allowed) return;
  // Previous sample was never read out - the ADC has already overwritten it.
  if (dataReady) adc_overrun_count++;
  dataReady = true;
}
















byte SPI_SendByte(byte data_byte, bool cont) {
  if (!cont) { digitalWrite(pin_CS_NUM, LOW); delayMicroseconds(1); }
  byte received = vspi->transfer(data_byte);
  if (!cont) { delayMicroseconds(1); digitalWrite(pin_CS_NUM, HIGH); }
  return received;
}
















void ADS1299_WREG(uint8_t regAdd, uint8_t *values, uint8_t numRegs) {
  if (_ADS1299_MODE != ADS1299_MODE_SDATAC) ADS1299_SDATAC();
  digitalWrite(pin_CS_NUM, LOW);
  delayMicroseconds(1); // FIX #1: Enforce CS setup time
  SPI_SendByte(0b01000000 | regAdd, true);
  SPI_SendByte(numRegs - 1, true);
  for (uint8_t i = 0; i < numRegs; i++) SPI_SendByte(values[i], true);
  delayMicroseconds(1); // FIX #1: Enforce CS hold time
  digitalWrite(pin_CS_NUM, HIGH);
  _ADS1299_PREV_CMD = _CMD_ADC_WREG;
}
















void ADS1299_RREG(uint8_t regAdd, uint8_t *buffer, uint8_t numRegs) {
  if (_ADS1299_MODE != ADS1299_MODE_SDATAC) ADS1299_SDATAC();
  digitalWrite(pin_CS_NUM, LOW);
  delayMicroseconds(1); // FIX #1: Enforce CS setup time
  SPI_SendByte(0b00100000 | regAdd, true);
  SPI_SendByte(numRegs - 1, true);
  for (uint8_t i = 0; i < numRegs; i++) buffer[i] = SPI_SendByte(0x00, true);
  delayMicroseconds(1); // FIX #1: Enforce CS hold time
  digitalWrite(pin_CS_NUM, HIGH);
  _ADS1299_PREV_CMD = _CMD_ADC_RREG;
}
















void ADS1299_SDATAC(void) {
  digitalWrite(pin_CS_NUM, LOW);
  delayMicroseconds(1); // FIX #1
  SPI_SendByte(_CMD_ADC_SDATAC, true);
  delayMicroseconds(1); // FIX #1
  digitalWrite(pin_CS_NUM, HIGH);
  _ADS1299_MODE = ADS1299_MODE_SDATAC;
  _ADS1299_PREV_CMD = _CMD_ADC_SDATAC;
}
















void ADS1299_RDATAC(void) {
  digitalWrite(pin_CS_NUM, LOW);
  delayMicroseconds(1); // FIX #1
  SPI_SendByte(_CMD_ADC_RDATAC, true);
  delayMicroseconds(1); // FIX #1
  digitalWrite(pin_CS_NUM, HIGH);
  _ADS1299_MODE = ADS1299_MODE_RDATAC;
  _ADS1299_PREV_CMD = _CMD_ADC_RDATAC;
}
















void ADS1299_START(void) {
  digitalWrite(pin_CS_NUM, LOW);
  delayMicroseconds(1); // FIX #1
  SPI_SendByte(_CMD_ADC_START, true);
  delayMicroseconds(1); // FIX #1
  digitalWrite(pin_CS_NUM, HIGH);
  _ADS1299_PREV_CMD = _CMD_ADC_START;
}
















void ADS1299_SETUP(void) {
  digitalWrite(pin_PWDN_NUM, LOW);
  digitalWrite(pin_RST_NUM, LOW);
  delay(100);
  digitalWrite(pin_PWDN_NUM, HIGH);
  digitalWrite(pin_RST_NUM, HIGH);
  delay(1000);
  ADS1299_SDATAC();
  uint8_t refbuf[] = {0b11101100};
  ADS1299_WREG(0x03, refbuf, 1);
  delay(10);
  uint8_t value[1];
  uint8_t i = 0;
  while (i < size_reg_ls) {
      const regVal_pair temp = ADS1299_REGISTER_LS[i];
      if (temp.add == -2) { i++; continue; }
      value[0] = {(uint8_t)temp.reg_val};
      ADS1299_WREG(temp.add, value, 1);
      delayMicroseconds(10);
      i++;
  }
}
















void read_ADS1299_data(byte *buffer) {
  digitalWrite(pin_CS_NUM, LOW);
  delayMicroseconds(1); // FIX #1
  for (int i = 0; i < ADS1299_TOTAL_DATA_BYTES; i++) {
      buffer[i] = SPI_SendByte(0x00, true);
  }
  delayMicroseconds(1); // FIX #1
  digitalWrite(pin_CS_NUM, HIGH);
}
















void print_all_ADS1299_registers_from_setup(void) {
  DEBUG_PRINTLN("---- ADS1299 Register Dump ----");
  for (int i = 0; i < size_reg_ls; i++) {
      int reg_addr = ADS1299_REGISTER_LS[i].add;
      if (reg_addr == -2) { i++; continue; }
      uint8_t reg_val[1];
      ADS1299_RREG((uint8_t)reg_addr, reg_val, 1);
      DEBUG_PRINT("Register 0x");
      if (reg_addr < 0x10) DEBUG_PRINT("0");
      DEBUG_PRINT(reg_addr, HEX);
      DEBUG_PRINT(" : ");
      uint16_t val_for_print = 0x100 | reg_val[0];
      DEBUG_PRINTLN(val_for_print, BIN);
      delayMicroseconds(2);
  }
  DEBUG_PRINTLN("-------------------------------");
}
















uint32_t get_baud_rate_from_config(uint8_t config_val) {
  switch (config_val) {
      case 0x00: return 9600;   case 0x01: return 19200;  case 0x02: return 38400;
      case 0x03: return 57600;  case 0x04: return 115200; case 0x05: return 230400;
      case 0x06: return 460800; case 0x07: return 921600; default: return 0;
  }
}
















// --- Acquisition control -------------------------------------------------
// The WiFi host firmware holds the ADS1299 in reset-idle (START low) until
// a client attaches, then runs the settle sequence. That timing is what
// makes the wireless stream clean, so it is reproduced here rather than
// free-running the ADC from setup() the way the USB firmware does. Over BLE
// the trigger is the host's CMD_START write instead of a TCP connect.
// --- BLE transmit path -----------------------------------------------------
// Work out how many packets fit in one notification, from the ATT MTU the host
// told us it negotiated. 3 bytes of that MTU are the ATT notification header.
void ble_configure_batching(void) {
  uint16_t payload = (ble_host_mtu > 3) ? (ble_host_mtu - 3) : 20;
  uint16_t n = payload / PACKET_TOTAL_SIZE;
  if (n < 1) n = 1;
  if (n > BLE_MAX_PACKETS_PER_NOTIFY) n = BLE_MAX_PACKETS_PER_NOTIFY;
  ble_notify_bytes = n * PACKET_TOTAL_SIZE;
  ble_chunk_max = payload;                     // never exceed the MTU per notify
  if (ble_chunk_max > ble_notify_bytes) ble_chunk_max = ble_notify_bytes;
}

// Move the next batch out of the ring once a full notification is waiting, or
// the oldest packet has waited BLE_NOTIFY_FLUSH_MS, then offer it to the stack.
// Only what the stack accepts is consumed, so a refusal costs time and no data.
void ble_service(void) {
  if (!bleConnected || dataChar == NULL) return;

  if (ble_tx_len == 0) {
      uint16_t batch = ble_notify_bytes / PACKET_TOTAL_SIZE;
      if (ble_ring_count == 0) return;
      if (ble_ring_count < batch && (millis() - ble_tx_first_ms) < BLE_NOTIFY_FLUSH_MS) return;
      if (batch > ble_ring_count) batch = ble_ring_count;
      for (uint16_t i = 0; i < batch; i++) {
          memcpy(&ble_tx_buf[ble_tx_len], &ble_ring[(uint32_t)ble_ring_tail * PACKET_TOTAL_SIZE], PACKET_TOTAL_SIZE);
          ble_tx_len += PACKET_TOTAL_SIZE;
          ble_ring_tail = (ble_ring_tail + 1) % BLE_RING_PACKETS;
      }
      ble_ring_count -= batch;
      ble_tx_off = 0;
      ble_tx_first_ms = millis();
  }

  if (ble_retry_wait) {
      if ((int32_t)(micros() - ble_retry_at_us) < 0) return;
      ble_retry_wait = false;
  }

  uint16_t n = ble_tx_len - ble_tx_off;
  if (n > ble_chunk_max) n = ble_chunk_max;

  ble_notify_failed = false;
  dataChar->setValue(&ble_tx_buf[ble_tx_off], n);
  dataChar->notify();

  if (ble_notify_failed) {
      ble_notify_retries++;
      ble_retry_at_us = micros() + BLE_RETRY_US;
      ble_retry_wait = true;
      return;
  }

  ble_tx_off += n;
  if (ble_tx_off >= ble_tx_len) {
      ble_sent_samples += ble_tx_len / PACKET_TOTAL_SIZE;
      ble_tx_len = 0;
      ble_tx_off = 0;
  }
}

// Add one packet to the ring. When it is full the oldest packet is discarded,
// so a link that never recovers still delivers fresh data with bounded latency.
void ble_queue_packet(const byte *packet) {
  if (ble_ring_count == BLE_RING_PACKETS) {
      ble_ring_tail = (ble_ring_tail + 1) % BLE_RING_PACKETS;
      ble_ring_count--;
      ble_dropped_samples++;
  }
  if (ble_ring_count == 0) ble_tx_first_ms = millis();
  memcpy(&ble_ring[(uint32_t)ble_ring_head * PACKET_TOTAL_SIZE], packet, PACKET_TOTAL_SIZE);
  ble_ring_head = (ble_ring_head + 1) % BLE_RING_PACKETS;
  ble_ring_count++;
  if (ble_ring_count > ble_ring_peak) ble_ring_peak = ble_ring_count;
}

static void ble_reset_tx(void) {
  ble_ring_head = 0;
  ble_ring_tail = 0;
  ble_ring_count = 0;
  ble_ring_peak = 0;
  ble_tx_len = 0;
  ble_tx_off = 0;
  ble_retry_wait = false;
}

// Called once the host has finished setting up, so these requests do not
// collide with its service discovery and MTU exchange.
static void ble_request_fast_link(void) {
  if (!ble_peer_known) return;
#if defined(CONFIG_BLUEDROID_ENABLED)
  esp_ble_gap_set_preferred_phy(ble_peer_bda, 0, ESP_BLE_GAP_PHY_2M_PREF_MASK,
                                ESP_BLE_GAP_PHY_2M_PREF_MASK, ESP_BLE_GAP_PHY_OPTIONS_NO_PREF);
  esp_ble_gap_set_pkt_data_len(ble_peer_bda, 251);
  esp_ble_gap_read_phy(ble_peer_bda);
#elif defined(CONFIG_NIMBLE_ENABLED)
  ble_gap_set_prefered_le_phy(ble_peer_handle, BLE_GAP_LE_PHY_2M_MASK, BLE_GAP_LE_PHY_2M_MASK, 0);
  link_data_len_rc = ble_gap_set_data_len(ble_peer_handle, 251, 2120);
#endif
}

// NimBLE reports these only through the library's own event handler, so they
// are read back here instead. Bluedroid gets them from ble_gap_observer().
static void ble_poll_link(void) {
#if defined(CONFIG_NIMBLE_ENABLED)
  if (!ble_peer_known) return;
  struct ble_gap_conn_desc desc;
  uint8_t tx_phy = 0, rx_phy = 0;
  if (ble_gap_conn_find(ble_peer_handle, &desc) != 0) return;
  ble_gap_read_le_phy(ble_peer_handle, &tx_phy, &rx_phy);
  if (desc.conn_itvl != link_interval || tx_phy != link_tx_phy || rx_phy != link_rx_phy) {
      link_interval = desc.conn_itvl;
      link_latency  = desc.conn_latency;
      link_timeout  = desc.supervision_timeout;
      link_tx_phy   = tx_phy;
      link_rx_phy   = rx_phy;
      ble_link_changed = true;
  }
#endif
}

void start_acquisition(void) {
  _millis_reference = millis();   // packet timestamps restart at 0 per session

  digitalWrite(pin_START_NUM, HIGH);
  delay(10);                      // let the chip settle before START
  ADS1299_START();

  // CRITICAL: wait for the ADC's digital filter to settle. This takes 4 data
  // periods (4 * 4ms = 16ms). A 20ms delay is safe and reliable.
  delay(20);
  ADS1299_RDATAC();

  dataReady = false;
  streaming_allowed = true;
  digitalWrite(pin_LED_DEBUG, HIGH);
}

void stop_acquisition(void) {
  streaming_allowed = false;
  dataReady = false;
  ble_reset_tx();                 // discard anything still staged
  ADS1299_SDATAC();
  digitalWrite(pin_START_NUM, LOW);
  digitalWrite(pin_LED_DEBUG, LOW);
}

void setup() {
  // Suppress ESP-IDF log output (sdmmc, vfs_fat, etc.) so it never
  // corrupts the binary data protocol on USB CDC Serial.
  esp_log_level_set("*", ESP_LOG_NONE);

  // Over BLE the USB CDC port carries status text only - the binary data
  // protocol goes out as GATT notifications - so it is safe to print here.
  Serial.begin(115200);
  // The USB CDC port stays enumerated by the OS even when no serial monitor is
  // draining it. With the core's default 100 ms tx timeout (and up to 20
  // consecutive retries) a single blocked println can stall this task for ~2 s,
  // which starves the ADC read loop and the BLE flush - the stream degrades over
  // successive sessions and eventually stops. Status text is expendable; the
  // data path is not. A 0 ms timeout drops the text instead of blocking.
  Serial.setTxTimeoutMs(0);
  #ifdef DEBUG_ENABLED
      delay(5000);
  #endif
  // Wait for USB CDC to be ready (host has enumerated the port).
  // Times out after 3 s so standalone SD-only logging still works.
  {
    unsigned long usb_start = millis();
    while (!Serial && (millis() - usb_start < 3000)) { delay(10); }
  }
  pinMode(pin_PWDN_NUM, OUTPUT);
  pinMode(pin_RST_NUM, OUTPUT);
  pinMode(pin_START_NUM, OUTPUT);
  pinMode(pin_CS_NUM, OUTPUT);
  pinMode(pin_DRDY_NUM, INPUT_PULLUP);
  pinMode(pin_LED_DEBUG, OUTPUT);
  digitalWrite(pin_CS_NUM, HIGH);
  delay(2000);
  digitalWrite(pin_LED_DEBUG, LOW);
















  vspi = new SPIClass(FSPI);
  vspi->begin(pin_SCK_NUM, pin_MISO_NUM, pin_MOSI_NUM, pin_CS_NUM);
  vspi->beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE1));
  delay(500);
















  ADS1299_SETUP();
















  // No serial handshake over BLE: Python_ble_LSL.py sends no timestamp
  // packet. The reference is re-zeroed each time the host starts the stream
  // (see loop), which is what the WiFi host firmware does on TCP connect.
  _unix_timestamp_reference = 0;
  _millis_reference = millis();
  _timestamp_initialized = true;

  // --- BLE peripheral ---
  BLEDevice::init(BLE_DEVICE_NAME);
#if defined(CONFIG_BLUEDROID_ENABLED)
  BLEDevice::setCustomGapHandler(ble_gap_observer);
#endif
  BLEDevice::setMTU(517);                   // ask for the largest ATT MTU we can use
  BLEDevice::setPower(ESP_PWR_LVL_P9);      // max TX power - keeps the link solid

  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new CerelogServerCallbacks());

  BLEService *bleService = bleServer->createService(SERVICE_UUID);

  dataChar = bleService->createCharacteristic(DATA_CHAR_UUID,
                                              BLECharacteristic::PROPERTY_NOTIFY);
  // arduino-esp32 3.x creates the CCCD itself; 2.x needs it added by hand.
  if (dataChar->getDescriptorByUUID(BLEUUID((uint16_t)0x2902)) == nullptr) {
      dataChar->addDescriptor(new BLE2902());
  }
  dataChar->setCallbacks(new CerelogDataCallbacks());

  ctrlChar = bleService->createCharacteristic(CTRL_CHAR_UUID,
                                              BLECharacteristic::PROPERTY_WRITE |
                                              BLECharacteristic::PROPERTY_WRITE_NR);
  ctrlChar->setCallbacks(new CerelogCtrlCallbacks());

  bleService->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);   // device name goes in the scan response
  adv->setMinPreferred(0x06);   // preferred connection interval, 7.5 ms
  adv->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("\n=== CERELOG V2 BLE Peripheral ===");
  Serial.print("Name:     "); Serial.println(BLE_DEVICE_NAME);
  Serial.print("Service:  "); Serial.println(SERVICE_UUID);
  Serial.print("Data:     "); Serial.println(DATA_CHAR_UUID);
  Serial.print("Control:  "); Serial.println(CTRL_CHAR_UUID);
  Serial.println("=================================");
 
  print_all_ADS1299_registers_from_setup();
  attachInterrupt(digitalPinToInterrupt(pin_DRDY_NUM), onDRDYFalling, FALLING);
















  // --- SD Card Init: runs on core 0 in background so it never blocks data streaming ---
  xTaskCreatePinnedToCore(sd_setup_task, "SD_Init", 8192, NULL, 1, NULL, 0);

  // --- Startup Sequence ---
  // Unlike the USB firmware, acquisition is NOT started here. The ADS1299 is
  // left idle (START low) until the host starts the stream; see start_acquisition().
  DEBUG_PRINTLN("Setup complete.");
  digitalWrite(pin_START_NUM, LOW);
  Serial.println("Ready. Advertising, waiting for BLE connection...");
  // --- End of Startup Sequence ---
















  digitalWrite(pin_LED_DEBUG, LOW);   // lights up on client connect
}
















void loop() {
  // ============================================================
  // PART 1: BLE connection handling
  // ============================================================
  // Connecting is not enough to start the ADC: the host also has to subscribe
  // to the data characteristic and write CMD_START. That write carries the ATT
  // MTU it negotiated, which sets how many packets go in one notification.
  if (bleConnected && hostRequestedStream && !streaming_allowed) {
      ble_configure_batching();
      ble_reset_tx();             // never emit samples staged before this session
      ble_dropped_samples = 0;
      ble_notify_retries = 0;
      adc_overrun_count = 0;
      ble_sent_samples = 0;
      ble_stats_ms = millis();
      Serial.print(">>> Host started stream - MTU ");
      Serial.print(ble_host_mtu);
      Serial.print(", ");
      Serial.print(ble_notify_bytes / PACKET_TOTAL_SIZE);
      Serial.println(" packet(s) per notification");
      start_acquisition();
      ble_request_fast_link();
  }

  // ============================================================
  // PART 2: Advertising - the BLE stand-in for UDP discovery
  // ============================================================
  if (bleConnected && !ble_was_connected) {
      ble_was_connected = true;
      Serial.println(">>> Central connected");
  } else if (!bleConnected && ble_was_connected) {
      ble_was_connected = false;
      delay(100);                    // let the stack finish tearing the link down
      BLEDevice::startAdvertising();
      Serial.println("Central disconnected - advertising again");
  }

  // ============================================================
  // PART 3: Data streaming
  // ============================================================
  if (!bleConnected || !hostRequestedStream) {
      if (streaming_allowed) {
          Serial.println("Host lost - acquisition stopped");
          stop_acquisition();
      }
      yield();
      return;
  }
  //got rid of the micro check because it was causing 6 second spics, this check is a redundancy to the data ready flag check
  //unsigned long currentMicros = micros();
 // if (currentMicros - _last_query_time >= SAMPLE_PRD_us) {
   //   _last_query_time = currentMicros;
      if (dataReady) {
          dataReady = false;
     
          byte raw_data[ADS1299_TOTAL_DATA_BYTES];
          read_ADS1299_data(raw_data);
















          const uint16_t START_MARKER = 0xABCD;
          const uint16_t END_MARKER = 0xDCBA;
          byte packet[PACKET_TOTAL_SIZE];
















          packet[PACKET_IDX_START_MARKER] = (START_MARKER >> 8) & 0xFF;
          packet[PACKET_IDX_START_MARKER + 1] = START_MARKER & 0xFF;
          packet[PACKET_IDX_LENGTH] = PACKET_MSG_LENGTH;
















           //New way of timestamping added 8_22
          // 1. Perform the calculation using 'double' for maximum precision.
         uint32_t millis_since_sync = millis() - _millis_reference;




         // 2. Pack this integer into the packet in BIG-ENDIAN order, which BrainFlow expects.
         packet[PACKET_IDX_TIMESTAMP]     = (millis_since_sync >> 24) & 0xFF;
         packet[PACKET_IDX_TIMESTAMP + 1] = (millis_since_sync >> 16) & 0xFF;
         packet[PACKET_IDX_TIMESTAMP + 2] = (millis_since_sync >> 8) & 0xFF;
         packet[PACKET_IDX_TIMESTAMP + 3] =  millis_since_sync & 0xFF;
















          for (uint8_t i = 0; i < ADS1299_TOTAL_DATA_BYTES; i++) {
              packet[PACKET_IDX_ADS1299_DATA + i] = raw_data[i];
          }
          uint8_t checksum = 0;
          for (uint8_t i = PACKET_IDX_LENGTH; i < PACKET_IDX_CHECKSUM; i++) {
              checksum += packet[i];
          }
          packet[PACKET_IDX_CHECKSUM] = checksum;
          packet[PACKET_IDX_END_MARKER] = (END_MARKER >> 8) & 0xFF;
          packet[PACKET_IDX_END_MARKER + 1] = END_MARKER & 0xFF;
















          ble_queue_packet(packet);

          // --- SD Card: enqueue sample (non-blocking) ---
          if (sd_queue != NULL) {
              sd_sample_t sample;
              sample.timestamp_ms = millis_since_sync;
              memcpy(sample.ads_data, raw_data, ADS1299_TOTAL_DATA_BYTES);
              if (xQueueSend(sd_queue, &sample, 0) != pdTRUE) {
                  sd_dropped_count++;
              }
          }
      }

  // ============================================================
  // PART 4: Flush staged packets over BLE
  // ============================================================
  // Runs every iteration: it decides when a batch is due, and paces its retries.
  if (streaming_allowed) ble_service();

  // ============================================================
  // PART 5: Link statistics
  // ============================================================
  if (millis() - ble_stats_ms >= BLE_STATS_PERIOD_MS) {
      ble_stats_ms = millis();
      Serial.print("[STATS] sent="); Serial.print(ble_sent_samples);
      Serial.print(" ble_dropped="); Serial.print(ble_dropped_samples);
      Serial.print(" (retries="); Serial.print(ble_notify_retries);
      Serial.print(") adc_overrun="); Serial.print(adc_overrun_count);
      Serial.print(" sd_dropped="); Serial.print(sd_dropped_count);
      Serial.print(" backlog="); Serial.print(ble_ring_count);
      Serial.print(" peak="); Serial.print(ble_ring_peak);
      Serial.print(" free_heap="); Serial.println(ESP.getFreeHeap());
      ble_ring_peak = ble_ring_count;
      ble_poll_link();
  }

  if (ble_link_changed) {
      ble_link_changed = false;
      Serial.print("[LINK] interval="); Serial.print(link_interval * 1.25f);
      Serial.print("ms latency="); Serial.print(link_latency);
      Serial.print(" timeout="); Serial.print(link_timeout * 10);
      Serial.print("ms phy tx/rx="); Serial.print(link_tx_phy);
      Serial.print("/"); Serial.print(link_rx_phy);
#if defined(CONFIG_NIMBLE_ENABLED)
      Serial.print(" data_len_request_rc="); Serial.println(link_data_len_rc);
#else
      Serial.print(" data_len tx/rx="); Serial.print(link_tx_octets);
      Serial.print("/"); Serial.println(link_rx_octets);
#endif
  }

  yield();
 
}




























































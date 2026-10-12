#include <Arduino.h>
#include <SPI.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>   // CCCD. arduino-esp32 3.x adds this descriptor on its own;
                       // the runtime check in setup() keeps both cores working.
#include "esp_log.h"

// ============================================================================
//  16 CHANNEL FIRMWARE - BLE HOST STREAMING
//
//  This is 16CH_EMUL_FW.ino with its serial layer replaced by the BLE
//  peripheral from V2_Devices_BLE_hostfw.ino.  The ADS1299 layer - dual-chip
//  register tables, bias policy, clock chaining, SPI primitives, pin map and
//  SD-card logging - is carried over unchanged.  What is gone is the OpenBCI
//  EMUL wire format: no 33-byte 0xA0 frames, no 'b'/'s' command parser, no
//  USB-OTG "VCP" product string, and no 125 Hz downsampling (that existed only
//  because BrainFlow's 16-channel serial board assumes 125 Hz - BLE has no such
//  constraint, so all 16 channels go out at the native 250 SPS).
//
//  In its place the board advertises as a BLE peripheral and emits the same
//  Cerelog packet the V1/V2 BLE firmwares emit, widened to 16 channels.
//  Python_ble_LSL.py handles all three boards; it tells them apart by the
//  advertised device name, with the packet's length field as a fallback.
//
//  *** REQUIRED Arduino IDE settings (same as V2_Devices_BLE_hostfw) ***
//    Tools -> Board            : ESP32S3 Dev Module
//    Tools -> USB CDC On Boot  : Enabled
//    Tools -> USB Mode         : Hardware CDC and JTAG
//
//  Unlike 16CH_EMUL_FW this sketch does NOT need USB-OTG (TinyUSB): nothing is
//  pretending to be a "VCP" serial port for the OpenBCI GUI any more.  The USB
//  CDC port carries status text only - the data goes out over GATT - so plain
//  hardware CDC is both sufficient and what the other BLE sketches use.
//
//  --- WIRE FORMAT (61 bytes) ------------------------------------------------
//    bytes  0-1    0xABCD                      start marker
//    byte   2      55                           message length (4 + 51)
//    bytes  3-6    uint32 BE, ms since boot     device sample clock
//    bytes  7-9    ADC1 status word             (ADC2's status goes to SD only)
//    bytes 10-57   16 channels x 24-bit BE      ch1..8 = ADC1, ch9..16 = ADC2
//    byte   58     checksum, sum(bytes 2..57) & 0xFF
//    bytes 59-60   0xDCBA                       end marker
//
//  The 8-channel boards send the identical layout with 8 channels and a
//  length field of 31, which is what makes one host parser serve all three.
// ============================================================================


// ==========================================
// ===  BLE PERIPHERAL CONFIGURATION      ===
// ==========================================
// These MUST match Python_ble_LSL.py. The host scans for BLE_DEVICE_NAME /
// SERVICE_UUID, subscribes to DATA_CHAR_UUID and then writes CMD_START to
// CTRL_CHAR_UUID. That write is the BLE equivalent of the 'b' command that
// started the stream in the USB firmware.
//
// The service and characteristic UUIDs are deliberately the SAME as the
// V1/V2 firmwares so one scan filter finds any Cerelog board. Only the
// advertised name differs, and that is what tells the host this board sends
// 16-channel packets: the host has to size its LSL outlet before the first
// sample arrives, so it cannot wait and find out from the data.
const char* BLE_DEVICE_NAME = "CERELOG_EEG_16";
#define SERVICE_UUID    "7a1e8b00-9c3d-4b7e-8f21-6d0a5c2e91f3"
#define DATA_CHAR_UUID  "7a1e8b01-9c3d-4b7e-8f21-6d0a5c2e91f3"  // notify: EEG packets
#define CTRL_CHAR_UUID  "7a1e8b02-9c3d-4b7e-8f21-6d0a5c2e91f3"  // write:  start/stop

const uint8_t CMD_START = 0x01;   // payload: [0x01, mtu_hi, mtu_lo]
const uint8_t CMD_STOP  = 0x00;

// BLE moves far less data per second than USB, so packets are coalesced into
// one notification instead of being written out one at a time. 8 * 61 = 488
// bytes, which fits the 517-byte ATT MTU that macOS/Windows/BlueZ negotiate.
// (The 8-channel firmware stages 13 of its 37-byte packets for the same 481
// bytes - same notification size, wider packets.)
const uint8_t  BLE_MAX_PACKETS_PER_NOTIFY = 8;
const uint16_t BLE_NOTIFY_FLUSH_MS = 25;   // bound on staging latency
const uint32_t BLE_STATS_PERIOD_MS = 5000; // how often drop counters are printed

BLEServer         *bleServer = NULL;
BLECharacteristic *dataChar  = NULL;
BLECharacteristic *ctrlChar  = NULL;

volatile bool     bleConnected = false;
volatile bool     hostRequestedStream = false;  // set by the CMD_START write
volatile uint16_t ble_host_mtu = 23;            // ATT MTU as reported by the host
bool              ble_was_connected = false;

// --- Drop accounting -------------------------------------------------------
// The BLE link can refuse packets when the controller's buffers back up. When
// that happens the staged batch is thrown away rather than queued, so the
// stream always stays live instead of drifting further and further behind.
volatile uint32_t ble_dropped_samples  = 0;   // samples lost to a failed notify
volatile uint32_t ble_dropped_notifies = 0;   // notifications the stack refused
volatile uint32_t adc_overrun_count    = 0;   // DRDY fired before loop() read the last sample
uint32_t          ble_sent_samples     = 0;
unsigned long     ble_stats_ms         = 0;   // last time counters were printed
volatile bool     ble_notify_failed    = false;  // set from the onStatus callback

// Set true only while a host is subscribed and streaming. Gates the BLE
// transmit path - NOT the ADC. See the note above start_streaming().
volatile bool streaming_allowed = false;

void start_streaming(void);
void stop_streaming(void);

// --- BLE callbacks ---------------------------------------------------------
class CerelogServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
      bleConnected = true;
  }
  void onDisconnect(BLEServer *pServer) {
      bleConnected = false;
      hostRequestedStream = false;
  }
  // The ESP32 core ships the BLE library on either Bluedroid or NimBLE, and the
  // two spell the connection handle differently. Both call the plain onConnect
  // above as well, so this overload only has to do the parameter update: ask
  // for the fastest connection interval the central will grant (7.5-15 ms),
  // which at 250 Hz x 16 channels is what keeps the link ahead of the ADC.
#if defined(CONFIG_BLUEDROID_ENABLED)
  void onConnect(BLEServer *pServer, esp_ble_gatts_cb_param_t *param) {
      bleConnected = true;
      pServer->updateConnParams(param->connect.remote_bda, 6, 12, 0, 200);
  }
#elif defined(CONFIG_NIMBLE_ENABLED)
  void onConnect(BLEServer *pServer, ble_gap_conn_desc *desc) {
      bleConnected = true;
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
// stack would not take the packet, i.e. those samples are gone.
class CerelogDataCallbacks : public BLECharacteristicCallbacks {
  void onStatus(BLECharacteristic *pChar, Status s, uint32_t code) {
      if (s != BLECharacteristicCallbacks::SUCCESS_NOTIFY) ble_notify_failed = true;
  }
};
// ==========================================


// #define DEBUG_ENABLED // Uncomment when debugging ONLY


#ifdef DEBUG_ENABLED
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__) // accepts variable arguments
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#endif




// --- ADS1299 geometry ---
const uint8_t ADS1299_NUM_DEVICES = 2;
const uint8_t ADS1299_NUM_STATUS_BYTES = 3;
const uint8_t ADS1299_NUM_CHANNELS = 8;   // per device
const uint8_t ADS1299_BYTES_PER_CHANNEL = 3;
const uint8_t ADS1299_TOTAL_DATA_BYTES = ADS1299_NUM_STATUS_BYTES + (ADS1299_NUM_CHANNELS * ADS1299_BYTES_PER_CHANNEL);
const uint8_t ADS1299_ALL_DATA_BYTES = ADS1299_TOTAL_DATA_BYTES * ADS1299_NUM_DEVICES;
const uint8_t TOTAL_NUM_CHANNELS = ADS1299_NUM_CHANNELS * ADS1299_NUM_DEVICES; // 16


// --- Packet/Protocol Global Variables ---
// Same framing as the 8-channel BLE firmwares, widened to 16 channels. Only
// ADC1's status word rides on the wire (both are logged to SD); the host
// parser skips the status bytes entirely and only needs the offset to be the
// same on every board, which it is.
const uint8_t PACKET_STATUS_BYTES = ADS1299_NUM_STATUS_BYTES;                  // 3
const uint8_t PACKET_CHANNEL_BYTES = TOTAL_NUM_CHANNELS * ADS1299_BYTES_PER_CHANNEL; // 48
const uint8_t PACKET_DATA_BYTES = PACKET_STATUS_BYTES + PACKET_CHANNEL_BYTES;  // 51
const uint8_t PACKET_TIMESTAMP_BYTES = 4;
const uint8_t PACKET_START_MARKER_BYTES = 2;
const uint8_t PACKET_END_MARKER_BYTES = 2;
const uint8_t PACKET_LENGTH_FIELD_BYTES = 1;
const uint8_t PACKET_CHECKSUM_BYTES = 1;
const uint8_t PACKET_MSG_LENGTH = PACKET_TIMESTAMP_BYTES + PACKET_DATA_BYTES;  // 55
const uint8_t PACKET_TOTAL_SIZE = PACKET_START_MARKER_BYTES + PACKET_LENGTH_FIELD_BYTES + PACKET_MSG_LENGTH + PACKET_CHECKSUM_BYTES + PACKET_END_MARKER_BYTES; // 61
const uint8_t PACKET_IDX_START_MARKER = 0;
const uint8_t PACKET_IDX_LENGTH = PACKET_IDX_START_MARKER + PACKET_START_MARKER_BYTES;
const uint8_t PACKET_IDX_TIMESTAMP = PACKET_IDX_LENGTH + PACKET_LENGTH_FIELD_BYTES;
const uint8_t PACKET_IDX_DATA = PACKET_IDX_TIMESTAMP + PACKET_TIMESTAMP_BYTES;
const uint8_t PACKET_IDX_CHECKSUM = PACKET_IDX_DATA + PACKET_DATA_BYTES;
const uint8_t PACKET_IDX_END_MARKER = PACKET_IDX_CHECKSUM + PACKET_CHECKSUM_BYTES;

// --- BLE notification staging ---
// Whole packets are collected here and pushed out as one notification. The
// buffer is deliberately small: if the link cannot drain it, the batch is
// dropped (and counted) so what does get through is always fresh.
uint8_t  ble_tx_buf[BLE_MAX_PACKETS_PER_NOTIFY * PACKET_TOTAL_SIZE];
uint16_t ble_tx_len = 0;            // bytes currently staged
uint16_t ble_notify_bytes = PACKET_TOTAL_SIZE;  // flush threshold, from the MTU
uint16_t ble_chunk_max = PACKET_TOTAL_SIZE;     // max bytes per notification
unsigned long ble_tx_first_ms = 0;  // when the oldest staged packet arrived




// --- Pin Mapping ---
static const uint8_t pin_BATT_MON = 1;     // ADC1_CH0, battery voltage monitor
static const uint8_t pin_TRIGGER_IN = 6;   // ADC1_CH5, reserved for future use
static const uint8_t pin_CS1_NUM = 10;     // Chip select, ADC1 (master / clock source / bias amp)
static const uint8_t pin_CS2_NUM = 48;     // Chip select, ADC2 (follower / external clock)
static const uint8_t pin_MOSI_NUM = 11;
static const uint8_t pin_SCK_NUM = 12;
static const uint8_t pin_MISO_NUM = 13;
static const uint8_t pin_DRDY_NUM = 14;    // shared by both devices
static const uint8_t pin_PWDN_NUM = 15;    // shared by both devices
static const uint8_t pin_RST_NUM = 16;     // shared by both devices
static const uint8_t pin_START_NUM = 17;   // shared by both devices - hardware sync
static const uint8_t pin_LED_DEBUG = 18;

// --- SD Card Pin Mapping ---
static const uint8_t pin_SD_CLK = 35;   // SD_CLK / SCLK
static const uint8_t pin_SD_CMD = 36;   // SD_CMD / MOSI
static const uint8_t pin_SD_DAT0 = 37;  // SD_DAT0 / MISO
static const uint8_t pin_SD_CS = 38;




// --- SPI instance ---
SPIClass *vspi = NULL;

static const int SPI_FREQ = 4000000;
static const int SAMPLE_FREQ = 250;




// --- Register Setup ---
typedef struct Deez { int add; int reg_val; } regVal_pair;
const int size_reg_ls = 24;

// ---------------- ADC1 (master) ----------------
// 0x01 CONFIG1 = 0b1011_0110 : CLK_EN = 1 -> internal oscillator routed OUT of
//                              the CLK pin to feed ADC2.  DR = 110 (250 SPS).
// 0x03 CONFIG3 = 0b1110_1100 : PD_REFBUF = 1, BIASREF_INT = 1, PD_BIAS = 1
//                              -> the ONLY bias amplifier that is powered.
// 0x0D/0x0E BIAS_SENSP/N = 0b0000_0001 -> only CH1P and CH1N feed the bias loop.
static const regVal_pair ADS1299_REGISTER_LS[size_reg_ls] = {
  {0x01, 0b10110110}, {0x02, 0b11010000}, {0x03, 0b11101100}, {0x04, 0}, {-2, -2},
  {0x05, 0b01100000}, {0x06, 0b01100000}, {0x07, 0b01100000}, {0x08, 0b01100000},
  {0x09, 0b01100000}, {0x0A, 0b01100000}, {0x0B, 0b01100000}, {0x0C, 0b01100000},
  {0x0D, 0b00000001}, {0x0E, 0b00000001}, {0x0F, 0}, {0x10, 0}, {0x11, 0}, {-2, -2},
  {0x15, 0b00000000}, {0x16, 0}, {0x17, 0} // DO NOT EDIT 0x15 (MISC1) - use the montage switch on the PCB to select between SRB1 and differential mode. Modifying this value will stop the device from working.
};

// ---------------- ADC2 (follower) ----------------
// 0x01 CONFIG1 = 0b1001_0110 : CLK_EN = 0.  ADC2's CLK pin is an INPUT fed by
//                              ADC1, so it must not drive the shared clock net.
// 0x03 CONFIG3 = 0b1110_1000 : PD_BIAS = 0 -> ADC2's bias amplifier POWERED DOWN.
// 0x0D/0x0E BIAS_SENSP/N = 0  -> no ADC2 channel enters any bias loop.
static const regVal_pair ADS1299_REGISTER_LS_2[size_reg_ls] = {
  {0x01, 0b10010110}, {0x02, 0b11010000}, {0x03, 0b11101000}, {0x04, 0}, {-2, -2},
  {0x05, 0b01100000}, {0x06, 0b01100000}, {0x07, 0b01100000}, {0x08, 0b01100000},
  {0x09, 0b01100000}, {0x0A, 0b01100000}, {0x0B, 0b01100000}, {0x0C, 0b01100000},
  {0x0D, 0b00000000}, {0x0E, 0b00000000}, {0x0F, 0}, {0x10, 0}, {0x11, 0}, {-2, -2},
  {0x15, 0b00000000}, {0x16, 0}, {0x17, 0} // DO NOT EDIT 0x15 (MISC1) - montage switch on the PCB controls SRB1 vs differential mode.
};




// --- ADS1299 State Management ---
int _ADS1299_MODE[ADS1299_NUM_DEVICES] = {-2, -2};
int ADS1299_MODE_SDATAC = 1;
int ADS1299_MODE_RDATAC = 2;
int _ADS1299_PREV_CMD = -1;
int _CMD_ADC_WREG = 3;
int _CMD_ADC_RREG = 4;
int _CMD_ADC_SDATAC = 17;
int _CMD_ADC_RDATAC = 16;
int _CMD_ADC_START = 8;

static inline uint8_t cs_to_index(uint8_t cs) { return (cs == pin_CS1_NUM) ? 0 : 1; }

// Live copy of each device's CHnSET registers, kept so the register tables and
// any future host-side channel command stay in agreement without an extra read.
uint8_t chset_shadow[ADS1299_NUM_DEVICES][ADS1299_NUM_CHANNELS] = {
  {0x60,0x60,0x60,0x60,0x60,0x60,0x60,0x60},
  {0x60,0x60,0x60,0x60,0x60,0x60,0x60,0x60}
};




// --- Interrupt Flag & Timestamp ---
volatile bool dataReady = false;
unsigned long _millis_reference = 0;




// --- Function Prototypes ---
void ADS1299_WREG(uint8_t cs, uint8_t regAdd, uint8_t *values, uint8_t numRegs);
void ADS1299_RREG(uint8_t cs, uint8_t regAdd, uint8_t *buffer, uint8_t numRegs);
void ADS1299_SETUP(void);
static void ADS1299_WRITE_TABLE(uint8_t cs, const regVal_pair *table);
void ADS1299_SDATAC(uint8_t cs);
void ADS1299_RDATAC(uint8_t cs);
void ADS1299_START(uint8_t cs);
byte SPI_SendByte(uint8_t cs, byte data_byte, bool cont);
void read_ADS1299_data(uint8_t cs, byte *buffer);
void read_ADS1299_data_all(byte *buffer);
void IRAM_ATTR onDRDYFalling(void);
void print_all_ADS1299_registers_from_setup(void);

// ============================================================
// === SD CARD LOGGING SECTION - START ========================
// ============================================================
#include "SD_MMC.h"
#include "SD.h"
#include "FS.h"

typedef struct {
    uint32_t timestamp_ms;
    int32_t  ch[16];
    uint32_t status1;
    uint32_t status2;
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
    // ch1..ch8 come from ADC1, ch9..ch16 from ADC2.  Each device has its own
    // 24-bit status word, so both are logged.
    sd_state.file.println("timestamp_ms,status1,ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8,status2,ch9,ch10,ch11,ch12,ch13,ch14,ch15,ch16");
    sd_state.logging_active = true;
    sd_state.samples_written = 0;
    DEBUG_PRINT("SD: Recording to "); DEBUG_PRINTLN(filename);
    return true;
}

void sd_log_task(void *param) {
    sd_sample_t sample;
    unsigned long last_flush = millis();
    uint32_t last_logged_drops = 0;
    char line[320];

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

            snprintf(line, sizeof(line),
                     "%lu,0x%06lX,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,0x%06lX,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\n",
                     (unsigned long)sample.timestamp_ms,
                     (unsigned long)sample.status1,
                     (long)sample.ch[0], (long)sample.ch[1], (long)sample.ch[2], (long)sample.ch[3],
                     (long)sample.ch[4], (long)sample.ch[5], (long)sample.ch[6], (long)sample.ch[7],
                     (unsigned long)sample.status2,
                     (long)sample.ch[8],  (long)sample.ch[9],  (long)sample.ch[10], (long)sample.ch[11],
                     (long)sample.ch[12], (long)sample.ch[13], (long)sample.ch[14], (long)sample.ch[15]);

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




void IRAM_ATTR onDRDYFalling(void) {
  // Previous sample was never read out - the ADC has already overwritten it.
  if (dataReady) adc_overrun_count++;
  dataReady = true;
}




// --- SPI primitives.  Every call takes the chip select of the device it is
//     talking to; only one CS is ever low at a time so the two ADS1299s never
//     both drive the shared MISO line. ---
byte SPI_SendByte(uint8_t cs, byte data_byte, bool cont) {
  if (!cont) { digitalWrite(cs, LOW); delayMicroseconds(1); }
  byte received = vspi->transfer(data_byte);
  if (!cont) { delayMicroseconds(1); digitalWrite(cs, HIGH); }
  return received;
}


void ADS1299_WREG(uint8_t cs, uint8_t regAdd, uint8_t *values, uint8_t numRegs) {
  if (_ADS1299_MODE[cs_to_index(cs)] != ADS1299_MODE_SDATAC) ADS1299_SDATAC(cs);
  digitalWrite(cs, LOW);
  delayMicroseconds(1);
  SPI_SendByte(cs, 0b01000000 | regAdd, true);
  SPI_SendByte(cs, numRegs - 1, true);
  for (uint8_t i = 0; i < numRegs; i++) SPI_SendByte(cs, values[i], true);
  delayMicroseconds(1);
  digitalWrite(cs, HIGH);
  _ADS1299_PREV_CMD = _CMD_ADC_WREG;
}


void ADS1299_RREG(uint8_t cs, uint8_t regAdd, uint8_t *buffer, uint8_t numRegs) {
  if (_ADS1299_MODE[cs_to_index(cs)] != ADS1299_MODE_SDATAC) ADS1299_SDATAC(cs);
  digitalWrite(cs, LOW);
  delayMicroseconds(1);
  SPI_SendByte(cs, 0b00100000 | regAdd, true);
  SPI_SendByte(cs, numRegs - 1, true);
  for (uint8_t i = 0; i < numRegs; i++) buffer[i] = SPI_SendByte(cs, 0x00, true);
  delayMicroseconds(1);
  digitalWrite(cs, HIGH);
  _ADS1299_PREV_CMD = _CMD_ADC_RREG;
}


void ADS1299_SDATAC(uint8_t cs) {
  digitalWrite(cs, LOW);
  delayMicroseconds(1);
  SPI_SendByte(cs, _CMD_ADC_SDATAC, true);
  delayMicroseconds(1);
  digitalWrite(cs, HIGH);
  _ADS1299_MODE[cs_to_index(cs)] = ADS1299_MODE_SDATAC;
  _ADS1299_PREV_CMD = _CMD_ADC_SDATAC;
}


void ADS1299_RDATAC(uint8_t cs) {
  digitalWrite(cs, LOW);
  delayMicroseconds(1);
  SPI_SendByte(cs, _CMD_ADC_RDATAC, true);
  delayMicroseconds(1);
  digitalWrite(cs, HIGH);
  _ADS1299_MODE[cs_to_index(cs)] = ADS1299_MODE_RDATAC;
  _ADS1299_PREV_CMD = _CMD_ADC_RDATAC;
}


// Kept for completeness.  Conversions are started with the shared hardware
// START pin (IO17) so both devices begin on the same edge of the same clock.
void ADS1299_START(uint8_t cs) {
  digitalWrite(cs, LOW);
  delayMicroseconds(1);
  SPI_SendByte(cs, _CMD_ADC_START, true);
  delayMicroseconds(1);
  digitalWrite(cs, HIGH);
  _ADS1299_PREV_CMD = _CMD_ADC_START;
}


static void ADS1299_WRITE_TABLE(uint8_t cs, const regVal_pair *table) {
  uint8_t value[1];
  uint8_t i = 0;
  while (i < size_reg_ls) {
      const regVal_pair temp = table[i];
      if (temp.add == -2) { i++; continue; }
      value[0] = (uint8_t)temp.reg_val;
      ADS1299_WREG(cs, temp.add, value, 1);
      if (temp.add >= 0x05 && temp.add <= 0x0C) {
          chset_shadow[cs_to_index(cs)][temp.add - 0x05] = (uint8_t)temp.reg_val;
      }
      delayMicroseconds(10);
      i++;
  }
}


void ADS1299_SETUP(void) {
  digitalWrite(pin_START_NUM, LOW);

  digitalWrite(pin_PWDN_NUM, LOW);
  digitalWrite(pin_RST_NUM, LOW);
  delay(100);
  digitalWrite(pin_PWDN_NUM, HIGH);
  digitalWrite(pin_RST_NUM, HIGH);
  delay(1000);

  // ADC1 MUST be configured first: a reset clears CONFIG1.CLK_EN, which stops
  // the clock ADC2 runs on, and an ADS1299 cannot decode SPI without a master
  // clock.  Writing ADC1's CONFIG1 restarts ADC2's clock.
  ADS1299_SDATAC(pin_CS1_NUM);
  uint8_t refbuf[] = {0b11101100};
  ADS1299_WREG(pin_CS1_NUM, 0x03, refbuf, 1);
  delay(10);
  ADS1299_WRITE_TABLE(pin_CS1_NUM, ADS1299_REGISTER_LS);

  delay(50); // let ADC2 come up on ADC1's clock

  ADS1299_SDATAC(pin_CS2_NUM);
  uint8_t refbuf2[] = {0b11101000};   // PD_REFBUF = 1, PD_BIAS = 0
  ADS1299_WREG(pin_CS2_NUM, 0x03, refbuf2, 1);
  delay(10);
  ADS1299_WRITE_TABLE(pin_CS2_NUM, ADS1299_REGISTER_LS_2);
}


void read_ADS1299_data(uint8_t cs, byte *buffer) {
  digitalWrite(cs, LOW);
  delayMicroseconds(1);
  for (int i = 0; i < ADS1299_TOTAL_DATA_BYTES; i++) {
      buffer[i] = SPI_SendByte(cs, 0x00, true);
  }
  delayMicroseconds(1);
  digitalWrite(cs, HIGH);
}

void read_ADS1299_data_all(byte *buffer) {
  read_ADS1299_data(pin_CS1_NUM, buffer);
  read_ADS1299_data(pin_CS2_NUM, buffer + ADS1299_TOTAL_DATA_BYTES);
}


static inline int32_t conv24(const byte *b) {
  int32_t v = ((int32_t)b[0] << 16) | ((int32_t)b[1] << 8) | (int32_t)b[2];
  if (v & 0x800000) v |= 0xFF000000; // sign extend
  return v;
}


// Dump every configured register from both chips over USB serial. Both devices
// are put back into continuous-read mode afterwards: any RREG/WREG leaves a
// device in SDATAC, where it clocks out zeros instead of samples.
void print_all_ADS1299_registers_from_setup(void) {
  const uint8_t cs_list[ADS1299_NUM_DEVICES] = {pin_CS1_NUM, pin_CS2_NUM};
  Serial.println("---- ADS1299 Register Dump ----");
  for (uint8_t d = 0; d < ADS1299_NUM_DEVICES; d++) {
      Serial.println(d == 0 ? "ADC1 (ch 1-8, master)" : "ADC2 (ch 9-16, follower)");
      for (int i = 0; i < size_reg_ls; i++) {
          int reg_addr = ADS1299_REGISTER_LS[i].add;
          // -2 marks a gap in the table, and the array's zero-filled tail
          // reads back as address 0x00.  Skip both so every real register -
          // and only the real registers - is listed exactly once.
          if (reg_addr <= 0) continue;
          uint8_t reg_val[1];
          ADS1299_RREG(cs_list[d], (uint8_t)reg_addr, reg_val, 1);
          Serial.print("Register 0x");
          if (reg_addr < 0x10) Serial.print("0");
          Serial.print(reg_addr, HEX);
          Serial.print(" : ");
          uint16_t val_for_print = 0x100 | reg_val[0];
          Serial.println(val_for_print, BIN);
          delayMicroseconds(2);
      }
  }
  Serial.println("-------------------------------");
  ADS1299_RDATAC(pin_CS1_NUM);
  ADS1299_RDATAC(pin_CS2_NUM);
}


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

// Push whatever is staged. The buffer is emptied unconditionally: a batch the
// link refused is discarded and counted rather than re-queued, which is what
// keeps latency bounded on a congested connection.
void ble_flush(void) {
  if (ble_tx_len == 0) return;
  if (!bleConnected || dataChar == NULL) {   // nobody to send to - drop it
      ble_dropped_samples += ble_tx_len / PACKET_TOTAL_SIZE;
      ble_tx_len = 0;
      return;
  }

  uint16_t staged = ble_tx_len;
  ble_tx_len = 0;                             // clear first: notify() can re-enter

  ble_notify_failed = false;
  for (uint16_t off = 0; off < staged; off += ble_chunk_max) {
      uint16_t n = staged - off;
      if (n > ble_chunk_max) n = ble_chunk_max;
      dataChar->setValue(&ble_tx_buf[off], n);
      dataChar->notify();
  }

  uint16_t samples = staged / PACKET_TOTAL_SIZE;
  if (ble_notify_failed) {
      ble_dropped_samples += samples;
      ble_dropped_notifies++;
  } else {
      ble_sent_samples += samples;
  }
}

// Stage one packet, flushing first if it would not fit.
void ble_queue_packet(const byte *packet) {
  if (ble_tx_len + PACKET_TOTAL_SIZE > sizeof(ble_tx_buf)) ble_flush();
  if (ble_tx_len + PACKET_TOTAL_SIZE > sizeof(ble_tx_buf)) {
      ble_dropped_samples++;                  // still no room - drop this sample
      return;
  }
  if (ble_tx_len == 0) ble_tx_first_ms = millis();
  memcpy(&ble_tx_buf[ble_tx_len], packet, PACKET_TOTAL_SIZE);
  ble_tx_len += PACKET_TOTAL_SIZE;
}


// --- Stream control --------------------------------------------------------
// Note the difference from the 8-channel BLE firmwares: there, acquisition
// itself is gated and the ADS1299 is held with START low until the host writes
// CMD_START. Here the ADCs free-run from setup() exactly as in 16CH_EMUL_FW,
// for two reasons:
//   * the SD card must keep recording whenever the board is powered, which is
//     how the 16-channel USB firmware behaves and is a feature of the board;
//   * the dual-chip START sequence is a single shared hardware edge on IO17
//     that also keeps ADC2 (clocked from ADC1) in step - re-running it per
//     session is risk with no benefit.
// So CMD_START/CMD_STOP open and close the BLE transmit path only. The ADC
// keeps converting underneath, and the host's timestamps are ms since boot
// rather than ms since the session began; Python_ble_LSL.py derives its LSL
// clock offset from the first packet it sees, so either is fine.
void start_streaming(void) {
  ble_tx_len = 0;                 // never emit samples staged before this session
  dataReady = false;
  streaming_allowed = true;
  digitalWrite(pin_LED_DEBUG, HIGH);
}

void stop_streaming(void) {
  streaming_allowed = false;
  ble_tx_len = 0;                 // discard anything still staged
  digitalWrite(pin_LED_DEBUG, LOW);
}


void setup() {
  // Suppress ESP-IDF log output (sdmmc, vfs_fat, etc.) so the console stays
  // readable and nothing unexpected is printed mid-session.
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
  pinMode(pin_CS1_NUM, OUTPUT);
  pinMode(pin_CS2_NUM, OUTPUT);
  pinMode(pin_DRDY_NUM, INPUT_PULLUP);
  pinMode(pin_LED_DEBUG, OUTPUT);
  digitalWrite(pin_CS1_NUM, HIGH);
  digitalWrite(pin_CS2_NUM, HIGH);
  digitalWrite(pin_START_NUM, LOW);
  delay(500);
  digitalWrite(pin_LED_DEBUG, LOW);

  vspi = new SPIClass(FSPI);
  vspi->begin(pin_SCK_NUM, pin_MISO_NUM, pin_MOSI_NUM, pin_CS1_NUM);
  vspi->beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE1));
  delay(200);

  ADS1299_SETUP();

  // No serial handshake over BLE: Python_ble_LSL.py sends no timestamp
  // packet, so the device clock is simply ms since boot.
  _millis_reference = millis();

  // --- BLE peripheral ---
  BLEDevice::init(BLE_DEVICE_NAME);
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

  Serial.println("\n=== CERELOG 16CH BLE Peripheral ===");
  Serial.print("Name:     "); Serial.println(BLE_DEVICE_NAME);
  Serial.print("Service:  "); Serial.println(SERVICE_UUID);
  Serial.print("Data:     "); Serial.println(DATA_CHAR_UUID);
  Serial.print("Control:  "); Serial.println(CTRL_CHAR_UUID);
  Serial.print("Channels: "); Serial.print(TOTAL_NUM_CHANNELS);
  Serial.print(" @ "); Serial.print(SAMPLE_FREQ); Serial.println(" Hz");
  Serial.print("Packet:   "); Serial.print(PACKET_TOTAL_SIZE); Serial.println(" bytes");
  Serial.println("===================================");

  print_all_ADS1299_registers_from_setup();

  attachInterrupt(digitalPinToInterrupt(pin_DRDY_NUM), onDRDYFalling, FALLING);

  // SD Card Init runs on core 0 in background so it never blocks streaming.
  xTaskCreatePinnedToCore(sd_setup_task, "SD_Init", 8192, NULL, 1, NULL, 0);

  // Single hardware START edge on IO17, shared by both devices.  Conversions
  // free-run from here (the SD card logs them all); nothing goes out over BLE
  // until the host writes CMD_START.
  digitalWrite(pin_START_NUM, HIGH);
  delay(20);                       // digital filter settle (4 data periods)
  ADS1299_RDATAC(pin_CS1_NUM);
  ADS1299_RDATAC(pin_CS2_NUM);

  dataReady = false;
  DEBUG_PRINTLN("Setup complete.");
  Serial.println("Ready. Advertising, waiting for BLE connection...");

  digitalWrite(pin_LED_DEBUG, LOW);   // lights up once the host starts the stream
}




void loop() {
  // ============================================================
  // PART 1: BLE connection handling
  // ============================================================
  // Connecting is not enough to start the stream: the host also has to
  // subscribe to the data characteristic and write CMD_START. That write
  // carries the ATT MTU it negotiated, which sets how many packets go in one
  // notification.
  if (bleConnected && hostRequestedStream && !streaming_allowed) {
      ble_configure_batching();
      ble_dropped_samples = 0;
      ble_dropped_notifies = 0;
      adc_overrun_count = 0;
      ble_sent_samples = 0;
      ble_stats_ms = millis();
      Serial.print(">>> Host started stream - MTU ");
      Serial.print(ble_host_mtu);
      Serial.print(", ");
      Serial.print(ble_notify_bytes / PACKET_TOTAL_SIZE);
      Serial.println(" packet(s) per notification");
      start_streaming();
  }

  // ============================================================
  // PART 2: Advertising - the BLE stand-in for USB enumeration
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

  if (!bleConnected || !hostRequestedStream) {
      if (streaming_allowed) {
          Serial.println("Host lost - BLE stream stopped");
          stop_streaming();
      }
      // Fall through: the ADC and the SD log keep running regardless.
  }

  // ============================================================
  // PART 3: Read one conversion from both ADS1299s
  // ============================================================
  // Deliberately not an early return: PART 5 has to keep running on iterations
  // where no conversion is pending, or a partly-filled batch would sit in the
  // staging buffer past its BLE_NOTIFY_FLUSH_MS deadline.
  if (dataReady) {
      dataReady = false;

      byte raw_data[ADS1299_ALL_DATA_BYTES];
      read_ADS1299_data_all(raw_data);

      uint32_t millis_since_sync = millis() - _millis_reference;

      // --- SD card: log every conversion, streaming or not. -----------------
      // This runs before the streaming gate on purpose: the card records
      // whenever the board is powered, exactly like 16CH_EMUL_FW.ino.
      if (sd_queue != NULL) {
          sd_sample_t sample;
          sample.timestamp_ms = millis_since_sync;
          sample.status1 = ((uint32_t)raw_data[0] << 16) | ((uint32_t)raw_data[1] << 8) | raw_data[2];
          sample.status2 = ((uint32_t)raw_data[ADS1299_TOTAL_DATA_BYTES] << 16) |
                           ((uint32_t)raw_data[ADS1299_TOTAL_DATA_BYTES + 1] << 8) |
                            (uint32_t)raw_data[ADS1299_TOTAL_DATA_BYTES + 2];
          for (uint8_t i = 0; i < ADS1299_NUM_CHANNELS; i++) {
              sample.ch[i]     = conv24(&raw_data[3 + 3 * i]);
              sample.ch[i + 8] = conv24(&raw_data[ADS1299_TOTAL_DATA_BYTES + 3 + 3 * i]);
          }
          if (xQueueSend(sd_queue, &sample, 0) != pdTRUE) {
              sd_dropped_count++;
          }
      }

      // ========================================================
      // PART 4: Build and stage the BLE packet
      // ========================================================
      if (streaming_allowed) {
          const uint16_t START_MARKER = 0xABCD;
          const uint16_t END_MARKER = 0xDCBA;
          byte packet[PACKET_TOTAL_SIZE];

          packet[PACKET_IDX_START_MARKER]     = (START_MARKER >> 8) & 0xFF;
          packet[PACKET_IDX_START_MARKER + 1] =  START_MARKER & 0xFF;
          packet[PACKET_IDX_LENGTH] = PACKET_MSG_LENGTH;

          // Big-endian, which is what the host parser (and BrainFlow) expects.
          packet[PACKET_IDX_TIMESTAMP]     = (millis_since_sync >> 24) & 0xFF;
          packet[PACKET_IDX_TIMESTAMP + 1] = (millis_since_sync >> 16) & 0xFF;
          packet[PACKET_IDX_TIMESTAMP + 2] = (millis_since_sync >> 8) & 0xFF;
          packet[PACKET_IDX_TIMESTAMP + 3] =  millis_since_sync & 0xFF;

          // ADC1's status word, then ch1-8 from ADC1 and ch9-16 from ADC2. The
          // two reads are already laid out that way in raw_data apart from
          // ADC2's own 3 status bytes, which are skipped here (they go to the
          // SD log, where there is a column for each device's status word).
          memcpy(&packet[PACKET_IDX_DATA], raw_data, ADS1299_TOTAL_DATA_BYTES);
          memcpy(&packet[PACKET_IDX_DATA + ADS1299_TOTAL_DATA_BYTES],
                 &raw_data[ADS1299_TOTAL_DATA_BYTES + ADS1299_NUM_STATUS_BYTES],
                 ADS1299_NUM_CHANNELS * ADS1299_BYTES_PER_CHANNEL);

          uint8_t checksum = 0;
          for (uint8_t i = PACKET_IDX_LENGTH; i < PACKET_IDX_CHECKSUM; i++) {
              checksum += packet[i];
          }
          packet[PACKET_IDX_CHECKSUM] = checksum;
          packet[PACKET_IDX_END_MARKER]     = (END_MARKER >> 8) & 0xFF;
          packet[PACKET_IDX_END_MARKER + 1] =  END_MARKER & 0xFF;

          ble_queue_packet(packet);
      }
  }

  // ============================================================
  // PART 5: Flush staged packets over BLE
  // ============================================================
  // Send once a full notification's worth is staged, or once the oldest staged
  // packet has been waiting BLE_NOTIFY_FLUSH_MS, whichever comes first.
  if (ble_tx_len >= ble_notify_bytes ||
      (ble_tx_len > 0 && (millis() - ble_tx_first_ms) >= BLE_NOTIFY_FLUSH_MS)) {
      ble_flush();
  }

  // ============================================================
  // PART 6: Link statistics
  // ============================================================
  if (streaming_allowed && (millis() - ble_stats_ms >= BLE_STATS_PERIOD_MS)) {
      ble_stats_ms = millis();
      Serial.print("[STATS] sent="); Serial.print(ble_sent_samples);
      Serial.print(" ble_dropped="); Serial.print(ble_dropped_samples);
      Serial.print(" (notifies="); Serial.print(ble_dropped_notifies);
      Serial.print(") adc_overrun="); Serial.print(adc_overrun_count);
      Serial.print(" sd_dropped="); Serial.println(sd_dropped_count);
  }

  yield();
}

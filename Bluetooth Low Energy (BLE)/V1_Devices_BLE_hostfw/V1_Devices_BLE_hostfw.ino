// V1 device firmware - BLE transport.
//
// The ADS1299 layer here is taken from the working V1 USB firmware
// (ESP-EEG/firmware/V1_ESP_EEG_firmware.ino): the pin map, SPI settings, the
// command helpers, the register table and the startup ordering are all copied
// from it unchanged. Only the transport is different - instead of streaming
// packets over USB serial, the device is a BLE peripheral and the packets go
// out as GATT notifications.
//
// This matters because the V1 WiFi and BLE firmwares that came before this one
// configured no ADS1299 registers at all. They reset the chip, sent SDATAC and
// then went straight to RDATAC, so the ADC ran on its power-on defaults: the
// internal reference buffer left powered down (CONFIG3 default 0x60), SRB1
// open, and every channel's MUX at 001 (inputs shorted) rather than 000
// (normal electrode input). That is why they never produced usable EEG. The
// full ADS1299_SETUP() below is what was missing.
//
// The V1 board is a classic ESP32 (ESP32-D0WDQ6), NOT the ESP32-S3 used by V2.
// Board: Tools > Board > ESP32 Arduino > 'ESP32 WROOM DA Module'.
// The CH340 bridge on this board is unreliable at the default upload speed -
// set Tools > Upload Speed to 115200.
//
// Host side: Python_ble_LSL.py, unchanged.

#include <Arduino.h>
#include <SPI.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>   // CCCD. arduino-esp32 3.x adds this descriptor on its own;
                       // the runtime check in setup() keeps both cores working.

// ==========================================
// ===  BLE PERIPHERAL CONFIGURATION      ===
// ==========================================
// These MUST match Python_ble_LSL.py.
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

// --- Packet geometry (same as the USB firmware) ---
const uint8_t ADS1299_NUM_CHANNELS      = 8;
const uint8_t ADS1299_NUM_STATUS_BYTES  = 3;
const uint8_t ADS1299_BYTES_PER_CHANNEL = 3;
const uint8_t ADS1299_TOTAL_DATA_BYTES  = ADS1299_NUM_STATUS_BYTES +
                                          (ADS1299_NUM_CHANNELS * ADS1299_BYTES_PER_CHANNEL);
const uint8_t PACKET_TIMESTAMP_BYTES = 4;
const uint8_t PACKET_MSG_LENGTH      = PACKET_TIMESTAMP_BYTES + ADS1299_TOTAL_DATA_BYTES; // 31
const uint8_t PACKET_TOTAL_SIZE      = 2 + 1 + PACKET_MSG_LENGTH + 1 + 2;                  // 37

// --- Pin Mapping --- (identical to the V1 USB firmware)
static const uint8_t pin_MOSI_NUM  = 23;
static const uint8_t pin_MISO_NUM  = 19;
static const uint8_t pin_SCK_NUM   = 18;
static const uint8_t pin_CS_NUM    = 5;
static const uint8_t pin_PWDN_NUM  = 13;
static const uint8_t pin_RST_NUM   = 12;
static const uint8_t pin_START_NUM = 14;
static const uint8_t pin_DRDY_NUM  = 27;
static const uint8_t pin_LED_DEBUG = 17;

static const int SPI_FREQ = 4000000;
SPIClass *vspi = NULL;

// --- ADS1299 State Management --- (from the USB firmware)
int _ADS1299_MODE = -2;
int ADS1299_MODE_SDATAC = 1;
int ADS1299_MODE_RDATAC = 2;
int _ADS1299_PREV_CMD = -1;
int _CMD_ADC_WREG = 3;
int _CMD_ADC_RREG = 4;
int _CMD_ADC_SDATAC = 17;
int _CMD_ADC_RDATAC = 16;
int _CMD_ADC_START = 8;

// --- Register Setup --- (copied verbatim from the V1 USB firmware)
// 0x15 (MISC1) = 0b00100000 closes SRB1, i.e. V1 runs in SRB1 referential mode.
// CHnSET = 0b01100000 -> gain 24, MUX 000 (normal electrode input).
typedef struct Deez { int add; int reg_val; } regVal_pair;
const int size_reg_ls = 24;
static const regVal_pair ADS1299_REGISTER_LS[size_reg_ls] = {
  {0x01, 0b10110110}, {0x02, 0b11010000}, {0x03, 0b11101100}, {0x04, 0}, {-2, -2},
  {0x05, 0b01100000}, {0x06, 0b01100000}, {0x07, 0b01100000}, {0x08, 0b01100000},
  {0x09, 0b01100000}, {0x0A, 0b01100000}, {0x0B, 0b01100000}, {0x0C, 0b01100000},
  {0x0D, 0b11111111}, {0x0E, 0b00000000}, {0x0F, 0}, {0x10, 0}, {0x11, 0}, {-2, -2},
  {0x15, 0b00100000}, {0x16, 0}, {0x17, 0}
};

BLEServer         *bleServer = NULL;
BLECharacteristic *dataChar  = NULL;
BLECharacteristic *ctrlChar  = NULL;

volatile bool     bleConnected = false;
volatile bool     hostRequestedStream = false;  // set by the CMD_START write
volatile uint16_t ble_host_mtu = 23;            // ATT MTU as reported by the host
bool              ble_was_connected = false;

// --- Drop accounting -------------------------------------------------------
volatile uint32_t ble_dropped_samples  = 0;   // samples lost to a failed notify
volatile uint32_t ble_dropped_notifies = 0;   // notifications the stack refused
volatile uint32_t adc_overrun_count    = 0;   // DRDY fired before loop() read the last sample
uint32_t          ble_sent_samples     = 0;
unsigned long     ble_stats_ms         = 0;
volatile bool     ble_notify_failed    = false;  // set from the onStatus callback

// --- BLE notification staging ---
uint8_t  ble_tx_buf[BLE_MAX_PACKETS_PER_NOTIFY * PACKET_TOTAL_SIZE];
uint16_t ble_tx_len = 0;                        // bytes currently staged
uint16_t ble_notify_bytes = PACKET_TOTAL_SIZE;  // flush threshold, from the MTU
uint16_t ble_chunk_max = PACKET_TOTAL_SIZE;     // max bytes per notification
unsigned long ble_tx_first_ms = 0;              // when the oldest staged packet arrived

// --- State Flags ---
volatile bool streaming_allowed = false;
volatile bool data_ready = false;
unsigned long _millis_reference = 0;

byte SPI_SendByte(byte data_byte, bool cont);
void ADS1299_WREG(uint8_t regAdd, uint8_t *values, uint8_t numRegs);
void ADS1299_SDATAC(void);
void ADS1299_RDATAC(void);
void ADS1299_START(void);
void ADS1299_SETUP(void);
void read_ADS1299_data(byte *buffer);
void start_acquisition(void);
void stop_acquisition(void);

// --- Interrupt ---
void IRAM_ATTR onDRDY() {
    if (!streaming_allowed) return;
    // Previous sample was never read out - the ADC has already overwritten it.
    if (data_ready) adc_overrun_count++;
    data_ready = true;
}

// ============================================================
// === ADS1299 low level - copied from the V1 USB firmware  ===
// ============================================================
byte SPI_SendByte(byte data_byte, bool cont) {
  if (!cont) { digitalWrite(pin_CS_NUM, LOW); delayMicroseconds(1); }
  byte received = vspi->transfer(data_byte);
  if (!cont) { delayMicroseconds(1); digitalWrite(pin_CS_NUM, HIGH); }
  return received;
}

void ADS1299_WREG(uint8_t regAdd, uint8_t *values, uint8_t numRegs) {
  if (_ADS1299_MODE != ADS1299_MODE_SDATAC) ADS1299_SDATAC();
  digitalWrite(pin_CS_NUM, LOW);
  delayMicroseconds(1);                       // enforce CS setup time
  SPI_SendByte(0b01000000 | regAdd, true);
  SPI_SendByte(numRegs - 1, true);
  for (uint8_t i = 0; i < numRegs; i++) SPI_SendByte(values[i], true);
  delayMicroseconds(1);                       // enforce CS hold time
  digitalWrite(pin_CS_NUM, HIGH);
  _ADS1299_PREV_CMD = _CMD_ADC_WREG;
}

void ADS1299_SDATAC(void) {
  digitalWrite(pin_CS_NUM, LOW);
  delayMicroseconds(1);
  SPI_SendByte(_CMD_ADC_SDATAC, true);
  delayMicroseconds(1);
  digitalWrite(pin_CS_NUM, HIGH);
  _ADS1299_MODE = ADS1299_MODE_SDATAC;
  _ADS1299_PREV_CMD = _CMD_ADC_SDATAC;
}

void ADS1299_RDATAC(void) {
  digitalWrite(pin_CS_NUM, LOW);
  delayMicroseconds(1);
  SPI_SendByte(_CMD_ADC_RDATAC, true);
  delayMicroseconds(1);
  digitalWrite(pin_CS_NUM, HIGH);
  _ADS1299_MODE = ADS1299_MODE_RDATAC;
  _ADS1299_PREV_CMD = _CMD_ADC_RDATAC;
}

void ADS1299_START(void) {
  digitalWrite(pin_CS_NUM, LOW);
  delayMicroseconds(1);
  SPI_SendByte(_CMD_ADC_START, true);
  delayMicroseconds(1);
  digitalWrite(pin_CS_NUM, HIGH);
  _ADS1299_PREV_CMD = _CMD_ADC_START;
}

// The power-up and register programming the wireless V1 firmwares were missing.
void ADS1299_SETUP(void) {
  digitalWrite(pin_PWDN_NUM, LOW);
  digitalWrite(pin_RST_NUM, LOW);
  delay(100);
  digitalWrite(pin_PWDN_NUM, HIGH);
  digitalWrite(pin_RST_NUM, HIGH);
  delay(1000);
  ADS1299_SDATAC();
  // CONFIG3 first: this powers up the internal reference buffer. Nothing else
  // is meaningful until the ADC has a reference.
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
  delayMicroseconds(1);
  for (int i = 0; i < ADS1299_TOTAL_DATA_BYTES; i++) {
      buffer[i] = SPI_SendByte(0x00, true);
  }
  delayMicroseconds(1);
  digitalWrite(pin_CS_NUM, HIGH);
}

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
  // which at 250 Hz is what keeps the link ahead of the ADC.
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

void ble_queue_packet(const uint8_t *packet) {
  if (ble_tx_len + PACKET_TOTAL_SIZE > sizeof(ble_tx_buf)) {
      // Staging is full and the flush has not run yet - drop the oldest batch
      // rather than stall the read loop.
      ble_dropped_samples += ble_tx_len / PACKET_TOTAL_SIZE;
      ble_tx_len = 0;
  }
  if (ble_tx_len == 0) ble_tx_first_ms = millis();
  memcpy(&ble_tx_buf[ble_tx_len], packet, PACKET_TOTAL_SIZE);
  ble_tx_len += PACKET_TOTAL_SIZE;
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

// --- Acquisition -----------------------------------------------------------
// This is the USB firmware's startup ordering, deferred until a BLE host asks
// for the stream instead of running unconditionally from setup().
void start_acquisition(void) {
    _millis_reference = millis();   // packet timestamps restart at 0 per session

    digitalWrite(pin_START_NUM, HIGH);
    delay(10);                      // let the chip settle before the START command
    ADS1299_START();

    // CRITICAL: wait for the ADC's digital filter to settle. This takes 4 data
    // periods (4 * 4ms = 16ms). A 20ms delay is safe and reliable.
    delay(20);
    ADS1299_RDATAC();

    data_ready = false;
    streaming_allowed = true;
    digitalWrite(pin_LED_DEBUG, HIGH);
}

// Over BLE the host can disconnect and come straight back, so the ADS1299 is
// taken out of continuous mode here and the next start_acquisition() begins
// from a known state.
void stop_acquisition(void) {
    streaming_allowed = false;
    data_ready = false;
    ble_tx_len = 0;                 // discard anything still staged
    ADS1299_SDATAC();
    digitalWrite(pin_START_NUM, LOW);
    digitalWrite(pin_LED_DEBUG, LOW);
}

void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(pin_PWDN_NUM, OUTPUT);
    pinMode(pin_RST_NUM, OUTPUT);
    pinMode(pin_START_NUM, OUTPUT);
    pinMode(pin_CS_NUM, OUTPUT);
    pinMode(pin_DRDY_NUM, INPUT_PULLUP);
    pinMode(pin_LED_DEBUG, OUTPUT);
    digitalWrite(pin_CS_NUM, HIGH);
    digitalWrite(pin_START_NUM, LOW);
    delay(2000);
    digitalWrite(pin_LED_DEBUG, LOW);

    Serial.println("\n--- CERELOG V1 BLE PERIPHERAL MODE ---");

    // SPI, then the full ADS1299 power-up and register programming, exactly as
    // the USB firmware does it. The chip is left in SDATAC with START low; the
    // stream itself does not begin until a host sends CMD_START.
    vspi = new SPIClass(VSPI);
    vspi->begin(pin_SCK_NUM, pin_MISO_NUM, pin_MOSI_NUM, pin_CS_NUM);
    vspi->beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE1));
    delay(500);

    ADS1299_SETUP();
    Serial.println("ADS1299 configured (SRB1 mode, gain 24, 250 SPS).");

    attachInterrupt(digitalPinToInterrupt(pin_DRDY_NUM), onDRDY, FALLING);

    // --- BLE peripheral: the stand-in for the USB serial link ---
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

    Serial.println("\n=== CERELOG V1 BLE Peripheral ===");
    Serial.print("Name:     "); Serial.println(BLE_DEVICE_NAME);
    Serial.println("Service:  " SERVICE_UUID);
    Serial.println("Data:     " DATA_CHAR_UUID);
    Serial.println("Control:  " CTRL_CHAR_UUID);
    Serial.println("=================================");
    Serial.println("Ready. Advertising, waiting for BLE connection...");

    ble_stats_ms = millis();
}

void loop() {
    // ============================================================
    // PART 1: Start the stream when the host asks for it
    // ============================================================
    if (bleConnected && hostRequestedStream && !streaming_allowed) {
        ble_configure_batching();
        ble_tx_len = 0;
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
        start_acquisition();
    }

    // ============================================================
    // PART 2: Advertising
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
    // PART 3: Data Streaming
    // ============================================================
    if (!bleConnected || !hostRequestedStream) {
        if (streaming_allowed) {
            Serial.println("Host lost - acquisition stopped");
            stop_acquisition();
        }
    } else if (data_ready) {
        data_ready = false;

        byte raw[ADS1299_TOTAL_DATA_BYTES];
        read_ADS1299_data(raw);

        uint8_t packet[PACKET_TOTAL_SIZE];
        uint32_t t = millis() - _millis_reference;

        packet[0] = 0xAB; packet[1] = 0xCD;
        packet[2] = PACKET_MSG_LENGTH;
        packet[3] = (t >> 24) & 0xFF;
        packet[4] = (t >> 16) & 0xFF;
        packet[5] = (t >> 8) & 0xFF;
        packet[6] = t & 0xFF;

        memcpy(&packet[7], raw, ADS1299_TOTAL_DATA_BYTES);

        uint8_t sum = 0;
        for(int i=2; i<34; i++) sum += packet[i];
        packet[34] = sum;

        packet[35] = 0xDC; packet[36] = 0xBA;

        // The one transport difference: instead of one write per sample,
        // packets are staged and coalesced into a single notification.
        ble_queue_packet(packet);
    }

    // ============================================================
    // PART 4: Flush staged packets over BLE
    // ============================================================
    if (ble_tx_len >= ble_notify_bytes ||
        (ble_tx_len > 0 && (millis() - ble_tx_first_ms) >= BLE_NOTIFY_FLUSH_MS)) {
        ble_flush();
    }

    // ============================================================
    // PART 5: Link statistics
    // ============================================================
    if (millis() - ble_stats_ms >= BLE_STATS_PERIOD_MS) {
        ble_stats_ms = millis();
        Serial.print("[STATS] sent="); Serial.print(ble_sent_samples);
        Serial.print(" ble_dropped="); Serial.print(ble_dropped_samples);
        Serial.print(" (notifies="); Serial.print(ble_dropped_notifies);
        Serial.print(") adc_overrun="); Serial.println(adc_overrun_count);
    }

    yield();
}

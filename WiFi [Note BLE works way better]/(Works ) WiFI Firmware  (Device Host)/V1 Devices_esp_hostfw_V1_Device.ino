// V1 device firmware - WiFi transport.
//
// The ADS1299 layer here is taken from the working V1 USB firmware
// (ESP-EEG/firmware/V1_ESP_EEG_firmware.ino): the pin map, SPI settings, the
// command helpers, the register table and the startup ordering are all copied
// from it unchanged. Only the transport differs - the device brings up a WiFi
// access point and streams packets to a TCP client.
//
// Earlier versions of this firmware configured no ADS1299 registers at all.
// They reset the chip, sent SDATAC and went straight to RDATAC, so the ADC ran
// on its power-on defaults: the internal reference buffer left powered down
// (CONFIG3 default 0x60), SRB1 open, and every channel's MUX at 001 (inputs
// shorted) rather than 000 (normal electrode input). That is why this firmware
// never produced usable EEG. The full ADS1299_SETUP() below is what was
// missing; the WiFi/TCP/UDP logic is unchanged.
//
// Board: Tools > Board > ESP32 Arduino > 'ESP32 WROOM DA Module'.
// The CH340 bridge on this board is unreliable at the default upload speed -
// set Tools > Upload Speed to 115200.

#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ==========================================
// ===  ACCESS POINT CONFIGURATION        ===
// ==========================================
const char* AP_SSID = "CERELOG_EEG";
const char* AP_PASS = "cerelog123";
IPAddress local_IP(192,168,4,1);
IPAddress gateway(192,168,4,1);
IPAddress subnet(255,255,255,0);

const int TCP_PORT = 1112;
const int UDP_PORT = 4445;
// ==========================================

WiFiServer *server = NULL;
WiFiClient client;
WiFiUDP udp;

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

// The power-up and register programming this firmware was missing.
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

// --- Acquisition -----------------------------------------------------------
// The USB firmware's startup ordering, deferred until a TCP client connects.
void start_acquisition(void) {
    _millis_reference = millis();

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

// Taken out of continuous mode so the next start_acquisition() begins from a
// known state when a client reconnects.
void stop_acquisition(void) {
    streaming_allowed = false;
    data_ready = false;
    ADS1299_SDATAC();
    digitalWrite(pin_START_NUM, LOW);
    digitalWrite(pin_LED_DEBUG, LOW);
}

void setup() {
    Serial.begin(115200);
    delay(500);

    // 1. Safe Pin Setup
    pinMode(pin_PWDN_NUM, OUTPUT); pinMode(pin_RST_NUM, OUTPUT);
    pinMode(pin_START_NUM, OUTPUT); pinMode(pin_CS_NUM, OUTPUT);
    pinMode(pin_DRDY_NUM, INPUT_PULLUP); pinMode(pin_LED_DEBUG, OUTPUT);

    // 2. Hardware Silence (Force OFF)
    digitalWrite(pin_CS_NUM, HIGH);
    digitalWrite(pin_START_NUM, LOW);
    digitalWrite(pin_PWDN_NUM, HIGH);
    digitalWrite(pin_RST_NUM, HIGH);

    Serial.println("\n--- CERELOG ACCESS POINT MODE ---");

    // 3. SPI, then the full ADS1299 power-up and register programming, exactly
    //    as the USB firmware does it. The chip is left in SDATAC with START
    //    low; the stream does not begin until a TCP client connects.
    vspi = new SPIClass(VSPI);
    vspi->begin(pin_SCK_NUM, pin_MISO_NUM, pin_MOSI_NUM, pin_CS_NUM);
    vspi->beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE1));
    delay(500);

    ADS1299_SETUP();
    Serial.println("ADS1299 configured (SRB1 mode, gain 24, 250 SPS).");

    attachInterrupt(digitalPinToInterrupt(pin_DRDY_NUM), onDRDY, FALLING);

    // 4. WiFi Access Point Setup
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(local_IP, gateway, subnet);
    WiFi.softAP(AP_SSID, AP_PASS);

    Serial.println("\n=== WiFi Access Point Started ===");
    Serial.print("SSID: "); Serial.println(AP_SSID);
    Serial.print("Password: "); Serial.println(AP_PASS);
    Serial.print("IP Address: "); Serial.println(WiFi.softAPIP());
    Serial.println("=====================================\n");

    // Start UDP
    udp.begin(UDP_PORT);
    Serial.print("UDP Port: "); Serial.println(UDP_PORT);

    // Start TCP
    server = new WiFiServer(TCP_PORT);
    server->begin();
    server->setNoDelay(true);
    Serial.print("TCP Port: "); Serial.println(TCP_PORT);

    Serial.println("Ready. Waiting for TCP Connection...");
}

void loop() {
    // ============================================================
    // PART 1: TCP Connection Handling
    // ============================================================
    if (!client || !client.connected()) {
        if (server->hasClient()) {
            WiFiClient newClient = server->available();

            if (newClient) {
                Serial.println(">>> New Client!");

                if (client) {
                    client.stop();
                }

                client = newClient;
                client.setNoDelay(true);
                Serial.println(">>> Connected!");

                start_acquisition();
            }
        }
    }

    // ============================================================
    // PART 2: UDP Discovery
    // ============================================================
    int packetSize = udp.parsePacket();
    if (packetSize) {
        char buf[64];
        int len = udp.read(buf, 63);
        buf[len] = 0;
        if (strstr(buf, "CERELOG_FIND_ME")) {
            Serial.println(">>> UDP Reply");
            udp.beginPacket(udp.remoteIP(), udp.remotePort());
            udp.print("CERELOG_HERE");
            udp.endPacket();
        }
    }

    // ============================================================
    // PART 3: Data Streaming
    // ============================================================
    if (client && client.connected()) {
        if (data_ready) {
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

            client.write(packet, PACKET_TOTAL_SIZE);
        }
    } else {
        if (streaming_allowed) {
            Serial.println("Client Lost.");
            stop_acquisition();
            if (client) client.stop();
        }
    }

    yield();
}

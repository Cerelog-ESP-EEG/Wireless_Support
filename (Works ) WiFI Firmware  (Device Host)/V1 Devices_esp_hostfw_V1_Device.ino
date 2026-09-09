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

// --- Pin Mapping ---
static const uint8_t pin_MOSI = 23;
static const uint8_t pin_MISO = 19;
static const uint8_t pin_SCK  = 18;
static const uint8_t pin_CS   = 5;
static const uint8_t pin_PWDN = 13;
static const uint8_t pin_RST  = 12;
static const uint8_t pin_START= 14;
static const uint8_t pin_DRDY = 27;
static const uint8_t pin_LED  = 17;

SPIClass *vspi = NULL;

// --- State Flags ---
volatile bool streaming_allowed = false;
volatile bool data_ready = false;
unsigned long _millis_reference = 0;

// --- Interrupt ---
void IRAM_ATTR onDRDY() {
    if (!streaming_allowed) return; 
    data_ready = true;
}

void setup() {
    Serial.begin(115200);
    delay(500);
    
    // 1. Safe Pin Setup
    pinMode(pin_PWDN, OUTPUT); pinMode(pin_RST, OUTPUT);
    pinMode(pin_START, OUTPUT); pinMode(pin_CS, OUTPUT);
    pinMode(pin_DRDY, INPUT_PULLUP); pinMode(pin_LED, OUTPUT);

    // 2. Hardware Silence (Force OFF)
    digitalWrite(pin_CS, HIGH);
    digitalWrite(pin_START, LOW);
    digitalWrite(pin_PWDN, HIGH);
    digitalWrite(pin_RST, HIGH);

    Serial.println("\n--- CERELOG ACCESS POINT MODE ---");

    // 3. WiFi Access Point Setup
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

    // 4. SPI Setup
    vspi = new SPIClass(VSPI);
    vspi->begin(pin_SCK, pin_MISO, pin_MOSI, pin_CS);
    vspi->beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));
    
    // 5. ADS1299 Soft Reset
    delay(100);
    digitalWrite(pin_RST, LOW); delay(100);
    digitalWrite(pin_RST, HIGH); delay(500);
    
    digitalWrite(pin_CS, LOW); delayMicroseconds(2);
    vspi->transfer(0x11);
    delayMicroseconds(2); digitalWrite(pin_CS, HIGH);

    attachInterrupt(digitalPinToInterrupt(pin_DRDY), onDRDY, FALLING);
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
                
                streaming_allowed = true;
                _millis_reference = millis();
                
                digitalWrite(pin_START, HIGH);
                delay(1);
                digitalWrite(pin_CS, LOW); 
                vspi->transfer(0x10);
                digitalWrite(pin_CS, HIGH);
                
                digitalWrite(pin_LED, HIGH);
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
            
            byte raw[27];
            digitalWrite(pin_CS, LOW);
            for(int i=0; i<27; i++) raw[i] = vspi->transfer(0x00);
            digitalWrite(pin_CS, HIGH);

            uint8_t packet[37];
            uint32_t t = millis() - _millis_reference;
            
            packet[0] = 0xAB; packet[1] = 0xCD;
            packet[2] = 31;
            packet[3] = (t >> 24) & 0xFF;
            packet[4] = (t >> 16) & 0xFF;
            packet[5] = (t >> 8) & 0xFF;
            packet[6] = t & 0xFF;
            
            memcpy(&packet[7], raw, 27);
            
            uint8_t sum = 0;
            for(int i=2; i<34; i++) sum += packet[i];
            packet[34] = sum;
            
            packet[35] = 0xDC; packet[36] = 0xBA;

            client.write(packet, 37);
        }
    } else {
        if (streaming_allowed) {
            Serial.println("Client Lost.");
            streaming_allowed = false;
            digitalWrite(pin_START, LOW);
            digitalWrite(pin_LED, LOW);
            if (client) client.stop();
        }
    }
    
    yield();
}
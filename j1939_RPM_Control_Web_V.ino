#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>
#include "driver/twai.h"

// 1. 하드웨어 및 전역 설정 (기존 유지)
#define CAN_TX_PIN GPIO_NUM_22
#define CAN_RX_PIN GPIO_NUM_21
const int STATUS_LED = 27;

Preferences prefs;
byte mySA = 0xE5; 
const byte engineDA = 0x00;
unsigned char myNAME[8] = {0x01, 0x00, 0x00, 0xE8, 0x00, 0x21, 0x00, 0x80}; 

bool addressConfirmed = false;
unsigned long claimTimer = 0;
volatile int targetRPM = 0;
volatile int currentRPM = 0;
bool is500k = false;
bool deviceConnected = false;
byte rollingCount = 0;

enum ControlState { IDLE, RUNNING, STOPPING };
ControlState currentState = IDLE;

BLECharacteristic *pCharacteristic;
BLEServer *pServer;

// --- 유틸리티 함수 (기존 유지) ---
byte calculateChecksum(byte* data, byte count) {
    byte ck = 0;
    for(int i=0; i<7; i++) ck ^= data[i];
    ck ^= (count & 0x0F); 
    ck ^= mySA; 
    ck ^= engineDA;
    return (ck << 4) | (count & 0x0F);
}

void sendAddressClaim() {
    twai_message_t msg;
    msg.identifier = 0x18EEFF00 | mySA; 
    msg.extd = 1;
    msg.data_length_code = 8;
    memcpy(msg.data, myNAME, 8);
    twai_transmit(&msg, pdMS_TO_TICKS(10));
}

void sendTSC1() {
    twai_message_t msg;
    msg.identifier = 0x0C000000 | ((uint32_t)engineDA << 8) | mySA;
    msg.extd = 1;
    msg.data_length_code = 8;
    
    unsigned int val = (unsigned int)(targetRPM / 0.125);
    msg.data[0] = 0x01; 
    msg.data[1] = (byte)(val & 0xFF);
    msg.data[2] = (byte)(val >> 8);
    msg.data[3] = 0xFF; msg.data[4] = 0xFF;
    msg.data[5] = 0xFF; msg.data[6] = 0xFF;
    msg.data[7] = calculateChecksum(msg.data, rollingCount);
    
    twai_transmit(&msg, pdMS_TO_TICKS(5));
    rollingCount = (rollingCount + 1) % 16;
}

// 2. CAN 초기화 (시리얼 출력 및 PGN 60928 재송신 유지)
void initCAN(bool use500k) {
    Serial.println("\n[CAN] Starting Initialization...");
    twai_stop();
    twai_driver_uninstall();
    delay(50);

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config;
    if (use500k) {
        t_config = TWAI_TIMING_CONFIG_500KBITS();
        Serial.println("[CAN] Configured to 500kbps.");
    } else {
        t_config = TWAI_TIMING_CONFIG_250KBITS();
        Serial.println("[CAN] Configured to 250kbps.");
    }

    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        if (twai_start() == ESP_OK) {
            sendAddressClaim(); 
            Serial.println("[CAN] Address Claim (PGN 60928) sent.");
            claimTimer = millis();
            addressConfirmed = false;
        }
    }
}

// 3. BLE 콜백 설정
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* p) { deviceConnected = true; }
    void onDisconnect(BLEServer* p) { 
        deviceConnected = false;
        ESP.restart(); // 블루투스 해제 시에만 리셋
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pChar) {
        String rx = pChar->getValue();
        if (rx.length() < 2) return;
        uint16_t val = (uint8_t)rx[0] | ((uint8_t)rx[1] << 8);

        // Resume: 가동 상태로 전환 및 초기 RPM 설정
        if (val == 1000) { 
            currentState = RUNNING; 
            targetRPM = 1000; 
        }
        // Stop: 정지 상태로 전환 (감속 시작)
        else if (val == 600) { 
            currentState = STOPPING; 
        }
        // Up/Down: RUNNING 상태에서만 작동하도록 제한
        else if (val == 1 && currentState == RUNNING) { 
            targetRPM = min(targetRPM + 25, 2000); // 상한 2000
        }
        else if (val == 2 && currentState == RUNNING) { 
            targetRPM = max(targetRPM - 25, 1000); // 하한 1000
        }
        else if (val == 0x5000) { is500k = true; prefs.putBool("is500k", is500k); initCAN(is500k); }
        else if (val == 0x2500) { is500k = false; prefs.putBool("is500k", is500k); initCAN(is500k); }
        else if ((val & 0xFF00) == 0xA500) { 
            mySA = val & 0xFF;
            prefs.putUChar("mySA", mySA); 
            initCAN(is500k); 
        }
    }
};

void setup() {
    Serial.begin(115200);
    pinMode(STATUS_LED, OUTPUT);
    prefs.begin("truck_cfg", false);
    is500k = prefs.getBool("is500k", false);
    mySA = prefs.getUChar("mySA", 0xE5);
    initCAN(is500k);

    BLEDevice::init("Truck_RPM_Control");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
    pCharacteristic = pService->createCharacteristic("beb5483e-36e1-4688-b7f5-ea07361b26a8", 
                BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
    pCharacteristic->setCallbacks(new MyCallbacks());
    pCharacteristic->addDescriptor(new BLE2902());
    pService->start();
    pServer->getAdvertising()->start();
}

void loop() {
    unsigned long now = millis();
    twai_message_t rx_msg;
    
    while (twai_receive(&rx_msg, 0) == ESP_OK) {
        if (!rx_msg.extd) continue;
        uint32_t id = rx_msg.identifier;
        uint8_t pf = (id >> 16) & 0xFF;
        uint8_t ps = (id >> 8) & 0xFF;

        if (pf == 0xEA && (ps == 0xFF || ps == mySA)) {
            if (rx_msg.data[0] == 0x00 && rx_msg.data[1] == 0xEE && rx_msg.data[2] == 0x00) {
                twai_message_t tx_msg;
                tx_msg.identifier = 0x18EE0000 | ((uint32_t)((ps == 0xFF) ? 0xFF : (id & 0xFF)) << 8) | mySA;
                tx_msg.extd = 1; tx_msg.data_length_code = 8;
                memcpy(tx_msg.data, myNAME, 8);
                twai_transmit(&tx_msg, pdMS_TO_TICKS(10));
            }
        }
        if (pf == 0xF0 && ps == 0x04) {
            currentRPM = (int)((rx_msg.data[4] * 256 + rx_msg.data[3]) * 0.125);
            digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
        }
    }

    if (!addressConfirmed && (now - claimTimer >= 500)) {
        addressConfirmed = true;
    }

    static unsigned long last10ms = 0;
    if (now - last10ms >= 10 && addressConfirmed) {
        if (currentState != IDLE) sendTSC1();
        last10ms = now;
    }

    static unsigned long last100ms = 0;
    if (now - last100ms >= 100) {
        // Stop 처리: 600 RPM까지 서서히 감소
        if (currentState == STOPPING) {
            targetRPM -= 50;
            if (targetRPM <= 600) { 
                targetRPM = 600; 
                currentState = IDLE; // 600 도달 시 IDLE로 전환하여 TSC1 중단 방지 혹은 유지 결정
            }
        }
        
        if (deviceConnected) {
            uint8_t tx[5] = { (uint8_t)(currentRPM & 0xFF), (uint8_t)(currentRPM >> 8), 
                             (uint8_t)(targetRPM & 0xFF), (uint8_t)(targetRPM >> 8), (uint8_t)(is500k ? 1 : 0) };
            pCharacteristic->setValue(tx, 5);
            pCharacteristic->notify();
        }
        last100ms = now;
    }
}
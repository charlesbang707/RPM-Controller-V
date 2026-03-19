#include <Arduino.h>                    // 기본 아두이노 함수들
#include <BLEDevice.h>                  // BLE 장치 설정
#include <BLEServer.h>                  // BLE 서버 기능
#include <BLEUtils.h>                   // BLE 유틸리티
#include <BLE2902.h>                    // Notify/Indicate를 위한 디스크립터
#include <Preferences.h>                // NVS(비휘발성 저장소)에 설정 저장
#include "driver/twai.h"                // ESP32 CAN/TWAI 드라이버

// ────────────────────────────────────────────────
// 1. 핀 및 기본 설정
// ────────────────────────────────────────────────
#define CAN_TX_PIN GPIO_NUM_22          // CAN 송신 핀 (TX)
#define CAN_RX_PIN GPIO_NUM_21          // CAN 수신 핀 (RX)
const int STATUS_LED = 27;              // 상태 LED 핀 번호

Preferences prefs;                      // NVS 저장소 객체 (설정 저장용)
byte mySA = 0xE5;                       // 기본 Source Address (J1939 ECU 주소)
const byte engineDA = 0x00;             // 엔진 목적지 주소 (Destination Address = 0x00)
unsigned char myNAME[8] = {0x01, 0x00, 0x00, 0xE8, 0x00, 0x21, 0x00, 0x80}; // J1939 NAME (고유 식별자)

// 상태 변수들
bool addressConfirmed = false;          // 주소 청구(Claim) 완료 여부
unsigned long claimTimer = 0;           // 주소 청구 타이머
volatile int targetRPM = 0;             // 목표 RPM (웹에서 설정)
volatile int currentRPM = 0;            // 현재 엔진 RPM (CAN으로 수신)
bool is500k = false;                    // CAN 속도 500kbps 여부
bool deviceConnected = false;           // BLE 연결 상태
byte rollingCount = 0;                  // TSC1 메시지 롤링 카운터 (체크섬용)

enum ControlState { IDLE, RUNNING, STOPPING }; // 제어 상태
ControlState currentState = IDLE;       // 현재 상태 (기본: IDLE)

BLECharacteristic *pCharacteristic;     // 읽기/쓰기용 특성
BLEServer *pServer;                     // BLE 서버 객체

// ────────────────────────────────────────────────
// 유틸리티 함수
// ────────────────────────────────────────────────

// TSC1 메시지 체크섬 계산 (J1939 규격)
byte calculateChecksum(byte* data, byte count) {
    byte ck = 0;
    for(int i = 0; i < 7; i++) ck ^= data[i];   // 데이터 0~6까지 XOR
    ck ^= (count & 0x0F);                       // 롤링 카운트 하위 4비트
    ck ^= mySA;                                 // 소스 주소
    ck ^= engineDA;                             // 목적지 주소
    return (ck << 4) | (count & 0x0F);          // 상위 4비트 체크섬 + 하위 4비트 카운트
}

// 주소 청구 메시지 (PGN 60928) 전송
void sendAddressClaim() {
    twai_message_t msg;
    msg.identifier = 0x18EEFF00 | mySA;     // 29비트 확장 ID
    msg.extd = 1;                           // 확장 프레임
    msg.data_length_code = 8;
    memcpy(msg.data, myNAME, 8);            // NAME 데이터 복사
    twai_transmit(&msg, pdMS_TO_TICKS(10)); // 10ms 타임아웃
}

// TSC1 제어 메시지 (PGN 0) 전송 - 목표 RPM 명령
void sendTSC1() {
    twai_message_t msg;
    msg.identifier = 0x0C000000 | ((uint32_t)engineDA << 8) | mySA;
    msg.extd = 1;
    msg.data_length_code = 8;

    // RPM을 J1939 스케일(0.125 rpm/bit)로 변환
    unsigned int val = (unsigned int)(targetRPM / 0.125);
    msg.data[0] = 0x01;                     // 속도 제어 모드
    msg.data[1] = (byte)(val & 0xFF);       // 하위 바이트
    msg.data[2] = (byte)(val >> 8);         // 상위 바이트
    msg.data[3] = 0xFF; msg.data[4] = 0xFF; // 예약
    msg.data[5] = 0xFF; msg.data[6] = 0xFF; // 예약
    msg.data[7] = calculateChecksum(msg.data, rollingCount); // 체크섬

    twai_transmit(&msg, pdMS_TO_TICKS(5));
    rollingCount = (rollingCount + 1) % 16; // 카운트 0~15 순환
}

// ────────────────────────────────────────────────
// CAN 버스 초기화 함수
// ────────────────────────────────────────────────
void initCAN(bool use500k) {
    Serial.println("\n[CAN] Starting Initialization...");
    twai_stop();                        // 기존 CAN 중지
    twai_driver_uninstall();            // 드라이버 제거
    delay(50);                          // 안정화 대기

    // 기본 설정
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);

    twai_timing_config_t t_config;
    if (use500k) {
        t_config = TWAI_TIMING_CONFIG_500KBITS();
        Serial.println("[CAN] Configured to 500kbps.");
    } else {
        t_config = TWAI_TIMING_CONFIG_250KBITS();
        Serial.println("[CAN] Configured to 250kbps.");
    }

    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); // 모든 메시지 수신

    // 드라이버 설치 및 시작
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        if (twai_start() == ESP_OK) {
            sendAddressClaim();             // 주소 청구 즉시 전송
            Serial.println("[CAN] Address Claim (PGN 60928) sent.");
            claimTimer = millis();
            addressConfirmed = false;
        }
    }
}

// ────────────────────────────────────────────────
// BLE 서버 콜백
// ────────────────────────────────────────────────
class MyServerCallbacks: public BLEServerCallbacks {
    // 클라이언트(폰)가 연결되었을 때
    void onConnect(BLEServer* p) {
        deviceConnected = true;
        Serial.println("BLE 클라이언트 연결됨");
    }

    // 클라이언트가 연결 해제되었을 때
    void onDisconnect(BLEServer* p) {
        deviceConnected = false;
        Serial.println("BLE 연결 해제됨 → ESP32 리셋");
        ESP.restart(); // 연결 해제 시 리셋 (재연결 문제 해결용)
    }
};

// ────────────────────────────────────────────────
// BLE 특성(Characteristic) 쓰기 콜백
// ────────────────────────────────────────────────
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pChar) {
        String rx = pChar->getValue();
        if (rx.length() < 2) return; // 데이터가 2바이트 미만이면 무시

        uint16_t val = (uint8_t)rx[0] | ((uint8_t)rx[1] << 8); // little-endian 16비트 값

        // 명령 처리
        if (val == 1000) {                  // RESUME
            currentState = RUNNING;
            targetRPM = 1000;
        }
        else if (val == 600) {              // STOP
            currentState = STOPPING;
        }
        else if (val == 1) {                // UP (+25 RPM)
            targetRPM = min(targetRPM + 25, 2500);
        }
        else if (val == 2) {                // DOWN (-25 RPM)
            targetRPM = max(targetRPM - 25, 0);
        }
        else if (val == 0x5000) {           // 500kbps로 변경
            is500k = true;
            prefs.putBool("is500k", is500k);
            initCAN(is500k);
        }
        else if (val == 0x2500) {           // 250kbps로 변경
            is500k = false;
            prefs.putBool("is500k", is500k);
            initCAN(is500k);
        }
        else if ((val & 0xFF00) == 0xA500) { // Source Address 변경 (0xA500 | 주소)
            mySA = val & 0xFF;
            prefs.putUChar("mySA", mySA);
            initCAN(is500k);
        }
    }
};

// ────────────────────────────────────────────────
// 초기화 (setup)
// ────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);               // 시리얼 모니터 115200bps
    pinMode(STATUS_LED, OUTPUT);        // LED 핀 출력 설정

    // 저장된 설정 불러오기
    prefs.begin("truck_cfg", false);
    is500k = prefs.getBool("is500k", false);
    mySA = prefs.getUChar("mySA", 0xE5);

    initCAN(is500k);                    // CAN 초기화

    // BLE 초기화
    BLEDevice::init("Truck_RPM_Control"); // 장치 이름

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // 서비스와 특성 생성
    BLEService *pService = pServer->createService("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
    pCharacteristic = pService->createCharacteristic(
        "beb5483e-36e1-4688-b7f5-ea07361b26a8",
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
    );
    pCharacteristic->setCallbacks(new MyCallbacks());
    pCharacteristic->addDescriptor(new BLE2902()); // Notify 활성화용 디스크립터
    pService->start();

    // 광고 시작
    pServer->getAdvertising()->start();
    Serial.println("BLE 광고 시작됨");
}

// ────────────────────────────────────────────────
// 메인 루프
// ────────────────────────────────────────────────
void loop() {
    unsigned long now = millis();
    twai_message_t rx_msg;

    // CAN 메시지 수신 처리
    while (twai_receive(&rx_msg, 0) == ESP_OK) {
        if (!rx_msg.extd) continue; // 표준 프레임은 무시

        uint32_t id = rx_msg.identifier;
        uint8_t pf = (id >> 16) & 0xFF; // PDU Format
        uint8_t ps = (id >> 8) & 0xFF;  // PDU Specific

        // 주소 청구 요청(PGN 60928) 수신 시 응답
        if (pf == 0xEA && (ps == 0xFF || ps == mySA)) {
            if (rx_msg.data[0] == 0x00 && rx_msg.data[1] == 0xEE && rx_msg.data[2] == 0x00) {
                twai_message_t tx_msg;
                tx_msg.identifier = 0x18EE0000 | ((uint32_t)((ps == 0xFF) ? 0xFF : (id & 0xFF)) << 8) | mySA;
                tx_msg.extd = 1; tx_msg.data_length_code = 8;
                memcpy(tx_msg.data, myNAME, 8);
                twai_transmit(&tx_msg, pdMS_TO_TICKS(10));
            }
        }

        // 엔진 RPM 수신 (PGN 61444)
        if (pf == 0xF0 && ps == 0x04) {
            currentRPM = (int)((rx_msg.data[4] * 256 + rx_msg.data[3]) * 0.125);
            digitalWrite(STATUS_LED, !digitalRead(STATUS_LED)); // LED 토글 (활동 표시)
        }
    }

    // 주소 청구 완료 판정 (500ms 후)
    if (!addressConfirmed && (now - claimTimer >= 500)) {
        addressConfirmed = true;
    }

    // 10ms마다 TSC1 메시지 전송 (제어 중일 때만)
    static unsigned long last10ms = 0;
    if (now - last10ms >= 10 && addressConfirmed) {
        if (currentState != IDLE) sendTSC1();
        last10ms = now;
    }

    // 100ms마다 처리 (STOP 감속 + BLE Notify)
    static unsigned long last100ms = 0;
    if (now - last100ms >= 100) {
        // STOP 상태일 때 RPM 서서히 감소
        if (currentState == STOPPING) {
            targetRPM -= 50;
            if (targetRPM <= 600) {
                targetRPM = 600;
                currentState = IDLE; // 600 도달 시 IDLE로 전환
            }
        }

        // BLE 연결되어 있으면 데이터 전송 (현재 RPM, 목표 RPM, BAUD 상태)
        if (deviceConnected) {
            uint8_t tx[5] = {
                (uint8_t)(currentRPM & 0xFF), (uint8_t)(currentRPM >> 8),
                (uint8_t)(targetRPM & 0xFF),  (uint8_t)(targetRPM >> 8),
                (uint8_t)(is500k ? 1 : 0)
            };
            pCharacteristic->setValue(tx, 5);
            pCharacteristic->notify(); // 웹으로 실시간 전송
        }

        last100ms = now;
    }
}

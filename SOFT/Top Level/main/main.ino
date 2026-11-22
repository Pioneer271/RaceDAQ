#include <stdarg.h>
#include <math.h>
#include <Wire.h>
#include <SPI.h>
#include <CAN.h>
#include <bluefruit.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include "PacketIdInfo.h"

using namespace Adafruit_LittleFS_Namespace;

// ============================================================================
// ВЕРСИЯ ПРОШИВКИ (ВЕРСИЯ ЖЕЛЕЗА ЧИТАЕТСЯ ИЗ NRF_UICR->CUSTOMER[0])
// ============================================================================
#define FW_VER_MAJOR  0
#define FW_VER_MINOR  1
#define FW_VER_PATCH  0
#define FW_BUILD_ID   2

// ============================================================================
// АКТИВНЫЕ МОДУЛИ ПЛАТЫ RACEDAQ
// ============================================================================
#define HAS_GPS          // GPS-модуль BE-880 на UART (P0.06 / P0.08)
#define HAS_IO_CHANNELS  // Аналоговые, цифровые входы/выходы и виртуальные CAN-пакеты
#define HAS_MAX6675_EGT  // Термопара EGT на микросхеме DA1 MAX6675ISA+ (SPI)
#define HAS_LSM6DS3_IMU  // 6-осевой IMU GY-LSM6DS3 (D1) по I2C (адрес 0x6B)
//#define HAS_CAN_BUS      // Физический CAN-трансивер MCP2515 (D2) по SPI

// Раскомментируй на ревизии платы с переключаемой подтяжкой NTC (3.9 кОм + 1 кОм на GPIO):
// #define HAS_SWITCHABLE_NTC_PULLUP

// ============================================================================
// РАСПИНОВКА ПЛАТЫ СОГЛАСНО СХЕМЕ (МОДУЛЬ E73-2G4M04S1B / nRF52832)
// ============================================================================
#define PIN_NRF_T_WATR        2   // P0.02 (AIN0) - Датчик температуры воды
#define PIN_NRF_T_OIL         3   // P0.03 (AIN1) - Датчик температуры масла
#define PIN_NRF_TWATR_PULLUP  15  // P0.15 - Переключаемая подтяжка 1 кОм для T_WATR (H0H1)
#define PIN_NRF_TOIL_PULLUP   17  // P0.17 - Переключаемая подтяжка 1 кОм для T_OIL (H0H1)

#define PIN_NRF_AIN1          4   // P0.04 (AIN2) - Аналоговый вход 1 (делитель R41=5.1K / R42=10K)
#define PIN_NRF_AIN2          5   // P0.05 (AIN3) - Аналоговый вход 2 (делитель R44=5.1K / R43=10K)
#define PIN_NRF_SENS          28  // P0.28 (AIN4) - Контроль зажигания IGN (делитель R13=100K / R16=33K, R33=200K)

#define PIN_NRF_DIN1          11  // P0.11 - Цифровой вход 1
#define PIN_NRF_DIN2          9   // P0.09 - Цифровой вход 2

#define PIN_NRF_DOUT1         29  // P0.29 - Силовой выход DOUT1
#define PIN_NRF_DOUT2         31  // P0.31 - Силовой выход DOUT2

#define PIN_NRF_HOLD          30  // P0.30 - Удержание DC-DC SQ20953AIC (D4)

#define RACEDAQ_SPI_SCK       12  // P0.12
#define RACEDAQ_SPI_MOSI      13  // P0.13
#define RACEDAQ_SPI_MISO      14  // P0.14
#define PIN_EGT_CS            16  // P0.16 - Chip Select термопары MAX6675 (DA1)
#define PIN_CAN_CS            22  // P0.22 - Chip Select контроллера MCP2515 (D2)
#define PIN_CAN_INT           10  // P0.10 - Прерывание MCP2515 (D2)

#define PIN_I2C_SDA           25  // P0.25
#define PIN_I2C_SCL           26  // P0.26
#define PIN_IMU_INT           27  // P0.27 - Прерывание INT1 от LSM6DS3

// ============================================================================
// Настройка EGT
// ============================================================================

// Смещение термопары EGT в °C (отрицательное уменьшает показания, положительное увеличивает)
static const float EGT_OFFSET_C = -6.0f;

// ============================================================================
// ПАРАМЕТРЫ ТЕРМИСТОРОВ NTC И ЦЕПИ ЗАЖИГАНИЯ
// ============================================================================
static const float NTC_FIXED_PULLUP_OHM = 1000.0f; // Текущие R35 и R37 = 1K 0.1% на плате V0.1.0
static const float NTC_SW_HIGH_OHM      = 3900.0f; // Постоянный верхний резистор (режим HIGH_OHM)
static const float NTC_SW_LOW_EQ_OHM    = 805.0f;  // 3.9K || (1K + ~18 Ом ключа H0H1) (режим LOW_OHM)

static const uint32_t IGN_OFF_DELAY_MS  = 30000;  //работает 30секунд после отключения зажигания.
static const uint16_t IGN_OFF_PIN_MV    = 1000;    // ~4.5 В на входе IGN

// ============================================================================
// ВИРТУАЛЬНЫЕ CAN PID ДЛЯ RACECHRONO И ПЛАНШЕТА
// ============================================================================
static const uint32_t PID_ANALOG_VOLTAGES = 0x100; // 256: T_WATR, T_OIL, AIN1, AIN2 (в мВ)
static const uint32_t PID_EGT_IGN_DIO     = 0x101; // 257: EGT (0.25°C), IGN (мВ), маски DIN/DOUT
static const uint32_t PID_DIGITAL_BYTES   = 0x102; // 258: DIN1, DIN2, DOUT1, DOUT2, IGN + RPM (uint16)
static const uint32_t PID_IMU_ACCEL_GYRO  = 0x103; // 259: IMU: AccX, AccY, AccZ (мg), YawRate (0.1 °/с)
static const uint32_t PID_NTC_TEMPS_GYRO  = 0x104; // 260: T_WATR (0.1°C), T_OIL (0.1°C), GyroX, GyroY

// ============================================================================
// СТРУКТУРА КОНФИГУРАЦИИ ВХОДОВ, CAN-СЛОТОВ И СЦЕНАРИЕВ ВЫХОДОВ (116 БАЙТ)
// ============================================================================
static const uint8_t CONFIG_MAGIC   = 0xDA;
static const uint8_t CONFIG_VERSION = 3;

enum DinMode : uint8_t {
    DIN_MODE_DIGITAL = 0, // Обычный дискретный вход (0 или 1)
    DIN_MODE_RPM     = 1  // Тахометр (измерение оборотов по спаду импульса)
};

enum TempPullupMode : uint8_t {
    TEMP_PULLUP_LOW_OHM  = 0, // Низкоомная подтяжка (805 Ом / 1 кОм) для датчиков 1..5 кОм (GM, Bosch)
    TEMP_PULLUP_HIGH_OHM = 1  // Высокоомная подтяжка (3.9 кОм) для датчиков 20..100 кОм (Defi 55k)
};

enum OutputMode : uint8_t {
    OUT_MODE_MANUAL    = 0, // Только ручное управление по BLE (0x0006)
    OUT_MODE_SINGLE    = 1, // Одно условие (condA)
    OUT_MODE_LOGIC_OR  = 2, // Условие A ИЛИ Условие B
    OUT_MODE_LOGIC_AND = 3  // Условие A И Условие B
};

enum SignalSource : uint8_t {
    SRC_NONE      = 0,
    SRC_AIN1_MV   = 1,  // Аналоговый вход 1 (мВ: 0..5000)
    SRC_AIN2_MV   = 2,  // Аналоговый вход 2 (мВ: 0..5000)
    SRC_TWATR_C10 = 3,  // Температура воды (в 0.1 °C, 950 = 95.0 °C)
    SRC_TOIL_C10  = 4,  // Температура масла (в 0.1 °C)
    SRC_EGT_C     = 5,  // Температура выхлопа (°C)
    SRC_IGN_MV    = 6,  // Напряжение зажигания (мВ)
    SRC_DIN1      = 7,  // Цифровой вход 1 (0 или 1)
    SRC_DIN2      = 8,  // Цифровой вход 2 (0 или 1)
    SRC_RPM       = 9,  // Обороты двигателя (об/мин с настроенного входа DIN)
    SRC_GPS_SPEED = 10, // Скорость по GPS (км/ч)
    SRC_CAN_SLOT0 = 11, // Автономный CAN-слот 0
    SRC_CAN_SLOT1 = 12, // Автономный CAN-слот 1
    SRC_CAN_SLOT2 = 13, // Автономный CAN-слот 2
    SRC_CAN_SLOT3 = 14  // Автономный CAN-слот 3
};

enum CompareOp : uint8_t {
    OP_GT = 0, // Больше (>)
    OP_LT = 1, // Меньше (<)
    OP_EQ = 2  // Равно (==)
};

static const uint8_t CAN_FLAG_BIG_ENDIAN = 0x01;
static const uint8_t CAN_FLAG_SIGNED     = 0x02;

static const uint8_t HW_CAP_SWITCHABLE_NTC = (1 << 0);
static const uint8_t HW_CAP_EGT_MAX6675    = (1 << 1);
static const uint8_t HW_CAP_IMU_PRESENT    = (1 << 2);
static const uint8_t HW_CAP_CAN_PRESENT    = (1 << 3);
static const uint8_t HW_CAP_GPS_PRESENT    = (1 << 4);

bool isGpsDetected = false;

#pragma pack(push, 1)
struct DinConfig {
    uint8_t mode;          // DinMode (0 = Digital, 1 = RPM)
    uint8_t pulsesPer2Rev; // Кол-во импульсов за 2 оборота коленвала
    uint8_t pullup;        // 1 = INPUT_PULLUP, 0 = INPUT
}; // 3 байта

struct TempChannelConfig {
    uint8_t  pullupMode;   // TempPullupMode (0 = LOW_OHM, 1 = HIGH_OHM)
    float    r25Ohm;       // Сопротивление датчика при +25 °C (Ом)
    uint16_t betaK;        // Коэффициент Beta термистора (K)
}; // 7 байт

struct RuleCondition {
    uint8_t  source;       // SignalSource
    uint8_t  op;           // CompareOp
    int32_t  threshold;    // Порог срабатывания
    uint16_t hysteresis;   // Гистерезис возврата
    uint16_t delayMs;      // Задержка перед включением (мс)
}; // 10 байт

struct DoutConfig {
    uint8_t       mode;    // OutputMode
    uint8_t       invert;  // 0 = прямой выход, 1 = инверсный
    RuleCondition condA;   // Основное условие
    RuleCondition condB;   // Второе условие для И / ИЛИ
}; // 22 байта

struct CanSignalSlot {
    uint8_t  enabled;      // 0 = выключен, 1 = активен
    uint32_t canId;        // CAN ID пакета (11-bit или 29-bit)
    uint8_t  startByte;    // Начальный байт в кадре (0..7)
    uint8_t  length;       // Длина значения (1 или 2 байта)
    uint8_t  flags;        // Бит 0: BigEndian; Бит 1: Signed
    int16_t  mul;          // Множитель в 0.01
    int16_t  offset;       // Смещение после умножения
}; // 12 байт

struct SystemConfig {
    uint8_t           magic;        // CONFIG_MAGIC (0xDA)
    uint8_t           version;      // CONFIG_VERSION (3)
    DinConfig         din[2];       // [0] = DIN1, [1] = DIN2 (6 байт)
    DoutConfig        dout[2];      // [0] = DOUT1, [1] = DOUT2 (44 байта)
    uint16_t          canSpeedKbps; // 250, 500 или 1000 кбит/с (2 байта)
    CanSignalSlot     canSlots[4];  // 4 автономных CAN-слота (48 байт)
    TempChannelConfig temp[2];      // [0] = T_WATR, [1] = T_OIL (14 байт)
}; // Итого: 116 байт

struct DeviceInfoPacket {
    uint8_t  hwMajor;
    uint8_t  hwMinor;
    uint8_t  hwPatch;
    uint8_t  fwMajor;
    uint8_t  fwMinor;
    uint8_t  fwPatch;
    uint8_t  hwCaps;
    uint16_t buildId;
}; // 9 байт
#pragma pack(pop)

SystemConfig sysConfig;
DeviceInfoPacket devInfo;

struct RuleRuntimeState {
    bool     latchedA;
    bool     latchedB;
    bool     outputActive;
    uint32_t activeSinceMs;
};
RuleRuntimeState doutRuntime[2] = { {false, false, false, 0}, {false, false, false, 0} };

int32_t  canSlotValues[4]   = { 0, 0, 0, 0 };
uint32_t canSlotLastRxMs[4] = { 0, 0, 0, 0 };

#ifdef HAS_GPS
void dummy_debug(...) {}
#define debug dummy_debug
#define debugln dummy_debug
#else
#define HAS_DEBUG
#define debugln Serial.println
#define debug Serial.print
#endif

BLEService mainService = BLEService(0x1ff8);
uint8_t tempData[20];

uint8_t dout1State = 0;
uint8_t dout2State = 0;
uint16_t latestEgtRaw = 0;
uint16_t latestGpsSpeedKmh = 0;

uint16_t filtMvWat    = 0;
uint16_t filtMvOil    = 0;
uint16_t filtMvAin1   = 0;
uint16_t filtMvAin2   = 0;
uint16_t filtMvIgnPin = 0;

volatile uint32_t dinLastEdgeUs[2] = { 0, 0 };
volatile uint32_t dinPeriodUs[2]   = { 0, 0 };

bool isImuInitialized = false;
int16_t imuAccX_mg = 0, imuAccY_mg = 0, imuAccZ_mg = 0;
int16_t imuGyrX_dps10 = 0, imuGyrY_dps10 = 0, imuGyrZ_dps10 = 0;

BLECharacteristic canBusMainCharacteristic      = BLECharacteristic(0x01);
BLECharacteristic canBusFilterCharacteristic    = BLECharacteristic(0x02);
BLECharacteristic ioOutputControlCharacteristic = BLECharacteristic(0x06);
BLECharacteristic systemConfigCharacteristic    = BLECharacteristic(0x08);
BLECharacteristic deviceInfoCharacteristic      = BLECharacteristic(0x09);

static const int CAN_BUS_CMD_DENY_ALL  = 0;
static const int CAN_BUS_CMD_ALLOW_ALL = 1;
static const int CAN_BUS_CMD_ADD_PID   = 2;

PacketIdInfo canBusPacketIdInfo;
bool canBusAllowUnknownPackets = true;
boolean isCanBusConnected = false;

#ifdef HAS_GPS
BLECharacteristic gpsMainCharacteristic = BLECharacteristic(0x03);
BLECharacteristic gpsTimeCharacteristic = BLECharacteristic(0x04);
BLECharacteristic gpsRateCharacteristic = BLECharacteristic(0x07);
uint8_t currentGpsHz = 10;

void sendUbxCommand(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len);

int gpsPreviousDateAndHour = 0;
uint8_t gpsSyncBits = 0;

#pragma pack(push, 1)
struct UbxNavPvt {
    uint32_t iTOW;
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  min;
    uint8_t  sec;
    uint8_t  valid;
    uint32_t tAcc;
    int32_t  nano;
    uint8_t  fixType;
    uint8_t  flags;
    uint8_t  flags2;
    uint8_t  numSV;
    int32_t  lon;
    int32_t  lat;
    int32_t  height;
    int32_t  hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t  velN;
    int32_t  velE;
    int32_t  velD;
    int32_t  gSpeed;
    int32_t  headMot;
    uint32_t sAcc;
    uint32_t headAcc;
    uint16_t pDOP;
    uint8_t  flags3;
    uint8_t  reserved1[5];
    int32_t  headVeh;
    int16_t  magDec;
    uint16_t magAcc;
};
#pragma pack(pop)

static UbxNavPvt pvt;
#endif

BLECharacteristic otaControlCharacteristic = BLECharacteristic(0x05);

void otaControlWriteCallback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
    if (len != 2) return;
    uint8_t mac[6] = { 0 };
    Bluefruit.getAddr(mac);
    if (data[0] == mac[4] && data[1] == mac[5]) {
        delay(100);
        enterOTADfu();
    }
}

void unlockNfcPinsAsGpio() {
    if ((NRF_UICR->NFCPINS & UICR_NFCPINS_PROTECT_Msk) == (UICR_NFCPINS_PROTECT_NFC << UICR_NFCPINS_PROTECT_Pos)) {
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}
        NRF_UICR->NFCPINS &= ~UICR_NFCPINS_PROTECT_Msk;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy) {}
        NVIC_SystemReset();
    }
}

void initDeviceInfo() {
    uint32_t uicrHw = NRF_UICR->CUSTOMER[0];
    // Формат метки в UICR (0x10001080): 0xDA <hwMajor> <hwMinor> <hwPatch>
    if ((uicrHw >> 24) == 0xDA) {
        devInfo.hwMajor = (uicrHw >> 16) & 0xFF;
        devInfo.hwMinor = (uicrHw >> 8) & 0xFF;
        devInfo.hwPatch = uicrHw & 0xFF;
    } else {
        devInfo.hwMajor = 0;
        devInfo.hwMinor = 0;
        devInfo.hwPatch = 0;
    }

    devInfo.fwMajor = FW_VER_MAJOR;
    devInfo.fwMinor = FW_VER_MINOR;
    devInfo.fwPatch = FW_VER_PATCH;
    devInfo.buildId = FW_BUILD_ID;

    uint8_t caps = 0;
#ifdef HAS_SWITCHABLE_NTC_PULLUP
    caps |= HW_CAP_SWITCHABLE_NTC;
#endif
#ifdef HAS_MAX6675_EGT
    caps |= HW_CAP_EGT_MAX6675;
#endif
    if (isImuInitialized) {
        caps |= HW_CAP_IMU_PRESENT;
    }
#ifdef HAS_CAN_BUS
    caps |= HW_CAP_CAN_PRESENT;
#endif
    if (isGpsDetected) {
        caps |= HW_CAP_GPS_PRESENT;
    }
    devInfo.hwCaps = caps;
}

void setDout1(bool active) {
    dout1State = active ? 1 : 0;
    digitalWrite(PIN_NRF_DOUT1, active ? HIGH : LOW);
}

void setDout2(bool active) {
    dout2State = active ? 1 : 0;
    digitalWrite(PIN_NRF_DOUT2, active ? HIGH : LOW);
}

// ============================================================================
// УПРАВЛЕНИЕ ПЕРЕКЛЮЧАЕМОЙ ПОДТЯЖКОЙ ДАТЧИКОВ ТЕМПЕРАТУРЫ (H0H1)
// ============================================================================
void setTempPullupLow(uint32_t ulPin, bool enableLowOhm) {
    if (enableLowOhm) {
        pinMode(ulPin, OUTPUT);
        uint32_t psel = g_ADigitalPinMap[ulPin];
        NRF_GPIO->PIN_CNF[psel] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos)
                                | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
                                | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
                                | (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos)
                                | (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
        digitalWrite(ulPin, HIGH);
    } else {
        pinMode(ulPin, INPUT);
    }
}

void applyTempSensorsHardwareConfig() {
#ifdef HAS_SWITCHABLE_NTC_PULLUP
    setTempPullupLow(PIN_NRF_TWATR_PULLUP, sysConfig.temp[0].pullupMode == TEMP_PULLUP_LOW_OHM);
    setTempPullupLow(PIN_NRF_TOIL_PULLUP,  sysConfig.temp[1].pullupMode == TEMP_PULLUP_LOW_OHM);
#endif
}

// ============================================================================
// ОБРАБОТЧИКИ ПРЕРЫВАНИЙ ТАХОМЕТРА И КОНФИГУРАЦИЯ ВХОДОВ DIN1 / DIN2
// ============================================================================
void din1Isr() {
    uint32_t now = micros();
    uint32_t dt = now - dinLastEdgeUs[0];
    if (dt < 500) return;
    dinPeriodUs[0] = dt;
    dinLastEdgeUs[0] = now;
}

void din2Isr() {
    uint32_t now = micros();
    uint32_t dt = now - dinLastEdgeUs[1];
    if (dt < 500) return;
    dinPeriodUs[1] = dt;
    dinLastEdgeUs[1] = now;
}

void applyDinHardwareConfig() {
    detachInterrupt(digitalPinToInterrupt(PIN_NRF_DIN1));
    detachInterrupt(digitalPinToInterrupt(PIN_NRF_DIN2));

    pinMode(PIN_NRF_DIN1, sysConfig.din[0].pullup ? INPUT_PULLUP : INPUT);
    pinMode(PIN_NRF_DIN2, sysConfig.din[1].pullup ? INPUT_PULLUP : INPUT);

    noInterrupts();
    dinPeriodUs[0] = 0;
    dinPeriodUs[1] = 0;
    interrupts();

    if (sysConfig.din[0].mode == DIN_MODE_RPM) {
        attachInterrupt(digitalPinToInterrupt(PIN_NRF_DIN1), din1Isr, FALLING);
    }
    if (sysConfig.din[1].mode == DIN_MODE_RPM) {
        attachInterrupt(digitalPinToInterrupt(PIN_NRF_DIN2), din2Isr, FALLING);
    }
}

uint16_t getCurrentRpm() {
    for (int idx = 0; idx < 2; idx++) {
        if (sysConfig.din[idx].mode == DIN_MODE_RPM) {
            noInterrupts();
            uint32_t period = dinPeriodUs[idx];
            uint32_t lastEdge = dinLastEdgeUs[idx];
            interrupts();

            if (period == 0 || (micros() - lastEdge) > 300000UL) {
                return 0;
            }
            uint8_t p2r = sysConfig.din[idx].pulsesPer2Rev;
            if (p2r == 0) p2r = 4;
            return (uint16_t)(120000000UL / (period * (uint32_t)p2r));
        }
    }
    return 0;
}

// ============================================================================
// РАБОТА С НАСТРОЙКАМИ В INTERNALFS (/racedaq_cfg.bin)
// ============================================================================
void setDefaultSystemConfig() {
    memset(&sysConfig, 0, sizeof(SystemConfig));
    sysConfig.magic        = CONFIG_MAGIC;
    sysConfig.version      = CONFIG_VERSION;
    sysConfig.canSpeedKbps = 500;

    sysConfig.din[0] = { DIN_MODE_DIGITAL, 4, 1 };
    sysConfig.din[1] = { DIN_MODE_DIGITAL, 4, 1 };

    sysConfig.dout[0].mode = OUT_MODE_MANUAL;
    sysConfig.dout[1].mode = OUT_MODE_MANUAL;

    // По умолчанию профиль GM (~2750 Ом при 25 °C, Beta = 3950 K)
    sysConfig.temp[0] = { TEMP_PULLUP_LOW_OHM, 2750.0f, 3950 };
    sysConfig.temp[1] = { TEMP_PULLUP_LOW_OHM, 2750.0f, 3950 };
}

void saveSystemConfigToFlash() {
    InternalFS.remove("/racedaq_cfg.bin");
    File file(InternalFS);
    if (file.open("/racedaq_cfg.bin", FILE_O_WRITE)) {
        file.write((uint8_t*)&sysConfig, sizeof(SystemConfig));
        file.close();
    }
}

void loadSystemConfigFromFlash() {
    InternalFS.begin();
    File file(InternalFS);
    bool loaded = false;
    if (file.open("/racedaq_cfg.bin", FILE_O_READ)) {
        if (file.available() >= (int)sizeof(SystemConfig)) {
            SystemConfig tmp;
            file.read((uint8_t*)&tmp, sizeof(SystemConfig));
            if (tmp.magic == CONFIG_MAGIC && tmp.version == CONFIG_VERSION) {
                sysConfig = tmp;
                loaded = true;
            }
        }
        file.close();
    }
    if (!loaded) {
        setDefaultSystemConfig();
        saveSystemConfigToFlash();
    }
}

void systemConfigWriteCallback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
    if (len != sizeof(SystemConfig)) return;
    SystemConfig incoming;
    memcpy(&incoming, data, sizeof(SystemConfig));
    if (incoming.magic != CONFIG_MAGIC || incoming.version != CONFIG_VERSION) return;

    bool canSpeedChanged = (sysConfig.canSpeedKbps != incoming.canSpeedKbps);
    sysConfig = incoming;

    memset(doutRuntime, 0, sizeof(doutRuntime));
    memset(canSlotValues, 0, sizeof(canSlotValues));
    applyDinHardwareConfig();
    applyTempSensorsHardwareConfig();

#ifdef HAS_CAN_BUS
    if (canSpeedChanged && isCanBusConnected) {
        CAN.end();
        isCanBusConnected = false;
    }
#endif

    saveSystemConfigToFlash();
    systemConfigCharacteristic.write((uint8_t*)&sysConfig, sizeof(SystemConfig));
}

// ============================================================================
// АНАЛОГОВЫЕ ВХОДЫ И ФИЛЬТРАЦИЯ АЦП
// ============================================================================
uint16_t readPinMillivoltsRaw(uint32_t ulPin) {
    int32_t raw = analogRead(ulPin);
    if (raw < 0) raw = 0;
    return (uint16_t)((raw * 3600UL) / 4095UL);
}

uint16_t smoothAdc(uint16_t prev, uint16_t sample, bool init) {
    if (init) return sample;
    return (uint16_t)((prev * 7UL + sample + 4UL) / 8UL);
}

void updateFilteredAdc() {
    static uint32_t lastAdcMs = 0;
    static bool isFirstSample = true;

    uint32_t now = millis();
    if (!isFirstSample && (now - lastAdcMs < 10)) return;
    lastAdcMs = now;

    uint16_t rawWat  = readPinMillivoltsRaw(PIN_NRF_T_WATR);
    uint16_t rawOil  = readPinMillivoltsRaw(PIN_NRF_T_OIL);
    uint16_t rawAin1 = (uint16_t)((readPinMillivoltsRaw(PIN_NRF_AIN1) * 151UL) / 100UL);
    uint16_t rawAin2 = (uint16_t)((readPinMillivoltsRaw(PIN_NRF_AIN2) * 151UL) / 100UL);
    uint16_t rawIgn  = readPinMillivoltsRaw(PIN_NRF_SENS);

    filtMvWat    = smoothAdc(filtMvWat, rawWat, isFirstSample);
    filtMvOil    = smoothAdc(filtMvOil, rawOil, isFirstSample);
    filtMvAin1   = smoothAdc(filtMvAin1, rawAin1, isFirstSample);
    filtMvAin2   = smoothAdc(filtMvAin2, rawAin2, isFirstSample);
    filtMvIgnPin = smoothAdc(filtMvIgnPin, rawIgn, isFirstSample);

    isFirstSample = false;
}

uint16_t readIgnRealMillivolts(uint16_t pinMv) {
    int32_t mv = ((int32_t)pinMv * 453L) / 100L - 1200L;
    return (mv > 0) ? (uint16_t)mv : 0;
}

int16_t calcNtcTempTenthsC(uint16_t pinMv, const TempChannelConfig& cfg) {
    if (pinMv >= 3285 || pinMv <= 15) {
        return -400; // Обрыв или КЗ датчика (-40.0 °C)
    }

#ifdef HAS_SWITCHABLE_NTC_PULLUP
    float rPullup = (cfg.pullupMode == TEMP_PULLUP_HIGH_OHM) ? NTC_SW_HIGH_OHM : NTC_SW_LOW_EQ_OHM;
#else
    float rPullup = NTC_FIXED_PULLUP_OHM;
#endif

    float r25  = (cfg.r25Ohm > 10.0f) ? cfg.r25Ohm : 2750.0f;
    float beta = (cfg.betaK >= 1000) ? (float)cfg.betaK : 3950.0f;

    float rNtc  = rPullup * ((float)pinMv / (3300.0f - (float)pinMv));
    float invT  = (1.0f / 298.15f) + (logf(rNtc / r25) / beta);
    float tempC = (1.0f / invT) - 273.15f;
    return (int16_t)roundf(tempC * 10.0f);
}

// ============================================================================
// ДВИЖОК СЦЕНАРИЕВ ВЫХОДОВ DOUT1 И DOUT2 (С ГИСТЕРЕЗИСОМ И ЗАДЕРЖКОЙ)
// ============================================================================
int32_t getSignalValue(uint8_t source) {
    uint32_t now = millis();
    switch (source) {
        case SRC_AIN1_MV:   return (int32_t)filtMvAin1;
        case SRC_AIN2_MV:   return (int32_t)filtMvAin2;
        case SRC_TWATR_C10: return (int32_t)calcNtcTempTenthsC(filtMvWat, sysConfig.temp[0]);
        case SRC_TOIL_C10:  return (int32_t)calcNtcTempTenthsC(filtMvOil, sysConfig.temp[1]);
        case SRC_EGT_C:     return (int32_t)(latestEgtRaw / 4);
        case SRC_IGN_MV:    return (int32_t)readIgnRealMillivolts(filtMvIgnPin);
        case SRC_DIN1:      return (sysConfig.din[0].mode == DIN_MODE_DIGITAL && digitalRead(PIN_NRF_DIN1) == LOW) ? 1 : 0;
        case SRC_DIN2:      return (sysConfig.din[1].mode == DIN_MODE_DIGITAL && digitalRead(PIN_NRF_DIN2) == LOW) ? 1 : 0;
        case SRC_RPM:       return (int32_t)getCurrentRpm();
        case SRC_GPS_SPEED: return (int32_t)latestGpsSpeedKmh;
        case SRC_CAN_SLOT0:
        case SRC_CAN_SLOT1:
        case SRC_CAN_SLOT2:
        case SRC_CAN_SLOT3: {
            uint8_t idx = source - SRC_CAN_SLOT0;
            if (!sysConfig.canSlots[idx].enabled || (now - canSlotLastRxMs[idx]) > 1000UL) {
                return 0;
            }
            return canSlotValues[idx];
        }
        default:
            return 0;
    }
}

bool evaluateCondition(const RuleCondition& cond, bool prevLatched) {
    if (cond.source == SRC_NONE) return false;
    int32_t val = getSignalValue(cond.source);
    int32_t hyst = (int32_t)cond.hysteresis;

    switch (cond.op) {
        case OP_GT:
            return prevLatched ? (val > (cond.threshold - hyst)) : (val > cond.threshold);
        case OP_LT:
            return prevLatched ? (val < (cond.threshold + hyst)) : (val < cond.threshold);
        case OP_EQ:
            return (val == cond.threshold);
        default:
            return false;
    }
}

void evaluateDoutRulesLoop() {
    uint32_t now = millis();

    for (int i = 0; i < 2; i++) {
        const DoutConfig& cfg = sysConfig.dout[i];
        if (cfg.mode == OUT_MODE_MANUAL) continue;

        RuleRuntimeState& rt = doutRuntime[i];
        rt.latchedA = evaluateCondition(cfg.condA, rt.latchedA);

        bool rawTrigger = false;
        if (cfg.mode == OUT_MODE_SINGLE) {
            rawTrigger = rt.latchedA;
        } else if (cfg.mode == OUT_MODE_LOGIC_OR) {
            rt.latchedB = evaluateCondition(cfg.condB, rt.latchedB);
            rawTrigger = (rt.latchedA || rt.latchedB);
        } else if (cfg.mode == OUT_MODE_LOGIC_AND) {
            rt.latchedB = evaluateCondition(cfg.condB, rt.latchedB);
            rawTrigger = (rt.latchedA && rt.latchedB);
        }

        uint16_t reqDelay = cfg.condA.delayMs;
        if (rawTrigger) {
            if (rt.activeSinceMs == 0) {
                rt.activeSinceMs = now;
            }
            if ((now - rt.activeSinceMs) >= reqDelay) {
                rt.outputActive = true;
            }
        } else {
            rt.activeSinceMs = 0;
            rt.outputActive = false;
        }

        bool finalPinState = (cfg.invert != 0) ? !rt.outputActive : rt.outputActive;
        if (i == 0) {
            setDout1(finalPinState);
        } else {
            setDout2(finalPinState);
        }
    }
}

// ============================================================================
// КОНТРОЛЬ ЗАЖИГАНИЯ И ОТКЛЮЧЕНИЕ ПИТАНИЯ ПЛАТЫ (NRF_HOLD -> P0.30)
// ============================================================================
void powerHoldLoop() {
    static uint32_t ignLowSinceMs = 0;

    if (filtMvIgnPin < IGN_OFF_PIN_MV) {
        if (ignLowSinceMs == 0) {
            ignLowSinceMs = millis();
        } else if (millis() - ignLowSinceMs >= IGN_OFF_DELAY_MS) {
            setDout1(false);
            setDout2(false);
            delay(50);
            digitalWrite(PIN_NRF_HOLD, LOW);
            while (1) {
                delay(100);
            }
        }
    } else {
        ignLowSinceMs = 0;
    }
}

// ============================================================================
// ДРАЙВЕР I2C IMU GY-LSM6DS3
// ============================================================================
#ifdef HAS_LSM6DS3_IMU
static uint8_t lsm6ds3Addr = 0x6B;

bool lsm6ds3WriteReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(lsm6ds3Addr);
    Wire.write(reg);
    Wire.write(val);
    return (Wire.endTransmission() == 0);
}

int lsm6ds3ReadReg(uint8_t reg) {
    Wire.beginTransmission(lsm6ds3Addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return -1;
    if (Wire.requestFrom(lsm6ds3Addr, (uint8_t)1) != 1) return -1;
    return Wire.read();
}

void imuSetup() {
    pinMode(PIN_IMU_INT, INPUT);
    delay(25); // Даём LSM6DS3 выйти из сброса после подачи питания
    Wire.setPins(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.begin();
    Wire.setClock(100000); // Снижаем частоту с 400 кГц до 100 кГц

    int who = lsm6ds3ReadReg(0x0F);
    if (who != 0x69 && who != 0x6A) {
        lsm6ds3Addr = 0x6A;
        who = lsm6ds3ReadReg(0x0F);
    }

    if (who == 0x69 || who == 0x6A) {
        lsm6ds3WriteReg(0x12, 0x44);
        lsm6ds3WriteReg(0x10, 0x4E);
        lsm6ds3WriteReg(0x11, 0x44);
        isImuInitialized = true;
    }
}

void updateImuReadings() {
    if (!isImuInitialized) return;
    static uint32_t lastImuReadMs = 0;
    uint32_t now = millis();
    if (now - lastImuReadMs < 20) return;
    lastImuReadMs = now;

    Wire.beginTransmission(lsm6ds3Addr);
    Wire.write(0x22);
    if (Wire.endTransmission(false) != 0) return;
    if (Wire.requestFrom(lsm6ds3Addr, (uint8_t)12) != 12) return;

    uint8_t buf[12];
    for (int i = 0; i < 12; i++) {
        buf[i] = Wire.read();
    }

    int16_t rawGx = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t rawGy = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t rawGz = (int16_t)((buf[5] << 8) | buf[4]);
    int16_t rawAx = (int16_t)((buf[7] << 8) | buf[6]);
    int16_t rawAy = (int16_t)((buf[9] << 8) | buf[8]);
    int16_t rawAz = (int16_t)((buf[11] << 8) | buf[10]);

    imuAccX_mg = (int16_t)((rawAx * 244L) / 1000L);
    imuAccY_mg = (int16_t)((rawAy * 244L) / 1000L);
    imuAccZ_mg = (int16_t)((rawAz * 244L) / 1000L);

    imuGyrX_dps10 = (int16_t)((rawGx * 7L) / 40L);
    imuGyrY_dps10 = (int16_t)((rawGy * 7L) / 40L);
    imuGyrZ_dps10 = (int16_t)((rawGz * 7L) / 40L);
}
#endif

// ============================================================================
// ЧТЕНИЕ ТЕРМОПАРЫ MAX6675ISA+ (DA1) ПО ШИНЕ SPI
// ============================================================================
#ifdef HAS_MAX6675_EGT
void updateMax6675Egt() {
    static uint32_t lastEgtReadMs = 0;
    uint32_t now = millis();
    if (now - lastEgtReadMs < 250) return;
    lastEgtReadMs = now;

    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    digitalWrite(PIN_EGT_CS, LOW);
    delayMicroseconds(2);
    uint16_t v = SPI.transfer16(0x0000);
    digitalWrite(PIN_EGT_CS, HIGH);
    SPI.endTransaction();

    if (v & 0x0004) {
        latestEgtRaw = 0; // Обрыв термопары
    } else {
        int32_t raw = (int32_t)((v >> 3) & 0x0FFF) + (int32_t)roundf(EGT_OFFSET_C * 4.0f);
        if (raw < 1) raw = 1;       // Защита от ложного срабатывания флага обрыва (0)
        if (raw > 4095) raw = 4095;
        latestEgtRaw = (uint16_t)raw;
    }
}
#endif

// ============================================================================
// ФИЗИЧЕСКИЙ КОНТРОЛЛЕР CAN-ШИНЫ MCP2515 (D2) + АВТОНОМНЫЕ CAN-СЛОТЫ
// ============================================================================
#ifdef HAS_CAN_BUS
bool hasActiveCanSlots() {
    for (int i = 0; i < 4; i++) {
        if (sysConfig.canSlots[i].enabled) return true;
    }
    return false;
}

long getConfiguredCanBaudrate() {
    if (sysConfig.canSpeedKbps == 250)  return 250E3;
    if (sysConfig.canSpeedKbps == 1000) return 1000E3;
    return 500E3;
}

void processCanSlotsFromFrame(uint32_t packetId, const uint8_t* frameData, uint8_t frameLen) {
    uint32_t now = millis();
    for (int i = 0; i < 4; i++) {
        const CanSignalSlot& slot = sysConfig.canSlots[i];
        if (!slot.enabled || slot.canId != packetId) continue;
        if (slot.startByte >= frameLen) continue;

        bool bigEndian = (slot.flags & CAN_FLAG_BIG_ENDIAN) != 0;
        bool isSigned  = (slot.flags & CAN_FLAG_SIGNED) != 0;
        int32_t raw = 0;

        if (slot.length == 2 && (slot.startByte + 1) < frameLen) {
            uint16_t u16 = bigEndian
                ? (((uint16_t)frameData[slot.startByte] << 8) | frameData[slot.startByte + 1])
                : (((uint16_t)frameData[slot.startByte + 1] << 8) | frameData[slot.startByte]);
            raw = isSigned ? (int32_t)(int16_t)u16 : (int32_t)u16;
        } else {
            uint8_t u8 = frameData[slot.startByte];
            raw = isSigned ? (int32_t)(int8_t)u8 : (int32_t)u8;
        }

        canSlotValues[i]   = ((raw * (int32_t)slot.mul) / 100L) + (int32_t)slot.offset;
        canSlotLastRxMs[i] = now;
    }
}

void canBusSetup() {
    CAN.setClockFrequency(8E6);
    CAN.setPins(PIN_CAN_CS, PIN_CAN_INT);
}

void canBusLoop() {
    static uint32_t lastCanRetryMs = 0;
    bool needCan = Bluefruit.connected() || hasActiveCanSlots();

    if (!isCanBusConnected && needCan) {
        if (millis() - lastCanRetryMs >= 3000) {
            lastCanRetryMs = millis();
            if (CAN.begin(getConfiguredCanBaudrate())) {
                isCanBusConnected = true;
            }
        }
    } else if (isCanBusConnected && !needCan) {
        CAN.end();
        isCanBusConnected = false;
    }

    if (isCanBusConnected) {
        int packetSize = CAN.parsePacket();
        if (packetSize > 0) {
            uint32_t packetId = CAN.packetId();
            uint8_t frameData[8];
            uint8_t frameLen = 0;

            int dataByte = CAN.read();
            while (dataByte != -1 && frameLen < 8) {
                frameData[frameLen++] = (uint8_t)dataByte;
                dataByte = CAN.read();
            }

            processCanSlotsFromFrame(packetId, frameData, frameLen);

            if (Bluefruit.connected()) {
                PacketIdInfoItem* infoItem = canBusPacketIdInfo.findItem(packetId, canBusAllowUnknownPackets);
                if (infoItem && infoItem->shouldNotify()) {
                    ((uint32_t*)tempData)[0] = packetId;
                    memcpy(&tempData[4], frameData, frameLen);
                    canBusMainCharacteristic.notify(tempData, 4 + frameLen);
                    infoItem->markNotified();
                }
            }
        }
    }
}
#endif

// ============================================================================
// ОБРАБОТКА КОМАНД RACECHRONO И ВИРТУАЛЬНЫХ КАНАЛОВ ВВОДА/ВЫВОДА
// ============================================================================
void canBusFilterWriteCallback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
    if (len < 1) return;
    uint8_t command = data[0];
    switch (command) {
        case CAN_BUS_CMD_DENY_ALL:
            if (len == 1) {
                canBusPacketIdInfo.reset();
                canBusAllowUnknownPackets = false;
            }
            break;
        case CAN_BUS_CMD_ALLOW_ALL:
            if (len == 3) {
                canBusPacketIdInfo.reset();
                uint16_t notifyIntervalMs = (data[1] << 8) | data[2];
                canBusPacketIdInfo.setDefaultNotifyInterval(notifyIntervalMs);
                canBusAllowUnknownPackets = true;
            }
            break;
        case CAN_BUS_CMD_ADD_PID:
            if (len == 7) {
                uint16_t notifyIntervalMs = (data[1] << 8) | data[2];
                uint32_t pid = ((uint32_t)data[3] << 24) | ((uint32_t)data[4] << 16) | ((uint32_t)data[5] << 8) | data[6];
                canBusPacketIdInfo.setNotifyInterval(pid, notifyIntervalMs);
            }
            break;
        default:
            break;
    }
}

void ioOutputWriteCallback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
    if (len < 2) return;
    uint8_t outChannel = data[0];
    bool enable = (data[1] != 0);
    if (outChannel == 0 || outChannel == 1) {
        if (sysConfig.dout[0].mode == OUT_MODE_MANUAL) setDout1(enable);
    } else if (outChannel == 2) {
        if (sysConfig.dout[1].mode == OUT_MODE_MANUAL) setDout2(enable);
    }
}

void sendVirtualCanPacket(uint32_t pid, const uint8_t* payload, uint8_t len) {
    if (len > 8) len = 8;
    uint8_t packet[12];
    packet[0] = (uint8_t)(pid & 0xFF);
    packet[1] = (uint8_t)((pid >> 8) & 0xFF);
    packet[2] = (uint8_t)((pid >> 16) & 0xFF);
    packet[3] = (uint8_t)((pid >> 24) & 0xFF);
    memcpy(&packet[4], payload, len);
    canBusMainCharacteristic.notify(packet, 4 + len);
}

#ifdef HAS_IO_CHANNELS
void ioChannelsSetup() {
    analogReference(AR_INTERNAL);
    analogReadResolution(12);
    analogOversampling(16);

    pinMode(PIN_NRF_DOUT1, OUTPUT);
    pinMode(PIN_NRF_DOUT2, OUTPUT);
    setDout1(false);
    setDout2(false);

    applyDinHardwareConfig();
    applyTempSensorsHardwareConfig();

    pinMode(PIN_EGT_CS, OUTPUT);
    digitalWrite(PIN_EGT_CS, HIGH);
    pinMode(PIN_CAN_CS, OUTPUT);
    digitalWrite(PIN_CAN_CS, HIGH);

    SPI.setPins(RACEDAQ_SPI_MISO, RACEDAQ_SPI_SCK, RACEDAQ_SPI_MOSI);
    SPI.begin();

    canBusPacketIdInfo.reset();
    canBusPacketIdInfo.setDefaultNotifyInterval(50);

    updateFilteredAdc();
}

void ioChannelsLoop() {
#ifdef HAS_MAX6675_EGT
    updateMax6675Egt();
#endif
#ifdef HAS_LSM6DS3_IMU
    updateImuReadings();
#endif

    evaluateDoutRulesLoop();

    if (!Bluefruit.connected()) return;

    // 1. Пакет 0x100 (256): Напряжения T_WATR, T_OIL, AIN1, AIN2 в мВ
    PacketIdInfoItem* itemAnalog = canBusPacketIdInfo.findItem(PID_ANALOG_VOLTAGES, canBusAllowUnknownPackets);
    if (itemAnalog && itemAnalog->shouldNotify()) {
        uint8_t payload[8] = {
            (uint8_t)(filtMvWat >> 8),  (uint8_t)(filtMvWat & 0xFF),
            (uint8_t)(filtMvOil >> 8),  (uint8_t)(filtMvOil & 0xFF),
            (uint8_t)(filtMvAin1 >> 8), (uint8_t)(filtMvAin1 & 0xFF),
            (uint8_t)(filtMvAin2 >> 8), (uint8_t)(filtMvAin2 & 0xFF)
        };
        sendVirtualCanPacket(PID_ANALOG_VOLTAGES, payload, 8);
        itemAnalog->markNotified();
    }

    // 2. Пакет 0x101 (257): Термопара EGT (0.25°C), напряжение IGN (мВ), маски DIN и DOUT
    PacketIdInfoItem* itemSys = canBusPacketIdInfo.findItem(PID_EGT_IGN_DIO, canBusAllowUnknownPackets);
    if (itemSys && itemSys->shouldNotify()) {
        uint16_t mvIgnReal = readIgnRealMillivolts(filtMvIgnPin);
        uint8_t din1 = (sysConfig.din[0].mode == DIN_MODE_DIGITAL && digitalRead(PIN_NRF_DIN1) == LOW) ? 1 : 0;
        uint8_t din2 = (sysConfig.din[1].mode == DIN_MODE_DIGITAL && digitalRead(PIN_NRF_DIN2) == LOW) ? 1 : 0;

        uint8_t payload[6] = {
            (uint8_t)(latestEgtRaw >> 8), (uint8_t)(latestEgtRaw & 0xFF),
            (uint8_t)(mvIgnReal >> 8),    (uint8_t)(mvIgnReal & 0xFF),
            (uint8_t)((din1 & 0x01) | ((din2 & 0x01) << 1)),
            (uint8_t)((dout1State & 0x01) | ((dout2State & 0x01) << 1))
        };
        sendVirtualCanPacket(PID_EGT_IGN_DIO, payload, 6);
        itemSys->markNotified();
    }

    // 3. Пакет 0x102 (258): Побайтные статусы DIN1, DIN2, DOUT1, DOUT2, IGN + RPM (uint16)
    PacketIdInfoItem* itemDig = canBusPacketIdInfo.findItem(PID_DIGITAL_BYTES, canBusAllowUnknownPackets);
    if (itemDig && itemDig->shouldNotify()) {
        uint16_t currentRpm = getCurrentRpm();
        uint8_t payload[7] = {
            (uint8_t)((sysConfig.din[0].mode == DIN_MODE_DIGITAL && digitalRead(PIN_NRF_DIN1) == LOW) ? 1 : 0),
            (uint8_t)((sysConfig.din[1].mode == DIN_MODE_DIGITAL && digitalRead(PIN_NRF_DIN2) == LOW) ? 1 : 0),
            dout1State,
            dout2State,
            (uint8_t)((filtMvIgnPin >= IGN_OFF_PIN_MV) ? 1 : 0),
            (uint8_t)(currentRpm >> 8),
            (uint8_t)(currentRpm & 0xFF)
        };
        sendVirtualCanPacket(PID_DIGITAL_BYTES, payload, 7);
        itemDig->markNotified();
    }

#ifdef HAS_LSM6DS3_IMU
    // 4. Пакет 0x103 (259): Данные IMU (AccX, AccY, AccZ в мg; GyroZ Yaw Rate в 0.1 град/с)
    PacketIdInfoItem* itemImu = canBusPacketIdInfo.findItem(PID_IMU_ACCEL_GYRO, canBusAllowUnknownPackets);
    if (itemImu && itemImu->shouldNotify()) {
        uint8_t payload[8] = {
            (uint8_t)(imuAccX_mg >> 8),    (uint8_t)(imuAccX_mg & 0xFF),
            (uint8_t)(imuAccY_mg >> 8),    (uint8_t)(imuAccY_mg & 0xFF),
            (uint8_t)(imuAccZ_mg >> 8),    (uint8_t)(imuAccZ_mg & 0xFF),
            (uint8_t)(imuGyrZ_dps10 >> 8), (uint8_t)(imuGyrZ_dps10 & 0xFF)
        };
        sendVirtualCanPacket(PID_IMU_ACCEL_GYRO, payload, 8);
        itemImu->markNotified();
    }
#endif

    // 5. Пакет 0x104 (260): Готовые температуры T_WATR и T_OIL (в 0.1 °C) + GyroX и GyroY (в 0.1 °/с)
    PacketIdInfoItem* itemTemp = canBusPacketIdInfo.findItem(PID_NTC_TEMPS_GYRO, canBusAllowUnknownPackets);
    if (itemTemp && itemTemp->shouldNotify()) {
        int16_t tWatTenths = calcNtcTempTenthsC(filtMvWat, sysConfig.temp[0]);
        int16_t tOilTenths = calcNtcTempTenthsC(filtMvOil, sysConfig.temp[1]);

        uint8_t payload[8] = {
            (uint8_t)(tWatTenths >> 8),    (uint8_t)(tWatTenths & 0xFF),
            (uint8_t)(tOilTenths >> 8),    (uint8_t)(tOilTenths & 0xFF),
            (uint8_t)(imuGyrX_dps10 >> 8), (uint8_t)(imuGyrX_dps10 & 0xFF),
            (uint8_t)(imuGyrY_dps10 >> 8), (uint8_t)(imuGyrY_dps10 & 0xFF)
        };
        sendVirtualCanPacket(PID_NTC_TEMPS_GYRO, payload, 8);
        itemTemp->markNotified();
    }
}
#endif

#ifdef HAS_GPS
void applyGpsRate(uint8_t hz) {
    if (hz < 1) hz = 1;
    if (hz > 25) hz = 25;
    currentGpsHz = hz;

    uint16_t measRateMs = 1000 / hz;
    uint8_t cfgRate[] = {
        (uint8_t)(measRateMs & 0xFF),
        (uint8_t)((measRateMs >> 8) & 0xFF),
        0x01, 0x00,
        0x01, 0x00
    };
    sendUbxCommand(0x06, 0x08, cfgRate, sizeof(cfgRate));
    gpsRateCharacteristic.write8(currentGpsHz);
}

void saveGpsRateToFlash(uint8_t hz) {
    InternalFS.remove("/gps_hz.bin");
    File file(InternalFS);
    if (file.open("/gps_hz.bin", FILE_O_WRITE)) {
        file.write(&hz, 1);
        file.close();
    }
}

uint8_t loadGpsRateFromFlash() {
    File file(InternalFS);
    uint8_t hz = 10;
    if (file.open("/gps_hz.bin", FILE_O_READ)) {
        if (file.available() >= 1) {
            uint8_t saved = file.read();
            if (saved >= 1 && saved <= 25) hz = saved;
        }
        file.close();
    }
    return hz;
}

void gpsRateWriteCallback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
    if (len < 1) return;
    uint8_t requestedHz = data[0];
    if (requestedHz < 1 || requestedHz > 25) return;

    applyGpsRate(requestedHz);
    saveGpsRateToFlash(requestedHz);
}
#endif

void bluetoothSetupMainService(void) {
    mainService.begin();

    otaControlCharacteristic.setProperties(CHR_PROPS_WRITE);
    otaControlCharacteristic.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
    otaControlCharacteristic.setFixedLen(2);
    otaControlCharacteristic.setWriteCallback(otaControlWriteCallback);
    otaControlCharacteristic.begin();

    canBusMainCharacteristic.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
    canBusMainCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    canBusMainCharacteristic.begin();

    canBusFilterCharacteristic.setProperties(CHR_PROPS_WRITE);
    canBusFilterCharacteristic.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
    canBusFilterCharacteristic.setWriteCallback(canBusFilterWriteCallback);
    canBusFilterCharacteristic.begin();

    ioOutputControlCharacteristic.setProperties(CHR_PROPS_WRITE | CHR_PROPS_READ);
    ioOutputControlCharacteristic.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    ioOutputControlCharacteristic.setWriteCallback(ioOutputWriteCallback);
    ioOutputControlCharacteristic.begin();

    systemConfigCharacteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
    systemConfigCharacteristic.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    systemConfigCharacteristic.setFixedLen(sizeof(SystemConfig));
    systemConfigCharacteristic.setWriteCallback(systemConfigWriteCallback);
    systemConfigCharacteristic.begin();
    systemConfigCharacteristic.write((uint8_t*)&sysConfig, sizeof(SystemConfig));

    deviceInfoCharacteristic.setProperties(CHR_PROPS_READ);
    deviceInfoCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    deviceInfoCharacteristic.setFixedLen(sizeof(DeviceInfoPacket));
    deviceInfoCharacteristic.begin();
    deviceInfoCharacteristic.write((uint8_t*)&devInfo, sizeof(DeviceInfoPacket));

#ifdef HAS_GPS
    gpsMainCharacteristic.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
    gpsMainCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    gpsMainCharacteristic.begin();

    gpsTimeCharacteristic.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
    gpsTimeCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    gpsTimeCharacteristic.begin();

    gpsRateCharacteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
    gpsRateCharacteristic.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    gpsRateCharacteristic.setFixedLen(1);
    gpsRateCharacteristic.setWriteCallback(gpsRateWriteCallback);
    gpsRateCharacteristic.begin();
    gpsRateCharacteristic.write8(currentGpsHz);
#endif
}

void bluetoothStartAdvertising(void) {
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addService(mainService);
    Bluefruit.Advertising.addName();
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(160, 160);
    Bluefruit.Advertising.setFastTimeout(30);
    Bluefruit.Advertising.start(0);
}

void connect_callback(uint16_t conn_handle) {
    BLEConnection* connection = Bluefruit.Connection(conn_handle);
    connection->requestConnectionParameter(12, 0, 400);
    connection->requestMtuExchange(247);
}

void bluetoothStart() {
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
    Bluefruit.begin();
    Bluefruit.Periph.setConnectCallback(connect_callback);
    Bluefruit.setName("RaceDAQ");
    bluetoothSetupMainService();
    bluetoothStartAdvertising();
}

#ifdef HAS_GPS
void sendUbxCommand(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len) {
    uint8_t ck_a = 0, ck_b = 0;
    Serial.write(0xB5);
    Serial.write(0x62);

    Serial.write(cls);
    ck_a += cls; ck_b += ck_a;

    Serial.write(id);
    ck_a += id; ck_b += ck_a;

    uint8_t lenL = len & 0xFF;
    uint8_t lenH = (len >> 8) & 0xFF;
    Serial.write(lenL);
    ck_a += lenL; ck_b += ck_a;

    Serial.write(lenH);
    ck_a += lenH; ck_b += ck_a;

    for (uint16_t i = 0; i < len; i++) {
        Serial.write(payload[i]);
        ck_a += payload[i]; ck_b += ck_a;
    }

    Serial.write(ck_a);
    Serial.write(ck_b);
}

void gpsSetup() {
    Serial.begin(230400);
    delay(200);

    uint8_t savedHz = loadGpsRateFromFlash();
    applyGpsRate(savedHz);
    delay(50);

    uint8_t cfgMsg[] = { 0x01, 0x07, 0x01 };
    sendUbxCommand(0x06, 0x01, cfgMsg, sizeof(cfgMsg));
    delay(50);

    uint8_t cfgPrt[] = {
        0x01, 0x00, 0x00, 0x00,
        0xD0, 0x08, 0x00, 0x00,
        0x00, 0x84, 0x03, 0x00,
        0x07, 0x00,
        0x01, 0x00,
        0x00, 0x00, 0x00, 0x00
    };
    sendUbxCommand(0x06, 0x00, cfgPrt, sizeof(cfgPrt));
}

void gpsLoop() {
    static uint32_t lastTimeNotify = 0;
    static uint8_t state = 0;
    static uint8_t msgClass = 0;
    static uint8_t msgId = 0;
    static uint16_t payloadLen = 0;
    static uint16_t payloadCounter = 0;
    static uint8_t ck_a = 0, ck_b = 0;
    static uint8_t rx_ck_a = 0;

    while (Serial.available()) {
        uint8_t b = Serial.read();

        switch (state) {
            case 0:
                if (b == 0xB5) state = 1;
                break;
            case 1:
                if (b == 0x62) {
                    state = 2;
                    ck_a = 0;
                    ck_b = 0;
                } else if (b != 0xB5) {
                    state = 0;
                }
                break;
            case 2:
                msgClass = b;
                ck_a += b; ck_b += ck_a;
                state = 3;
                break;
            case 3:
                msgId = b;
                ck_a += b; ck_b += ck_a;
                state = 4;
                break;
            case 4:
                payloadLen = b;
                ck_a += b; ck_b += ck_a;
                state = 5;
                break;
            case 5:
                payloadLen |= ((uint16_t)b << 8);
                ck_a += b; ck_b += ck_a;
                payloadCounter = 0;
                state = 6;
                break;
            case 6:
                if (msgClass == 0x01 && msgId == 0x07) {
                    if (payloadCounter < sizeof(UbxNavPvt)) {
                        ((uint8_t*)&pvt)[payloadCounter] = b;
                    }
                }
                ck_a += b; ck_b += ck_a;
                payloadCounter++;
                if (payloadCounter >= payloadLen) {
                    state = 7;
                }
                break;
            case 7:
                rx_ck_a = b;
                state = 8;
                break;
            case 8:
                if (rx_ck_a == ck_a && b == ck_b) {
                    if (!isGpsDetected) {
                        isGpsDetected = true;
                        devInfo.hwCaps |= HW_CAP_GPS_PRESENT;
                        deviceInfoCharacteristic.write((uint8_t*)&devInfo, sizeof(DeviceInfoPacket));
                    }
                    if (msgClass == 0x01 && msgId == 0x07 && payloadLen >= sizeof(UbxNavPvt)) {
                        int year2k = (pvt.year >= 2000) ? (pvt.year - 2000) : (pvt.year % 100);
                        int dateAndHour = (year2k * 8928) + ((pvt.month - 1) * 744) + ((pvt.day - 1) * 24) + pvt.hour;
                        bool hourChanged = false;

                        if (gpsPreviousDateAndHour != dateAndHour) {
                            gpsPreviousDateAndHour = dateAndHour;
                            gpsSyncBits++;
                            hourChanged = true;
                        }

                        int ms = (pvt.nano >= 0) ? (pvt.nano / 1000000) : 0;
                        int timeSinceHourStart = (pvt.min * 30000) + (pvt.sec * 500) + (ms / 2);

                        int fixStatus = 0;
                        if (pvt.fixType >= 3) {
                            fixStatus = (pvt.flags & 0x02) ? 2 : 1;
                        } else if (pvt.fixType == 2) {
                            fixStatus = 1;
                        }

                        float altM = (pvt.fixType >= 2) ? (pvt.hMSL / 1000.0f) : 0.0f;
                        int altitude = altM > 6000.f ?
                            (max(0, (int)round(altM + 500.f)) & 0x7FFF) | 0x8000 :
                            max(0, (int)round((altM + 500.f) * 10.f)) & 0x7FFF;

                        float speedKmh = (pvt.fixType >= 2) ? (pvt.gSpeed * 0.0036f) : 0.0f;
                        latestGpsSpeedKmh = (uint16_t)roundf(speedKmh);
                        int speed = speedKmh > 600.f ?
                            ((max(0, (int)round(speedKmh * 10.f))) & 0x7FFF) | 0x8000 :
                            (max(0, (int)round(speedKmh * 100.f))) & 0x7FFF;

                        int bearing = (pvt.fixType >= 2) ? max(0, (int)(pvt.headMot / 1000)) : 0;

                        tempData[0] = ((gpsSyncBits & 0x7) << 5) | ((timeSinceHourStart >> 16) & 0x1F);
                        tempData[1] = timeSinceHourStart >> 8;
                        tempData[2] = timeSinceHourStart;
                        tempData[3] = ((fixStatus & 0x3) << 6) | (min(0x3F, (int)pvt.numSV) & 0x3F);

                        int32_t latVal = (pvt.fixType >= 2) ? pvt.lat : 0;
                        int32_t lonVal = (pvt.fixType >= 2) ? pvt.lon : 0;

                        tempData[4] = latVal >> 24;
                        tempData[5] = latVal >> 16;
                        tempData[6] = latVal >> 8;
                        tempData[7] = latVal >> 0;
                        tempData[8] = lonVal >> 24;
                        tempData[9] = lonVal >> 16;
                        tempData[10] = lonVal >> 8;
                        tempData[11] = lonVal >> 0;
                        tempData[12] = altitude >> 8;
                        tempData[13] = altitude;
                        tempData[14] = speed >> 8;
                        tempData[15] = speed;
                        tempData[16] = bearing >> 8;
                        tempData[17] = bearing;

                        tempData[18] = (pvt.fixType >= 2) ? (uint8_t)min(255, (int)((pvt.pDOP * 0.6f) / 10)) : 0;
                        tempData[19] = 0xFF;

                        if (Bluefruit.connected()) {
                            gpsMainCharacteristic.notify(tempData, 20);

                            if (hourChanged || (millis() - lastTimeNotify >= 1000)) {
                                lastTimeNotify = millis();
                                uint8_t timeData[3];
                                timeData[0] = ((gpsSyncBits & 0x7) << 5) | ((dateAndHour >> 16) & 0x1F);
                                timeData[1] = dateAndHour >> 8;
                                timeData[2] = dateAndHour;
                                gpsTimeCharacteristic.notify(timeData, 3);
                            }
                        }
                    }
                }
                state = 0;
                break;
        }
    }
}
#endif

void setup() {
    unlockNfcPinsAsGpio();

    pinMode(PIN_NRF_HOLD, OUTPUT);
    digitalWrite(PIN_NRF_HOLD, HIGH);

#ifdef HAS_DEBUG
    Serial.begin(230400);
    while (!Serial);
#endif

    loadSystemConfigFromFlash();

#ifdef HAS_LSM6DS3_IMU
    imuSetup();
#endif
#ifdef HAS_CAN_BUS
    canBusSetup();
#endif

    initDeviceInfo();
    bluetoothStart();

#ifdef HAS_IO_CHANNELS
    ioChannelsSetup();
#endif
#ifdef HAS_GPS
    gpsSetup();
#endif
}

void loop() {
    updateFilteredAdc();
    powerHoldLoop();

#ifdef HAS_GPS
    gpsLoop();
#endif
#ifdef HAS_IO_CHANNELS
    ioChannelsLoop();
#endif
#ifdef HAS_CAN_BUS
    canBusLoop();
#endif
}

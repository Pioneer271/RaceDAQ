#include <stdarg.h>
#include <math.h>
#include <Wire.h>
#include <SPI.h>
#include <CAN.h>
#include <bluefruit.h>
#include "PacketIdInfo.h"

// ============================================================================
// АКТИВНЫЕ МОДУЛИ ПЛАТЫ RACEDAQ
// ============================================================================
#define HAS_GPS          // GPS-модуль BE-880 на UART (P0.06 / P0.08)
#define HAS_IO_CHANNELS  // Аналоговые, цифровые входы/выходы и виртуальные CAN-пакеты
#define HAS_MAX6675_EGT  // Термопара EGT на микросхеме DA1 MAX6675ISA+ (SPI)
#define HAS_LSM6DS3_IMU  // 6-осевой IMU GY-LSM6DS3 (D1) по I2C (адрес 0x6B)
#define HAS_CAN_BUS      // Физический CAN-трансивер MCP2515 (D2) по SPI

// ============================================================================
// РАСПИНОВКА ПЛАТЫ СОГЛАСНО СХЕМЕ (МОДУЛЬ E73-2G4M04S1B / nRF52832)
// ============================================================================

// 1. Аналоговые входы
#define PIN_NRF_T_WATR      2   // P0.02 (AIN0) - Датчик температуры воды (подтяжка R35 = 1K 0.1% к 3.3V)
#define PIN_NRF_T_OIL       3   // P0.03 (AIN1) - Датчик температуры масла (подтяжка R37 = 1K 0.1% к 3.3V)
#define PIN_NRF_AIN1        4   // P0.04 (AIN2) - Аналоговый вход 1 (делитель R41=5.1K / R42=10K)
#define PIN_NRF_AIN2        5   // P0.05 (AIN3) - Аналоговый вход 2 (делитель R44=5.1K / R43=10K)
#define PIN_NRF_SENS        28  // P0.28 (AIN4) - Контроль зажигания IGN (делитель R13=100K / R16=33K, R33=200K)

// 2. Цифровые входы
#define PIN_NRF_DIN1        11  // P0.11 - Цифровой вход 1 (внешняя подтяжка R25_PU = 10K к 3.3V)
#define PIN_NRF_DIN2        9   // P0.09 - Цифровой вход 2 (внешняя подтяжка R26_PU = 10K к 3.3V)

// 3. Цифровые силовые выходы (N-MOSFET ключи VT2 и VT3 WM10N33M)
#define PIN_NRF_DOUT1       29  // P0.29 (вывод 9 модуля E73) - Силовой выход DOUT1
#define PIN_NRF_DOUT2       31  // P0.31 (вывод 11 модуля E73) - Силовой выход DOUT2

// 4. Управление питанием
#define PIN_NRF_HOLD        30  // P0.30 (вывод 10 модуля E73) - Удержание DC-DC SQ20953AIC (D4)

// 5. Шина SPI (общая для MAX6675 и MCP2515)
#define RACEDAQ_SPI_SCK     12  // P0.12
#define RACEDAQ_SPI_MOSI    13  // P0.13
#define RACEDAQ_SPI_MISO    14  // P0.14
#define PIN_EGT_CS          16  // P0.16 - Chip Select термопары MAX6675 (DA1)
#define PIN_CAN_CS          22  // P0.22 - Chip Select контроллера MCP2515 (D2)
#define PIN_CAN_INT         10  // P0.10 - Прерывание MCP2515 (D2)

// 6. Шина I2C (модуль GY-LSM6DS3 D1)
#define PIN_I2C_SDA         25  // P0.25
#define PIN_I2C_SCL         26  // P0.26
#define PIN_IMU_INT         27  // P0.27 - Прерывание INT1 от LSM6DS3

// ============================================================================
// ПАРАМЕТРЫ ТЕРМИСТОРОВ NTC И ЦЕПИ ЗАЖИГАНИЯ
// ============================================================================
static const float NTC_PULLUP_OHM = 1000.0f; // R35 и R37 = 1K 0.1%
static const float NTC_R25_OHM    = 2500.0f; // Номинал датчика при +25 °C
static const float NTC_BETA_K     = 3900.0f; // Коэффициент Beta термистора

// Задержка перед отключением питания после пропадания IGN (защита от просадки при стартере)
static const uint32_t IGN_OFF_DELAY_MS = 3000;
static const uint16_t IGN_OFF_PIN_MV   = 1000; // ~4.5 В на входе IGN

// ============================================================================
// ВИРТУАЛЬНЫЕ CAN PID ДЛЯ RACECHRONO
// ============================================================================
static const uint32_t PID_ANALOG_VOLTAGES = 0x100; // 256: Напряжения T_WATR, T_OIL, AIN1, AIN2 (в мВ)
static const uint32_t PID_EGT_IGN_DIO     = 0x101; // 257: EGT (0.25°C), IGN (мВ), битовые маски DIN/DOUT
static const uint32_t PID_DIGITAL_BYTES   = 0x102; // 258: Побайтно (0/1): DIN1, DIN2, DOUT1, DOUT2, IGN
static const uint32_t PID_IMU_ACCEL_GYRO  = 0x103; // 259: IMU LSM6DS3: AccX, AccY, AccZ (мg), YawRate (0.1 °/с)
static const uint32_t PID_NTC_TEMPS_GYRO  = 0x104; // 260: Готовые T_WATR (0.1°C), T_OIL (0.1°C), GyroX, GyroY

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

// Хранилище данных IMU LSM6DS3
bool isImuInitialized = false;
int16_t imuAccX_mg = 0, imuAccY_mg = 0, imuAccZ_mg = 0;
int16_t imuGyrX_dps10 = 0, imuGyrY_dps10 = 0, imuGyrZ_dps10 = 0;

// Характеристики CAN-Bus (0x01 и 0x02)
BLECharacteristic canBusMainCharacteristic      = BLECharacteristic(0x01);
BLECharacteristic canBusFilterCharacteristic    = BLECharacteristic(0x02);
BLECharacteristic ioOutputControlCharacteristic = BLECharacteristic(0x06);

static const int CAN_BUS_CMD_DENY_ALL  = 0;
static const int CAN_BUS_CMD_ALLOW_ALL = 1;
static const int CAN_BUS_CMD_ADD_PID   = 2;

PacketIdInfo canBusPacketIdInfo;
bool canBusAllowUnknownPackets = true;
boolean isCanBusConnected = false;

#ifdef HAS_GPS
BLECharacteristic gpsMainCharacteristic = BLECharacteristic(0x03);
BLECharacteristic gpsTimeCharacteristic = BLECharacteristic(0x04);

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

// Характеристика запуска OTA DFU (0x05)
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

// Перевод пинов P0.09 / P0.10 из режима NFC в обычные GPIO
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

void setDout1(bool active) {
    dout1State = active ? 1 : 0;
    digitalWrite(PIN_NRF_DOUT1, active ? HIGH : LOW);
}

void setDout2(bool active) {
    dout2State = active ? 1 : 0;
    digitalWrite(PIN_NRF_DOUT2, active ? HIGH : LOW);
}

// Чтение АЦП в милливольтах (опорное AR_INTERNAL -> 3600 мВ при 12 битах)
uint16_t readPinMillivolts(uint32_t ulPin) {
    int32_t raw = analogRead(ulPin);
    if (raw < 0) raw = 0;
    return (uint16_t)((raw * 3600UL) / 4095UL);
}

// Расчет реального напряжения на линии IGN (в мВ) с учетом R13=100K, R16=33K и R33=200K
uint16_t readIgnRealMillivolts(uint16_t pinMv) {
    return (uint16_t)((pinMv * 453UL) / 100UL);
}

// Расчет температуры NTC-термистора в десятых долях градуса (0.1 °C)
int16_t calcNtcTempTenthsC(uint16_t pinMv) {
    if (pinMv >= 3250 || pinMv <= 15) {
        return -400; // Обрыв или КЗ датчика (-40.0 °C)
    }
    float rNtc = NTC_PULLUP_OHM * ((float)pinMv / (3300.0f - (float)pinMv));
    float invT = (1.0f / 298.15f) + (logf(rNtc / NTC_R25_OHM) / NTC_BETA_K);
    float tempC = (1.0f / invT) - 273.15f;
    return (int16_t)roundf(tempC * 10.0f);
}

// ============================================================================
// КОНТРОЛЬ ЗАЖИГАНИЯ И ОТКЛЮЧЕНИЕ ПИТАНИЯ ПЛАТЫ (NRF_HOLD -> P0.30)
// ============================================================================
void powerHoldLoop() {
    static uint32_t ignLowSinceMs = 0;
    uint16_t mvIgnPin = readPinMillivolts(PIN_NRF_SENS);

    if (mvIgnPin < IGN_OFF_PIN_MV) {
        if (ignLowSinceMs == 0) {
            ignLowSinceMs = millis();
        } else if (millis() - ignLowSinceMs >= IGN_OFF_DELAY_MS) {
            setDout1(false);
            setDout2(false);
            delay(50);
            digitalWrite(PIN_NRF_HOLD, LOW); // Снимаем удержание с EN DC-DC SQ20953AIC
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
    Wire.setPins(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.begin();
    Wire.setClock(400000);

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
        latestEgtRaw = 0;
    } else {
        latestEgtRaw = (v >> 3) & 0x0FFF;
    }
}
#endif

// ============================================================================
// ФИЗИЧЕСКИЙ КОНТРОЛЛЕР CAN-ШИНЫ MCP2515 (D2) НА ПИНАХ P0.22 (CS) И P0.10 (INT)
// ============================================================================
#ifdef HAS_CAN_BUS
void canBusNotifyLatestPacket(BLECharacteristic* characteristic) {
    ((uint32_t*)tempData)[0] = CAN.packetId();
    int pos = 4;
    int dataByte = CAN.read();
    while (dataByte != -1 && pos < (int)sizeof(tempData)) {
        tempData[pos] = (uint8_t)dataByte;
        dataByte = CAN.read();
        pos++;
    }
    characteristic->notify(tempData, pos);
}

void canBusSetup() {
    CAN.setClockFrequency(8E6);
    CAN.setPins(PIN_CAN_CS, PIN_CAN_INT);
}

void canBusLoop() {
    static uint32_t lastCanRetryMs = 0;

    if (!isCanBusConnected && Bluefruit.connected()) {
        if (millis() - lastCanRetryMs >= 3000) {
            lastCanRetryMs = millis();
            if (CAN.begin(500E3)) {
                isCanBusConnected = true;
            }
        }
    } else if (isCanBusConnected && !Bluefruit.connected()) {
        CAN.end();
        isCanBusConnected = false;
    }

    if (isCanBusConnected) {
        int packetSize = CAN.parsePacket();
        if (packetSize > 0) {
            uint32_t packetId = CAN.packetId();
            PacketIdInfoItem* infoItem = canBusPacketIdInfo.findItem(packetId, canBusAllowUnknownPackets);
            if (infoItem && infoItem->shouldNotify()) {
                canBusNotifyLatestPacket(&canBusMainCharacteristic);
                infoItem->markNotified();
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
        setDout1(enable);
    } else if (outChannel == 2) {
        setDout2(enable);
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
    analogReference(AR_INTERNAL); // Диапазон 0..3600 мВ (Gain 1/6 от 0.6 В)
    analogReadResolution(12);

    pinMode(PIN_NRF_DIN1, INPUT);
    pinMode(PIN_NRF_DIN2, INPUT);

    pinMode(PIN_NRF_DOUT1, OUTPUT);
    pinMode(PIN_NRF_DOUT2, OUTPUT);
    setDout1(false);
    setDout2(false);

    pinMode(PIN_EGT_CS, OUTPUT);
    digitalWrite(PIN_EGT_CS, HIGH);
    pinMode(PIN_CAN_CS, OUTPUT);
    digitalWrite(PIN_CAN_CS, HIGH);

    SPI.setPins(RACEDAQ_SPI_MISO, RACEDAQ_SPI_SCK, RACEDAQ_SPI_MOSI);
    SPI.begin();

    canBusPacketIdInfo.reset();
    canBusPacketIdInfo.setDefaultNotifyInterval(50);
}

void ioChannelsLoop() {
#ifdef HAS_MAX6675_EGT
    updateMax6675Egt();
#endif
#ifdef HAS_LSM6DS3_IMU
    updateImuReadings();
#endif

    if (!Bluefruit.connected()) return;

    // 1. Пакет 0x100 (256): Напряжения T_WATR, T_OIL, AIN1, AIN2 в мВ (uint16 Big-Endian)
    PacketIdInfoItem* itemAnalog = canBusPacketIdInfo.findItem(PID_ANALOG_VOLTAGES, canBusAllowUnknownPackets);
    if (itemAnalog && itemAnalog->shouldNotify()) {
        uint16_t mvWat  = readPinMillivolts(PIN_NRF_T_WATR);
        uint16_t mvOil  = readPinMillivolts(PIN_NRF_T_OIL);
        uint16_t mvAin1 = (uint16_t)((readPinMillivolts(PIN_NRF_AIN1) * 151UL) / 100UL);
        uint16_t mvAin2 = (uint16_t)((readPinMillivolts(PIN_NRF_AIN2) * 151UL) / 100UL);

        uint8_t payload[8] = {
            (uint8_t)(mvWat >> 8),  (uint8_t)(mvWat & 0xFF),
            (uint8_t)(mvOil >> 8),  (uint8_t)(mvOil & 0xFF),
            (uint8_t)(mvAin1 >> 8), (uint8_t)(mvAin1 & 0xFF),
            (uint8_t)(mvAin2 >> 8), (uint8_t)(mvAin2 & 0xFF)
        };
        sendVirtualCanPacket(PID_ANALOG_VOLTAGES, payload, 8);
        itemAnalog->markNotified();
    }

    // 2. Пакет 0x101 (257): Термопара EGT (0.25°C), напряжение IGN (мВ), маски DIN и DOUT
    PacketIdInfoItem* itemSys = canBusPacketIdInfo.findItem(PID_EGT_IGN_DIO, canBusAllowUnknownPackets);
    if (itemSys && itemSys->shouldNotify()) {
        uint16_t mvIgnPin  = readPinMillivolts(PIN_NRF_SENS);
        uint16_t mvIgnReal = readIgnRealMillivolts(mvIgnPin);
        uint8_t din1 = (digitalRead(PIN_NRF_DIN1) == LOW) ? 1 : 0;
        uint8_t din2 = (digitalRead(PIN_NRF_DIN2) == LOW) ? 1 : 0;

        uint8_t payload[6] = {
            (uint8_t)(latestEgtRaw >> 8), (uint8_t)(latestEgtRaw & 0xFF),
            (uint8_t)(mvIgnReal >> 8),    (uint8_t)(mvIgnReal & 0xFF),
            (uint8_t)((din1 & 0x01) | ((din2 & 0x01) << 1)),
            (uint8_t)((dout1State & 0x01) | ((dout2State & 0x01) << 1))
        };
        sendVirtualCanPacket(PID_EGT_IGN_DIO, payload, 6);
        itemSys->markNotified();
    }

    // 3. Пакет 0x102 (258): Цифровые статусы по отдельным байтам (0 или 1)
    PacketIdInfoItem* itemDig = canBusPacketIdInfo.findItem(PID_DIGITAL_BYTES, canBusAllowUnknownPackets);
    if (itemDig && itemDig->shouldNotify()) {
        uint16_t mvIgnPin = readPinMillivolts(PIN_NRF_SENS);
        uint8_t payload[5] = {
            (uint8_t)((digitalRead(PIN_NRF_DIN1) == LOW) ? 1 : 0),
            (uint8_t)((digitalRead(PIN_NRF_DIN2) == LOW) ? 1 : 0),
            dout1State,
            dout2State,
            (uint8_t)((mvIgnPin >= IGN_OFF_PIN_MV) ? 1 : 0)
        };
        sendVirtualCanPacket(PID_DIGITAL_BYTES, payload, 5);
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
        int16_t tWatTenths = calcNtcTempTenthsC(readPinMillivolts(PIN_NRF_T_WATR));
        int16_t tOilTenths = calcNtcTempTenthsC(readPinMillivolts(PIN_NRF_T_OIL));

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

#ifdef HAS_GPS
    gpsMainCharacteristic.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
    gpsMainCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    gpsMainCharacteristic.begin();

    gpsTimeCharacteristic.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
    gpsTimeCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    gpsTimeCharacteristic.begin();
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

    uint8_t cfgRate[] = { 0x64, 0x00, 0x01, 0x00, 0x01, 0x00 };
    sendUbxCommand(0x06, 0x08, cfgRate, sizeof(cfgRate));
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

    bluetoothStart();

#ifdef HAS_IO_CHANNELS
    ioChannelsSetup();
#endif
#ifdef HAS_LSM6DS3_IMU
    imuSetup();
#endif
#ifdef HAS_CAN_BUS
    canBusSetup();
#endif
#ifdef HAS_GPS
    gpsSetup();
#endif
}

void loop() {
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

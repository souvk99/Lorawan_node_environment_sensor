#include <Arduino.h>
#include <LoRaWan-RAK4630.h>
#include <Wire.h>
#include "SparkFun_SHTC3.h"
#include "SparkFun_SCD30_Arduino_Library.h"  // CO2
#include <SensirionI2CSgp40.h>               // VOC

#ifdef RAK4630
  #define BOARD "RAK4631"
  #define RAK4631_BOARD true
#else
  #define RAK4631_BOARD false
#endif

/* ===== RAK1901 Temp/Humidity Sensor ===== */
SHTC3 g_shtc3;

/* ===== RAK12037 CO2 Sensor ===== */
SCD30 co2Sensor;

/* ===== RAK12047 VOC Sensor ===== */
SensirionI2CSgp40 sgp40;

/* =============================================
   RAK12004 MQ2 — I2C ADC Settings
   ============================================= */
#define ADC_ADDRESS       0x51
#define MQ2_POWER_PIN     WB_IO6
#define MQ2_RO_CLEAN_AIR  9.83f
#define MQ2_RL            10.0f
#define ADC_VREF          3.3f
#define ADC_MAX_VAL       4095.0f

/* =============================================
   POWER PINS
   ============================================= */
#define CO2_POWER_PIN     WB_IO2   // RAK12037 power
#define VOC_POWER_PIN     WB_IO5   // RAK12047 power (use different slot pin)

/* =============================================
   GAS THRESHOLDS (ppm)
   ============================================= */
#define GAS_WARNING_PPM   500
#define GAS_ALERT_PPM     1000

/* =============================================
   CO2 THRESHOLDS (ppm)
   ============================================= */
#define CO2_NORMAL_MAX    500
#define CO2_WARNING_MAX   1000

/* =============================================
   VOC INDEX THRESHOLDS
   ============================================= */
#define VOC_NORMAL_MAX    100
#define VOC_WARNING_MAX   200

/* =============================================
   ALERT THRESHOLDS (temp / hum)
   ============================================= */
#define TEMP_MIN     10.0
#define TEMP_MAX     40.0
#define HUM_MIN      20.0
#define HUM_MAX      80.0

/* =============================================
   MODULE
   ============================================= */
#define THIS_MODULE        0x656D
#define THIS_MODULE_NAME   "em"
#define DEVICE_ID_TEXT     "dv01"

/* =============================================
   ALARM
   ============================================= */
#define ALARM_NONE    0x00
#define ALARM_ACTIVE  0x01

/* ===== Send Intervals ===== */
#define SEND_INTERVAL      15000
#define ALERT_INTERVAL      5000

/* ===== LoRaWAN Parameters ===== */
#define LORAWAN_ADR_ON      true
#define LORAWAN_DATERATE    DR_2
#define LORAWAN_TX_POWER    TX_POWER_5
#define JOINREQ_NBTRIALS    10

DeviceClass_t    g_CurrentClass   = CLASS_A;
LoRaMacRegion_t  g_CurrentRegion  = LORAMAC_REGION_EU868;
lmh_confirm      g_CurrentConfirm = LMH_UNCONFIRMED_MSG;
uint8_t          gAppPort         = 2;

/* ===== OTAA Keys ===== */
uint8_t nodeDeviceEUI[8] = { 0xAC, 0x1F, 0x09, 0xFF, 0xFE, 0x22, 0x02, 0x4E };
uint8_t nodeAppEUI[8]    = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x02, 0x6B, 0x87 };
uint8_t nodeAppKey[16]   = {
  0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
  0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C
};

static lmh_param_t lora_param_init = {
  LORAWAN_ADR_ON, LORAWAN_DATERATE, LORAWAN_PUBLIC_NETWORK,
  JOINREQ_NBTRIALS, LORAWAN_TX_POWER, LORAWAN_DUTYCYCLE_OFF
};

#define LORAWAN_APP_DATA_BUFF_SIZE 64
static uint8_t        m_lora_buffer[LORAWAN_APP_DATA_BUFF_SIZE];
static lmh_app_data_t m_lora_app_data = { m_lora_buffer, 0, 0, 0, 0 };

static uint32_t count      = 0;
static uint32_t count_fail = 0;

/* =============================================
   STRUCT — 12 bytes (removed module, device_id, mcu_bat)
   ============================================= */
#pragma pack(1)
typedef struct {
    uint16_t  alarm_status;  // 2 byte  — but using 0 or 1 --> inefficiently
    int16_t  temp;          // 2 bytes — x100
    uint16_t hum;           // 2 bytes — x100
    uint16_t gas;           // 2 bytes — ppm  (MQ2)
    uint16_t co2;           // 2 bytes — ppm  (SCD30)
    uint16_t voc;           // 2 bytes — index x10 (SGP40)   
} EnvironmentMonitoring_t;  // TOTAL = 11 bytes
#pragma pack()

/* ===== Forward Declarations ===== */
static void lorawan_joined_handler(void);
static void lorawan_join_failed_handler(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);

static lmh_callback_t lora_callbacks = {
  BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
  lorawan_rx_handler, lorawan_joined_handler,
  lorawan_confirm_class_handler, lorawan_join_failed_handler
};

/* =============================================
   READ ADC via I2C — RAK12004
   ============================================= */
uint16_t readADC_I2C()
{
    Wire.beginTransmission(ADC_ADDRESS);
    Wire.write(0x00);
    Wire.endTransmission(false);
    Wire.requestFrom(ADC_ADDRESS, 2);

    if (Wire.available() == 2) {
        uint16_t raw = ((Wire.read() & 0x0F) << 8) | Wire.read();
        return raw;
    }
    return 0;
}

/* =============================================
   READ GAS PPM — MQ2 via I2C ADC
   ============================================= */
uint16_t readGasPPM()
{
    uint16_t adc_raw = readADC_I2C();
    float voltage = (adc_raw * ADC_VREF) / ADC_MAX_VAL;
    if (voltage < 0.01f) return 0;
    float rs      = ((ADC_VREF - voltage) / voltage) * MQ2_RL;
    float ratio   = rs / MQ2_RO_CLEAN_AIR;
    float log_ppm = (log10(ratio) - 1.67f) / -0.47f;
    float ppm_f   = pow(10.0f, log_ppm);
    if (ppm_f < 0.0f)    ppm_f = 0.0f;
    if (ppm_f > 9999.0f) ppm_f = 9999.0f;
    return (uint16_t)ppm_f;
}

/* =============================================
   READ CO2 PPM — RAK12037 SCD30
   Returns 0 if not ready
   ============================================= */
uint16_t readCO2PPM()
{
    if (co2Sensor.dataAvailable()) {
        float co2 = co2Sensor.getCO2();
        return (uint16_t)co2;
    }
    return 0;
}

/* =============================================
   READ VOC INDEX — RAK12047 SGP40
   Returns VOC index x10 (stored as uint16)
   e.g. index 84.5 → stored as 845
   ============================================= */
uint16_t readVOCIndex()
{
    uint16_t srawVoc   = 0;
    uint16_t defaultRh = 0x8000;  // 50 %RH
    uint16_t defaultT  = 0x6666;  // 25 °C

    uint16_t error = sgp40.measureRawSignal(defaultRh, defaultT, srawVoc);
    if (error) return 0;

    float vocIndex = (float)srawVoc / 131.07f;
    return (uint16_t)(vocIndex * 10);  // store x10 to keep 1 decimal
}

/* =============================================
   BATTERY
   ============================================= */
uint8_t getBatteryPercent()
{
    float voltage = analogRead(WB_A0) * 3.6f / 1024.0f * 2.0f;
    return (uint8_t)constrain(
        map((long)(voltage * 100), 320, 420, 0, 100), 0, 100
    );
}

/* =============================================
   CHECK ALARMS — Returns 0 or 1
   Now includes CO2 and VOC
   ============================================= */
uint8_t checkAlarms(float temperature, float humidity,
                    uint16_t gas, uint16_t co2, uint16_t voc_x10)
{
    float vocIndex = voc_x10 / 10.0f;

    if (temperature > TEMP_MAX)           return ALARM_ACTIVE;
    if (temperature < TEMP_MIN)           return ALARM_ACTIVE;
    if (humidity    > HUM_MAX)            return ALARM_ACTIVE;
    if (humidity    < HUM_MIN)            return ALARM_ACTIVE;
    if (gas         >= GAS_WARNING_PPM)   return ALARM_ACTIVE;
    if (co2         >= CO2_WARNING_MAX)   return ALARM_ACTIVE;
    if (vocIndex    >  VOC_WARNING_MAX)   return ALARM_ACTIVE;
    return ALARM_NONE;
}

/* =============================================
   PRINT ALARM REASON
   ============================================= */
void printAlarmReason(float temp, float hum,
                      uint16_t gas, uint16_t co2, uint16_t voc_x10)
{
    float vocIndex = voc_x10 / 10.0f;

    Serial.println("ALARM REASON:");
    Serial.println("────────────────────────────────────");
    if (temp > TEMP_MAX)
        Serial.printf("[TEMP] HIGH   : %.2f C  > %.1f C\n",   temp, TEMP_MAX);
    if (temp < TEMP_MIN)
        Serial.printf("[TEMP] LOW    : %.2f C  < %.1f C\n",   temp, TEMP_MIN);
    if (hum > HUM_MAX)
        Serial.printf("[HUM]  HIGH   : %.2f%%  > %.1f%%\n",   hum,  HUM_MAX);
    if (hum < HUM_MIN)
        Serial.printf("[HUM]  LOW    : %.2f%%  < %.1f%%\n",   hum,  HUM_MIN);
    if (gas > GAS_ALERT_PPM)
        Serial.printf("[GAS]  ALERT  : %d ppm  > %d\n",       gas,  GAS_ALERT_PPM);
    else if (gas >= GAS_WARNING_PPM)
        Serial.printf("[GAS]  WARNING: %d ppm  >= %d\n",      gas,  GAS_WARNING_PPM);
    if (co2 > CO2_WARNING_MAX)
        Serial.printf("[CO2]  ALERT  : %d ppm  > %d\n",       co2,  CO2_WARNING_MAX);
    else if (co2 >= CO2_NORMAL_MAX)
        Serial.printf("[CO2]  WARNING: %d ppm  >= %d\n",      co2,  CO2_NORMAL_MAX);
    if (vocIndex > VOC_WARNING_MAX)
        Serial.printf("[VOC]  ALERT  : %.1f idx > %d\n",      vocIndex, VOC_WARNING_MAX);
    else if (vocIndex > VOC_NORMAL_MAX)
        Serial.printf("[VOC]  WARNING: %.1f idx > %d\n",      vocIndex, VOC_NORMAL_MAX);
    Serial.println("────────────────────────────────────");
}

/* =============================================
   PRINT ALL SENSOR STATUS
   ============================================= */
void printSensorStatus(float temp, float hum,
                       uint16_t gas, uint16_t co2,
                       uint16_t voc_x10, uint8_t alarm)
{
    float vocIndex = voc_x10 / 10.0f;

    Serial.println("\n========== READINGS ==========");
    Serial.printf("TEMP  : %.2f C\n",  temp);
    Serial.printf("HUM   : %.2f %%\n", hum);
    Serial.printf("GAS   : %d ppm  ", gas);
    if      (gas > GAS_ALERT_PPM)    Serial.println("🔴 ALERT");
    else if (gas >= GAS_WARNING_PPM) Serial.println("🟡 WARNING");
    else                              Serial.println("🟢 NORMAL");

    Serial.printf("CO2   : %d ppm  ", co2);
    if      (co2 > CO2_WARNING_MAX)  Serial.println("🔴 ALERT");
    else if (co2 >= CO2_NORMAL_MAX)  Serial.println("🟡 WARNING");
    else                              Serial.println("🟢 NORMAL");

    Serial.printf("VOC   : %.1f idx  ", vocIndex);
    if      (vocIndex > VOC_WARNING_MAX) Serial.println("🔴 ALERT");
    else if (vocIndex > VOC_NORMAL_MAX)  Serial.println("🟡 WARNING");
    else                                  Serial.println("🟢 NORMAL");

    Serial.println("\n===== ALARM STATUS =====");
    Serial.printf("ALARM : %d  (%s)\n",
                  alarm,
                  alarm == ALARM_ACTIVE ? "⚠️  ALARM ACTIVE!" : "✅ NORMAL");

    if (alarm == ALARM_ACTIVE)
        printAlarmReason(temp, hum, gas, co2, voc_x10);

    Serial.println("========================");
}

/* =============================================
   BUILD PACKET
   ============================================= */
void lora_frame_setup(float temperature, float humidity,
                      uint16_t gas, uint16_t co2,
                      uint16_t voc_x10, uint8_t alarm)
{
    EnvironmentMonitoring_t pkt;
    pkt.temp         = (int16_t)(temperature * 100);
    pkt.hum          = (uint16_t)(humidity   * 100);
    pkt.gas          = gas;
    pkt.co2          = co2;
    pkt.voc          = voc_x10;
    pkt.alarm_status = alarm;

    m_lora_app_data.port     = gAppPort;
    m_lora_app_data.buffsize = sizeof(EnvironmentMonitoring_t);
    memcpy(m_lora_app_data.buffer, &pkt,
           sizeof(EnvironmentMonitoring_t));

    uint8_t battery = getBatteryPercent();
    
    Serial.println("\n====== ENV PACKET ======");
    Serial.printf("TEMP        : %d  (%.2f C)\n",  pkt.temp, temperature);
    Serial.printf("HUM         : %d  (%.2f %%)\n", pkt.hum,  humidity);
    Serial.printf("GAS         : %d ppm\n",  pkt.gas);
    Serial.printf("CO2         : %d ppm\n",  pkt.co2);
    Serial.printf("VOC         : %d  (%.1f idx)\n", pkt.voc, voc_x10 / 10.0f);
    Serial.printf("ALARM_STATUS: %d  (%s)\n",
                  pkt.alarm_status,
                  pkt.alarm_status == ALARM_ACTIVE ? "ALARM" : "NORMAL");
    Serial.printf("MCU_BAT     : %d%% (local only)\n", battery);
    Serial.printf("SIZE        : %d bytes\n", sizeof(EnvironmentMonitoring_t));

    Serial.print("RAW HEX     : ");
    uint8_t *raw = (uint8_t*)&pkt;
    for (int i = 0; i < (int)sizeof(EnvironmentMonitoring_t); i++)
        Serial.printf("%02X ", raw[i]);
    Serial.println();
    Serial.println("========================");
}

/* =============================================
   SEND LORA FRAME
   ============================================= */
void send_lora_frame(float temperature, float humidity,
                     uint16_t gas, uint16_t co2,
                     uint16_t voc_x10, uint8_t alarm)
{
    if (lmh_join_status_get() != LMH_SET) {
        Serial.println("Not ready yet...");
        return;
    }

    lora_frame_setup(temperature, humidity, gas, co2, voc_x10, alarm);

    lmh_error_status error = lmh_send(&m_lora_app_data, g_CurrentConfirm);
    if (error == LMH_SUCCESS) {
        count++;
        Serial.printf("Sent! Count: %d\n", count);
        int blinks = (alarm == ALARM_ACTIVE) ? 3 : 1;
        for (int b = 0; b < blinks; b++) {
            digitalWrite(LED_BUILTIN, HIGH); delay(100);
            digitalWrite(LED_BUILTIN, LOW);  delay(100);
        }
    } else {
        count_fail++;
        Serial.printf("Failed! Fail count: %d\n", count_fail);
    }
}

/* =============================================
   SETUP
   ============================================= */
void setup()
{
    Serial.begin(115200);
    delay(3000);

    Wire.begin();
    Wire.setClock(400000);

    pinMode(LED_BUILTIN,  OUTPUT);
    pinMode(MQ2_POWER_PIN, OUTPUT);
    pinMode(CO2_POWER_PIN, OUTPUT);
    pinMode(VOC_POWER_PIN, OUTPUT);

    // Power ON all sensors
    digitalWrite(MQ2_POWER_PIN, HIGH);
    digitalWrite(CO2_POWER_PIN, HIGH);
    digitalWrite(VOC_POWER_PIN, HIGH);
    Serial.println("Powering sensors...");
    delay(2000);

    Serial.println("================================");
    Serial.printf("Module   : %s\n",     THIS_MODULE_NAME);
    Serial.printf("Module ID: 0x%04X\n", THIS_MODULE);
    Serial.printf("Device ID: %s\n",     DEVICE_ID_TEXT);
    Serial.println("--- Thresholds ---");
    Serial.printf("TEMP  : %.1f to %.1f C\n",    TEMP_MIN,        TEMP_MAX);
    Serial.printf("HUM   : %.1f to %.1f %%\n",   HUM_MIN,         HUM_MAX);
    Serial.printf("GAS   : WARNING>=%d ALERT>%d ppm\n",
                  GAS_WARNING_PPM, GAS_ALERT_PPM);
    Serial.printf("CO2   : WARNING>=%d ALERT>%d ppm\n",
                  CO2_NORMAL_MAX,  CO2_WARNING_MAX);
    Serial.printf("VOC   : WARNING>%d  ALERT>%d idx\n",
                  VOC_NORMAL_MAX,  VOC_WARNING_MAX);
    Serial.printf("Size  : %d bytes\n", sizeof(EnvironmentMonitoring_t));
    Serial.println("================================");

    // Init SHTC3
    Serial.print("SHTC3  init: ");
    Serial.println(g_shtc3.begin() == SHTC3_Status_Nominal ? "OK" : "FAILED!");

    // Init SCD30 (CO2)
    Serial.print("SCD30  init: ");
    if (co2Sensor.begin()) {
        co2Sensor.setMeasurementInterval(2);
        Serial.println("OK");
    } else {
        Serial.println("FAILED! Check wiring.");
    }

    // Init SGP40 (VOC)
    Serial.print("SGP40  init: ");
    sgp40.begin(Wire);
    uint16_t testResult;
    uint16_t err = sgp40.executeSelfTest(testResult);
    if (err || testResult != 0xD400) {
        Serial.println("FAILED! Check wiring.");
    } else {
        Serial.println("OK");
    }

    Serial.println("Warming up VOC sensor (10s)...");
    delay(10000);

    Serial.print("DevEUI: ");
    for (int i = 0; i < 8; i++)
        Serial.printf("%02X", nodeDeviceEUI[i]);
    Serial.println();

    lora_rak4630_init();
    lmh_setDevEui(nodeDeviceEUI);
    lmh_setAppEui(nodeAppEUI);
    lmh_setAppKey(nodeAppKey);

    uint32_t lora_err = lmh_init(&lora_callbacks, lora_param_init,
                                  true, g_CurrentClass, g_CurrentRegion);
    if (lora_err != 0) {
        Serial.printf("LoRa init failed: %d\n", lora_err);
        return;
    }

    Serial.println("Joining network (OTAA)...");
    lmh_join();
}

/* =============================================
   LOOP
   ============================================= */
void loop()
{
    static uint32_t last_send  = 0;
    static uint8_t  last_alarm = 0xFF;

    uint32_t interval = (last_alarm == ALARM_ACTIVE) ?
                         ALERT_INTERVAL : SEND_INTERVAL;

    if (millis() - last_send > interval) {
        last_send = millis();

        if (lmh_join_status_get() != LMH_SET) {
            Serial.println("Not joined yet...");
            return;
        }

        // Read SHTC3
        g_shtc3.update();
        if (g_shtc3.lastStatus != SHTC3_Status_Nominal) {
            Serial.println("SHTC3 read failed!");
            return;
        }
        float temperature = g_shtc3.toDegC();
        float humidity    = g_shtc3.toPercent();

        // Read MQ2
        uint16_t gas = readGasPPM();

        // Read CO2
        uint16_t co2 = readCO2PPM();

        // Read VOC
        uint16_t voc_x10 = readVOCIndex();

        // Check all alarms
        uint8_t alarm = checkAlarms(temperature, humidity,
                                    gas, co2, voc_x10);

        // Print all readings and status
        printSensorStatus(temperature, humidity,
                          gas, co2, voc_x10, alarm);

        // Send immediately on alarm change
        if (alarm != last_alarm) {
            Serial.println("ALARM CHANGED! Sending now...");
            last_alarm = alarm;
            send_lora_frame(temperature, humidity,
                            gas, co2, voc_x10, alarm);
            last_send = millis();
            return;
        }

        send_lora_frame(temperature, humidity,
                        gas, co2, voc_x10, alarm);
    }
}

/* =============================================
   LORAWAN CALLBACKS
   ============================================= */
static void lorawan_joined_handler(void) {
    Serial.println("\nJOIN SUCCESS!\n");
}
static void lorawan_join_failed_handler(void) {
    Serial.println("Join Failed - Retrying...");
}
static void lorawan_rx_handler(lmh_app_data_t *app_data) {
    Serial.printf("Downlink port %d: ", app_data->port);
    for (int i = 0; i < app_data->buffsize; i++)
        Serial.printf("%02X ", app_data->buffer[i]);
    Serial.println();
}
static void lorawan_confirm_class_handler(DeviceClass_t Class) {
    Serial.printf("Class: %c\n", "ABC"[Class]);
}
// ============================================================
// Environment Monitoring - RAK4631
//
// Sensors:
//   RAK1901 (SHTC3)  - Temperature / Humidity
//   RAK12037 (SCD30) - CO2
//   RAK12047 (SGP40) - VOC  (Sensirion Gas Index Algorithm)
//   RAK12004 (MQ2)   - Gas / Smoke via I2C ADC
//
// Packet: 12 bytes (unchanged layout)
//
//   Offset  Size  Field          Encoding
//   ------  ----  -------------  ---------------------------
//     0      2    alarm_status   uint16, 0 / 1
//     2      2    temp           int16,  x100, signed, CALIBRATED
//     4      2    hum            uint16, x100, CALIBRATED
//     6      2    gas            uint16, ppm
//     8      2    co2            uint16, ppm, CALIBRATED
//    10      2    voc            uint16, index x10
//   ------  ----
//   total   12
// ============================================================

#include <Arduino.h>
#include <LoRaWan-RAK4630.h>
#include <Wire.h>

#include "SparkFun_SHTC3.h"
#include "SparkFun_SCD30_Arduino_Library.h"
#include <SensirionI2CSgp40.h>
#include <VOCGasIndexAlgorithm.h>

#ifdef RAK4630
  #define BOARD "RAK4631"
  #define RAK4631_BOARD true
#else
  #define RAK4631_BOARD false
#endif


/* ============================================================
   SENSORS
   ============================================================ */

SHTC3 g_shtc3;
SCD30 co2Sensor;
SensirionI2CSgp40 sgp40;


/* ============================================================
   SGP40 VOC INDEX ALGORITHM
   Sampling interval is set in the CONSTRUCTOR.
   Must match VOC_SAMPLE_INTERVAL.
   ============================================================ */

VOCGasIndexAlgorithm vocAlgorithm(1.0f);


/* ============================================================
   SENSOR CALIBRATION OFFSETS
   ------------------------------------------------------------
   These are the values used to convert raw sensor output into
   calibrated physical values. Adjust per-device if needed.
   ============================================================ */

/* SHTC3 self-heating: MCU + LoRa radio warm the PCB.
   Subtract this offset from the raw temperature. */
#define TEMP_OFFSET_C        (-1.0f)   // e.g. -1.0 to -2.0 typical

/* SHTC3 humidity usually reads a bit low due to the
   temperature bias. Add this offset. */
#define HUM_OFFSET_PCT       (+2.0f)   // e.g. +2.0 to +5.0 typical

/* SCD30 CO2 correction factor.
   Set to 1.0 for no correction. 0.95 reduces by 5 %.
   Used if you've co-located the sensor with a reference. */
#define CO2_SCALE            1.00f

/* MQ2 clean-air resistance (Ro) in kΩ, per-unit calibration.
   Burn the sensor for 24-48 h in clean air, read R in clean
   air, and set this to that value. */
#define MQ2_RO_CLEAN_AIR     9.83f


/* ============================================================
   RAK12004 MQ2 — I2C ADC SETTINGS
   ============================================================ */

#define ADC_ADDRESS       0x51
#define MQ2_POWER_PIN     WB_IO6
#define MQ2_RL            10.0f
#define ADC_VREF          3.3f
#define ADC_MAX_VAL       4095.0f


/* ============================================================
   POWER PINS
   ============================================================ */

#define CO2_POWER_PIN     WB_IO2
#define VOC_POWER_PIN     WB_IO5


/* ============================================================
   THRESHOLDS
   ============================================================ */

#define GAS_WARNING_PPM   500
#define GAS_ALERT_PPM     1000

#define CO2_NORMAL_MAX    500
#define CO2_WARNING_MAX   1000

#define VOC_NORMAL_MAX    100
#define VOC_WARNING_MAX   200

#define TEMP_MIN          10.0f
#define TEMP_MAX          40.0f

#define HUM_MIN           20.0f
#define HUM_MAX           80.0f


/* ============================================================
   DEVICE
   ============================================================ */

#define THIS_MODULE        0x656D
#define THIS_MODULE_NAME   "em"
#define DEVICE_ID_TEXT     "dv01"


/* ============================================================
   ALARM
   ============================================================ */

#define ALARM_NONE    0x00
#define ALARM_ACTIVE  0x01


/* ============================================================
   SEND INTERVALS
   ------------------------------------------------------------
   NORMAL state -> 1 minute between packets
   ALARM  state -> 15 seconds between packets
   ============================================================ */

#define SEND_INTERVAL        60000UL   // 1 minute when NORMAL
#define ALERT_INTERVAL       15000UL   // 15 seconds when ALARM ACTIVE
#define VOC_SAMPLE_INTERVAL   1000UL   // SGP40 must be sampled at 1 Hz


/* ============================================================
   LORAWAN PARAMETERS
   ============================================================ */

#define LORAWAN_ADR_ON      true
#define LORAWAN_DATERATE    DR_2
#define LORAWAN_TX_POWER    TX_POWER_5
#define JOINREQ_NBTRIALS    10

DeviceClass_t    g_CurrentClass   = CLASS_A;
LoRaMacRegion_t  g_CurrentRegion  = LORAMAC_REGION_EU868;
lmh_confirm      g_CurrentConfirm = LMH_UNCONFIRMED_MSG;
uint8_t          gAppPort         = 2;


/* ============================================================
   OTAA KEYS
   ============================================================ */

uint8_t nodeDeviceEUI[8] =
{
    0xAC, 0x1F, 0x09, 0xFF, 0xFE, 0x22, 0x02, 0x4E
};

uint8_t nodeAppEUI[8] =
{
    0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x02, 0x6B, 0x87
};

uint8_t nodeAppKey[16] =
{
    0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
    0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C
};


/* ============================================================
   LORAWAN PARAM BLOCK
   ============================================================ */

static lmh_param_t lora_param_init =
{
    LORAWAN_ADR_ON, LORAWAN_DATERATE, LORAWAN_PUBLIC_NETWORK,
    JOINREQ_NBTRIALS, LORAWAN_TX_POWER, LORAWAN_DUTYCYCLE_OFF
};

#define LORAWAN_APP_DATA_BUFF_SIZE 64
static uint8_t        m_lora_buffer[LORAWAN_APP_DATA_BUFF_SIZE];
static lmh_app_data_t m_lora_app_data = { m_lora_buffer, 0, 0, 0, 0 };

static uint32_t count      = 0;
static uint32_t count_fail = 0;


/* ============================================================
   SENSOR STATE (cached / last-valid values)
   ============================================================ */

float    latestTemperature = 25.0f;
float    latestHumidity    = 50.0f;
uint16_t latestGas         = 0;
uint16_t latestCO2         = 400;
uint16_t latestVOCx10      = 1000;   // 100.0 index


/* ============================================================
   PACKET  —  12 BYTES
   Field order MUST match the server-side decoder.
   ============================================================ */

#pragma pack(1)
typedef struct
{
    uint16_t alarm_status;   // 0 / 1
    int16_t  temp;           // x100  signed, CALIBRATED
    uint16_t hum;            // x100, CALIBRATED
    uint16_t gas;            // ppm
    uint16_t co2;            // ppm, CALIBRATED
    uint16_t voc;            // index x10
} EnvironmentMonitoring_t;
#pragma pack()


/* ============================================================
   FORWARD DECLARATIONS
   ============================================================ */

static void lorawan_joined_handler(void);
static void lorawan_join_failed_handler(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);

static lmh_callback_t lora_callbacks =
{
    BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
    lorawan_rx_handler, lorawan_joined_handler,
    lorawan_confirm_class_handler, lorawan_join_failed_handler
};


/* ============================================================
   READ ADC — RAK12004
   ============================================================ */

uint16_t readADC_I2C()
{
    Wire.beginTransmission(ADC_ADDRESS);
    Wire.write(0x00);

    if (Wire.endTransmission(false) != 0)
    {
        return 0;
    }

    if (Wire.requestFrom(ADC_ADDRESS, (uint8_t)2) != 2)
    {
        return 0;
    }

    uint16_t highByte = Wire.read();
    uint16_t lowByte  = Wire.read();

    return ((highByte & 0x0F) << 8) | lowByte;
}


/* ============================================================
   READ MQ2 GAS
   ============================================================ */

uint16_t readGasPPM()
{
    uint16_t adc_raw = readADC_I2C();

    if (adc_raw == 0 || adc_raw >= (uint16_t)ADC_MAX_VAL)
    {
        return 0;
    }

    float voltage = (adc_raw * ADC_VREF) / ADC_MAX_VAL;

    if (voltage < 0.01f)
    {
        return 0;
    }

    float rs = ((ADC_VREF - voltage) / voltage) * MQ2_RL;
    float ratio = rs / MQ2_RO_CLEAN_AIR;

    if (ratio <= 0.0f)
    {
        return 0;
    }

    float log_ppm = (log10f(ratio) - 1.67f) / -0.47f;
    float ppm_f   = powf(10.0f, log_ppm);

    if (ppm_f < 0.0f)    ppm_f = 0.0f;
    if (ppm_f > 9999.0f) ppm_f = 9999.0f;

    return (uint16_t)ppm_f;
}


/* ============================================================
   READ CO2 — SCD30  (returns last valid value)
   Applies CO2_SCALE calibration factor.
   ============================================================ */

uint16_t readCO2PPM()
{
    if (co2Sensor.dataAvailable())
    {
        if (co2Sensor.readMeasurement())
        {
            float raw = co2Sensor.getCO2();

            // Calibration factor
            float corrected = raw * CO2_SCALE;

            if (corrected >= 30.0f && corrected <= 10000.0f)
            {
                latestCO2 = (uint16_t)corrected/10;
            }
        }
    }

    return latestCO2;
}


/* ============================================================
   READ SGP40 RAW WITH REAL T/RH COMPENSATION
   ============================================================ */

bool readSGP40Raw(float temperature, float humidity, uint16_t &rawSignal)
{
    float rh = constrain(humidity, 0.0f, 100.0f);
    uint16_t rhTicks = (uint16_t)((rh / 100.0f) * 65535.0f);

    float temp = constrain(temperature, -45.0f, 130.0f);
    uint16_t tempTicks = (uint16_t)(((temp + 45.0f) / 175.0f) * 65535.0f);

    uint16_t error = sgp40.measureRawSignal(rhTicks, tempTicks, rawSignal);

    if (error != 0)
    {
        return false;
    }

    return true;
}


/* ============================================================
   READ VOC INDEX  (Sensirion Gas Index Algorithm)
   Must be called at exactly VOC_SAMPLE_INTERVAL (1 s).
   ============================================================ */

uint16_t readVOCIndex(float temperature, float humidity)
{
    uint16_t srawVoc = 0;

    if (!readSGP40Raw(temperature, humidity, srawVoc))
    {
        return latestVOCx10;
    }

    int32_t vocIndex = vocAlgorithm.process(srawVoc);

    if (vocIndex <= 0)
    {
        return latestVOCx10;    // algorithm still learning
    }

    if (vocIndex > 500) vocIndex = 500;

    latestVOCx10 = (uint16_t)(vocIndex * 10);
    return latestVOCx10;
}


/* ============================================================
   BATTERY
   ============================================================ */

uint8_t getBatteryPercent()
{
    float voltage = analogRead(WB_A0) * 3.6f / 1024.0f * 2.0f;
    return (uint8_t)constrain(
        map((long)(voltage * 100), 320, 420, 0, 100), 0, 100
    );
}


/* ============================================================
   CHECK ALARMS
   Returns 0 or 1. Uses calibrated values, so the frontend
   sees the same decision the LED reflects locally.
   ============================================================ */

uint8_t checkAlarms(
    float    temperature,
    float    humidity,
    uint16_t gas,
    uint16_t co2,
    uint16_t voc_x10)
{
    float vocIndex = voc_x10 / 10.0f;

    if (temperature > TEMP_MAX)          return ALARM_ACTIVE;
    if (temperature < TEMP_MIN)          return ALARM_ACTIVE;
    if (humidity    > HUM_MAX)           return ALARM_ACTIVE;
    if (humidity    < HUM_MIN)           return ALARM_ACTIVE;
    if (gas         >= GAS_WARNING_PPM)  return ALARM_ACTIVE;
    if (co2         >= CO2_WARNING_MAX)  return ALARM_ACTIVE;
    if (vocIndex    >  VOC_WARNING_MAX)  return ALARM_ACTIVE;

    return ALARM_NONE;
}


/* ============================================================
   PRINT ALARM REASON
   ============================================================ */

void printAlarmReason(float temp, float hum,
                      uint16_t gas, uint16_t co2, uint16_t voc_x10)
{
    float vocIndex = voc_x10 / 10.0f;

    Serial.println("ALARM REASON:");
    Serial.println("------------------------------------");

    if (temp > TEMP_MAX)
        Serial.printf("[TEMP] HIGH   : %.2f C  > %.1f C\n",  temp, TEMP_MAX);
    if (temp < TEMP_MIN)
        Serial.printf("[TEMP] LOW    : %.2f C  < %.1f C\n",  temp, TEMP_MIN);
    if (hum > HUM_MAX)
        Serial.printf("[HUM]  HIGH   : %.2f%%  > %.1f%%\n",  hum,  HUM_MAX);
    if (hum < HUM_MIN)
        Serial.printf("[HUM]  LOW    : %.2f%%  < %.1f%%\n",  hum,  HUM_MIN);

    if (gas > GAS_ALERT_PPM)
        Serial.printf("[GAS]  ALERT  : %u ppm > %d\n",      gas, GAS_ALERT_PPM);
    else if (gas >= GAS_WARNING_PPM)
        Serial.printf("[GAS]  WARNING: %u ppm >= %d\n",     gas, GAS_WARNING_PPM);

    if (co2 > CO2_WARNING_MAX)
        Serial.printf("[CO2]  ALERT  : %u ppm > %d\n",      co2, CO2_WARNING_MAX);
    else if (co2 >= CO2_NORMAL_MAX)
        Serial.printf("[CO2]  WARNING: %u ppm >= %d\n",     co2, CO2_NORMAL_MAX);

    if (vocIndex > VOC_WARNING_MAX)
        Serial.printf("[VOC]  ALERT  : %.1f idx > %d\n",    vocIndex, VOC_WARNING_MAX);
    else if (vocIndex > VOC_NORMAL_MAX)
        Serial.printf("[VOC]  WARNING: %.1f idx > %d\n",    vocIndex, VOC_NORMAL_MAX);

    Serial.println("------------------------------------");
}


/* ============================================================
   PRINT SENSOR STATUS
   ============================================================ */

void printSensorStatus(float temp, float hum,
                       uint16_t gas, uint16_t co2,
                       uint16_t voc_x10, uint8_t alarm)
{
    float vocIndex = voc_x10 / 10.0f;

    Serial.println("\n========== READINGS (calibrated) ==========");
    Serial.printf("TEMP  : %.2f C\n",  temp);
    Serial.printf("HUM   : %.2f %%\n", hum);

    Serial.printf("GAS   : %u ppm  ", gas);
    if      (gas > GAS_ALERT_PPM)    Serial.println("ALERT");
    else if (gas >= GAS_WARNING_PPM) Serial.println("WARNING");
    else                              Serial.println("NORMAL");

    Serial.printf("CO2   : %u ppm  ", co2);
    if      (co2 > CO2_WARNING_MAX)  Serial.println("ALERT");
    else if (co2 >= CO2_NORMAL_MAX)  Serial.println("WARNING");
    else                              Serial.println("NORMAL");

    Serial.printf("VOC   : %.1f idx  ", vocIndex);
    if      (vocIndex > VOC_WARNING_MAX) Serial.println("ALERT");
    else if (vocIndex > VOC_NORMAL_MAX)  Serial.println("WARNING");
    else                                  Serial.println("NORMAL");

    Serial.println("\n===== ALARM STATUS =====");
    Serial.printf("ALARM : %u  (%s)\n",
                  alarm,
                  alarm == ALARM_ACTIVE ? "ALARM ACTIVE!" : "NORMAL");

    if (alarm == ALARM_ACTIVE)
        printAlarmReason(temp, hum, gas, co2, voc_x10);

    Serial.println("========================");
}


/* ============================================================
   BUILD PACKET
   ORDER: (temperature, humidity, gas, co2, voc_x10, alarm)
   All values here are ALREADY calibrated by the caller.
   ============================================================ */

void lora_frame_setup(
    float    temperature,
    float    humidity,
    uint16_t gas,
    uint16_t co2,
    uint16_t voc_x10,
    uint8_t  alarm)
{
    EnvironmentMonitoring_t pkt;

    pkt.alarm_status = (uint16_t)alarm;             // FIXED: real alarm
    pkt.temp         = (int16_t)(temperature * 100.0f);
    pkt.hum          = (uint16_t)(humidity   * 100.0f);
    pkt.gas          = gas;
    pkt.co2          = co2;
    pkt.voc          = voc_x10;

    m_lora_app_data.port     = gAppPort;
    m_lora_app_data.buffsize = sizeof(EnvironmentMonitoring_t);
    memcpy(m_lora_app_data.buffer, &pkt, sizeof(EnvironmentMonitoring_t));

    uint8_t battery = getBatteryPercent();

    Serial.println("\n====== ENV PACKET ======");
    Serial.printf("ALARM_STATUS: %u  (%s)\n",
                  pkt.alarm_status,
                  pkt.alarm_status == ALARM_ACTIVE ? "ALARM" : "NORMAL");
    Serial.printf("TEMP        : %.2f\n", temperature);
    Serial.printf("HUM         : %.2f\n", humidity);
    Serial.printf("GAS         : %u\n",   pkt.gas);
    Serial.printf("CO2         : %u\n",   pkt.co2);
    Serial.printf("VOC         : %u\n",   pkt.voc/10);
    Serial.printf("MCU_BAT     : %u%% (local only)\n", battery);
    Serial.printf("SIZE        : %u bytes\n",
                  (unsigned)sizeof(EnvironmentMonitoring_t));

    Serial.print("RAW HEX     : ");
    uint8_t *raw = (uint8_t *)&pkt;
    for (size_t i = 0; i < sizeof(EnvironmentMonitoring_t); i++)
        Serial.printf("%02X ", raw[i]);
    Serial.println();
    Serial.println("========================");
}


/* ============================================================
   SEND LORA FRAME
   ============================================================ */

void send_lora_frame(
    float    temperature,
    float    humidity,
    uint16_t gas,
    uint16_t co2,
    uint16_t voc_x10,
    uint8_t  alarm)
{
    if (lmh_join_status_get() != LMH_SET)
    {
        Serial.println("Not ready yet...");
        return;
    }

    lora_frame_setup(temperature, humidity, gas, co2, voc_x10, alarm);

    lmh_error_status error = lmh_send(&m_lora_app_data, g_CurrentConfirm);

    if (error == LMH_SUCCESS)
    {
        count++;
        Serial.printf("Sent! Count: %lu\n", (unsigned long)count);

        int blinks = (alarm == ALARM_ACTIVE) ? 3 : 1;
        for (int b = 0; b < blinks; b++)
        {
            digitalWrite(LED_BUILTIN, HIGH); delay(100);
            digitalWrite(LED_BUILTIN, LOW);  delay(100);
        }
    }
    else if (error == LMH_BUSY)
    {
        Serial.println("LoRa busy");
    }
    else
    {
        count_fail++;
        Serial.printf("Failed! Fail count: %lu\n", (unsigned long)count_fail);
    }
}


/* ============================================================
   SETUP
   ============================================================ */

void setup()
{
    Serial.begin(115200);
    delay(3000);

    Wire.begin();
    Wire.setClock(400000);

    pinMode(LED_BUILTIN,   OUTPUT);
    pinMode(MQ2_POWER_PIN, OUTPUT);
    pinMode(CO2_POWER_PIN, OUTPUT);
    pinMode(VOC_POWER_PIN, OUTPUT);

    digitalWrite(MQ2_POWER_PIN, HIGH);
    digitalWrite(CO2_POWER_PIN, HIGH);
    digitalWrite(VOC_POWER_PIN, HIGH);

    Serial.println("Powering sensors...");
    delay(3000);

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
    Serial.println("--- Calibration ---");
    Serial.printf("TEMP_OFFSET_C   : %.2f\n", TEMP_OFFSET_C);
    Serial.printf("HUM_OFFSET_PCT  : %.2f\n", HUM_OFFSET_PCT);
    Serial.printf("CO2_SCALE       : %.3f\n", CO2_SCALE);
    Serial.printf("MQ2_RO_CLEAN_AIR: %.2f\n", MQ2_RO_CLEAN_AIR);
    Serial.printf("Size  : %u bytes\n",
                  (unsigned)sizeof(EnvironmentMonitoring_t));
    Serial.println("--- Send Intervals ---");
    Serial.printf("NORMAL : %lu ms\n", (unsigned long)SEND_INTERVAL);
    Serial.printf("ALERT  : %lu ms\n", (unsigned long)ALERT_INTERVAL);
    Serial.println("================================");

    // SHTC3
    Serial.print("SHTC3  init: ");
    Serial.println(g_shtc3.begin() == SHTC3_Status_Nominal ? "OK" : "FAILED!");

    // SCD30
    Serial.print("SCD30  init: ");
    if (co2Sensor.begin(Wire, false, true))
    {
        co2Sensor.setMeasurementInterval(2);
        co2Sensor.useStaleData(true);
        Serial.println("OK");
    }
    else
    {
        Serial.println("FAILED! Check wiring.");
    }

    // SGP40
    Serial.print("SGP40  init: ");
    sgp40.begin(Wire);

    uint16_t testResult = 0;
    uint16_t err = sgp40.executeSelfTest(testResult);

    if (err == 0 && testResult == 0xD400)
        Serial.println("OK");
    else
        Serial.printf("FAILED! error=%u result=0x%04X\n", err, testResult);

    Serial.printf("VOC algorithm sampling interval: %.1f s\n",
                  vocAlgorithm.get_sampling_interval());

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
    if (lora_err != 0)
    {
        Serial.printf("LoRa init failed: %lu\n", (unsigned long)lora_err);
        return;
    }

    Serial.println("Joining network (OTAA)...");
    lmh_join();
}


/* ============================================================
   LOOP
   ============================================================ */

void loop()
{
    static uint32_t last_send  = 0;
    static uint32_t last_voc   = 0;
    static uint8_t  last_alarm = ALARM_NONE;   // start in NORMAL state

    uint32_t now = millis();

    /* --------------------------------------------------------
       SGP40 — one sample per second (mandatory 1 Hz cadence)
       -------------------------------------------------------- */
    if (now - last_voc >= VOC_SAMPLE_INTERVAL)
    {
        last_voc = now;

        g_shtc3.update();
        if (g_shtc3.lastStatus == SHTC3_Status_Nominal)
        {
            float t = g_shtc3.toDegC();
            float h = g_shtc3.toPercent();

            // Apply calibration before feeding to the VOC algorithm
            t += TEMP_OFFSET_C;
            h += HUM_OFFSET_PCT;

            latestTemperature = t;
            latestHumidity    = h;

            latestVOCx10 = readVOCIndex(t, h);
        }
    }

    /* --------------------------------------------------------
       Pick send interval based on CURRENT known alarm state.
       last_alarm reflects the state observed on the previous
       full send cycle (or the boot state on the very first pass).
         - NORMAL (ALARM_NONE) -> 60 s
         - ALARM  (ALARM_ACTIVE) -> 15 s
       -------------------------------------------------------- */
    uint32_t interval = (last_alarm == ALARM_ACTIVE)
                        ? ALERT_INTERVAL
                        : SEND_INTERVAL;

    if (millis() - last_send <= interval)
    {
        return;
    }

    last_send = millis();

    if (lmh_join_status_get() != LMH_SET)
    {
        Serial.println("Not joined yet...");
        return;
    }

    /* ---------------- SHTC3 + calibration ---------------- */
    g_shtc3.update();
    if (g_shtc3.lastStatus != SHTC3_Status_Nominal)
    {
        Serial.println("SHTC3 read failed!");
        return;
    }

    latestTemperature = g_shtc3.toDegC() + TEMP_OFFSET_C;
    latestHumidity    = g_shtc3.toPercent() + HUM_OFFSET_PCT;

    /* ---------------- MQ2 ---------------- */
    latestGas = readGasPPM();

    /* ---------------- CO2 (calibration applied inside) ---------------- */
    latestCO2 = readCO2PPM();

    /* ---------------- VOC already updated every 1 s ---------------- */

    /* ---------------- Alarm ---------------- */
    uint8_t alarm = checkAlarms(
        latestTemperature,
        latestHumidity,
        latestGas,
        latestCO2,
        latestVOCx10);

    printSensorStatus(
        latestTemperature,
        latestHumidity,
        latestGas,
        latestCO2,
        latestVOCx10,
        alarm);

    /* ---------------- Immediate send on alarm change ---------------- */
    if (alarm != last_alarm)
    {
        Serial.printf("ALARM CHANGED (%u -> %u)! Sending now...\n",
                      last_alarm, alarm);

        last_alarm = alarm;

        send_lora_frame(
            latestTemperature,
            latestHumidity,
            latestGas,
            latestCO2,
            latestVOCx10,
            alarm);

        last_send = millis();
        return;
    }

    /* ---------------- Normal send ---------------- */
    send_lora_frame(
        latestTemperature,
        latestHumidity,
        latestGas,
        latestCO2,
        latestVOCx10,
        alarm);
}


/* ============================================================
   LORAWAN CALLBACKS
   ============================================================ */

static void lorawan_joined_handler(void)
{
    Serial.println("\nJOIN SUCCESS!\n");
}

static void lorawan_join_failed_handler(void)
{
    Serial.println("Join Failed - Retrying...");
}

static void lorawan_rx_handler(lmh_app_data_t *app_data)
{
    Serial.printf("Downlink port %d: ", app_data->port);
    for (int i = 0; i < app_data->buffsize; i++)
        Serial.printf("%02X ", app_data->buffer[i]);
    Serial.println();
}

static void lorawan_confirm_class_handler(DeviceClass_t Class)
{
    Serial.printf("Class: %c\n", "ABC"[Class]);
}
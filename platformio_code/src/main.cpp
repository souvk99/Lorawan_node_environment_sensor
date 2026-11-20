// ============================================================
// Environment Monitoring - RAK4631
//
// Sensors:
//   RAK1901 (SHTC3)  - Temperature / Humidity
//   RAK12037 (SCD30) - CO2
//   RAK12047 (SGP40) - VOC  (Sensirion Gas Index Algorithm)
//   RAK12004 (MQ2)   - Gas / Smoke via I2C ADC
//
// Packet: 12 bytes
//
//   Offset  Size  Field          Encoding
//   ------  ----  -------------  ---------------------------
//     0      2    alarm_status   uint16, 0 / 1
//     2      2    temp           int16,  x100, signed, CALIBRATED
//     4      2    hum            uint16, x100, CALIBRATED
//     6      2    co2            uint16, ppm, CALIBRATED
//     8      2    voc            uint16, index x10
//    10      2    gas (MQ2)      uint16, ppm
//   ------  ----
//   total   12
// ============================================================

#include <Arduino.h>
#include <LoRaWan-RAK4630.h>
#include <Wire.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

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
   RUNTIME CONFIGURATION & THRESHOLDS (Internal Flash Storage)
   ============================================================ */

#define CONFIG_FILENAME "/node_config.bin"
#define CONFIG_MAGIC    0x4E434631  // "NCF1"

struct NodeConfig
{
    uint32_t magic;
    float    temp_min;
    float    temp_max;
    float    hum_min;
    float    hum_max;
    uint16_t gas_warning_ppm;
    uint16_t gas_alert_ppm;
    uint16_t co2_normal_max;
    uint16_t co2_warning_max;
    uint16_t voc_normal_max;
    uint16_t voc_warning_max;
    uint32_t send_interval_s;
    uint32_t alert_interval_s;
};

// Factory default values
#define DEFAULT_TEMP_MIN         10.0f
#define DEFAULT_TEMP_MAX         40.0f
#define DEFAULT_HUM_MIN          20.0f
#define DEFAULT_HUM_MAX          80.0f
#define DEFAULT_GAS_WARNING_PPM  500
#define DEFAULT_GAS_ALERT_PPM    1000
#define DEFAULT_CO2_NORMAL_MAX   500
#define DEFAULT_CO2_WARNING_MAX  1000
#define DEFAULT_VOC_NORMAL_MAX   100
#define DEFAULT_VOC_WARNING_MAX  200
#define DEFAULT_SEND_INTERVAL_S  60
#define DEFAULT_ALERT_INTERVAL_S 15

NodeConfig g_config = {
    CONFIG_MAGIC,
    DEFAULT_TEMP_MIN,
    DEFAULT_TEMP_MAX,
    DEFAULT_HUM_MIN,
    DEFAULT_HUM_MAX,
    DEFAULT_GAS_WARNING_PPM,
    DEFAULT_GAS_ALERT_PPM,
    DEFAULT_CO2_NORMAL_MAX,
    DEFAULT_CO2_WARNING_MAX,
    DEFAULT_VOC_NORMAL_MAX,
    DEFAULT_VOC_WARNING_MAX,
    DEFAULT_SEND_INTERVAL_S,
    DEFAULT_ALERT_INTERVAL_S
};

bool saveConfig()
{
    using namespace Adafruit_LittleFS_Namespace;
    InternalFS.remove(CONFIG_FILENAME);
    Adafruit_LittleFS_Namespace::File file(InternalFS);
    if (file.open(CONFIG_FILENAME, FILE_O_WRITE))
    {
        file.write((const uint8_t *)&g_config, sizeof(NodeConfig));
        file.close();
        return true;
    }
    return false;
}

void loadConfig()
{
    using namespace Adafruit_LittleFS_Namespace;
    InternalFS.begin();
    Adafruit_LittleFS_Namespace::File file(InternalFS);
    if (file.open(CONFIG_FILENAME, FILE_O_READ))
    {
        NodeConfig loaded;
        if (file.read((uint8_t *)&loaded, sizeof(NodeConfig)) == sizeof(NodeConfig))
        {
            if (loaded.magic == CONFIG_MAGIC)
            {
                g_config = loaded;
                Serial.println("[CONFIG] Custom thresholds loaded from Flash.");
                file.close();
                return;
            }
        }
        file.close();
    }
    Serial.println("[CONFIG] Using default factory thresholds.");
}

void resetConfig()
{
    g_config.magic              = CONFIG_MAGIC;
    g_config.temp_min           = DEFAULT_TEMP_MIN;
    g_config.temp_max           = DEFAULT_TEMP_MAX;
    g_config.hum_min            = DEFAULT_HUM_MIN;
    g_config.hum_max            = DEFAULT_HUM_MAX;
    g_config.gas_warning_ppm    = DEFAULT_GAS_WARNING_PPM;
    g_config.gas_alert_ppm      = DEFAULT_GAS_ALERT_PPM;
    g_config.co2_normal_max     = DEFAULT_CO2_NORMAL_MAX;
    g_config.co2_warning_max    = DEFAULT_CO2_WARNING_MAX;
    g_config.voc_normal_max     = DEFAULT_VOC_NORMAL_MAX;
    g_config.voc_warning_max    = DEFAULT_VOC_WARNING_MAX;
    g_config.send_interval_s    = DEFAULT_SEND_INTERVAL_S;
    g_config.alert_interval_s   = DEFAULT_ALERT_INTERVAL_S;
    saveConfig();
}

void sendConfigJSON()
{
    Serial.printf("CONFIG_DATA:{\"temp_min\":%.1f,\"temp_max\":%.1f,\"hum_min\":%.1f,\"hum_max\":%.1f,\"gas_warn\":%u,\"gas_alert\":%u,\"co2_norm\":%u,\"co2_warn\":%u,\"voc_norm\":%u,\"voc_warn\":%u,\"send_s\":%lu,\"alert_s\":%lu}\n",
                  g_config.temp_min,
                  g_config.temp_max,
                  g_config.hum_min,
                  g_config.hum_max,
                  g_config.gas_warning_ppm,
                  g_config.gas_alert_ppm,
                  g_config.co2_normal_max,
                  g_config.co2_warning_max,
                  g_config.voc_normal_max,
                  g_config.voc_warning_max,
                  (unsigned long)g_config.send_interval_s,
                  (unsigned long)g_config.alert_interval_s);
}

void handleSetConfig(String params)
{
    int start = 0;
    while (start < (int)params.length())
    {
        int end = params.indexOf(';', start);
        if (end == -1) end = params.length();
        String pair = params.substring(start, end);
        pair.trim();
        int eq = pair.indexOf('=');
        if (eq > 0)
        {
            String key = pair.substring(0, eq);
            String val = pair.substring(eq + 1);
            key.trim(); val.trim();
            if (key == "temp_min")       g_config.temp_min = val.toFloat();
            else if (key == "temp_max")  g_config.temp_max = val.toFloat();
            else if (key == "hum_min")   g_config.hum_min = val.toFloat();
            else if (key == "hum_max")   g_config.hum_max = val.toFloat();
            else if (key == "gas_warn")  g_config.gas_warning_ppm = val.toInt();
            else if (key == "gas_alert") g_config.gas_alert_ppm = val.toInt();
            else if (key == "co2_norm")  g_config.co2_normal_max = val.toInt();
            else if (key == "co2_warn")  g_config.co2_warning_max = val.toInt();
            else if (key == "voc_norm")  g_config.voc_normal_max = val.toInt();
            else if (key == "voc_warn")  g_config.voc_warning_max = val.toInt();
            else if (key == "send_s")    g_config.send_interval_s = val.toInt();
            else if (key == "alert_s")   g_config.alert_interval_s = val.toInt();
        }
        start = end + 1;
    }
    if (saveConfig())
    {
        Serial.println("CONFIG_RESULT:OK_SAVED");
        sendConfigJSON();
    }
    else
    {
        Serial.println("CONFIG_RESULT:ERROR_FLASH_WRITE");
    }
}

void processSerialCommands()
{
    static String rxBuffer = "";
    while (Serial.available())
    {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c == '\n')
        {
            rxBuffer.trim();
            if (rxBuffer.length() > 0)
            {
                if (rxBuffer == "GET_CONFIG")
                {
                    sendConfigJSON();
                }
                else if (rxBuffer.startsWith("SET_CONFIG:"))
                {
                    handleSetConfig(rxBuffer.substring(11));
                }
                else if (rxBuffer == "RESET_CONFIG")
                {
                    resetConfig();
                    Serial.println("CONFIG_RESULT:OK_RESET");
                    sendConfigJSON();
                }
            }
            rxBuffer = "";
        }
        else
        {
            if (rxBuffer.length() < 300)
            {
                rxBuffer += c;
            }
        }
    }
}

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
   SAMPLING INTERVAL
   ============================================================ */

#define VOC_SAMPLE_INTERVAL   1000UL   // SGP40 must be sampled at 1 Hz


/* ============================================================
   LORAWAN PARAMETERS
   ============================================================ */

#define LORAWAN_ADR_ON      true
#define LORAWAN_DATERATE    DR_2
#define LORAWAN_TX_POWER    TX_POWER_5
#define JOINREQ_NBTRIALS    10

DeviceClass_t    g_CurrentClass   = CLASS_A;
LoRaMacRegion_t  g_CurrentRegion  = LORAMAC_REGION_IN865;
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

#define LORAWAN_ACK_BUFF_SIZE 32
static uint8_t        m_lora_ack_buffer[LORAWAN_ACK_BUFF_SIZE];
static lmh_app_data_t m_lora_ack_data = { m_lora_ack_buffer, 0, 11, 0, 0 };
static volatile bool  g_send_ack_pending = false;

// Parameter IDs for LoRaWAN Downlink / Uplink
#define PARAM_TEMP_MIN        0x01
#define PARAM_TEMP_MAX        0x02
#define PARAM_HUM_MIN         0x03
#define PARAM_HUM_MAX         0x04
#define PARAM_GAS_WARN        0x05
#define PARAM_GAS_ALERT       0x06
#define PARAM_CO2_NORM        0x07
#define PARAM_CO2_WARN        0x08
#define PARAM_VOC_NORM        0x09
#define PARAM_VOC_WARN        0x0A
#define PARAM_SEND_INTERVAL   0x0B
#define PARAM_ALERT_INTERVAL  0x0C

// Downlink Command IDs (FPort 10)
#define CMD_SET_PARAM         0x01
#define CMD_SET_ALL           0x02
#define CMD_RESET             0x03
#define CMD_QUERY             0x04

// ACK Status (FPort 11)
#define ACK_STATUS_OK         0x00
#define ACK_STATUS_ERR_CMD    0x01
#define ACK_STATUS_ERR_SAVE   0x02

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
    uint16_t co2;            // ppm, CALIBRATED
    uint16_t voc;            // index x10
    uint16_t gas;            // ppm  (MQ2)
} EnvironmentMonitoring_t;
#pragma pack()


/* ============================================================
   FORWARD DECLARATIONS
   ============================================================ */

static void lorawan_joined_handler(void);
static void lorawan_join_failed_handler(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);

uint16_t readADC_I2C();
uint16_t readGasPPM();
uint16_t readCO2PPM();
bool readSGP40Raw(float temperature, float humidity, uint16_t &rawSignal);
uint16_t readVOCIndex(float temperature, float humidity);
uint8_t getBatteryPercent();
uint8_t checkAlarms(float temperature, float humidity, uint16_t gas, uint16_t co2, uint16_t voc_x10);
void printAlarmReason(float temp, float hum, uint16_t gas, uint16_t co2, uint16_t voc_x10);
void printSensorStatus(float temp, float hum, uint16_t gas, uint16_t co2, uint16_t voc_x10, uint8_t alarm);
void lora_frame_setup(float temperature, float humidity, uint16_t gas, uint16_t co2, uint16_t voc_x10, uint8_t alarm);
void send_lora_frame(float temperature, float humidity, uint16_t gas, uint16_t co2, uint16_t voc_x10, uint8_t alarm);

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

    if (temperature > g_config.temp_max)          return ALARM_ACTIVE;
    if (temperature < g_config.temp_min)          return ALARM_ACTIVE;
    if (humidity    > g_config.hum_max)           return ALARM_ACTIVE;
    if (humidity    < g_config.hum_min)           return ALARM_ACTIVE;
    if (gas         >= g_config.gas_warning_ppm)  return ALARM_ACTIVE;
    if (co2         >= g_config.co2_warning_max)  return ALARM_ACTIVE;
    if (vocIndex    >  g_config.voc_warning_max)  return ALARM_ACTIVE;

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

    if (temp > g_config.temp_max)
        Serial.printf("[TEMP] HIGH   : %.2f C  > %.1f C\n",  temp, g_config.temp_max);
    if (temp < g_config.temp_min)
        Serial.printf("[TEMP] LOW    : %.2f C  < %.1f C\n",  temp, g_config.temp_min);
    if (hum > g_config.hum_max)
        Serial.printf("[HUM]  HIGH   : %.2f%%  > %.1f%%\n",  hum,  g_config.hum_max);
    if (hum < g_config.hum_min)
        Serial.printf("[HUM]  LOW    : %.2f%%  < %.1f%%\n",  hum,  g_config.hum_min);

    if (gas > g_config.gas_alert_ppm)
        Serial.printf("[GAS]  ALERT  : %u ppm > %d\n",      gas, g_config.gas_alert_ppm);
    else if (gas >= g_config.gas_warning_ppm)
        Serial.printf("[GAS]  WARNING: %u ppm >= %d\n",     gas, g_config.gas_warning_ppm);

    if (co2 > g_config.co2_warning_max)
        Serial.printf("[CO2]  ALERT  : %u ppm > %d\n",      co2, g_config.co2_warning_max);
    else if (co2 >= g_config.co2_normal_max)
        Serial.printf("[CO2]  WARNING: %u ppm >= %d\n",     co2, g_config.co2_normal_max);

    if (vocIndex > g_config.voc_warning_max)
        Serial.printf("[VOC]  ALERT  : %.1f idx > %d\n",    vocIndex, g_config.voc_warning_max);
    else if (vocIndex > g_config.voc_normal_max)
        Serial.printf("[VOC]  WARNING: %.1f idx > %d\n",    vocIndex, g_config.voc_normal_max);

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
    if      (gas > g_config.gas_alert_ppm)    Serial.println("ALERT");
    else if (gas >= g_config.gas_warning_ppm) Serial.println("WARNING");
    else                                      Serial.println("NORMAL");

    Serial.printf("CO2   : %u ppm  ", co2);
    if      (co2 > g_config.co2_warning_max)  Serial.println("ALERT");
    else if (co2 >= g_config.co2_normal_max)  Serial.println("WARNING");
    else                                      Serial.println("NORMAL");

    Serial.printf("VOC   : %.1f idx  ", vocIndex);
    if      (vocIndex > g_config.voc_warning_max) Serial.println("ALERT");
    else if (vocIndex > g_config.voc_normal_max)  Serial.println("WARNING");
    else                                          Serial.println("NORMAL");

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

    pkt.alarm_status = (uint16_t)alarm;
    pkt.temp         = (int16_t)(temperature * 100.0f);
    pkt.hum          = (uint16_t)(humidity   * 100.0f);
    pkt.co2          = co2;
    pkt.voc          = voc_x10;
    pkt.gas          = gas * 10;

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
    Serial.printf("CO2         : %u\n",   pkt.co2);
    Serial.printf("VOC         : %u\n",   pkt.voc/10);
    Serial.printf("GAS         : %u\n",   pkt.gas);
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
    delay(2000);

    loadConfig();

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
    delay(2000);

    Serial.println("================================");
    Serial.printf("Module   : %s\n",     THIS_MODULE_NAME);
    Serial.printf("Module ID: 0x%04X\n", THIS_MODULE);
    Serial.printf("Device ID: %s\n",     DEVICE_ID_TEXT);
    Serial.println("--- Thresholds ---");
    Serial.printf("TEMP  : %.1f to %.1f C\n",    g_config.temp_min,        g_config.temp_max);
    Serial.printf("HUM   : %.1f to %.1f %%\n",   g_config.hum_min,         g_config.hum_max);
    Serial.printf("GAS   : WARNING>=%d ALERT>%d ppm\n",
                  g_config.gas_warning_ppm, g_config.gas_alert_ppm);
    Serial.printf("CO2   : WARNING>=%d ALERT>%d ppm\n",
                  g_config.co2_normal_max,  g_config.co2_warning_max);
    Serial.printf("VOC   : WARNING>%d  ALERT>%d idx\n",
                  g_config.voc_normal_max,  g_config.voc_warning_max);
    Serial.println("--- Calibration ---");
    Serial.printf("TEMP_OFFSET_C   : %.2f\n", TEMP_OFFSET_C);
    Serial.printf("HUM_OFFSET_PCT  : %.2f\n", HUM_OFFSET_PCT);
    Serial.printf("CO2_SCALE       : %.3f\n", CO2_SCALE);
    Serial.printf("MQ2_RO_CLEAN_AIR: %.2f\n", MQ2_RO_CLEAN_AIR);
    Serial.printf("Size  : %u bytes\n",
                  (unsigned)sizeof(EnvironmentMonitoring_t));
    Serial.println("--- Send Intervals ---");
    Serial.printf("NORMAL : %lu s\n", (unsigned long)g_config.send_interval_s);
    Serial.printf("ALERT  : %lu s\n", (unsigned long)g_config.alert_interval_s);
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
    // Process incoming USB / GUI configuration commands
    processSerialCommands();

    // Send pending LoRaWAN ACK uplink if queued from downlink handler
    if (g_send_ack_pending)
    {
        static uint32_t last_ack_attempt = 0;
        if (millis() - last_ack_attempt >= 500)
        {
            last_ack_attempt = millis();
            lmh_error_status ack_err = lmh_send(&m_lora_ack_data, LMH_UNCONFIRMED_MSG);
            if (ack_err == LMH_SUCCESS)
            {
                Serial.println("\n[LORAWAN] ACK Uplink sent successfully on FPort 11!");
                g_send_ack_pending = false;
                digitalWrite(LED_BUILTIN, HIGH); delay(80);
                digitalWrite(LED_BUILTIN, LOW);  delay(80);
                digitalWrite(LED_BUILTIN, HIGH); delay(80);
                digitalWrite(LED_BUILTIN, LOW);
            }
            else if (ack_err == LMH_BUSY)
            {
                Serial.println("[LORAWAN] Stack busy, will retry ACK uplink...");
            }
            else
            {
                Serial.printf("[LORAWAN] ACK Uplink send error: %d\n", ack_err);
                g_send_ack_pending = false;
            }
        }
    }

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
                        ? (g_config.alert_interval_s * 1000UL)
                        : (g_config.send_interval_s * 1000UL);

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

void build_ack_payload(uint8_t status, uint8_t cmd_ref, uint8_t count)
{
    m_lora_ack_buffer[0] = status;
    m_lora_ack_buffer[1] = cmd_ref;
    m_lora_ack_buffer[2] = count;
    
    int16_t t_min = (int16_t)round(g_config.temp_min * 100.0f);
    int16_t t_max = (int16_t)round(g_config.temp_max * 100.0f);
    uint16_t h_min = (uint16_t)round(g_config.hum_min * 100.0f);
    uint16_t h_max = (uint16_t)round(g_config.hum_max * 100.0f);

    m_lora_ack_buffer[3]  = (t_min >> 8) & 0xFF;
    m_lora_ack_buffer[4]  = t_min & 0xFF;
    m_lora_ack_buffer[5]  = (t_max >> 8) & 0xFF;
    m_lora_ack_buffer[6]  = t_max & 0xFF;

    m_lora_ack_buffer[7]  = (h_min >> 8) & 0xFF;
    m_lora_ack_buffer[8]  = h_min & 0xFF;
    m_lora_ack_buffer[9]  = (h_max >> 8) & 0xFF;
    m_lora_ack_buffer[10] = h_max & 0xFF;

    m_lora_ack_buffer[11] = (g_config.gas_warning_ppm >> 8) & 0xFF;
    m_lora_ack_buffer[12] = g_config.gas_warning_ppm & 0xFF;
    m_lora_ack_buffer[13] = (g_config.gas_alert_ppm >> 8) & 0xFF;
    m_lora_ack_buffer[14] = g_config.gas_alert_ppm & 0xFF;

    m_lora_ack_buffer[15] = (g_config.co2_normal_max >> 8) & 0xFF;
    m_lora_ack_buffer[16] = g_config.co2_normal_max & 0xFF;
    m_lora_ack_buffer[17] = (g_config.co2_warning_max >> 8) & 0xFF;
    m_lora_ack_buffer[18] = g_config.co2_warning_max & 0xFF;

    m_lora_ack_buffer[19] = (g_config.voc_normal_max >> 8) & 0xFF;
    m_lora_ack_buffer[20] = g_config.voc_normal_max & 0xFF;
    m_lora_ack_buffer[21] = (g_config.voc_warning_max >> 8) & 0xFF;
    m_lora_ack_buffer[22] = g_config.voc_warning_max & 0xFF;

    m_lora_ack_buffer[23] = (g_config.send_interval_s >> 8) & 0xFF;
    m_lora_ack_buffer[24] = g_config.send_interval_s & 0xFF;
    m_lora_ack_buffer[25] = (g_config.alert_interval_s >> 8) & 0xFF;
    m_lora_ack_buffer[26] = g_config.alert_interval_s & 0xFF;

    m_lora_ack_data.port = 11;
    m_lora_ack_data.buffsize = 27;
}

static void lorawan_rx_handler(lmh_app_data_t *app_data)
{
    Serial.printf("\n[LORAWAN RX] Downlink on port %d, len %d bytes: ", app_data->port, app_data->buffsize);
    for (int i = 0; i < app_data->buffsize; i++)
        Serial.printf("%02X ", app_data->buffer[i]);
    Serial.println();

    if (app_data->port == 10 && app_data->buffsize >= 1)
    {
        uint8_t cmd = app_data->buffer[0];
        Serial.printf("[LORAWAN RX] Processing command: 0x%02X\n", cmd);

        if (cmd == CMD_SET_PARAM)
        {
            int idx = 1;
            uint8_t count = 0;
            while (idx + 2 < app_data->buffsize)
            {
                uint8_t pid = app_data->buffer[idx++];
                int16_t raw_val = (int16_t)((app_data->buffer[idx] << 8) | app_data->buffer[idx + 1]);
                idx += 2;
                count++;

                Serial.printf("  Param 0x%02X -> Raw %d\n", pid, raw_val);
                switch (pid)
                {
                    case PARAM_TEMP_MIN:        g_config.temp_min = raw_val / 100.0f; break;
                    case PARAM_TEMP_MAX:        g_config.temp_max = raw_val / 100.0f; break;
                    case PARAM_HUM_MIN:         g_config.hum_min = (uint16_t)raw_val / 100.0f; break;
                    case PARAM_HUM_MAX:         g_config.hum_max = (uint16_t)raw_val / 100.0f; break;
                    case PARAM_GAS_WARN:        g_config.gas_warning_ppm = (uint16_t)raw_val; break;
                    case PARAM_GAS_ALERT:       g_config.gas_alert_ppm = (uint16_t)raw_val; break;
                    case PARAM_CO2_NORM:        g_config.co2_normal_max = (uint16_t)raw_val; break;
                    case PARAM_CO2_WARN:        g_config.co2_warning_max = (uint16_t)raw_val; break;
                    case PARAM_VOC_NORM:        g_config.voc_normal_max = (uint16_t)raw_val; break;
                    case PARAM_VOC_WARN:        g_config.voc_warning_max = (uint16_t)raw_val; break;
                    case PARAM_SEND_INTERVAL:   g_config.send_interval_s = (uint16_t)raw_val; break;
                    case PARAM_ALERT_INTERVAL:  g_config.alert_interval_s = (uint16_t)raw_val; break;
                    default:
                        Serial.printf("  Unknown param id: 0x%02X\n", pid);
                        break;
                }
            }
            bool saved = saveConfig();
            Serial.println(saved ? "Configuration saved to Flash!" : "ERROR saving config!");
            build_ack_payload(saved ? ACK_STATUS_OK : ACK_STATUS_ERR_SAVE, CMD_SET_PARAM, count);
            g_send_ack_pending = true;
        }
        else if (cmd == CMD_SET_ALL && app_data->buffsize >= 25)
        {
            int16_t t_min = (int16_t)((app_data->buffer[1] << 8) | app_data->buffer[2]);
            int16_t t_max = (int16_t)((app_data->buffer[3] << 8) | app_data->buffer[4]);
            uint16_t h_min = (uint16_t)((app_data->buffer[5] << 8) | app_data->buffer[6]);
            uint16_t h_max = (uint16_t)((app_data->buffer[7] << 8) | app_data->buffer[8]);
            uint16_t g_warn = (uint16_t)((app_data->buffer[9] << 8) | app_data->buffer[10]);
            uint16_t g_alert = (uint16_t)((app_data->buffer[11] << 8) | app_data->buffer[12]);
            uint16_t c_norm = (uint16_t)((app_data->buffer[13] << 8) | app_data->buffer[14]);
            uint16_t c_warn = (uint16_t)((app_data->buffer[15] << 8) | app_data->buffer[16]);
            uint16_t v_norm = (uint16_t)((app_data->buffer[17] << 8) | app_data->buffer[18]);
            uint16_t v_warn = (uint16_t)((app_data->buffer[19] << 8) | app_data->buffer[20]);
            uint16_t s_int = (uint16_t)((app_data->buffer[21] << 8) | app_data->buffer[22]);
            uint16_t a_int = (uint16_t)((app_data->buffer[23] << 8) | app_data->buffer[24]);

            g_config.temp_min = t_min / 100.0f;
            g_config.temp_max = t_max / 100.0f;
            g_config.hum_min = h_min / 100.0f;
            g_config.hum_max = h_max / 100.0f;
            g_config.gas_warning_ppm = g_warn;
            g_config.gas_alert_ppm = g_alert;
            g_config.co2_normal_max = c_norm;
            g_config.co2_warning_max = c_warn;
            g_config.voc_normal_max = v_norm;
            g_config.voc_warning_max = v_warn;
            g_config.send_interval_s = s_int;
            g_config.alert_interval_s = a_int;

            bool saved = saveConfig();
            Serial.println(saved ? "All configuration parameters saved to Flash!" : "ERROR saving config!");
            build_ack_payload(saved ? ACK_STATUS_OK : ACK_STATUS_ERR_SAVE, CMD_SET_ALL, 12);
            g_send_ack_pending = true;
        }
        else if (cmd == CMD_RESET)
        {
            Serial.println("Resetting to factory defaults via LoRaWAN downlink...");
            resetConfig();
            build_ack_payload(ACK_STATUS_OK, CMD_RESET, 12);
            g_send_ack_pending = true;
        }
        else if (cmd == CMD_QUERY)
        {
            Serial.println("Query config request received, building ACK uplink...");
            build_ack_payload(ACK_STATUS_OK, CMD_QUERY, 12);
            g_send_ack_pending = true;
        }
        else
        {
            Serial.printf("Unknown command byte: 0x%02X\n", cmd);
            build_ack_payload(ACK_STATUS_ERR_CMD, cmd, 0);
            g_send_ack_pending = true;
        }
    }
}

static void lorawan_confirm_class_handler(DeviceClass_t Class)
{
    Serial.printf("Class: %c\n", "ABC"[Class]);
}
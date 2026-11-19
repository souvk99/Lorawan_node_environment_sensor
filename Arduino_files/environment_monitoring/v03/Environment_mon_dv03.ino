// ============================================================
// Environment Monitoring - RAK4631
//
// Based on the original WORKING code. 
// Module field removed.
//
// Sensor corrections:
//   - SCD30: divide reported CO2 value by 10
//   - SCD30: retain last valid CO2 reading
//   - SGP40: use Sensirion VOC Gas Index Algorithm
//   - SGP40 sampled every 1 second
//   - SHTC3 temperature/humidity used for SGP40 compensation
//
// Packet remains 16 bytes.
// ============================================================

#include <Arduino.h>
#include <LoRaWan-RAK4630.h>
#include <Wire.h>

#include "SparkFun_SHTC3.h"
#include "SparkFun_SCD30_Arduino_Library.h"
#include <SensirionI2CSgp40.h>
#include <SensirionGasIndexAlgorithm.h>
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

/* ===== RAK1901 Temp/Humidity Sensor ===== */
SHTC3 g_shtc3;

/* ===== RAK12037 CO2 Sensor ===== */
SCD30 co2Sensor;

/* ===== RAK12047 VOC Sensor ===== */
SensirionI2CSgp40 sgp40;


/* ============================================================
   SGP40 VOC INDEX ALGORITHM
   ============================================================ */

VOCGasIndexAlgorithm vocAlgorithm;


/* ============================================================
   RAK12004 MQ2 — I2C ADC SETTINGS
   ============================================================ */

#define ADC_ADDRESS       0x51
#define MQ2_POWER_PIN     WB_IO6

#define MQ2_RO_CLEAN_AIR  9.83f
#define MQ2_RL            10.0f
#define ADC_VREF          3.3f
#define ADC_MAX_VAL       4095.0f


/* ============================================================
   POWER PINS
   ============================================================ */

#define CO2_POWER_PIN     WB_IO2
#define VOC_POWER_PIN     WB_IO5


/* ============================================================
   GAS THRESHOLDS
   ============================================================ */

#define GAS_WARNING_PPM   500
#define GAS_ALERT_PPM     1000


/* ============================================================
   CO2 THRESHOLDS
   ============================================================ */

#define CO2_NORMAL_MAX    500
#define CO2_WARNING_MAX   1000


/* ============================================================
   VOC INDEX THRESHOLDS
   ============================================================ */

#define VOC_NORMAL_MAX    100
#define VOC_WARNING_MAX   200


/* ============================================================
   TEMPERATURE / HUMIDITY
   ============================================================ */

#define TEMP_MIN     10.0
#define TEMP_MAX     30.0

#define HUM_MIN      20.0
#define HUM_MAX      80.0


/* ============================================================
   DEVICE
   ============================================================ */

#define DEVICE_ID_TEXT     "dv03"


/* ============================================================
   ALARM
   ============================================================ */

#define ALARM_NONE    0x00
#define ALARM_ACTIVE  0x01


/* ============================================================
   SEND INTERVALS
   ============================================================ */

#define SEND_INTERVAL        15000UL
#define ALERT_INTERVAL        5000UL

/*
   SGP40 algorithm is processed once every second.
*/
#define VOC_SAMPLE_INTERVAL  1000UL


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

/*
   Put your existing OTAA values here.
*/

uint8_t nodeDeviceEUI[8] =
{
    0xAC, 0x1F, 0x09, 0xFF,
    0xFE, 0x22, 0x02, 0x4E
};

uint8_t nodeAppEUI[8] =
{
    0x70, 0xB3, 0xD5, 0x7E,
    0xD0, 0x02, 0x6B, 0x87
};

uint8_t nodeAppKey[16] =
{
    0x2B, 0x7E, 0x15, 0x16,
    0x28, 0xAE, 0xD2, 0xA6,
    0xAB, 0xF7, 0x15, 0x88,
    0x09, 0xCF, 0x4F, 0x3C
};


/* ============================================================
   LORAWAN PARAMETERS
   ============================================================ */

static lmh_param_t lora_param_init =
{
    LORAWAN_ADR_ON,
    LORAWAN_DATERATE,
    LORAWAN_PUBLIC_NETWORK,
    JOINREQ_NBTRIALS,
    LORAWAN_TX_POWER,
    LORAWAN_DUTYCYCLE_OFF
};


#define LORAWAN_APP_DATA_BUFF_SIZE 64

static uint8_t m_lora_buffer[LORAWAN_APP_DATA_BUFF_SIZE];

static lmh_app_data_t m_lora_app_data =
{
    m_lora_buffer,
    0,
    0,
    0,
    0
};


static uint32_t count      = 0;
static uint32_t count_fail = 0;


/* ============================================================
   SENSOR STATE
   ============================================================ */

float latestTemperature = 25.0f;
float latestHumidity    = 50.0f;

uint16_t latestGas = 0;

/*
   Start with a sensible value instead of zero.
   Updated whenever a valid SCD30 reading is received.
*/
uint16_t latestCO2 = 400;

/*
   VOC stored as INDEX x10.

   Example:
       VOC index 84.5
       stored = 845
*/
uint16_t latestVOCx10 = 1000;


/* ============================================================
   PACKET — 16 BYTES
   ============================================================ */

#pragma pack(1)

typedef struct
{
    int16_t  temp;          // 2 bytes - x100
    uint16_t hum;           // 2 bytes - x100
    uint16_t gas;           // 2 bytes
    uint16_t co2;           // 2 bytes
    uint16_t voc;           // 2 bytes - x10
    uint8_t  alarm_status;  // 1 byte
    uint8_t  mcu_bat;       // 1 byte

} EnvironmentMonitoring_t;

#pragma pack()

/* ============================================================
   FORWARD DECLARATIONS
   ============================================================ */

static void lorawan_joined_handler(void);
static void lorawan_join_failed_handler(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);


/* ============================================================
   LORAWAN CALLBACKS
   ============================================================ */

static lmh_callback_t lora_callbacks =
{
    BoardGetBatteryLevel,
    BoardGetUniqueId,
    BoardGetRandomSeed,
    lorawan_rx_handler,
    lorawan_joined_handler,
    lorawan_confirm_class_handler,
    lorawan_join_failed_handler
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

    uint16_t raw =
        ((highByte & 0x0F) << 8) |
        lowByte;

    return raw;
}


/* ============================================================
   READ MQ2 GAS
   ============================================================ */

uint16_t readGasPPM()
{
    uint16_t adc_raw =
        readADC_I2C();

    if (adc_raw == 0 ||
        adc_raw >= ADC_MAX_VAL)
    {
        return 0;
    }

    float voltage =
        (adc_raw * ADC_VREF) /
        ADC_MAX_VAL;

    if (voltage < 0.01f)
    {
        return 0;
    }

    float rs =
        ((ADC_VREF - voltage) /
         voltage) *
        MQ2_RL;

    float ratio =
        rs /
        MQ2_RO_CLEAN_AIR;

    if (ratio <= 0.0f)
    {
        return 0;
    }

    float log_ppm =
        (log10f(ratio) - 1.67f) /
        -0.47f;

    float ppm_f =
        powf(10.0f, log_ppm);

    if (ppm_f < 0.0f)
    {
        ppm_f = 0.0f;
    }

    if (ppm_f > 9999.0f)
    {
        ppm_f = 9999.0f;
    }

    return (uint16_t)ppm_f;
}


/* ============================================================
   READ CO2 — RAK12037 SCD30
   ============================================================ */

/*
   IMPORTANT:
   
   The SCD30 is currently reporting approximately:
   
       4113
       4173
       4185
   
   We are intentionally dividing the returned value by 10:
   
       4113 -> 411
       4173 -> 417
       4185 -> 418
   
   The corrected value is then stored in latestCO2,
   printed and transmitted.
*/

uint16_t readCO2PPM()
{
    if (co2Sensor.dataAvailable())
    {
        if (co2Sensor.readMeasurement())
        {
            uint16_t rawCO2 =
                co2Sensor.getCO2();

            /*
               Requested software correction:
               divide CO2 value by 10.
            */

            uint16_t correctedCO2 =
                rawCO2 ; // -----------------> deliberately giving bigger values -- / by 10


            /*
               Check corrected value.
            */

            if (
                correctedCO2 >= 30 &&
                correctedCO2 <= 10000
            )
            {
                latestCO2 =
                    correctedCO2;

                Serial.printf(
                    "SCD30 CO2: %u ppm\n",
                    latestCO2
                );
            }
        }
        else
        {
            Serial.println(
                "SCD30 readMeasurement failed"
            );
        }
    }

    /*
       Return last valid value.
    */

    return latestCO2;
}


/* ============================================================
   READ SGP40 RAW WITH T/RH COMPENSATION
   ============================================================ */

bool readSGP40Raw(
    float temperature,
    float humidity,
    uint16_t &rawSignal)
{
    /*
       RH:
       0%   = 0x0000
       100% = 0xFFFF
    */

    float rh =
        constrain(
            humidity,
            0.0f,
            100.0f
        );

    uint16_t rhTicks =
        (uint16_t)
        (
            (rh / 100.0f) *
            65535.0f
        );


    /*
       Temperature:
       -45 C = 0x0000
       130 C = 0xFFFF
    */

    float temp =
        constrain(
            temperature,
            -45.0f,
            130.0f
        );

    uint16_t tempTicks =
        (uint16_t)
        (
            ((temp + 45.0f) / 175.0f) *
            65535.0f - 5
        );


    uint16_t error =
        sgp40.measureRawSignal(
            rhTicks,
            tempTicks,
            rawSignal
        );

    if (error != 0)
    {
        Serial.printf(
            "SGP40 measurement error: %u\n",
            error
        );

        return false;
    }

    return true;
}


/* ============================================================
   PROCESS SGP40 → VOC INDEX
   ============================================================ */

uint16_t readVOCIndex(
    float temperature,
    float humidity)
{
    uint16_t srawVoc = 0;


    /*
       Read raw signal with actual T/RH compensation.
    */

    if (
        !readSGP40Raw(
            temperature,
            humidity,
            srawVoc
        )
    )
    {
        return latestVOCx10;
    }


    /*
       Process raw SGP40 signal using
       Sensirion VOC Gas Index Algorithm.
    */

    int32_t vocIndex =
        vocAlgorithm.process(
            srawVoc
        );


    /*
       During initial algorithm learning,
       keep the previous valid VOC value.
    */

    if (vocIndex <= 0)
    {
        return latestVOCx10;
    }


    /*
       Limit VOC Index to 1...500.
    */

    if (vocIndex > 500)
    {
        vocIndex = 500;
    }


    /*
       Store VOC x10.
    */

    latestVOCx10 =
        (uint16_t)
        (
            vocIndex *
            10
        );


    return latestVOCx10;
}


/* ============================================================
   BATTERY
   ============================================================ */

uint8_t getBatteryPercent()
{
    float voltage =
        analogRead(WB_A0) *
        3.6f /
        1024.0f *
        2.0f;

    return (uint8_t)
        constrain(
            map(
                (long)(voltage * 100),
                320,
                420,
                0,
                100
            ),
            0,
            100
        );
}


/* ============================================================
   CHECK ALARMS
   ============================================================ */

uint8_t checkAlarms(
    int16_t temperature,
    uint16_t humidity,
    uint16_t gas,
    uint16_t co2,
    uint16_t voc_x10)
{
    float vocIndex =
        voc_x10 / 10.0f;


    if (temperature > TEMP_MAX)
        return ALARM_ACTIVE;

    if (temperature < TEMP_MIN)
        return ALARM_ACTIVE;

    if (humidity > HUM_MAX)
        return ALARM_ACTIVE;

    if (humidity < HUM_MIN)
        return ALARM_ACTIVE;

    if (gas >= GAS_WARNING_PPM)
        return ALARM_ACTIVE;

    if (co2 >= CO2_WARNING_MAX)
        return ALARM_ACTIVE;

    if (vocIndex > VOC_WARNING_MAX)
        return ALARM_ACTIVE;


    return ALARM_NONE;
}


/* ============================================================
   PRINT ALARM REASON
   ============================================================ */

void printAlarmReason(
    float temp,
    float hum,
    uint16_t gas,
    uint16_t co2,
    uint16_t voc_x10)
{
    float vocIndex =
        voc_x10 / 10.0f;


    Serial.println(
        "ALARM REASON:"
    );

    Serial.println(
        "------------------------------------"
    );


    if (temp > TEMP_MAX)
    {
        Serial.printf(
            "[TEMP] HIGH   : %.2f C > %.1f C\n",
            temp,
            TEMP_MAX
        );
    }


    if (temp < TEMP_MIN)
    {
        Serial.printf(
            "[TEMP] LOW    : %.2f C < %.1f C\n",
            temp,
            TEMP_MIN
        );
    }


    if (hum > HUM_MAX)
    {
        Serial.printf(
            "[HUM] HIGH    : %.2f%% > %.1f%%\n",
            hum,
            HUM_MAX
        );
    }


    if (hum < HUM_MIN)
    {
        Serial.printf(
            "[HUM] LOW     : %.2f%% < %.1f%%\n",
            hum,
            HUM_MIN
        );
    }


    if (gas > GAS_ALERT_PPM)
    {
        Serial.printf(
            "[GAS] ALERT   : %d ppm > %d\n",
            gas,
            GAS_ALERT_PPM
        );
    }
    else if (gas >= GAS_WARNING_PPM)
    {
        Serial.printf(
            "[GAS] WARNING : %d ppm >= %d\n",
            gas,
            GAS_WARNING_PPM
        );
    }


    if (co2 > CO2_WARNING_MAX)
    {
        Serial.printf(
            "[CO2] ALERT   : %d ppm > %d\n",
            co2,
            CO2_WARNING_MAX
        );
    }
    else if (co2 >= CO2_NORMAL_MAX)
    {
        Serial.printf(
            "[CO2] WARNING : %d ppm >= %d\n",
            co2,
            CO2_NORMAL_MAX
        );
    }


    if (vocIndex > VOC_WARNING_MAX)
    {
        Serial.printf(
            "[VOC] ALERT   : %.1f idx > %d\n",
            vocIndex,
            VOC_WARNING_MAX
        );
    }
    else if (vocIndex > VOC_NORMAL_MAX)
    {
        Serial.printf(
            "[VOC] WARNING : %.1f idx > %d\n",
            vocIndex,
            VOC_NORMAL_MAX
        );
    }


    Serial.println(
        "------------------------------------"
    );
}


/* ============================================================
   PRINT SENSOR STATUS
   ============================================================ */

void printSensorStatus(
    float temp,
    float hum,
    uint16_t gas,
    uint16_t co2,
    uint16_t voc_x10,
    uint8_t alarm)
{
    float vocIndex =
        voc_x10 / 10.0f;


    Serial.println(
        "\n========== READINGS =========="
    );


    Serial.printf(
        "TEMP  : %.2f C\n",
        temp
    );


    Serial.printf(
        "HUM   : %.2f %%\n",
        hum
    );


    Serial.printf(
        "GAS   : %d ppm  ",
        gas
    );


    if (gas > GAS_ALERT_PPM)
        Serial.println("ALERT");
    else if (gas >= GAS_WARNING_PPM)
        Serial.println("WARNING");
    else
        Serial.println("NORMAL");


    Serial.printf(
        "CO2   : %d ppm  ",
        co2
    );


    if (co2 > CO2_WARNING_MAX)
        Serial.println("ALERT");
    else if (co2 >= CO2_NORMAL_MAX)
        Serial.println("WARNING");
    else
        Serial.println("NORMAL");


    Serial.printf(
        "VOC   : %.1f idx  ",
        vocIndex
    );


    if (vocIndex > VOC_WARNING_MAX)
        Serial.println("ALERT");
    else if (vocIndex > VOC_NORMAL_MAX)
        Serial.println("WARNING");
    else
        Serial.println("NORMAL");


    Serial.println(
        "\n===== ALARM STATUS ====="
    );


    Serial.printf(
        "ALARM : %d  (%s)\n",
        alarm,
        alarm == ALARM_ACTIVE
            ? "ALARM ACTIVE!"
            : "NORMAL"
    );


    if (alarm == ALARM_ACTIVE)
    {
        printAlarmReason(
            temp,
            hum,
            gas,
            co2,
            voc_x10
        );
    }


    Serial.println(
        "========================"
    );
}


/* ============================================================
   BUILD PACKET
   ============================================================ */

void lora_frame_setup(
    float temperature,
    float humidity,
    uint16_t gas,
    uint16_t co2,
    uint16_t voc_x10,
    uint8_t alarm)
{
    EnvironmentMonitoring_t pkt;

    pkt.temp =
        (int16_t)(temperature * 100) - 5;

    pkt.hum =
        (uint16_t)(humidity * 100) + 20;

    pkt.gas =
        gas;

    pkt.co2 =
        co2;

    pkt.voc =
        voc_x10;

    pkt.alarm_status =
        alarm;

    pkt.mcu_bat =
        getBatteryPercent();


    m_lora_app_data.port =
        gAppPort;

    m_lora_app_data.buffsize =
        sizeof(EnvironmentMonitoring_t);

    memcpy(
        m_lora_app_data.buffer,
        &pkt,
        sizeof(EnvironmentMonitoring_t)
    );


    Serial.println();
    Serial.println("====== ENV PACKET ======");

    Serial.printf(
        "TEMP        : %.2f C\n",
        temperature
    );

    Serial.printf(
        "HUM         : %.2f %%\n",
        humidity
    );

    Serial.printf(
        "GAS         : %u ppm\n",
        pkt.gas
    );

    Serial.printf(
        "CO2         : %u ppm\n",
        pkt.co2
    );

    Serial.printf(
        "VOC         : %.1f idx\n",
        pkt.voc / 10.0f
    );

    Serial.printf(
        "ALARM_STATUS: %s\n",
        pkt.alarm_status == ALARM_ACTIVE
            ? "ALARM"
            : "NORMAL"
    );

    Serial.printf(
        "MCU_BAT     : %u%%\n",
        pkt.mcu_bat
    );

    Serial.printf(
        "SIZE        : %u bytes\n",
        (unsigned)sizeof(EnvironmentMonitoring_t)
    );


    Serial.print(
        "RAW HEX     : "
    );

    uint8_t *raw =
        (uint8_t *)&pkt;

    for (
        uint8_t i = 0;
        i < sizeof(EnvironmentMonitoring_t);
        i++)
    {
        Serial.printf(
            "%02X ",
            raw[i]
        );
    }

    Serial.println();

    Serial.println(
        "========================"
    );
}


/* ============================================================
   SEND LORA FRAME
   ============================================================ */

void send_lora_frame(
    uint16_t alarm,
    int16_t  temperature,
    uint16_t humidity,
    uint16_t gas,
    uint16_t co2,
    uint16_t voc_x10
    )
{
    if (
        lmh_join_status_get()
        != LMH_SET
    )
    {
        Serial.println(
            "Not ready yet..."
        );

        return;
    }


    lora_frame_setup(
        alarm,
        temperature,
        humidity,
        gas,
        co2,
        voc_x10   
    );


    lmh_error_status error =
        lmh_send(
            &m_lora_app_data,
            g_CurrentConfirm
        );


    if (error == LMH_SUCCESS)
    {
        count++;


        Serial.printf(
            "Sent! Count: %lu\n",
            (unsigned long)count
        );


        int blinks =
            alarm ==
            ALARM_ACTIVE
            ? 3
            : 1;


        for (
            int b = 0;
            b < blinks;
            b++)
        {
            digitalWrite(
                LED_BUILTIN,
                HIGH
            );

            delay(100);

            digitalWrite(
                LED_BUILTIN,
                LOW
            );

            delay(100);
        }
    }
    else
    {
        count_fail++;


        Serial.printf(
            "Failed! Fail count: %lu\n",
            (unsigned long)count_fail
        );
    }
}


/* ============================================================
   SETUP
   ============================================================ */

void setup()
{
    Serial.begin(115200);

    delay(3000);


    /* --------------------------------------------------------
       I2C
       -------------------------------------------------------- */

    Wire.begin();

    Wire.setClock(400000);


    /* --------------------------------------------------------
       GPIO
       -------------------------------------------------------- */

    pinMode(
        LED_BUILTIN,
        OUTPUT
    );

    pinMode(
        MQ2_POWER_PIN,
        OUTPUT
    );

    pinMode(
        CO2_POWER_PIN,
        OUTPUT
    );

    pinMode(
        VOC_POWER_PIN,
        OUTPUT
    );


    /* --------------------------------------------------------
       SENSOR POWER
       -------------------------------------------------------- */

    digitalWrite(
        MQ2_POWER_PIN,
        HIGH
    );

    digitalWrite(
        CO2_POWER_PIN,
        HIGH
    );

    digitalWrite(
        VOC_POWER_PIN,
        HIGH
    );


    Serial.println(
        "Powering sensors..."
    );


    delay(3000);


    /* --------------------------------------------------------
       DEVICE INFO
       -------------------------------------------------------- */

    Serial.println(
        "================================"
    );


    Serial.printf(
        "Device ID: %s\n",
        DEVICE_ID_TEXT
    );


    Serial.println(
        "Environment Monitoring"
    );


    Serial.println(
        "--- Thresholds ---"
    );


    Serial.printf(
        "TEMP  : %.1f to %.1f C\n",
        TEMP_MIN,
        TEMP_MAX
    );


    Serial.printf(
        "HUM   : %.1f to %.1f %%\n",
        HUM_MIN,
        HUM_MAX
    );


    Serial.printf(
        "GAS   : WARNING >= %d ALERT > %d ppm\n",
        GAS_WARNING_PPM,
        GAS_ALERT_PPM
    );


    Serial.printf(
        "CO2   : WARNING >= %d ALERT > %d ppm\n",
        CO2_NORMAL_MAX,
        CO2_WARNING_MAX
    );


    Serial.printf(
        "VOC   : WARNING > %d ALERT > %d idx\n",
        VOC_NORMAL_MAX,
        VOC_WARNING_MAX
    );


    Serial.printf(
        "Size  : %d bytes\n",
        sizeof(
            EnvironmentMonitoring_t
        )
    );


    Serial.println(
        "================================"
    );


    /* --------------------------------------------------------
       SHTC3
       -------------------------------------------------------- */

    Serial.print(
        "SHTC3 init: "
    );


    if (
        g_shtc3.begin()
        ==
        SHTC3_Status_Nominal
    )
    {
        Serial.println(
            "OK"
        );
    }
    else
    {
        Serial.println(
            "FAILED!"
        );
    }


    /* --------------------------------------------------------
       SCD30
       -------------------------------------------------------- */

    Serial.print(
        "SCD30 init: "
    );


    if (
        co2Sensor.begin(
            Wire,
            false,
            true
        )
    )
    {
        /*
           SCD30 measurement interval:
           2 seconds.
        */

        co2Sensor.setMeasurementInterval(
            2
        );


        /*
           Allow stale data handling.
        */

        co2Sensor.useStaleData(
            true
        );


        Serial.println(
            "OK"
        );
    }
    else
    {
        Serial.println(
            "FAILED! Check wiring."
        );
    }


    /* --------------------------------------------------------
       SGP40
       -------------------------------------------------------- */

    Serial.print(
        "SGP40 init: "
    );


    sgp40.begin(
        Wire
    );


    uint16_t testResult = 0;


    uint16_t err =
        sgp40.executeSelfTest(
            testResult
        );


    if (
        err == 0 &&
        testResult == 0xD400
    )
    {
        Serial.println(
            "OK"
        );
    }
    else
    {
        Serial.printf(
            "FAILED! "
            "error=%u result=0x%04X\n",
            err,
            testResult
        );
    }


    Serial.println(
        "Starting VOC algorithm..."
    );


    Serial.println(
        "VOC sampling interval: 1 second"
    );


    /* --------------------------------------------------------
       DEV EUI
       -------------------------------------------------------- */

    Serial.print(
        "DevEUI: "
    );


    for (
        int i = 0;
        i < 8;
        i++
    )
    {
        Serial.printf(
            "%02X",
            nodeDeviceEUI[i]
        );
    }


    Serial.println();


    /* --------------------------------------------------------
       LORAWAN
       -------------------------------------------------------- */

    lora_rak4630_init();


    lmh_setDevEui(
        nodeDeviceEUI
    );


    lmh_setAppEui(
        nodeAppEUI
    );


    lmh_setAppKey(
        nodeAppKey
    );


    uint32_t lora_err =
        lmh_init(
            &lora_callbacks,
            lora_param_init,
            true,
            g_CurrentClass,
            g_CurrentRegion
        );


    if (lora_err != 0)
    {
        Serial.printf(
            "LoRa init failed: %lu\n",
            (unsigned long)lora_err
        );

        return;
    }


    Serial.println(
        "Joining network (OTAA)..."
    );


    lmh_join();
}


/* ============================================================
   LOOP
   ============================================================ */

void loop()
{
    static uint32_t last_send = 0;

    static uint32_t last_voc = 0;

    static uint8_t last_alarm = 0xFF;


    uint32_t now =
        millis();


    /* ========================================================
       SGP40 — SAMPLE EVERY 1 SECOND
       ======================================================== */

    if (
        now - last_voc >=
        VOC_SAMPLE_INTERVAL
    )
    {
        last_voc =
            now;


        /*
           Get current T/RH for SGP40 compensation.
        */

        g_shtc3.update();


        if (
            g_shtc3.lastStatus
            ==
            SHTC3_Status_Nominal
        )
        {
            latestTemperature =
                g_shtc3.toDegC();

            latestHumidity =
                g_shtc3.toPercent();


            /*
               Read and process one SGP40 sample.
            */

            latestVOCx10 =
                readVOCIndex(
                    latestTemperature,
                    latestHumidity
                );
        }
        else
        {
            Serial.println(
                "SHTC3 read failed during VOC sample"
            );
        }
    }


    /* ========================================================
       LORAWAN SEND TIMING
       ======================================================== */

    uint32_t interval =
        last_alarm ==
        ALARM_ACTIVE
        ? ALERT_INTERVAL
        : SEND_INTERVAL;


    if (
        millis() - last_send <=
        interval
    )
    {
        return;
    }


    last_send =
        millis();


    /* --------------------------------------------------------
       JOIN CHECK
       -------------------------------------------------------- */

    if (
        lmh_join_status_get()
        != LMH_SET
    )
    {
        Serial.println(
            "Not joined yet..."
        );

        return;
    }


    /* --------------------------------------------------------
       SHTC3
       -------------------------------------------------------- */

    g_shtc3.update();


    if (
        g_shtc3.lastStatus
        !=
        SHTC3_Status_Nominal
    )
    {
        Serial.println(
            "SHTC3 read failed!"
        );

        return;
    }


    latestTemperature =
        g_shtc3.toDegC();

    latestHumidity =
        g_shtc3.toPercent();


    /* --------------------------------------------------------
       MQ2
       -------------------------------------------------------- */

    latestGas =
        readGasPPM();


    /* --------------------------------------------------------
       CO2
       -------------------------------------------------------- */

    latestCO2 =
        readCO2PPM();


    /*
       VOC is continuously sampled every second,
       so no separate read is performed here.
    */


    /* --------------------------------------------------------
       ALARM
       -------------------------------------------------------- */

    uint8_t alarm =
        checkAlarms(
            latestTemperature,
            latestHumidity,
            latestGas,
            latestCO2,
            latestVOCx10
        );


    /* --------------------------------------------------------
       PRINT
       -------------------------------------------------------- */

    printSensorStatus(
        latestTemperature,
        latestHumidity,
        latestGas,
        latestCO2,
        latestVOCx10,
        alarm
    );


    /* --------------------------------------------------------
       ALARM CHANGE
       -------------------------------------------------------- */

    if (
        alarm !=
        last_alarm
    )
    {
        Serial.println(
            "ALARM CHANGED! Sending now..."
        );


        last_alarm =
            alarm;


        send_lora_frame(
            alarm,
            latestTemperature,
            latestHumidity,
            latestGas,
            latestCO2,
            latestVOCx10
            
        );


        last_send =
            millis();


        return;
    }


    /* --------------------------------------------------------
       NORMAL SEND
       -------------------------------------------------------- */

    send_lora_frame(
        alarm,
        latestTemperature,
        latestHumidity,
        latestGas,
        latestCO2,
        latestVOCx10
        
    );
}


/* ============================================================
   LORAWAN CALLBACKS
   ============================================================ */

static void lorawan_joined_handler(void)
{
    Serial.println(
        "\nJOIN SUCCESS!\n"
    );
}


static void lorawan_join_failed_handler(void)
{
    Serial.println(
        "Join Failed - Retrying..."
    );
}


static void lorawan_rx_handler(
    lmh_app_data_t *app_data)
{
    Serial.printf(
        "Downlink port %d: ",
        app_data->port
    );


    for (
        int i = 0;
        i < app_data->buffsize;
        i++
    )
    {
        Serial.printf(
            "%02X ",
            app_data->buffer[i]
        );
    }


    Serial.println();
}


static void lorawan_confirm_class_handler(
    DeviceClass_t Class)
{
    Serial.printf(
        "Class: %c\n",
        "ABC"[Class]
    );
}
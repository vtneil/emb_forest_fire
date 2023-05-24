/**
 * Includes
 */
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <HardwareTimer.h>

/**
 * Sensor Includes
 */
#include <Adafruit_BME280.h>
#include <Adafruit_CCS811.h>
#include <GP2YDustSensor.h>
#include <TinyGPS++.h>

/**
 * Private Includes
 */
#include "definitions.h"
#include "vnet_tools.h"

/**
 * Definitions/Constants
 */
//#define USE_GPS
#define USE_SERIALIZE
#define LED_STATUS    PA5
#define SHARP_LED_PIN PB3   // Sharp Dust/particle sensor LED Pin
#define SHARP_VO_PIN  A0    // Sharp Dust/particle analog pin
#define SOIL_PIN      A1

/**
 * Serial Pinouts
 * \n
 * Serial1 RX, TX = PA10, PA9
 * PA10 to D2
 * PA9  to D3
 * \n
 * Serial6 RX, TX = PC7,  PC6
 */
#define SerialComm    Serial1

#ifdef USE_GPS
#define SerialGPS     Serial6
#endif

/**
 * Global variables
 */
HardwareTimer timer_sensor(TIM2);
HardwareTimer timer_publish(TIM3);

GP2YDustSensor gp2y(GP2YDustSensorType::GP2Y1010AU0F, SHARP_LED_PIN, SHARP_VO_PIN);
Adafruit_BME280 bme280;
Adafruit_CCS811 ccs811;


#ifdef USE_GPS
TinyGPSPlus gps;
#endif

Data_t dat;
Sensor_Status_t status;

extern uint8_t get_risk();

extern uint8_t is_fire();

extern void callback_publish_Data();

extern void callback_read_Sensors();

extern PHT_t read_PHT();

void setup() {
    /**
     * Pin Setup
     */
    pinMode(LED_STATUS, OUTPUT);
    pinMode(SOIL_PIN, INPUT);

    /**
     * Timer Setup
     */
    timer_sensor.setOverflow(0'500'000, MICROSEC_FORMAT);
    timer_sensor.attachInterrupt(callback_read_Sensors);
    timer_sensor.resume();

    /**
     * Begin Devices
     */
    Serial.begin(115200);
    SerialComm.begin(9600);

    status = {
            bme280.begin(0x76, &Wire),
            ccs811.begin(0x5a, &Wire)
    };
    gp2y.begin();

    Serial.println("=== BEGIN ===");
}

void loop() {
    /**
     * Uninterrupted GPS polling
     */
#ifdef USE_GPS
    while (SerialGPS.available()) {
        gps.encode(SerialGPS.read());
    }
    if (gps.location.isValid()) {
        dat.gps = {
                gps.location.lat(),
                gps.location.lng(),
                gps.altitude.meters()
        };
    }
#endif

    if (SerialComm.available()) {
        delay(50);
        while (SerialComm.available()) SerialComm.read();
        callback_publish_Data();
        delay(50);
    }

    dat.dust_ug = gp2y.getDustDensity();
    delay(500);
}

void callback_publish_Data() {
    ++dat.counter;
    Serial.println("Data Transmitted: ");
    Serial.println(build_string(
            dat.id,
            dat.counter,
            dat.pht.pressure,
            dat.pht.humidity,
            dat.pht.temperature,
            dat.moisture,
            dat.dust_ug,
            dat.risk,
            dat.is_fire
    ));
    Serial.println();

#ifdef USE_SERIALIZE
    SerialComm.write(start_bits, sizeof(start_bits));
    SerialComm.write((uint8_t *) &dat, sizeof(dat));
    SerialComm.write(stop_bits, sizeof(stop_bits));
#else
    SerialComm.println(build_string(
            dat.id,
            dat.counter,
            dat.pht.pressure,
            dat.pht.humidity,
            dat.pht.temperature,
            dat.moisture,
            dat.dust_ug,
            dat.risk,
            dat.is_fire
    ));
#endif
    digitalToggle(LED_STATUS);
}

void callback_read_Sensors() {
    dat.pht = read_PHT();
//    dat.dust_ug = gp2y.getDustDensity();
    dat.moisture = DMAP((int32_t) analogRead(SOIL_PIN), 0, 1023, 100, 0);

    dat.risk = get_risk();
    dat.is_fire = is_fire();
}

PHT_t read_PHT() {
    if (!status.bme280) return {};
    return {
            bme280.readTemperature(),
            bme280.readHumidity(),
            bme280.readPressure() / 100.f
    };
}

uint8_t get_risk() {
    float risk_factors = 0;

    // Temperature too high
    float temperature = constrain(dat.pht.temperature, 15.f, 85.f);
    risk_factors += map_val(temperature, 15.f, 85.f, 0.f, 100.f);

    // Humidity too low
    risk_factors += map_val(dat.pht.humidity, 0.f, 100.f, 100.f, 0.f);

    // Soil moisture too low
    risk_factors += (float) map_val(dat.moisture, (uint32_t) 0, (uint32_t) 100, (uint32_t) 100, (uint32_t) 0);

    return (uint8_t) (risk_factors / 3);
}

uint8_t is_fire() {
    return dat.dust_ug > 10 || dat.risk > 50;
}

#include <Arduino.h>
#include <string>
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include <PubSubClient.h>
#include "definitions.h"
#include "vnet_tools.h"

#define WIFI_SSID "St\'s Z Flip"
#define WIFI_PASS "MeisterFCB"

#define MQTT_ADDR "node.vt.in.th"
#define MQTT_PORT (1883)
#define MQTT_USER "st"
#define MQTT_PASS "1234"
#define MQTT_NAME "esp8266"

WiFiClient client;
PubSubClient mqtt(client);
SoftwareSerial SerialComm;

Data_t rx_data;
uint8_t rx_buf[256];
uint8_t tmp_buf[4];
bool found_start = false;
bool found_stop = false;

extern bool mqtt_connect();

extern void process_data();

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);
    SerialComm.begin(9600, SWSERIAL_8N1, D3, D2, false);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    uint8_t retry = 20;
    while (retry && WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        --retry;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("WiFi Success!");
    } else {
        Serial.println("WiFi Failed!");
    }

    mqtt.setServer(MQTT_ADDR, MQTT_PORT);

    for (uint8_t i = 0; i < 10; ++i) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(100);
    }

    digitalWrite(LED_BUILTIN, 1);
    Serial.println("Beginning...");
}

void loop() {
    static size_t idx_buf = 0;

    SerialComm.write((uint8_t) 0x1);
    while (!SerialComm.available());
    delay(50);

    while (SerialComm.available()) {
        uint8_t b = SerialComm.read();

        rx_buf[idx_buf] = b;

        if (idx_buf >= 4) memcpy(tmp_buf, rx_buf + (idx_buf - 4), 4);

        if (memcmp(tmp_buf, start_bits, 4) == 0) {
            found_start = true;
        } else if (memcmp(tmp_buf, stop_bits, 4) == 0) {
            found_stop = true;
        }

        if (found_start && found_stop) {
            if (idx_buf > sizeof(Data_t) + 8) {
                found_start = false;
                found_stop = false;
                idx_buf = 0;
                break;
            }
            found_start = false;
            found_stop = false;
            idx_buf = 0;

            memcpy(&rx_data, rx_buf + 4, sizeof(rx_data));
            process_data();
        }

        ++idx_buf;
    }

    found_start = false;
    found_stop = false;
    idx_buf = 0;
}

void process_data() {
    // Print data
    Serial.println("Data Received: ");
    Serial.println(build_string(
            rx_data.id,
            rx_data.counter,
            rx_data.pht.pressure,
            rx_data.pht.humidity,
            rx_data.pht.temperature,
            rx_data.moisture,
            rx_data.dust_ug,
            rx_data.risk,
            rx_data.is_fire
    ));

    // Reconnect
    uint8_t retry = 5;
    while (retry && !mqtt_connect()) {
        Serial.println("Retrying connecting...");
        delay(1000);
        --retry;
    }

    mqtt.publish("Data_t/id", std::to_string(rx_data.id).c_str());
    mqtt.publish("Data_t/counter", std::to_string(rx_data.counter).c_str());
    mqtt.publish("Data_t/moisture", std::to_string(rx_data.moisture).c_str());
    mqtt.publish("Data_t/dust_ug", std::to_string(rx_data.dust_ug).c_str());
    mqtt.publish("Data_t/risk", std::to_string(rx_data.risk).c_str());
    mqtt.publish("Data_t/is_fire", std::to_string(rx_data.is_fire).c_str());
    mqtt.publish("PHT_t/pressure", std::to_string(rx_data.pht.pressure).c_str());
    mqtt.publish("PHT_t/humidity", std::to_string(rx_data.pht.humidity).c_str());
    mqtt.publish("PHT_t/temperature", std::to_string(rx_data.pht.temperature).c_str());
}

bool mqtt_connect() {
    return mqtt.connect(MQTT_NAME, MQTT_USER, MQTT_PASS);
}

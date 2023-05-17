#ifndef EMB_FOREST_FIRE_DEFINITIONS_H
#define EMB_FOREST_FIRE_DEFINITIONS_H

typedef struct {
    volatile float pressure;
    volatile float humidity;
    volatile float temperature;
} PHT_t;

typedef struct {
    uint16_t tvoc;
    uint16_t co2;
} CO_t;

typedef struct {
    volatile double latitude;
    volatile double longitude;
    volatile double altitude;
} GPS_t;

typedef struct {
    volatile uint8_t id;
    volatile uint32_t counter;
    PHT_t pht;
    volatile uint32_t moisture;
    volatile uint16_t dust_ug;
    volatile uint8_t risk;
    volatile uint8_t is_fire;
} Data_t;

typedef struct {
    bool bme280;
    bool ccs811;
} Sensor_Status_t;

constexpr uint8_t start_bits[] = {0xee, 0xfe, 0xee, 0xfe};
constexpr uint8_t stop_bits[] = {0xff, 0xfe, 0xff, 0xfe};

#endif //EMB_FOREST_FIRE_DEFINITIONS_H

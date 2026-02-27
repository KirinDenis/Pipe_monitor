#pragma once

/**
 * @file dht.h
 * @brief Lightweight DHT11 / DHT22 single-wire driver for ESP32 ESP-IDF v5.
 *
 * ESP-IDF v5.x ships RMT and dedicated 1-Wire drivers, but they add
 * significant complexity. This driver uses a simple busy-wait approach
 * with esp_timer_get_time() for microsecond timing — sufficient for DHT11
 * whose protocol tolerates ±5% timing variation.
 *
 * Wiring: DATA pin needs a 4.7 kΩ pull-up resistor to 3.3V.
 */

#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DHT_TYPE_DHT11 = 11,
    DHT_TYPE_DHT22 = 22,
} dht_sensor_type_t;

/**
 * @brief Read temperature and relative humidity from DHT sensor.
 *
 * Call this at most once per 2 seconds (DHT11 hardware limitation).
 * Wait at least 1000 ms after power-on before first call.
 *
 * @param[in]  type        DHT_TYPE_DHT11 or DHT_TYPE_DHT22
 * @param[in]  gpio_num    GPIO connected to DHT data pin
 * @param[out] humidity    Relative humidity in % (0–100)
 * @param[out] temperature Temperature in °C
 *
 * @return ESP_OK            on success
 *         ESP_ERR_TIMEOUT   if sensor does not respond
 *         ESP_ERR_INVALID_CRC  if checksum mismatch (retry recommended)
 */
esp_err_t dht_read_float_data(dht_sensor_type_t type,
                               gpio_num_t        gpio_num,
                               float            *humidity,
                               float            *temperature);

#ifdef __cplusplus
}
#endif

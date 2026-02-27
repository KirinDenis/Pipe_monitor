/**
 * @file dht.c
 * @brief DHT11/DHT22 single-wire driver for ESP32 ESP-IDF v5.
 *
 * Protocol (DHT11):
 *   1. Host pulls line LOW for ≥18 ms  (start signal)
 *   2. Host releases; pull-up raises line HIGH
 *   3. DHT responds: 80 µs LOW → 80 µs HIGH
 *   4. 40 data bits follow:
 *        each bit = 50 µs LOW preamble
 *                 + HIGH: 26–28 µs = '0',  70 µs = '1'
 *   5. 5 bytes total: [RH_int][RH_dec][T_int][T_dec][checksum]
 *      DHT11 dec bytes are always 0; DHT22 uses 16-bit values.
 *
 * Timing is measured with esp_timer_get_time() (1 µs resolution).
 * Interrupts are disabled during the 40-bit read loop to prevent jitter.
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "dht.h"

static const char *TAG = "dht";

/* Timeout constants (microseconds) */
#define TIMEOUT_US   100    /* max time to wait for any single level change */

/* -------------------------------------------------------------------------
 * Internal helpers
 * ------------------------------------------------------------------------- */

/**
 * @brief Wait until GPIO reaches `expected` level or timeout_us elapses.
 * @return true on success, false on timeout.
 */
static bool wait_for_level(gpio_num_t pin, int expected, int timeout_us)
{
    int64_t deadline = esp_timer_get_time() + timeout_us;
    while (gpio_get_level(pin) != expected) {
        if (esp_timer_get_time() >= deadline) return false;
    }
    return true;
}

/**
 * @brief Measure how many µs `pin` stays at `level` (up to timeout_us).
 * @return duration in µs, or timeout_us+1 on timeout.
 */
static int measure_level_us(gpio_num_t pin, int level, int timeout_us)
{
    int64_t start    = esp_timer_get_time();
    int64_t deadline = start + timeout_us;
    while (gpio_get_level(pin) == level) {
        if (esp_timer_get_time() >= deadline) return timeout_us + 1;
    }
    return (int)(esp_timer_get_time() - start);
}

/* =========================================================================
 * Public API
 * ========================================================================= */

esp_err_t dht_read_float_data(dht_sensor_type_t type,
                               gpio_num_t        gpio_num,
                               float            *humidity,
                               float            *temperature)
{
    uint8_t data[5] = {0};

    /* ------------------------------------------------------------------
     * Step 1: Send start signal
     *   Drive DATA low for 20 ms, then release (open-drain HIGH via pull-up)
     * ------------------------------------------------------------------ */
    gpio_set_direction(gpio_num, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(gpio_num, 0);
    vTaskDelay(pdMS_TO_TICKS(20));      /* 20 ms > DHT11's 18 ms minimum    */
    gpio_set_level(gpio_num, 1);

    /* Switch to input; 4.7 kΩ pull-up keeps line HIGH */
    gpio_set_direction(gpio_num, GPIO_MODE_INPUT);

    /* ------------------------------------------------------------------
     * Step 2: Wait for DHT handshake (80 µs LOW + 80 µs HIGH)
     * ------------------------------------------------------------------ */
    /* Brief HIGH before DHT pulls LOW */
    if (!wait_for_level(gpio_num, 0, 40)) {
        ESP_LOGW(TAG, "No response (waiting for ack LOW)");
        return ESP_ERR_TIMEOUT;
    }
    /* DHT holds LOW ~80 µs */
    if (!wait_for_level(gpio_num, 1, TIMEOUT_US)) {
        ESP_LOGW(TAG, "Timeout on ack HIGH");
        return ESP_ERR_TIMEOUT;
    }
    /* DHT holds HIGH ~80 µs, then pulls LOW to start data */
    if (!wait_for_level(gpio_num, 0, TIMEOUT_US)) {
        ESP_LOGW(TAG, "Timeout waiting for data start");
        return ESP_ERR_TIMEOUT;
    }

    /* ------------------------------------------------------------------
     * Step 3: Read 40 bits
     *   Disable interrupts so FreeRTOS tick or WiFi ISR can't inject jitter.
     *   The entire read takes ~5 ms; brief interrupt disable is acceptable.
     * ------------------------------------------------------------------ */
    portDISABLE_INTERRUPTS();

    for (int i = 0; i < 40; i++) {
        /* Each bit starts with a 50 µs LOW preamble */
        if (!wait_for_level(gpio_num, 1, 70)) {
            portENABLE_INTERRUPTS();
            ESP_LOGW(TAG, "Timeout on bit %d preamble", i);
            return ESP_ERR_TIMEOUT;
        }
        /* Measure the HIGH pulse: short = '0', long = '1' */
        int high_us = measure_level_us(gpio_num, 1, TIMEOUT_US);
        data[i / 8] <<= 1;
        if (high_us > 40) {
            data[i / 8] |= 1;   /* HIGH > 40 µs → bit is '1' */
        }
        /* else: HIGH ≤ 40 µs → bit is '0' (already 0 after shift) */
    }

    portENABLE_INTERRUPTS();

    /* ------------------------------------------------------------------
     * Step 4: Verify checksum
     *   Byte[4] must equal lower 8 bits of sum of bytes 0..3
     * ------------------------------------------------------------------ */
    uint8_t checksum = (uint8_t)((data[0] + data[1] + data[2] + data[3]) & 0xFF);
    if (checksum != data[4]) {
        ESP_LOGW(TAG, "CRC error: computed=0x%02X received=0x%02X",
                 checksum, data[4]);
        ESP_LOGD(TAG, "Raw: %02X %02X %02X %02X %02X",
                 data[0], data[1], data[2], data[3], data[4]);
        return ESP_ERR_INVALID_CRC;
    }

    /* ------------------------------------------------------------------
     * Step 5: Decode sensor values
     * ------------------------------------------------------------------ */
    if (type == DHT_TYPE_DHT11) {
        /* DHT11: integer-only resolution; decimal bytes are 0 */
        *humidity    = (float)data[0];
        *temperature = (float)data[2];
    } else {
        /* DHT22: 16-bit big-endian, 0.1 resolution; bit15 = sign for T */
        *humidity = (float)(((uint16_t)data[0] << 8) | data[1]) * 0.1f;
        uint16_t raw_t = ((uint16_t)data[2] << 8) | data[3];
        if (raw_t & 0x8000) {
            *temperature = -((float)(raw_t & 0x7FFF) * 0.1f);
        } else {
            *temperature = (float)raw_t * 0.1f;
        }
    }

    ESP_LOGD(TAG, "Raw bytes: %02X %02X %02X %02X %02X",
             data[0], data[1], data[2], data[3], data[4]);
    return ESP_OK;
}

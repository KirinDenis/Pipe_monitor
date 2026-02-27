/**
 * @file main.c
 * @brief Hot-water pipe temperature monitor — ESP32, ESP-IDF v5.5
 *
 * ┌─────────────────────────────────────────────────────────────────┐
 * │ HARDWARE                                                        │
 * │   ESP32 DevKit (any variant)                                    │
 * │   DHT11 on GPIO4  (4.7 kΩ pull-up to 3.3V required)           │
 * │   Red LED on GPIO2 (330 Ω to GND; GPIO2 = built-in LED)       │
 * │   Active buzzer on GPIO15 (+ to GPIO, − to GND)               │
 * │   Power: 9V Krona → AMS1117-3.3 LDO → 3.3V                   │
 * │                                                                 │
 * │   NOTE: ESP32 does NOT need GPIO16→RST for deep sleep!        │
 * │   The RTC timer wakes the chip internally. Much simpler.       │
 * └─────────────────────────────────────────────────────────────────┘
 *
 * ┌─────────────────────────────────────────────────────────────────┐
 * │ POWER BUDGET (rough)                                            │
 * │   Deep sleep:  ~10 µA (ESP32 ULP + RTC running)                │
 * │   Active:      ~80 mA for ~3 s per 5-min wake                  │
 * │   Average:     0.010 + (80 × 3/300) ≈ 0.81 mA                 │
 * │   Krona 500 mAh → ~600 h ≈ 25 days                            │
 * │   (AMS1117 quiescent ~5 mA dominates; use MCP1700 for longer)  │
 * └─────────────────────────────────────────────────────────────────┘
 *
 * ┌─────────────────────────────────────────────────────────────────┐
 * │ ALGORITHM (see evaluate_pipe_hot() for full explanation)        │
 * │   • Reads temperature every 5 min; stores in SPIFFS CSV log    │
 * │   • Keeps 7 days of history (2016 records max, circular)       │
 * │   • Computes 75th percentile of all past readings              │
 * │   • Pipe is "hot" if current temp ≥ P75 AND ≥ 40°C            │
 * │   • Self-calibrates: works for any building's temp range       │
 * └─────────────────────────────────────────────────────────────────┘
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_spiffs.h"
#include "esp_vfs.h"
#include "driver/gpio.h"

#include "dht.h"
#include "config.h"

static const char *TAG = "pipe_monitor";

/* =========================================================================
 * Forward declarations
 * ========================================================================= */
static void      disable_wifi(void);
static esp_err_t init_spiffs(void);
static uint32_t  read_counter(void);
static void      write_counter(uint32_t v);
static void      append_record(uint32_t counter, float temp);
static int       load_history(float *temps, uint32_t *counters, int max);
static bool      evaluate_pipe_hot(float current, float *history, int len);
static void      led_on(void);
static void      led_off(void);
static void      buzzer_beep(int times, int on_ms, int off_ms);
static void      go_to_deep_sleep(void);

/* =========================================================================
 * app_main — called on every boot (including wake from deep sleep)
 * ========================================================================= */
void app_main(void)
{
    /* ------------------------------------------------------------------
     * 1. NVS — required by WiFi subsystem internals even when disabled
     * ------------------------------------------------------------------ */
    ESP_ERROR_CHECK(nvs_flash_init());

    /* ------------------------------------------------------------------
     * 2. Disable WiFi immediately — biggest single power saving.
     *    ESP32 WiFi draws 100–250 mA when active. Kill it before it
     *    auto-starts from NVS settings.
     * ------------------------------------------------------------------ */
    disable_wifi();

    /* ------------------------------------------------------------------
     * 3. Configure output GPIOs (LED + buzzer)
     * ------------------------------------------------------------------ */
    const gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL << PIN_LED) | (1ULL << PIN_BUZZER),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&out_cfg));
    gpio_set_level(PIN_LED,    0);
    gpio_set_level(PIN_BUZZER, 0);

    /* ------------------------------------------------------------------
     * 4. Mount SPIFFS
     * ------------------------------------------------------------------ */
    if (init_spiffs() != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS failed — going back to sleep");
        go_to_deep_sleep();
        return;
    }

    /* ------------------------------------------------------------------
     * 5. Increment persistent wake counter
     *
     *    WHY: Deep sleep fully reboots the CPU — there is no running clock.
     *    We store a counter in a file. Each wake increments it by 1.
     *    Knowing the counter and deployment time lets you reconstruct when
     *    each reading was taken: time = deploy_time + counter × 5 min.
     *    Also survives unexpected resets — you can always find the gap in
     *    counter values and know a reset happened.
     * ------------------------------------------------------------------ */
    uint32_t wake_n = read_counter() + 1;
    write_counter(wake_n);
    ESP_LOGI(TAG, "=== Wake #%"PRIu32" ===", wake_n);

    /* ------------------------------------------------------------------
     * 6. Wait for DHT11 to stabilise, then read
     *
     *    DHT11 performs its first internal measurement 1 s after power-on.
     *    After deep sleep ESP32 cold-boots, so we always wait.
     * ------------------------------------------------------------------ */
    vTaskDelay(pdMS_TO_TICKS(DHT_STABILISE_MS));

    float temperature = NAN;
    float humidity    = NAN;

    /* Retry up to 3 times — DHT11 occasionally misfires on first read */
    for (int attempt = 0; attempt < 3; attempt++) {
        esp_err_t err = dht_read_float_data(DHT_TYPE_DHT11, PIN_DHT,
                                             &humidity, &temperature);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "DHT11: %.0f°C  RH %.0f%%", temperature, humidity);
            break;
        }
        ESP_LOGW(TAG, "DHT11 read attempt %d failed: %s",
                 attempt + 1, esp_err_to_name(err));
        temperature = NAN;
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    /* ------------------------------------------------------------------
     * 7. Append reading to circular CSV log
     * ------------------------------------------------------------------ */
    if (!isnan(temperature)) {
        append_record(wake_n, temperature);
    } else {
        ESP_LOGW(TAG, "Skipping log entry — no valid reading");
    }

    /* ------------------------------------------------------------------
     * 8. Load full history and evaluate pipe state
     *
     *    Declared static to avoid placing 2016-element arrays on the stack.
     *    (ESP32 default stack for app_main is 3.5 KB — would overflow.)
     * ------------------------------------------------------------------ */
    static float    hist_temps[MAX_RECORDS];
    static uint32_t hist_counters[MAX_RECORDS];
    int hist_len = load_history(hist_temps, hist_counters, MAX_RECORDS);

    bool pipe_is_hot = false;

    if (!isnan(temperature)) {
        if (hist_len < MIN_HISTORY_RECORDS) {
            ESP_LOGI(TAG, "Building baseline: %d / %d records",
                     hist_len, MIN_HISTORY_RECORDS);
        } else {
            pipe_is_hot = evaluate_pipe_hot(temperature, hist_temps, hist_len);
        }
    }

    ESP_LOGI(TAG, "Result: pipe is %s", pipe_is_hot ? "HOT ✓" : "cold ✗");

    /* ------------------------------------------------------------------
     * 9. Signal user
     *
     *    Hot pipe sequence:
     *      LED on → 5 beeps → LED stays on 3 s → LED off → sleep
     *
     *    Note on LED during deep sleep: GPIO state is NOT held during
     *    deep sleep on ESP32 — the LED will go dark when we sleep.
     *    The buzzer sequence is the primary user alert.
     *    If you need LED to stay on across sleep cycles, you'd need
     *    an external latch circuit — overkill for this project.
     * ------------------------------------------------------------------ */
    if (pipe_is_hot) {
        led_on();
        buzzer_beep(BUZZER_BEEP_COUNT, BUZZER_ON_MS, BUZZER_OFF_MS);
        vTaskDelay(pdMS_TO_TICKS(LED_VISIBLE_MS));
        led_off();
    }

    /* ------------------------------------------------------------------
     * 10. Unmount SPIFFS — flushes write buffers before power-down
     * ------------------------------------------------------------------ */
    esp_vfs_spiffs_unregister(NULL);

    /* ------------------------------------------------------------------
     * 11. Deep sleep for 5 minutes
     *     No external wiring needed — ESP32 RTC timer wakes chip internally
     * ------------------------------------------------------------------ */
    go_to_deep_sleep();
}

/* =========================================================================
 * WiFi
 * ========================================================================= */

static void disable_wifi(void)
{
    /* These may return errors if WiFi was never initialised — that's fine */
    esp_wifi_stop();
    esp_wifi_deinit();
    ESP_LOGI(TAG, "WiFi disabled");
}

/* =========================================================================
 * SPIFFS
 * ========================================================================= */

static esp_err_t init_spiffs(void)
{
    esp_vfs_spiffs_conf_t cfg = {
        .base_path              = "/spiffs",
        .partition_label        = NULL,      /* uses first spiffs partition   */
        .max_files              = 5,
        .format_if_mount_failed = true,      /* auto-format blank flash       */
    };
    esp_err_t ret = esp_vfs_spiffs_register(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS: %s", esp_err_to_name(ret));
        return ret;
    }
    size_t total = 0, used = 0;
    esp_spiffs_info(NULL, &total, &used);
    ESP_LOGI(TAG, "SPIFFS: %d / %d bytes", used, total);
    return ESP_OK;
}

/* =========================================================================
 * Persistent counter
 *
 * File: /spiffs/counter.txt
 * Format: single ASCII decimal number followed by newline.
 * Stored as text so it can be read/edited via serial terminal if needed.
 * ========================================================================= */

static uint32_t read_counter(void)
{
    FILE *f = fopen("/spiffs/counter.txt", "r");
    if (!f) return 0;   /* doesn't exist = first boot */
    uint32_t v = 0;
    fscanf(f, "%"SCNu32, &v);
    fclose(f);
    return v;
}

static void write_counter(uint32_t v)
{
    FILE *f = fopen("/spiffs/counter.txt", "w");
    if (!f) { ESP_LOGE(TAG, "Cannot write counter.txt"); return; }
    fprintf(f, "%"PRIu32"\n", v);
    fclose(f);
}

/* =========================================================================
 * Circular temperature log
 *
 * File: /spiffs/log.csv
 * Format: one record per line — "counter,temperature\n"
 *   Example: 1042,47.0
 *
 * The file is kept at most MAX_RECORDS lines (one week at 5-min interval).
 * When full, the oldest record is dropped and the file is rewritten.
 *
 * Memory note: 2016 lines × 14 bytes ≈ 28 KB on SPIFFS; arrays in RAM
 * ≈ 2016 × (4+4) bytes = 16 KB. ESP32 has 320 KB RAM, so no problem.
 * ========================================================================= */

/**
 * @brief Load log file into caller-supplied arrays.
 * @return Number of records loaded (0 if file doesn't exist yet).
 */
static int load_history(float *temps, uint32_t *counters, int max)
{
    FILE *f = fopen("/spiffs/log.csv", "r");
    if (!f) return 0;

    int  n = 0;
    char line[32];
    while (fgets(line, sizeof(line), f) && n < max) {
        uint32_t cnt;
        float    t;
        if (sscanf(line, "%"SCNu32",%f", &cnt, &t) == 2) {
            counters[n] = cnt;
            temps[n]    = t;
            n++;
        }
    }
    fclose(f);
    return n;
}

/**
 * @brief Append one record; evict oldest if buffer is full; rewrite file.
 */
static void append_record(uint32_t counter, float temp)
{
    static float    temps[MAX_RECORDS];
    static uint32_t counters[MAX_RECORDS];
    int n = load_history(temps, counters, MAX_RECORDS);

    /* Circular buffer: discard oldest when full */
    if (n >= MAX_RECORDS) {
        /* Shift left — index 0 (oldest) is overwritten */
        memmove(temps,    temps    + 1, (MAX_RECORDS - 1) * sizeof(float));
        memmove(counters, counters + 1, (MAX_RECORDS - 1) * sizeof(uint32_t));
        n = MAX_RECORDS - 1;
    }

    counters[n] = counter;
    temps[n]    = temp;
    n++;

    /* Rewrite entire file */
    FILE *f = fopen("/spiffs/log.csv", "w");
    if (!f) { ESP_LOGE(TAG, "Cannot write log.csv"); return; }
    for (int i = 0; i < n; i++) {
        fprintf(f, "%"PRIu32",%.1f\n", counters[i], temps[i]);
    }
    fclose(f);
    ESP_LOGI(TAG, "Log: %d records | oldest=%"PRIu32" newest=%"PRIu32,
             n, counters[0], counters[n - 1]);
}

/* =========================================================================
 * CORE ALGORITHM — evaluate_pipe_hot()
 * =========================================================================
 *
 * PROBLEM: "Hot" is relative to the building. In one house hot water is
 * 45°C; in another it's 65°C. We cannot use a fixed threshold — it would
 * either miss genuine delivery windows or fire false positives.
 *
 * SOLUTION — adaptive percentile threshold:
 *
 *   1. Sort all historical temperature readings.
 *   2. Find the HOT_PERCENTILE-th percentile (default: 75th).
 *      → The temperature exceeded by only the top 25% of past readings.
 *   3. Declare pipe "hot" if:
 *        current_temp ≥ threshold − HYSTERESIS_C
 *        AND current_temp ≥ ABSOLUTE_MIN_HOT_C
 *
 * HOW IT SELF-CALIBRATES:
 *   In a building where hot water runs only 4 hours/day (common in CIS
 *   housing where water supply follows a schedule), the temperature
 *   distribution is bimodal: ~20°C most of the time, ~55°C during delivery.
 *   The 75th percentile naturally falls inside the hot cluster, so the
 *   algorithm triggers exactly during delivery windows.
 *
 *   In a building with 24/7 hot water the 75th percentile sits near the
 *   top of the normal operating range — still useful as a "peak heat"
 *   indicator (e.g. freshly pumped vs re-circulated water).
 *
 * HYSTERESIS prevents flickering when temperature oscillates near threshold.
 *
 * ABSOLUTE FLOOR prevents declaring a 22°C pipe "hot" just because the
 * 75th percentile happens to be 21°C (building had no hot water all week).
 *
 * STARTUP GUARD: algorithm is silent for the first MIN_HISTORY_RECORDS
 * readings to avoid false positives when we have too little data.
 * ========================================================================= */

/* qsort comparator — ascending float */
static int cmp_float(const void *a, const void *b)
{
    float fa = *(const float *)a;
    float fb = *(const float *)b;
    return (fa > fb) - (fa < fb);
}

static bool evaluate_pipe_hot(float current, float *history, int len)
{
    /* Hard minimum — skip percentile math if clearly cold */
    if (current < ABSOLUTE_MIN_HOT_C) {
        ESP_LOGI(TAG, "Below floor: %.1f < %.1f°C", current, ABSOLUTE_MIN_HOT_C);
        return false;
    }

    /* Copy history into a temporary array and sort ascending */
    int    n      = (len < MAX_RECORDS) ? len : MAX_RECORDS;
    float *sorted = (float *)malloc(n * sizeof(float));
    if (!sorted) {
        ESP_LOGE(TAG, "malloc failed — skipping evaluation");
        return false;
    }
    memcpy(sorted, history, n * sizeof(float));
    qsort(sorted, n, sizeof(float), cmp_float);

    /* Linear-interpolated percentile
     *
     * Index of the P-th percentile in a sorted array of length n:
     *   idx = (P / 100) * (n - 1)
     *
     * Example: P=75, n=100 → idx=74.25
     *   threshold = sorted[74] + 0.25 * (sorted[75] - sorted[74])
     */
    float pidx     = (HOT_PERCENTILE / 100.0f) * (float)(n - 1);
    int   lo       = (int)pidx;
    float frac     = pidx - (float)lo;
    float threshold;

    if (lo + 1 < n) {
        threshold = sorted[lo] + frac * (sorted[lo + 1] - sorted[lo]);
    } else {
        threshold = sorted[lo];
    }
    free(sorted);

    float effective_threshold = threshold - HYSTERESIS_C;
    ESP_LOGI(TAG, "History %d pts | P%.0f = %.1f°C | hysteresis floor = %.1f°C | now = %.1f°C",
             n, HOT_PERCENTILE, threshold, effective_threshold, current);

    return (current >= effective_threshold);
}

/* =========================================================================
 * GPIO output helpers
 * ========================================================================= */

static void led_on(void)  { gpio_set_level(PIN_LED, 1); }
static void led_off(void) { gpio_set_level(PIN_LED, 0); }

/**
 * @brief Drive active buzzer for `times` beeps.
 * @param times   Number of beeps
 * @param on_ms   Buzzer ON duration per beep (ms)
 * @param off_ms  Silence between beeps (ms)
 */
static void buzzer_beep(int times, int on_ms, int off_ms)
{
    for (int i = 0; i < times; i++) {
        gpio_set_level(PIN_BUZZER, 1);
        vTaskDelay(pdMS_TO_TICKS(on_ms));
        gpio_set_level(PIN_BUZZER, 0);
        if (i < times - 1) {
            vTaskDelay(pdMS_TO_TICKS(off_ms));
        }
    }
}

/**
 * @brief Enter deep sleep for SLEEP_DURATION_US microseconds.
 *
 * ESP32 advantage over ESP8266: the RTC timer wakes the chip internally.
 * No GPIO16→RST wire needed. After wake, the chip reboots and app_main()
 * runs again from the top.
 */
static void go_to_deep_sleep(void)
{
    ESP_LOGI(TAG, "Entering deep sleep for 5 minutes...");
    vTaskDelay(pdMS_TO_TICKS(100));     /* let UART flush log output */

    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(SLEEP_DURATION_US));
    esp_deep_sleep_start();
    /* Never reached */
}

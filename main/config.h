#pragma once

/**
 * @file config.h
 * @brief All hardware pins and algorithm tuning constants in one place.
 *        Change values here — no need to touch any other source file.
 */

/* -------------------------------------------------------------------------
 * GPIO pin assignments
 * -------------------------------------------------------------------------
 *  PIN_DHT     - DHT11 data line. Use a 4.7kΩ pull-up to 3.3V.
 *  PIN_LED     - Red LED (anode via 330Ω to GPIO, cathode to GND).
 *  PIN_BUZZER  - Active or passive buzzer (active: simpler wiring).
 *
 *  CRITICAL: GPIO16 (D0 on NodeMCU) MUST be wired to RST for deep sleep
 *  wake-up. This is a hardware requirement of the ESP8266 RTC timer.
 * ------------------------------------------------------------------------- */
#define PIN_DHT      4    /* GPIO4 = D2 on NodeMCU                         */
#define PIN_LED      12   /* GPIO12 = D6 on NodeMCU                        */
#define PIN_BUZZER   14   /* GPIO14 = D5 on NodeMCU                        */

/* -------------------------------------------------------------------------
 * DHT11 timing
 * -------------------------------------------------------------------------
 *  After deep-sleep wake the ESP8266 cold-boots. DHT11 needs at least
 *  1000 ms to stabilise its internal measurement before the first read.
 *  1200 ms gives comfortable margin.
 * ------------------------------------------------------------------------- */
#define DHT_STABILISE_MS   1200

/* -------------------------------------------------------------------------
 * Data log / history settings
 * -------------------------------------------------------------------------
 *  Samples every 5 minutes:
 *    5 min * 12/hour * 24 h * 7 days = 2016 records per week
 *  We keep exactly one week of data in the circular buffer.
 *
 *  MIN_HISTORY_RECORDS: algorithm won't trigger until we have this many
 *  readings (avoids false positives on first day of deployment).
 *  48 records = 4 hours — enough to see at least one full hot/cold cycle.
 * ------------------------------------------------------------------------- */
#define MAX_RECORDS          2016   /* 7 days × 288 samples/day            */
#define MIN_HISTORY_RECORDS    48   /* Must collect 4 h of data first       */

/* -------------------------------------------------------------------------
 * Algorithm thresholds
 * -------------------------------------------------------------------------
 *  HOT_PERCENTILE:
 *    The current temperature must exceed the Pth percentile of history
 *    to be declared "hot". 75 means "hotter than 75% of past readings".
 *    Raise to 80-90 for stricter detection; lower to 60-70 for looser.
 *
 *  ABSOLUTE_MIN_HOT_C:
 *    Hard floor. Even if the 75th percentile happens to be 25°C (because
 *    the building had no hot water for a long time), we won't declare the
 *    pipe "hot" unless it's at least this warm. Typical hot water supply
 *    in CIS apartment buildings should be ≥ 40°C by sanitary norms.
 *
 *  HYSTERESIS_C:
 *    Small tolerance below the percentile threshold to avoid flickering
 *    output when temperature is right on the boundary. E.g. if threshold
 *    is 50°C and HYSTERESIS_C is 1.0, we trigger at 49°C too.
 * ------------------------------------------------------------------------- */
#define HOT_PERCENTILE       75.0f  /* percentile (0–100)                  */
#define ABSOLUTE_MIN_HOT_C   40.0f  /* degrees Celsius                     */
#define HYSTERESIS_C          1.0f  /* degrees Celsius                     */

/* -------------------------------------------------------------------------
 * Buzzer / LED signal parameters
 * -------------------------------------------------------------------------
 *  BUZZER_BEEP_COUNT : number of beeps when pipe becomes "hot"
 *  BUZZER_ON_MS      : duration of each beep in milliseconds
 *  BUZZER_OFF_MS     : silence between beeps in milliseconds
 *  LED_VISIBLE_MS    : how long LED stays on after buzzer sequence (ms)
 *                      (so user has time to notice it before deep sleep)
 * ------------------------------------------------------------------------- */
#define BUZZER_BEEP_COUNT   5
#define BUZZER_ON_MS        120
#define BUZZER_OFF_MS       180
#define LED_VISIBLE_MS      2000

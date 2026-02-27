# Pipe Temperature Monitor — ESP8266 / ESP-IDF

Monitors a hot-water supply pipe, stores 7 days of temperature history,
and beeps + lights an LED when the pipe is carrying genuinely hot water
(determined by adaptive percentile analysis of historical readings).

---

## Hardware

### Bill of Materials
| Part | Notes |
|------|-------|
| ESP8266 module (NodeMCU v2/v3 or bare ESP-12) | Any variant works |
| DHT11 sensor | 3-pin or 4-pin package |
| 4.7 kΩ resistor | DHT11 data line pull-up |
| 330 Ω resistor | LED current limiter |
| Red LED (any 3 mm/5 mm) | |
| Active buzzer 3–5 V | "Active" = buzzes at DC; simpler than passive |
| 9V "Krona" battery | |
| AMS1117-3.3 LDO regulator | Or any 3.3V LDO, 300 mA capable |
| 10 µF + 100 nF capacitors | LDO bypass |

### Wiring

```
                  3.3V
                   │
               [4.7kΩ]
                   │
DHT11 DATA ────────┤──── GPIO4  (D2 on NodeMCU)
DHT11 VCC  ────────── 3.3V
DHT11 GND  ────────── GND

GPIO12 (D6) ──[330Ω]──>|── GND     (LED, anode to GPIO)

GPIO14 (D5) ──────────[Buzzer+]    (active buzzer)
                       [Buzzer-] ── GND

!!! CRITICAL !!!
GPIO16 (D0) ──────────── RST       (REQUIRED for deep sleep wake-up)
Without this wire the chip sleeps forever!

9V battery → LDO IN
LDO OUT → 3.3V rail → ESP8266 VCC, DHT11 VCC
LDO GND → GND rail
```

### Expected battery life (Krona 500 mAh)
- Deep sleep current:   ~20 µA
- Active time per wake: ~3 s @ ~80 mA
- Average current:      ≈ 0.02 + (80 × 3 / 300) ≈ 0.82 mA
- Estimated life:       500 / 0.82 ≈ **600 h ≈ 25 days**

> Note: LDO quiescent current (~5 mA for AMS1117!) dominates in practice.
> Use an MCP1700 or XC6206 LDO (2–4 µA Iq) for much longer battery life.
> With a 4 µA LDO: avg ≈ 0.024 mA → ~20,000 h → impractical limit.

---

## Software

### Prerequisites
- ESP8266 RTOS SDK (ESP-IDF style) installed
- Python 3, idf.py in PATH

### Build & Flash
```bash
cd pipe_monitor
idf.py set-target esp8266
idf.py -D SDKCONFIG_DEFAULTS=sdkconfig.defaults menuconfig
# Verify: Component config → SPIFFS → partition size = 65536
# Verify: CPU freq = 80 MHz
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

### File Structure
```
pipe_monitor/
├── CMakeLists.txt
├── partitions.csv
├── sdkconfig.defaults
├── main/
│   ├── CMakeLists.txt
│   ├── config.h          ← pins & algorithm constants
│   └── main.c            ← main logic
└── components/
    └── dht/
        ├── CMakeLists.txt
        ├── dht.h
        └── dht.c          ← DHT11 bit-bang driver
```

---

## Algorithm

The key question is: "Is the pipe **currently** hotter than usual?"

### Adaptive percentile threshold

1. Every 5 minutes, temperature is appended to a circular log (7 days max).
2. On each wake, sort all historical readings and compute the **75th percentile**.
3. The pipe is declared **hot** if:
   - `current_temp ≥ P75(history) − 1°C` (hysteresis)
   - **AND** `current_temp ≥ 40°C` (sanitary minimum floor)
4. If there are fewer than 48 historical readings (< 4 hours), the algorithm
   waits silently to avoid false positives.

### Why percentile and not a fixed threshold?
Different buildings, different water temperatures. In one building "hot"
is 45°C, in another it's 65°C. The percentile adapts automatically:
if hot water flows 20% of the day, the 75th percentile sits right at the
boundary between cold and hot — we catch every genuine delivery window
regardless of the building's absolute temperature range.

### Tuning (config.h)
| Constant | Default | Effect |
|---|---|---|
| `HOT_PERCENTILE` | 75.0 | Raise → stricter (only very hot triggers) |
| `ABSOLUTE_MIN_HOT_C` | 40.0 | Raise if building always has ≥50°C water |
| `HYSTERESIS_C` | 1.0 | Raise to reduce flicker near threshold |
| `MIN_HISTORY_RECORDS` | 48 | Minimum baseline before algorithm activates |

---

## Data Recovery

SPIFFS log format (`/spiffs/log.csv`):
```
<wake_counter>,<temperature>\n
1001,23.0
1002,24.0
...
2048,58.0
```

To decode timestamps: `time = deploy_epoch + counter × 300 seconds`

The counter file (`/spiffs/counter.txt`) persists across deep sleep reboots
and lets you reconstruct the time axis even after partial data loss.

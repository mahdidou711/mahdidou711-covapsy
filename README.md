# CoVAPSy 2026

Autonomous 1/10-scale race car for the ENS Paris-Saclay CoVAPSy 2026 competition.
Runtime: reactive Python control loop at 50 Hz on Raspberry Pi 4.

---

## Hardware

| Component | Model | Interface |
| --- | --- | --- |
| SBC | Raspberry Pi 4 | — |
| Lidar | RPLidar A2M12 | UART `/dev/ttyUSB0` @ 256000 baud |
| ESC | Hobby ESC | Hardware PWM channel 0 (GPIO 12) |
| Servo | Hobby servo | Hardware PWM channel 1 (GPIO 13) |
| Rear sonar | SRF10 | I2C bus 1 (GPIO 2/3), address 0x70 |

All PWM signals run at 50 Hz. Duty cycle range: 5.0 – 10.0 %.

---

## Repository structure

```text
main.py                   Control loop entry point (50 Hz)
navigation.py             Pure direction and speed computation
actuators.py              ESC + servo PWM driver, blocking reverse maneuver
config.py                 All runtime parameters and validation
lidar_thread.py           Background thread — continuous 360° scan acquisition
lidar_consumer.py         Non-blocking scan polling with freshness tracking
sonar.py                  SRF10 rear sonar thread + thread-safe accessor
steering.py               Logical angle → servo duty cycle conversion
test_direction_simple.py  Offline unit tests for direction law
requirements.txt          Python dependencies
```

Generated at runtime (not tracked):

- `run_log_<timestamp>.csv` — per-tick telemetry written by `main.py`

---

## Dependencies

```text
rplidar-roboticia    # RPLidar A2M12 driver
rpi_hardware_pwm     # Raspberry Pi hardware PWM
smbus2               # I2C (SRF10 sonar)
numpy                # Used in escape angle selection
```

Install:

```bash
pip install -r requirements.txt
```

---

## Running

```bash
sudo python3 main.py
```

`sudo` is required for hardware PWM access. Stop with `Ctrl+C` or `SIGTERM`; the shutdown handler zeroes ESC and servo before exit.

---

## Architecture

### Control loop (`main.py`)

Runs at `CONTROL_HZ = 50 Hz` (`DT_S = 0.020 s` per tick).

```text
┌──────────────┐     get_latest_scan()     ┌──────────────────┐
│  Lidar360    │ ─────────────────────────► │  LidarConsumer   │
│  (thread)    │                            │  poll()          │
└──────────────┘                            └────────┬─────────┘
                                                     │ scan[360]
                                          ┌──────────▼──────────┐
                                          │      main.py         │
                                          │  watchdog / stuck    │
                                          │  calculer_direction  │
                                          │  calculer_vitesse    │
                                          └──────┬───────┬───────┘
                                                 │       │
                                    set_direction│       │set_vitesse
                                          ┌──────▼───────▼──────┐
                                          │     Actuators        │
                                          │  ESC  |  Servo PWM   │
                                          └─────────────────────┘
```

Optional `sonar.py` thread runs in parallel; `actuators.reculer()` reads it during reverse.

### Lidar angle convention

| Index | Physical direction |
| --- | --- |
| `scan[0]` | Forward |
| `scan[90]` | Left |
| `scan[180]` | Rear |
| `scan[270]` | Right |
| `scan[i] == 0` | No valid measurement |

The driver applies `idx = (-int(round(angle))) % 360` to remap from RPLidar native angles.

---

## Navigation

### Direction law

Sectors: left `[30°–60°]`, right `[300°–330°]` (relative to forward axis).

```text
angle = NAV_K * (G - D) / (G + D + NAV_EPS)
```

Clamped to `[-STEER_ANGLE_MAX_DEG, +STEER_ANGLE_MAX_DEG]` = `[-18°, +18°]`.
Zero measurements are excluded from sector means.
If one sector is entirely invalid, the valid side is mirrored.
If both sectors are invalid, output is `0.0`.

### Speed law

Lateral sectors: left `[35°–55°]`, right `[305°–325°]`.

```text
r       = |left - right| / (max(left, right) + SPEED_EPS)
v_lat   = VITESSE_PLANCHER + (VITESSE_CROISIERE - VITESSE_PLANCHER) * exp(-SPEED_ALPHA * r)
f_front = min(1.0, front_min / FRONT_D_REF_MM)
v       = clamp(v_lat * f_front, VITESSE_PLANCHER * f_front, VITESSE_CROISIERE)
```

Current values: `NAV_K=30.0`, `SPEED_ALPHA=3.0`, `VITESSE_CROISIERE=0.30 m/s`, `VITESSE_PLANCHER=0.25 m/s`, `FRONT_D_REF_MM=600 mm`.

---

## Lidar watchdog

- A scan is **fresh** if it has a new `scan_id` and age ≤ `LIDAR_FRESH_MAX_S` (0.50 s).
- `ticks_sans_scan` increments each tick without a fresh scan.
- At `ticks_sans_scan >= LIDAR_TIMEOUT_TICKS` (15 ticks = 0.3 s): ESC and servo are forced to 0.
- If `LIDAR_REUSE_LAST_SCAN = True`, the last valid scan is reused until the timeout.

---

## Stuck recovery

Controlled by `STUCK_RECOVERY_ACTIVE = True`.

```text
if front_min < STUCK_DIST_MM (500 mm):
    stuck_count += 1
else:
    stuck_count = max(0, stuck_count - 1)

if stuck_count >= STUCK_TICKS (6):
    reculer(scan=last_scan)          # blocking
    cooldown = STUCK_COOLDOWN_TICKS  # 50 ticks immunity
```

### Escape angle selection

```text
left_space  = mean(scan[20:80])
right_space = mean(scan[280:340])
delta = right_space - left_space

if |delta| >= ESCAPE_SYMMETRY_MM (100):
    angle = -sign(delta) * 18°      # pivot toward larger gap
else:
    angle = 0°                       # straight reverse

if gap in pivot direction < ESCAPE_MIN_PIVOT_MM (200 mm):
    angle = 0°                       # cancel pivot
```

### ESC double-tap sequence

```text
REV_START (7.20%) → NEUTRAL → REV_START → NEUTRAL → REV_STABLE (7.00%) → NEUTRAL
```

Each tap: `REVERSE_TAP_S = 0.20 s`. Stable phase: up to `REVERSE_DUREE_S = 1.5 s`.
Sonar stops the reverse early if rear obstacle detected within `SONAR_ARRIERE_SEUIL_CM = 25 cm`.

---

## Servo/ESC calibration

| Parameter | Value | Meaning |
| --- | --- | --- |
| `SERVO_DUTY_CENTER` | 8.00 % | Wheels straight |
| `SERVO_DUTY_MIN` | 6.40 % | Max left lock |
| `SERVO_DUTY_MAX` | 9.60 % | Max right lock |
| `STEER_ANGLE_MAX_DEG` | 18.0° | Physical steering limit |
| `ESC_DUTY_NEUTRAL` | 7.78 % | Motor off (critical — measure per vehicle) |
| `ESC_DUTY_REV_START` | 7.20 % | Reverse priming pulse |
| `ESC_DUTY_REV_STABLE` | 7.00 % | Sustained reverse |
| `ESC_FWD_DEADBAND` | 0.10 % | Minimum threshold above neutral |
| `ESC_SPEED_SCALE_MS` | 0.80 m/s | Velocity at max PWM range |

**`ESC_DUTY_NEUTRAL` must be measured and set per physical vehicle before first run.**

---

## Safety properties

- `config.py` raises `ValueError` at import if any parameter is out of its valid range.
- `main.py` forces propulsion and steering to zero if the lidar scan expires.
- `actuators.stop()` always runs in the `finally` block; each step is wrapped in `try/except`.
- `steering.angle_deg_to_duty()` and `_speed_ms_to_duty()` clamp all outputs to hardware-safe ranges.
- The lidar thread reconnects automatically on `RPLidarException` with 0.5 s backoff.

---

## Telemetry

`run_log_<timestamp>.csv` is written to the working directory each tick with immediate `flush()`.

Columns: `timestamp`, `g_mm`, `d_mm`, `fresh`, `ticks_sans_scan`, `front_mm`, `angle_deg`, `vitesse_ms`.
Missing values are filled with `-1`.

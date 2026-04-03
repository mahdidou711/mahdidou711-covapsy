# CoVAPSy 2026

![Python](https://img.shields.io/badge/language-Python%203-blue)
![License](https://img.shields.io/badge/license-MIT-green)

**Autonomous 1/10-scale RC car for the ENS Paris-Saclay CoVAPSy 2026 competition.**

The runtime is modular, runs on a Raspberry Pi 4, and uses a RPLidar A2M12 for reactive navigation at 50 Hz. The active entry point is `main.py`.

---

## Table of contents

1. [Hardware](#hardware)
2. [State of the project](#state-of-the-project)
3. [Architecture](#architecture)
4. [File roles](#file-roles)
5. [Driving logic](#driving-logic)
6. [Key parameters](#key-parameters)
7. [Execution flow](#execution-flow)
8. [Dependencies and prerequisites](#dependencies-and-prerequisites)
9. [Usage](#usage)
10. [Safety and precautions](#safety-and-precautions)
11. [Known limits](#known-limits)
12. [Next steps](#next-steps)
13. [Project tree](#project-tree)
14. [License](#license)

---

## Hardware

| Component | Model | Role |
|---|---|---|
| Single-board computer | Raspberry Pi 4 | Main compute, runs all threads |
| Lidar | RPLidar A2M12 | 360-degree distance sensing, 256000 baud, `/dev/ttyUSB0` |
| ESC | Hobby-grade brushless ESC | Forward and reverse via hardware PWM on GPIO 12 (channel 0) |
| Servo | Standard RC servo | Steering via hardware PWM on GPIO 13 (channel 1) |
| Rear sonar | SRF10 | Obstacle detection during reverse, I2C bus 1, address `0x70` |

### Calibrated PWM values (`config.py`)

| Signal | Parameter | Value |
|---|---|---|
| ESC neutral | `ESC_DUTY_NEUTRAL` | 7.78 % duty |
| ESC forward start | `ESC_DUTY_FWD_START` | 7.88 % duty (neutral + `ESC_FWD_DEADBAND`) |
| ESC reverse engage | `ESC_DUTY_REV_START` | 7.20 % duty |
| ESC reverse stable | `ESC_DUTY_REV_STABLE` | 7.00 % duty |
| Servo center | `SERVO_DUTY_CENTER` | 8.00 % duty |
| Servo left stop | `SERVO_DUTY_MIN` | 6.40 % duty |
| Servo right stop | `SERVO_DUTY_MAX` | 9.60 % duty |

Hardware PWM must be enabled in `/boot/config.txt`:

```text
dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
```

---

## State of the project

The project is **ready under conditions, not final**.

- `main.py` is the real entry point.
- `navigation.py` is the active navigation module.
- FTG is no longer used in the runtime.
- Reverse is intentionally blocking.
- On-car physical validation is still required before the final cleanup.

---

## Architecture

```text
main.py
  |-- LidarConsumer (lidar_consumer.py)
  |     |-- Lidar360 (lidar_thread.py)          # acquisition thread, reconnect loop
  |-- Actuators (actuators.py)
  |     |-- HardwarePWM x2 (rpi_hardware_pwm)   # ESC (ch 0) + servo (ch 1)
  |     |-- steering.py                          # angle_deg -> duty cycle
  |     |-- sonar.py (optional)                 # rear SRF10 sonar during reverse
  |-- navigation.py                              # pure computation: direction + speed
  |-- config.py                                  # single source of truth for parameters
  |-- sonar.py (optional thread)                # rear sonar continuous measurement
```

`navigation.py` is a pure computation module: no hardware access, no side effects. It takes a `scan` array and config parameters, and returns floats.

---

## File roles

| File | Role |
|---|---|
| `main.py` | Entry point. 50 Hz control loop, lidar watchdog, stuck detection, CSV logging. |
| `navigation.py` | Normalized steering law and exponential speed law. Pure computation. |
| `config.py` | All tuning parameters. Single source of truth for the runtime. |
| `actuators.py` | ESC and servo commands. Blocking reverse with ESC double-tap and sonar safety. |
| `steering.py` | Asymmetric angle-to-duty conversion using calibrated servo endpoints. |
| `lidar_thread.py` | RPLidar acquisition thread. Mirror transform to runtime convention. Reconnect loop. |
| `lidar_consumer.py` | Non-blocking scan consumer. Freshness filtering. Scan rate estimation. |
| `sonar.py` | SRF10 rear sonar thread and thread-safe read function. |

---

## Driving logic

### Lidar convention

The `Lidar360` thread applies a mirror transform at publication:

```python
idx = (-int(round(angle))) % 360
```

Runtime convention throughout all modules:
- `scan[0]` = front
- `scan[90]` = left
- `scan[270]` = right
- `scan[i] == 0` means no valid measurement (not free space).

### Direction (`navigation.py` — `calculer_direction`)

Sectors used: `DIR_LEFT_SECTOR = (30, 60)`, `DIR_RIGHT_SECTOR = (300, 330)`.

Each sector mean ignores zero values. If both sectors are invalid, the function returns `0.0`. If one sector is invalid, the other side is copied (neutralization).

Steering formula:

```text
angle = NAV_K * (G_moy - D_moy) / (G_moy + D_moy + NAV_EPS)
```

Result is clamped to `[-STEER_ANGLE_MAX_DEG, +STEER_ANGLE_MAX_DEG]` (±18°).

The normalization by `(G_moy + D_moy + NAV_EPS)` makes the steering response independent of the absolute wall distance.

### Speed (`navigation.py` — `calculer_vitesse`)

Sectors used: `SPEED_LEFT_SECTOR = (35, 55)`, `SPEED_RIGHT_SECTOR = (305, 325)`.

Lateral contrast ratio:

```text
r = |s_g - s_d| / (max(s_g, s_d) + SPEED_EPS)
```

Exponential speed law:

```text
v_lat = VITESSE_PLANCHER + (VITESSE_CROISIERE - VITESSE_PLANCHER) * exp(-SPEED_ALPHA * r)
```

- Straight line (r near 0): `v_lat` approaches `VITESSE_CROISIERE` (0.30 m/s).
- Tight turn (r near 1): `v_lat` approaches `VITESSE_PLANCHER` (0.25 m/s).

If both speed sectors are invalid, `v_lat = VITESSE_PLANCHER`.

### Frontal term (`navigation.py` — `calculer_vitesse`)

`main.py` computes `front_min` (minimum valid distance in the frontal sector, half-angle `FRONT_SECTOR_HALF = 10°`) and passes it to `calculer_vitesse`.

```text
f_front = min(1.0, front_min / FRONT_D_REF_MM)    if front_min is not None and > 0
f_front = 1.0                                       otherwise

v = v_lat * f_front
v_min = VITESSE_PLANCHER * f_front    if f_front > 0 else 0.0
v = clamp(v, v_min, VITESSE_CROISIERE)
```

`FRONT_D_REF_MM = 600 mm`. Below 600 mm in front, speed is progressively reduced.

### Lidar watchdog (`main.py`)

Each tick, `ticks_sans_scan` is incremented when no fresh scan is available. A scan is considered fresh if it is new (`scan_id` changed) and its age does not exceed `LIDAR_FRESH_MAX_S = 0.50 s`.

If `scan is None` or `ticks_sans_scan >= LIDAR_TIMEOUT_TICKS` (15 ticks = 0.30 s at 50 Hz):
- propulsion is set to 0.
- steering is set to 0.
- A single warning is printed.

When `LIDAR_REUSE_LAST_SCAN = True`, the last valid scan is reused when no fresh scan is available, until the timeout fires.

### Stuck detection and reverse (`main.py` + `actuators.py`)

Controlled by `STUCK_RECOVERY_ACTIVE = True`.

Detection counter logic (per tick):
- If `cooldown > 0`: decrement cooldown, reset `stuck_count`.
- Else if `front_min < STUCK_DIST_MM` (500 mm): increment `stuck_count`.
- Else: `stuck_count = max(0, stuck_count - 1)`.

Trigger: reverse starts when `stuck_count >= STUCK_TICKS` (threshold 6 ticks ≈ 0.12 s at 50 Hz).

On trigger:
1. `set_vitesse(0.0)` — stop forward motion.
2. `act.reculer(scan=last_scan)` — blocking reverse.
3. `cooldown = STUCK_COOLDOWN_TICKS` (50 ticks = 1.0 s immunity).
4. `stuck_count = 0`.

### Blocking reverse (`actuators.py` — `reculer`)

**Escape angle selection:**

Lateral spaces are computed from `ESCAPE_LEFT_SECTOR = (20, 80)` and `ESCAPE_RIGHT_SECTOR = (280, 340)`, ignoring zero measurements.

```text
delta = right_space - left_space
if abs(delta) >= ESCAPE_SYMMETRY_MM:
    angle_deg = -sign(delta) * STEER_ANGLE_MAX_DEG
else:
    angle_deg = 0.0
```

A positive delta (more space on right) results in a negative angle (steer left), pivoting the front right during reverse. Small left/right differences below `ESCAPE_SYMMETRY_MM` keep the reverse straight.

**Pivot guard:**

Before sending any PWM:
- `angle_deg < 0` (pivot left): if `left_space < ESCAPE_MIN_PIVOT_MM` (200 mm) → `angle_deg = 0.0`.
- `angle_deg > 0` (pivot right): if `right_space < ESCAPE_MIN_PIVOT_MM` (200 mm) → `angle_deg = 0.0`.

If `scan` is `None`, `left_space = right_space = 0.0`; with no lateral clearance information, the guard can force a straight reverse.

**ESC double-tap sequence:**

```text
REV_START  →  sleep(REVERSE_TAP_S)   # 0.20 s
NEUTRAL    →  sleep(REVERSE_TAP_S)   # 0.20 s
REV_START  →  sleep(REVERSE_TAP_S)   # 0.20 s
NEUTRAL    →  sleep(REVERSE_TAP_S)   # 0.20 s
REV_STABLE →  loop until REVERSE_DUREE_S (1.0 s) or sonar stop
NEUTRAL                               # final neutral
```

`set_direction(angle_deg)` is called after each ESC step to maintain the steering angle through the sequence.

**Sonar safety during stable reverse:**

If `SONAR_ACTIF = True` and the SRF10 reads `1 < dist < SONAR_ARRIERE_SEUIL_CM` (25 cm), the reverse loop exits early.

### Servo conversion (`steering.py`)

Asymmetric interpolation around the calibrated center:

```text
if angle >= 0:  duty = SERVO_DUTY_CENTER + (angle/STEER_ANGLE_MAX_DEG) * (SERVO_DUTY_MAX - SERVO_DUTY_CENTER)
if angle < 0:   duty = SERVO_DUTY_CENTER + (angle/STEER_ANGLE_MAX_DEG) * (SERVO_DUTY_CENTER - SERVO_DUTY_MIN)
```

Sign convention in `actuators.set_direction`: the logical angle is inverted before conversion (`-angle_deg`) to compensate for the mechanical mounting direction.

---

## Key parameters

### Navigation — direction

| Parameter | Value | Description |
|---|---|---|
| `NAV_K` | 12.0 | Steering gain |
| `NAV_EPS` | 1.0 mm | Denominator guard (prevents division by zero) |
| `DIR_LEFT_SECTOR` | (30, 60) | Left lateral sector for steering |
| `DIR_RIGHT_SECTOR` | (300, 330) | Right lateral sector for steering |
| `STEER_ANGLE_MAX_DEG` | 18.0° | Maximum steering angle |

### Navigation — speed

| Parameter | Value | Description |
|---|---|---|
| `VITESSE_CROISIERE` | 0.30 m/s | Cruise speed (straight line) |
| `VITESSE_PLANCHER` | 0.25 m/s | Floor speed (tight turn) |
| `ESC_SPEED_SCALE_MS` | 0.80 m/s | Absolute software speed cap |
| `SPEED_ALPHA` | 2.0 | Exponential attenuation factor |
| `SPEED_EPS` | 1.0 mm | Speed contrast denominator guard |
| `SPEED_LEFT_SECTOR` | (35, 55) | Left lateral sector for speed law |
| `SPEED_RIGHT_SECTOR` | (305, 325) | Right lateral sector for speed law |

### Navigation — frontal term

| Parameter | Value | Description |
|---|---|---|
| `FRONT_SECTOR_HALF` | 10° | Half-angle of the front sector |
| `FRONT_D_REF_MM` | 600 mm | Reference distance for speed reduction |

### Lidar

| Parameter | Value | Description |
|---|---|---|
| `PORT` | `/dev/ttyUSB0` | Serial port |
| `BAUDRATE` | 256000 | Serial baud rate |
| `LIDAR_FRESH_MAX_S` | 0.50 s | Max scan age before considered stale |
| `LIDAR_TIMEOUT_TICKS` | 15 ticks | Ticks without fresh scan before emergency stop |
| `LIDAR_REUSE_LAST_SCAN` | True | Reuse last valid scan when no fresh scan |

### Stuck detection

| Parameter | Value | Description |
|---|---|---|
| `STUCK_DIST_MM` | 500 mm | Front distance threshold for stuck counter increment |
| `STUCK_TICKS` | 6 ticks | Stuck counter threshold before reverse trigger |
| `STUCK_COOLDOWN_TICKS` | 50 ticks | Post-reverse immunity ticks |
| `STUCK_RECOVERY_ACTIVE` | True | Enables stuck detection and reverse |

### Reverse

| Parameter | Value | Description |
|---|---|---|
| `REVERSE_TAP_S` | 0.20 s | Duration of each ESC double-tap step |
| `REVERSE_DUREE_S` | 1.0 s | Maximum stable reverse duration |
| `ESCAPE_LEFT_SECTOR` | (20, 80) | Left sector for escape angle selection |
| `ESCAPE_RIGHT_SECTOR` | (280, 340) | Right sector for escape angle selection |
| `ESCAPE_MIN_PIVOT_MM` | 200 mm | Minimum lateral clearance to allow a pivot |
| `ESCAPE_SYMMETRY_MM` | 100 mm | Minimum left/right difference before choosing a side |

### Sonar

| Parameter | Value | Description |
|---|---|---|
| `SONAR_ACTIF` | True | Enable rear sonar (set to False if hardware absent) |
| `SONAR_ARRIERE_SEUIL_CM` | 25 cm | Rear obstacle distance that stops reverse early |

### Control loop

| Parameter | Value | Description |
|---|---|---|
| `CONTROL_HZ` | 50 Hz | Loop frequency |
| `DT_S` | 0.02 s | Loop period |

---

## Execution flow

```text
main()
  │
  ├─ Lidar360.start()              # acquisition thread starts
  ├─ sonar_thread (optional)       # SRF10 measurement thread starts
  │
  └─ control loop at 50 Hz:
       │
       ├─ consumer.poll()          # read latest scan (freshness-filtered)
       │     fresh? → update last_scan, reset ticks_sans_scan
       │     stale? → increment ticks_sans_scan
       │
       ├─ select scan:
       │     LIDAR_REUSE_LAST_SCAN → use last_scan if no fresh scan
       │
       ├─ _compute_front_min(scan) # min valid distance in ±FRONT_SECTOR_HALF
       ├─ get_lateral_means(scan)  # (g, d) for CSV log
       │
       ├─ if lidar_perime:
       │     set_vitesse(0) + set_direction(0) + warning
       │
       └─ else:
             if STUCK_RECOVERY_ACTIVE:
               update cooldown / stuck_count
               if stuck_count >= STUCK_TICKS:
                 set_vitesse(0)
                 act.reculer(scan=last_scan)   ← blocking, up to ~1.8 s worst case
                 cooldown = STUCK_COOLDOWN_TICKS
             │
             if not recovery_triggered:
               angle = calculer_direction(scan, NAV_K, NAV_EPS)
               vitesse = calculer_vitesse(scan, front_min)
               act.set_direction(angle)
               act.set_vitesse(vitesse)
             │
             _write_log(...)        # CSV flush at each tick
```

On `SIGINT` or `SIGTERM`:
1. `stop_event.set()`
2. `act.stop()` — ESC neutral + servo center + PWM channels stopped.
3. `lidar.stop()` — thread join (timeout 3.0 s).
4. CSV file closed.

---

## Dependencies and prerequisites

### Python packages (`requirements.txt`)

```text
rplidar-roboticia
rpi_hardware_pwm
smbus2
numpy
```

Install:

```bash
pip install -r requirements.txt
```

### System prerequisites

- Raspberry Pi 4 running a Linux distribution with Python 3.
- Hardware PWM enabled on GPIO 12 and 13.
- RPLidar A2M12 connected on `/dev/ttyUSB0`.
- I2C enabled (for SRF10 sonar if `SONAR_ACTIF = True`).
- User in the `dialout` group for serial port access.
- User in the `i2c` group for I2C access.

Enable hardware PWM in `/boot/config.txt`:

```text
dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
```

---

## Usage

### Run the car

```bash
python3 main.py
```

Stop cleanly with `Ctrl+C` or `kill <pid>`. The shutdown handler cuts propulsion and servo before exiting.

### CSV log

`run_log.csv` is generated at runtime at 50 Hz in the working directory. It is an execution artifact, not a file that needs to appear in the repository tree.

Columns:

```text
t, moyenne_gauche, moyenne_droite, angle, vitesse, front_min, fresh, ticks_sans_scan
```

| Column | Description |
|---|---|
| `t` | Monotonic timestamp (s) |
| `moyenne_gauche` | Mean left lateral distance (mm), -1 if invalid |
| `moyenne_droite` | Mean right lateral distance (mm), -1 if invalid |
| `angle` | Commanded steering angle (deg) |
| `vitesse` | Commanded speed (m/s) |
| `front_min` | Minimum front distance (mm), -1 if invalid |
| `fresh` | 1 if the scan used this tick was fresh |
| `ticks_sans_scan` | Consecutive ticks without a fresh scan |

---

## Safety and precautions

- Do not run `main.py` without a working lidar: the watchdog will stop propulsion after `LIDAR_TIMEOUT_TICKS` ticks (0.30 s) if no fresh scan arrives.
- Set `SONAR_ACTIF = False` in `config.py` if the SRF10 is physically absent; the import is conditional but the thread will fail without the hardware.
- The blocking reverse (`actuators.reculer`) cannot be interrupted by `stop_event` once started. `SIGINT` will only take effect after the reverse completes.
- `scan[i] == 0` is not free space; it means the lidar returned no measurement at that angle. All navigation functions ignore zero values explicitly.
- Hardware PWM must be enabled and stable before running. An incorrect duty cycle can damage the ESC or servo.
- Validate the steering sign physically before the first real run: `set_direction(+angle)` must turn the wheels to the right.
- Validate the lidar convention physically: `scan[0]` must correspond to the physical front of the car.

---

## Known limits

- A blocking reverse cannot be interrupted immediately once `act.reculer()` is called. Total blocking time can reach about `4 × REVERSE_TAP_S + REVERSE_DUREE_S`, so up to roughly 1.8 s in the current tuning if sonar does not stop it earlier.
- If `scan` passed to `reculer()` is `None`, both lateral spaces fall back to `0.0` and the pivot guard can force a straight reverse.
- The lidar convention (mirror transform in `lidar_thread.py`) must be validated on the physical car before trusting the sector indices.
- The full steering sign chain (`navigation.py` → `actuators.set_direction` → `steering.angle_deg_to_duty`) must be validated on the physical car.

---

## Next steps

- [ ] Validate the lidar convention (`scan[0]` = front) on the physical car.
- [ ] Validate the full steering sign on the physical car.
- [ ] Validate the ESC double-tap reverse sequence on the physical car.
- [ ] Validate the sonar stop during reverse.
- [ ] Tune `NAV_K`, `SPEED_ALPHA`, `FRONT_D_REF_MM` from on-car log data.

---

## Project tree

```text
covapsy/
├── main.py                 # entry point, control loop
├── navigation.py           # pure steering and speed computation
├── config.py               # all tuning parameters
├── actuators.py            # ESC + servo + blocking reverse
├── steering.py             # angle-to-duty conversion
├── lidar_thread.py         # acquisition thread
├── lidar_consumer.py       # non-blocking scan consumer
├── sonar.py                # SRF10 rear sonar thread
├── requirements.txt
├── README.md
└── LICENSE
```

---

## License

MIT License — Copyright (c) 2026 mahdidou711. See [LICENSE](LICENSE) for details.

# CoVAPSy 2026

![Python](https://img.shields.io/badge/language-Python-blue)
![License](https://img.shields.io/github/license/mahdidou711/covapsy)
![Last Commit](https://img.shields.io/github/last-commit/mahdidou711/covapsy)
![Repo Size](https://img.shields.io/github/repo-size/mahdidou711/covapsy)

**Autonomous 1/10-scale RC car — ENS Paris-Saclay CoVAPSy 2026 competition**

---

## Hardware

| Component | Model | Role |
|---|---|---|
| Single-board computer | Raspberry Pi 4 | Main compute + communications |
| Lidar | RPLidar A2M12 | 360° distance sensing at 256000 baud |
| ESC | Hobby-grade brushless ESC | Motor speed control (forward + reverse double-tap) |
| Servo | Standard RC servo | Steering control (hardware PWM GPIO 12/13) |
| Rear sonar | SRF10 | Obstacle detection during reverse (I2C bus 1, address 0x70) |

### Calibrated PWM values

| Signal | Parameter | Value |
|---|---|---|
| ESC neutral | `ESC_DUTY_NEUTRAL` | 7.78 % duty |
| ESC forward start | `ESC_DUTY_FWD_START` | 7.88 % (above neutral) |
| ESC reverse engage | `ESC_DUTY_REV_START` | 7.20 % |
| ESC reverse stable | `ESC_DUTY_REV_STABLE` | 6.70 % |
| Servo center | `SERVO_DUTY_CENTER` | 7.85 % |
| Servo left (MIN) | `SERVO_DUTY_MIN` | 6.85 % |
| Servo right (MAX) | `SERVO_DUTY_MAX` | 8.60 % |

### ESC notes
- **Forward**: above neutral (7.88+) — below neutral is the braking/deadband zone
- **Reverse**: requires a double-tap sequence (brake → neutral → brake → stable reverse)
- Reverse is interrupted early if the rear sonar detects an obstacle

### LiDAR frame convention
- `scan[0]` = front, `scan[90]` = left, `scan[270]` = right
- `0` = no measurement (below minimum range ~200 mm per datasheet) — treat as obstacle, NOT empty space

---

## Software Architecture

| File | Role |
|---|---|
| `main.py` | Entry point — main control loop, stuck detection, escape maneuver, CSV logging |
| `ftg.py` | Follow-the-Gap algorithm — gap detection, best gap selection, angle output |
| `config.py` | **Single source of truth** — all tuning parameters (speed, angles, lidar sectors) |
| `actuators.py` | PWM control for motor (ESC) and steering servo, reverse double-tap sequence |
| `sonar.py` | SRF10 rear ultrasonic sensor thread (I2C, context-manager reconnection) |
| `lidar_thread.py` | RPLidar A2M12 acquisition thread (express mode, ~16K pts/s, auto-reconnect) |
| `lidar_consumer.py` | Non-blocking scan buffer with freshness check and rate estimation |
| `steering.py` | Asymmetric angle-to-duty-cycle conversion for servo |
| `cal_servo.py` | Interactive servo calibration script |
| `cal_esc_avant.py` | Interactive forward ESC calibration script |
| `cal_esc_reverse.py` | Interactive reverse ESC calibration script |
| `cal_lidar.py` | Lidar live display — verify angle convention and distance thresholds |

---

## Algorithms

### Follow-the-Gap (FTG)

Reactive navigation at 50 Hz:
1. Extract the forward lidar sector (`±FTG_SECTOR_DEG` around front)
2. Threshold: points below `D_MIN_MM` are treated as obstacles (gap closed)
3. Find contiguous free sequences of at least `W_MIN_DEG` points
4. Select the best gap: highest average depth; tie-break by gap width
5. Steer toward the gap center with gain `K_FTG`, clamped to `±STEER_ANGLE_MAX_DEG`
6. Speed proportionally reduced when `front_min < D_MIN_MM`, down to `VITESSE_MIN` at `COLLISION_DIST_MM`

### Stuck detection — two triggers

**Trigger 1 — counter**: `front_min < STUCK_DIST_MM` for `STUCK_TICKS` consecutive ticks (25 = 0.5 s at 50 Hz)

**Trigger 2 — zero front**: `scan[0] == 0` (too close to measure) AND lateral walls both < 1000 mm for `STUCK_AV_ZERO_TICKS` ticks (15 = 0.3 s) — handles the case where the car is wedged so close to a wall that the lidar returns no front measurement

### Reverse / Escape sequence

```
stuck detected
  → choose open side: left_min vs right_min over sector [85–95°] / [265–275°]
  → set steering toward open side
  → reculer():
      1. duty → REV_START  (brake,  REVERSE_ENGAGE_S)
      2. duty → NEUTRAL    (reset,  REVERSE_ENGAGE_S)
      3. duty → REV_START  (brake,  REVERSE_ENGAGE_S)   ← double-tap
      4. duty → REV_STABLE for T_REVERSE_S
         (interrupted early if rear sonar < SONAR_ARRIERE_SEUIL_CM)
      5. duty → NEUTRAL
  → escape phase: drive forward with same steering for ESCAPE_TICKS ticks (40 = 0.8 s)
  → cooldown: ESCAPE_COOLDOWN_TICKS ticks (50 = 1.0 s) before stuck detection resumes
```

### Telemetry

Each run writes `run_log.csv` in the working directory at 50 Hz:

```
t, front_min, angle, vitesse, etat, stuck
```

`etat` is `"AVANCE"` (normal FTG) or `"EVITE"` (escape phase active).

---

## Installation

```bash
git clone https://github.com/mahdidou711/covapsy.git
cd covapsy
pip install -r requirements.txt
```

> **Note**: must run on a Raspberry Pi 4 with the RPLidar connected on `/dev/ttyUSB0` and hardware PWM enabled on GPIO 12 and GPIO 13.

Enable hardware PWM in `/boot/config.txt`:
```
dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
```

---

## Usage

### Run the car
```bash
python3 main.py
```

Stop: `Ctrl+C` or `kill` — cleanly stops PWM, centers servo, and stops LiDAR motor.

### Deployment from dev machine
```bash
# Quick (tuning only)
scp config.py main.py actuators.py voituree3a6@192.168.123.97:~/covapsy/

# Full redeploy
scp *.py voituree3a6@192.168.123.97:~/covapsy/
```

### Calibration (run once before first use)
```bash
python3 cal_servo.py        # find servo CENTER, MIN, MAX duty cycles
python3 cal_esc_avant.py    # find ESC NEUTRAL and FWD_START duty cycles
python3 cal_esc_reverse.py  # find REV_START and REV_STABLE duty cycles
python3 cal_lidar.py        # verify angle mapping and distance thresholds live
```

---

## Configuration

All tuning parameters are centralized in `config.py`. **This is the only file to edit on race day.**

### Speed

| Parameter | Default | Description |
|---|---|---|
| `VITESSE_MS` | 0.3 m/s | Target forward speed — start here, increase if stable |
| `VITESSE_MIN` | 0.28 m/s | Minimum speed — ESC deadband floor |
| `VITESSE_MAX_MS` | 2.0 m/s | Hard software cap — do not exceed |

### FTG

| Parameter | Default | Description |
|---|---|---|
| `D_MIN_MM` | 1500 mm | Safety bubble radius — increase = more conservative |
| `W_MIN_DEG` | 20° | Minimum angular width of a valid gap |
| `K_FTG` | 1.0 | Steering gain — increase = more aggressive turns |
| `FTG_SECTOR_DEG` | 150° | Half-angle of the forward lidar sector analysed by FTG |

### Collision / Speed control

| Parameter | Default | Description |
|---|---|---|
| `COLLISION_DIST_MM` | 900 mm | Distance at which speed starts reducing (must be < D_MIN_MM) |
| `COLLISION_SECTOR_DEG` | 15° | Half-angle of the front detection sector |

### Stuck detection & Reverse

| Parameter | Default | Description |
|---|---|---|
| `STUCK_DIST_MM` | 400 mm | Front distance threshold to increment stuck counter |
| `STUCK_TICKS` | 25 (0.5 s) | Consecutive ticks below threshold to trigger reverse |
| `STUCK_AV_ZERO_TICKS` | 15 (0.3 s) | Ticks with no front measurement + lateral walls close → stuck |
| `T_REVERSE_S` | 1.0 s | Max reverse duration (can be cut short by rear sonar) |
| `REVERSE_ENGAGE_S` | 0.10 s | Delay between steps of the double-tap sequence |
| `ESCAPE_TICKS` | 40 (0.8 s) | Forward escape phase duration after reverse |
| `ESCAPE_COOLDOWN_TICKS` | 50 (1.0 s) | Cooldown before stuck detection resumes after escape |

### Sonar

| Parameter | Default | Description |
|---|---|---|
| `SONAR_ACTIF` | True | Set to False if sonar is not connected |
| `SONAR_ARRIERE_SEUIL_CM` | 20 cm | Rear obstacle distance that interrupts reverse early |

---

## Project Structure

```
covapsy/
├── main.py               # Main control loop
├── ftg.py                # Follow-the-Gap algorithm
├── config.py             # All tuning parameters
├── actuators.py          # Motor + servo PWM control
├── sonar.py              # Rear SRF10 ultrasonic sensor
├── lidar_thread.py       # Lidar acquisition (express mode)
├── lidar_consumer.py     # Non-blocking scan buffer
├── steering.py           # Servo angle conversion
├── cal_servo.py          # Servo calibration
├── cal_esc_avant.py      # Forward ESC calibration
├── cal_esc_reverse.py    # Reverse ESC calibration
├── cal_lidar.py          # Lidar calibration
├── requirements.txt      # Python dependencies
└── .github/
    ├── workflows/
    │   └── lint.yml      # CI: flake8 linting
    └── ISSUE_TEMPLATE/   # Bug report template
```

---

## Roadmap

- [ ] Verify LiDAR angle convention on hardware (`cal_lidar.py`)
- [ ] PID lateral controller (wall-following complement to FTG)
- [ ] PPO training in Webots simulation
- [ ] Sim-to-real policy transfer
- [ ] Multi-car avoidance / overtaking

---

## Author

**mahdidou711** — ENS Paris-Saclay, CoVAPSy 2026

## License

This project is licensed under the MIT License — see [LICENSE](LICENSE) for details.

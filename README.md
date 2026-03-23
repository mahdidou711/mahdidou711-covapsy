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

### ESC notes
- **Neutral**: 7.78% duty — **Forward**: above neutral (7.88+)
- **Reverse**: requires a double-tap sequence (brake → neutral → brake → stable reverse)
- Reverse is physically confirmed working on this ESC

### LiDAR frame convention
- `scan[0]` = front, `scan[90]` = left, `scan[270]` = right
- `0` = no measurement (below minimum range ~200 mm per datasheet) — treat as obstacle, NOT empty space

---

## Software Architecture

| File | Role |
|---|---|
| `main.py` | Entry point — main control loop, state machine (stuck detection, escape mode) |
| `ftg.py` | Follow-the-Gap algorithm — gap detection, best gap selection, angle output |
| `config.py` | **Single source of truth** — all tuning parameters (speed, angles, lidar sectors) |
| `actuators.py` | PWM control for motor (ESC) and steering servo, reverse double-tap sequence |
| `sonar.py` | SRF10 rear ultrasonic sensor thread (I2C, auto-reconnection) |
| `lidar_thread.py` | RPLidar A2M12 acquisition thread (express mode, ~16K pts/s) |
| `lidar_consumer.py` | Non-blocking scan buffer with freshness check |
| `steering.py` | Angle-to-duty-cycle conversion for servo |
| `cal_servo.py` | Servo calibration script |
| `cal_esc_avant.py` | Forward ESC calibration script |
| `cal_esc_reverse.py` | Reverse ESC calibration script |
| `cal_lidar.py` | Lidar calibration and angle verification script |

---

## Algorithms

### Follow-the-Gap (FTG) — current
The car uses a gap-based reactive navigation algorithm:
1. Extract a forward lidar sector (`FTG_SECTOR_DEG`)
2. Detect free gaps (distances above threshold)
3. Select the best gap by average depth
4. Steer toward the gap center, with proportional speed reduction based on frontal distance

**Stuck detection**: if `front_min < STUCK_DIST_MM` for `STUCK_TICKS` consecutive ticks → trigger reverse + escape maneuver.

**Escape mode**: after reversing, force steering toward the open side for `ESCAPE_TICKS` ticks, then hand back control to FTG.

### Stuck / Reverse / Escape sequence

```
front_min < STUCK_DIST_MM for STUCK_TICKS ticks
  → choose open side (left_d vs right_d, treating 0 as minimum range)
  → set steering toward open side
  → reculer() : brake → neutral → brake → stable reverse → T_REVERSE_S → neutral
                (interrupted early if rear sonar detects obstacle < SONAR_ARRIERE_SEUIL_CM)
  → escape phase: drive forward with same steering for ESCAPE_TICKS ticks (40 = 0.8 s)
  → cooldown 50 ticks (1 s) before stuck detection resumes
```

### Planned — PPO sim-to-real transfer
- Train a PPO agent (Stable-Baselines3) in Webots simulation
- Transfer policy weights to the RPi 4 for real-world inference
- Curriculum: oval track → chicanes → full race track

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

### Deployment from dev machine
```bash
scp config.py main.py actuators.py voituree3a6@192.168.123.97:~/covapsy/
```

### Calibration (run once before first use)
```bash
python3 cal_servo.py        # find servo CENTER, MIN, MAX duty cycles
python3 cal_esc_avant.py    # find ESC NEUTRAL and FWD_START duty cycles
python3 cal_esc_reverse.py  # find ESC reverse duty cycles and double-tap timing
python3 cal_lidar.py        # verify lidar angle mapping (scan[0]=front, scan[90]=left)
```

Stop: `Ctrl+C` — cleanly stops PWM and LiDAR motor.

---

## Configuration

All tuning parameters are centralized in `config.py`. **This is the only file to edit on race day.**

### Speed

| Parameter | Default | Description |
|---|---|---|
| `VITESSE_MS` | 0.3 m/s | Target forward speed — start here, increase if stable |
| `VITESSE_MIN` | 0.28 m/s | Minimum speed (ESC deadband floor) |

### FTG

| Parameter | Default | Description |
|---|---|---|
| `D_MIN_MM` | 1500 mm | Safety bubble radius — increase = more conservative |
| `K_FTG` | 1.0 | Steering gain — increase = more aggressive turns |
| `FTG_SECTOR_DEG` | 150° | Forward lidar sector width |

### Collision / Speed control

| Parameter | Default | Description |
|---|---|---|
| `COLLISION_DIST_MM` | 900 mm | Distance at which speed starts reducing |
| `COLLISION_SECTOR_DEG` | 15° | Front detection sector width |

### Stuck detection & Reverse

| Parameter | Default | Description |
|---|---|---|
| `STUCK_DIST_MM` | 400 mm | Front distance threshold to declare stuck |
| `STUCK_TICKS` | 25 (0.5 s) | Consecutive ticks below threshold to trigger reverse |
| `STUCK_AV_ZERO_TICKS` | 15 (0.3 s) | Ticks with no front measurement + lateral walls close → stuck |
| `T_REVERSE_S` | 1.0 s | Reverse duration (can be cut short by rear sonar) |
| `REVERSE_ENGAGE_S` | 0.10 s | Delay between steps of the double-tap sequence |
| `ESCAPE_TICKS` | 40 (0.8 s) | Forward escape phase duration after reverse |

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
    └── ISSUE_TEMPLATE/   # Bug report / feature request templates
```

---

## Roadmap

- [ ] PID lateral controller (wall-following complement to FTG)
- [ ] PPO training in Webots simulation
- [ ] Sim-to-real policy transfer
- [ ] Verify LiDAR angle convention on hardware (`cal_lidar.py`)
- [ ] Multi-car avoidance / overtaking

---

## Author

**mahdidou711** — ENS Paris-Saclay, CoVAPSy 2026

## License

This project is licensed under the MIT License — see [LICENSE](LICENSE) for details.

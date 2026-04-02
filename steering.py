"""Conversion angle -> duty cycle servo."""

import config  # calibrations servo et bornes PWM


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x  # saturation générique


def angle_deg_to_duty(angle_deg: float) -> float:
    """Convertit un angle logique en PWM calibré."""
    # Repli sûr si la calibration max manque.
    amax = float(getattr(config, "STEER_ANGLE_MAX_DEG", 18.0))  # angle max du servo
    angle_deg = clamp(angle_deg, -amax, amax)  # saturation avant interpolation

    x = angle_deg / amax if amax > 0 else 0.0  # -1 gauche, 0 centre, +1 droite

    # Interpolation asymétrique autour du centre calibré.
    if x >= 0:
        duty = config.SERVO_DUTY_CENTER + x * (config.SERVO_DUTY_MAX - config.SERVO_DUTY_CENTER)  # côté droite
    else:
        duty = config.SERVO_DUTY_CENTER + x * (config.SERVO_DUTY_CENTER - config.SERVO_DUTY_MIN)  # côté gauche

    return clamp(duty, config.DUTY_MIN_CLAMP, config.DUTY_MAX_CLAMP)  # sécurité finale

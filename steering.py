# ~/covapsy/steering.py
# Module de conversion angle volant → duty cycle PWM servo.
# Expose : clamp() (saturation générique) et angle_deg_to_duty() (conversion).

import config


# Saturation générique : renvoie x borné entre lo et hi.
def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def angle_deg_to_duty(angle_deg: float) -> float:
    """
    Map angle_deg in [-STEER_ANGLE_MAX_DEG, +STEER_ANGLE_MAX_DEG] to duty in [SERVO_DUTY_MIN, SERVO_DUTY_MAX].

    Valeurs calibrées :
      centre = 7.85 %  (roues droites)
      min    = 6.85 %  (butée gauche, -18°)
      max    = 8.60 %  (butée droite, +18°)
      débattement = ±18°
    """
    # getattr avec fallback 18.0 : protège contre un config incomplet lors des tests.
    amax = float(getattr(config, "STEER_ANGLE_MAX_DEG", 18.0))
    # On sature l'angle AVANT de normaliser pour garantir que x reste dans [-1, +1].
    angle_deg = clamp(angle_deg, -amax, amax)

    # x = position normalisée du volant : -1 = butée gauche, 0 = centre, +1 = butée droite.
    # Normalisation [-1, +1]
    x = angle_deg / amax if amax > 0 else 0.0

    # Interpolation asymétrique autour du centre :
    # Le centre (7.85) n'est pas au milieu de [6.85, 8.60], donc la course
    # gauche (1.00 %) et droite (0.75 %) sont interpolées séparément.
    # x=+1 → 7.85 + 1×(8.60-7.85) = 8.60  (butée droite)
    # x= 0 → 7.85                           (centre)
    # x=-1 → 7.85 + (-1)×(7.85-6.85) = 6.85 (butée gauche)
    if x >= 0:
        duty = config.SERVO_DUTY_CENTER + x * (config.SERVO_DUTY_MAX - config.SERVO_DUTY_CENTER)
    else:
        duty = config.SERVO_DUTY_CENTER + x * (config.SERVO_DUTY_CENTER - config.SERVO_DUTY_MIN)

    # Deuxième clamp de sécurité : empêche tout dépassement des limites
    # physiques [5.0, 10.0] même si config contient des valeurs incohérentes.
    return clamp(duty, config.DUTY_MIN_CLAMP, config.DUTY_MAX_CLAMP)

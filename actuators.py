"""Commande servo/ESC pour le runtime final."""

import time  # temporisations recul

import numpy as np  # calcul np.sign pour le choix d'angle de recul

from rpi_hardware_pwm import HardwarePWM  # PWM hardware Raspberry Pi
import config  # calibrations et seuils
from steering import angle_deg_to_duty, clamp  # conversion angle -> PWM et saturation

sonar_module = None  # import conditionnel du sonar arrière
if getattr(config, "SONAR_ACTIF", False):
    import sonar as sonar_module


class Actuators:

    def __init__(self):
        # Canal 0 = ESC, canal 1 = servo.
        self.pwm_prop = HardwarePWM(pwm_channel=config.PWM_PROP_CHANNEL, hz=config.PWM_HZ)  # propulsion
        self.pwm_dir = HardwarePWM(pwm_channel=config.PWM_DIR_CHANNEL, hz=config.PWM_HZ)  # direction
        # Démarrage au neutre et roues droites.
        self.pwm_prop.start(config.ESC_DUTY_NEUTRAL)  # ESC armé au point mort
        self.pwm_dir.start(config.SERVO_DUTY_CENTER)  # servo centré

    def set_direction(self, angle_deg: float):
        # Convention logique : + = droite.
        dc = angle_deg_to_duty(-angle_deg)  # inversion mécanique
        self.pwm_dir.change_duty_cycle(dc)  # envoi PWM servo

    def _speed_ms_to_duty(self, ms: float) -> float:
        ms = max(-config.VITESSE_MAX_MS, min(ms, config.VITESSE_MAX_MS))
        if ms == 0:
            dc = config.ESC_DUTY_NEUTRAL
        elif ms > 0:
            delta = ms * config.PROP_DELTA_PWM_MAX / config.VITESSE_MAX_MS
            dc = config.ESC_DUTY_NEUTRAL + config.PROP_POINT_MORT_PWM + delta
        else:
            delta = abs(ms) * config.PROP_DELTA_PWM_MAX / config.VITESSE_MAX_MS
            dc = config.ESC_DUTY_NEUTRAL - (config.PROP_POINT_MORT_PWM + delta)
        return clamp(dc, config.DUTY_MIN_CLAMP, config.DUTY_MAX_CLAMP)

    def set_vitesse(self, ms: float):
        dc = self._speed_ms_to_duty(ms)  # mapping terrain de test.py
        self.pwm_prop.change_duty_cycle(dc)  # envoi PWM ESC

    def reculer(self, scan=None, duree_s=None):
        """Recul bloquant. Calcule l'angle d'échappement par np.sign sur les secteurs latéraux."""
        if duree_s is None:
            duree_s = config.REVERSE_DUREE_S

        # --- Choix de l'angle ---
        angle_deg = 0.0
        left_space = 0.0
        right_space = 0.0
        if scan is not None:
            ls, le = config.ESCAPE_LEFT_SECTOR
            rs, re = config.ESCAPE_RIGHT_SECTOR
            left_vals  = [scan[i] for i in range(ls, le) if scan[i] > 0]
            right_vals = [scan[i] for i in range(rs, re) if scan[i] > 0]
            left_space  = float(np.mean(left_vals)) if left_vals else 0.0
            right_space = float(np.mean(right_vals)) if right_vals else 0.0
            delta = right_space - left_space
            if abs(delta) >= config.ESCAPE_SYMMETRY_MM:
                angle_deg = -float(np.sign(delta)) * config.STEER_ANGLE_MAX_DEG

        # --- Garde espace minimal de pivot ---
        if angle_deg < 0 and left_space < config.ESCAPE_MIN_PIVOT_MM:
            angle_deg = 0.0  # espace gauche insuffisant pour le pivot, recul droit
        elif angle_deg > 0 and right_space < config.ESCAPE_MIN_PIVOT_MM:
            angle_deg = 0.0  # espace droit insuffisant pour le pivot, recul droit

        # --- Braquage initial ---
        self.set_direction(angle_deg)

        # --- Séquence ESC double-tap avec neutre intermédiaire ---
        self.pwm_prop.change_duty_cycle(config.ESC_DUTY_REV_START)  # tap 1
        time.sleep(config.REVERSE_TAP_S)
        self.set_direction(angle_deg)

        self.pwm_prop.change_duty_cycle(config.ESC_DUTY_NEUTRAL)    # neutre 1
        time.sleep(config.REVERSE_TAP_S)
        self.set_direction(angle_deg)

        self.pwm_prop.change_duty_cycle(config.ESC_DUTY_REV_START)  # tap 2
        time.sleep(config.REVERSE_TAP_S)
        self.set_direction(angle_deg)

        self.pwm_prop.change_duty_cycle(config.ESC_DUTY_NEUTRAL)    # neutre 2 avant stable
        time.sleep(config.REVERSE_TAP_S)
        self.set_direction(angle_deg)

        # --- Recul stable + sonar ---
        self.pwm_prop.change_duty_cycle(config.ESC_DUTY_REV_STABLE)
        t0 = time.monotonic()
        while time.monotonic() - t0 < duree_s:
            self.set_direction(angle_deg)
            if sonar_module:
                dist = sonar_module.get_sonar_arriere()
                if dist is not None and 1 < dist < config.SONAR_ARRIERE_SEUIL_CM:
                    print(f"[SONAR] Obstacle arrière à {dist}cm, arrêt recul.")
                    break
            time.sleep(0.02)

        # --- Neutre final ---
        self.pwm_prop.change_duty_cycle(config.ESC_DUTY_NEUTRAL)
        return angle_deg

    def stop(self):
        # Chaque étape est isolée pour garantir l'arrêt même si une sortie plante.
        try:
            self.set_direction(0)  # roues droites
        except Exception:
            pass
        try:
            self.pwm_prop.change_duty_cycle(config.ESC_DUTY_NEUTRAL)  # point mort ESC
        except Exception:
            pass
        try:
            self.pwm_dir.stop()  # arrêt PWM servo
        except Exception:
            pass
        try:
            self.pwm_prop.stop()  # arrêt PWM ESC
        except Exception:
            pass

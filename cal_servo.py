# cal_servo.py — Calibration du servo de direction
# Raspberry Pi 4, hardware PWM canal 1 (GPIO 13), 50 Hz
#
# Touches :
#   +/−   → ajuste le duty cycle de ±0.05 %
#   c/C   → enregistre le centre
#   g/G   → enregistre la butée gauche (SERVO_DUTY_MIN)
#   d/D   → enregistre la butée droite (SERVO_DUTY_MAX)
#   q/Q   → quitte et affiche le bloc config.py
#
# Usage : python3 cal_servo.py

import sys
import tty
import termios
from rpi_hardware_pwm import HardwarePWM

# ── Constantes ────────────────────────────────────────────────────
PWM_HZ          = 50
PWM_DIR_CHANNEL = 1       # GPIO 13
DUTY_MIN_CLAMP  = 5.0
DUTY_MAX_CLAMP  = 10.0

# Valeurs de départ (dernière calibration connue — sync config.py)
SERVO_DUTY_CENTER = 7.85
SERVO_DUTY_MIN    = 6.85
SERVO_DUTY_MAX    = 8.60

STEP = 0.05               # Incrément par pression de touche


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def lire_touche():
    """Lit un caractère depuis stdin sans bloquer (mode cbreak)."""
    return sys.stdin.read(1)


def afficher(duty, centre, gauche, droite):
    print(f"\rduty={duty:.2f} | centre={centre:.2f} | gauche={gauche:.2f} | droite={droite:.2f}   ", end="", flush=True)


def main():
    duty   = SERVO_DUTY_CENTER
    centre = SERVO_DUTY_CENTER
    gauche = SERVO_DUTY_MIN
    droite = SERVO_DUTY_MAX

    # Sauvegarde de l'état terminal pour restauration dans finally
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    pwm = HardwarePWM(pwm_channel=PWM_DIR_CHANNEL, hz=PWM_HZ)
    pwm.start(duty)
    print("=== Calibration servo ===")
    print("  +/−  : ajuste duty ±0.05")
    print("  c/C  : enregistre centre")
    print("  g/G  : enregistre butée gauche")
    print("  d/D  : enregistre butée droite")
    print("  q/Q  : quitte")
    print()
    afficher(duty, centre, gauche, droite)

    try:
        # Mode cbreak : chaque touche est lue immédiatement, sans attendre Entrée
        tty.setcbreak(fd)

        while True:
            ch = lire_touche()

            if ch == '+':
                duty = clamp(duty + STEP, DUTY_MIN_CLAMP, DUTY_MAX_CLAMP)
                pwm.change_duty_cycle(duty)

            elif ch == '-':
                duty = clamp(duty - STEP, DUTY_MIN_CLAMP, DUTY_MAX_CLAMP)
                pwm.change_duty_cycle(duty)

            elif ch in ('c', 'C'):
                centre = duty

            elif ch in ('g', 'G'):
                gauche = duty

            elif ch in ('d', 'D'):
                droite = duty

            elif ch in ('q', 'Q'):
                break

            afficher(duty, centre, gauche, droite)

    finally:
        # Restauration du terminal quoi qu'il arrive (exception, Ctrl+C, etc.)
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        pwm.stop()

    # Résultat final
    print("\n")
    print("=" * 45)
    print("Bloc à copier-coller dans config.py :")
    print("=" * 45)
    print(f"SERVO_DUTY_CENTER = {centre:.2f}")
    print(f"SERVO_DUTY_MIN    = {gauche:.2f}")
    print(f"SERVO_DUTY_MAX    = {droite:.2f}")
    print("=" * 45)


if __name__ == "__main__":
    main()

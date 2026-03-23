# cal_esc_avant.py — Calibration ESC : neutral et seuil marche avant
# Raspberry Pi 4, hardware PWM canal 0 (GPIO 12), 50 Hz
#
# Touches :
#   +/−   → ajuste le duty cycle de ±0.02 %
#   n/N   → enregistre le duty courant comme neutral
#   f/F   → enregistre le duty courant comme seuil marche avant
#   q/Q   → retour au neutral, stop PWM, quitte
#
# Usage : python3 cal_esc_avant.py

import sys
import tty
import termios
from rpi_hardware_pwm import HardwarePWM

# ── Constantes ────────────────────────────────────────────────────
PWM_HZ           = 50
PWM_PROP_CHANNEL = 0       # GPIO 12
DUTY_MIN_CLAMP   = 5.0
DUTY_MAX_CLAMP   = 10.0

# Valeurs de départ (dernière calibration connue — sync config.py)
ESC_DUTY_NEUTRAL   = 7.78
ESC_DUTY_FWD_START = 7.88

STEP = 0.02               # Incrément par pression de touche (plus fin que servo)


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def lire_touche():
    """Lit un caractère depuis stdin sans bloquer (mode cbreak)."""
    return sys.stdin.read(1)


def afficher(duty, neutral, fwd_start):
    print(f"\rduty={duty:.2f} | neutral={neutral:.2f} | fwd_start={fwd_start:.2f}   ", end="", flush=True)


def main():
    duty      = ESC_DUTY_NEUTRAL
    neutral   = ESC_DUTY_NEUTRAL
    fwd_start = ESC_DUTY_FWD_START

    # Sauvegarde de l'état terminal pour restauration dans finally
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    pwm = HardwarePWM(pwm_channel=PWM_PROP_CHANNEL, hz=PWM_HZ)
    # Démarrage au neutral : l'ESC s'arme et émet ses bips de confirmation
    pwm.start(duty)
    print("=== Calibration ESC — marche avant ===")
    print("  Attendre les bips d'armement ESC avant de continuer.")
    print()
    print("  +/−  : ajuste duty ±0.02")
    print("  n/N  : enregistre neutral (moteur à l'arrêt, bips d'armement)")
    print("  f/F  : enregistre seuil marche avant (moteur commence à tourner)")
    print("  q/Q  : retour au neutral, quitte")
    print()
    afficher(duty, neutral, fwd_start)

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

            elif ch in ('n', 'N'):
                neutral = duty

            elif ch in ('f', 'F'):
                fwd_start = duty

            elif ch in ('q', 'Q'):
                break

            afficher(duty, neutral, fwd_start)

    finally:
        # Retour au neutral avant d'arrêter : évite de laisser le moteur
        # alimenté ou l'ESC dans un état indéfini.
        pwm.change_duty_cycle(neutral)
        # Restauration du terminal quoi qu'il arrive
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        pwm.stop()

    # Résultat final
    print("\n")
    print("=" * 45)
    print("Bloc à copier-coller dans config.py :")
    print("=" * 45)
    print(f"ESC_DUTY_NEUTRAL   = {neutral:.2f}")
    print(f"ESC_DUTY_FWD_START = {fwd_start:.2f}")
    print("=" * 45)


if __name__ == "__main__":
    main()

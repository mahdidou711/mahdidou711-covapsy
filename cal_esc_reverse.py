# cal_esc_reverse.py — Calibration ESC : reverse (REV_START et REV_STABLE)
# Raspberry Pi 4, hardware PWM canal 0 (GPIO 12), 50 Hz
#
# Touches :
#   s      → REV_START  += 0.02
#   S      → REV_START  -= 0.02
#   t      → REV_STABLE += 0.02
#   T      → REV_STABLE -= 0.02
#   r/R    → exécute la séquence reverse complète
#   q/Q    → retour au neutral, stop PWM, quitte
#
# Usage : python3 cal_esc_reverse.py

import sys
import tty
import termios
import time
from rpi_hardware_pwm import HardwarePWM

# ── Constantes ────────────────────────────────────────────────────
PWM_HZ           = 50
PWM_PROP_CHANNEL = 0       # GPIO 12
DUTY_MIN_CLAMP   = 5.0
DUTY_MAX_CLAMP   = 10.0

# Valeurs calibrées à l'étape précédente (sync config.py)
ESC_DUTY_NEUTRAL = 7.78

# Valeurs de départ pour le reverse (à affiner)
ESC_DUTY_REV_START  = 7.20   # Duty d'engagement reverse (doit être < neutral)
ESC_DUTY_REV_STABLE = 6.70   # Duty de recul stable
T_REVERSE_S         = 0.4    # Durée du recul stable (secondes)
REVERSE_ENGAGE_S    = 0.1    # Durée d'engagement (secondes)

STEP = 0.02


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def afficher(rev_start, rev_stable):
    print(f"\rrev_start={rev_start:.2f} | rev_stable={rev_stable:.2f}   ", end="", flush=True)


def sequence_reverse(pwm, rev_start, rev_stable, neutral):
    """
    Exécute la séquence reverse avec time.sleep(0.01) pour céder le CPU.
    1. REV_START pendant REVERSE_ENGAGE_S
    2. REV_STABLE pendant T_REVERSE_S
    3. Retour au neutral
    """
    print("\n  → Séquence reverse en cours...", flush=True)

    # Phase 1 : engagement reverse
    pwm.change_duty_cycle(rev_start)
    t0 = time.monotonic()
    while time.monotonic() - t0 < REVERSE_ENGAGE_S:
        time.sleep(0.01)

    # Phase 2 : recul stable
    pwm.change_duty_cycle(rev_stable)
    t0 = time.monotonic()
    while time.monotonic() - t0 < T_REVERSE_S:
        time.sleep(0.01)

    # Phase 3 : retour neutral
    pwm.change_duty_cycle(neutral)
    print("  → Retour neutral.", flush=True)


def main():
    rev_start  = ESC_DUTY_REV_START
    rev_stable = ESC_DUTY_REV_STABLE

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    pwm = HardwarePWM(pwm_channel=PWM_PROP_CHANNEL, hz=PWM_HZ)
    pwm.start(ESC_DUTY_NEUTRAL)

    print("=== Calibration ESC — reverse ===")
    print(f"  Neutral fixé à {ESC_DUTY_NEUTRAL} (ne pas modifier ici).")
    print()
    print("  s      : rev_start  += 0.02")
    print("  S      : rev_start  -= 0.02")
    print("  t      : rev_stable += 0.02")
    print("  T      : rev_stable -= 0.02")
    print("  r/R    : exécute la séquence reverse")
    print("  q/Q    : quitte")
    print()
    afficher(rev_start, rev_stable)

    try:
        tty.setcbreak(fd)

        while True:
            ch = sys.stdin.read(1)

            if ch == 's':
                rev_start = clamp(rev_start + STEP, DUTY_MIN_CLAMP, DUTY_MAX_CLAMP)
            elif ch == 'S':
                rev_start = clamp(rev_start - STEP, DUTY_MIN_CLAMP, DUTY_MAX_CLAMP)
            elif ch == 't':
                rev_stable = clamp(rev_stable + STEP, DUTY_MIN_CLAMP, DUTY_MAX_CLAMP)
            elif ch == 'T':
                rev_stable = clamp(rev_stable - STEP, DUTY_MIN_CLAMP, DUTY_MAX_CLAMP)
            elif ch in ('r', 'R'):
                # Restaurer le terminal pendant la séquence pour voir les prints
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                sequence_reverse(pwm, rev_start, rev_stable, ESC_DUTY_NEUTRAL)
                tty.setcbreak(fd)
            elif ch in ('q', 'Q'):
                break

            afficher(rev_start, rev_stable)

    finally:
        pwm.change_duty_cycle(ESC_DUTY_NEUTRAL)
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        pwm.stop()

    print("\n")
    print("=" * 45)
    print("Bloc à copier-coller dans config.py :")
    print("=" * 45)
    print(f"ESC_DUTY_REV_START  = {rev_start:.2f}")
    print(f"ESC_DUTY_REV_STABLE = {rev_stable:.2f}")
    print("=" * 45)


if __name__ == "__main__":
    main()

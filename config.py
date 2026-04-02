# Paramètres matériels et runtime de CoVAPSy — source unique de vérité.

# -----------------------------------------------------------------------------
# PWM — Général
# -----------------------------------------------------------------------------

PWM_HZ = 50                  # fréquence PWM standard servo/ESC

PWM_PROP_CHANNEL = 0         # canal ESC sur GPIO 12
PWM_DIR_CHANNEL  = 1         # canal servo sur GPIO 13

DUTY_MIN_CLAMP = 5.0         # borne basse de sécurité générale
DUTY_MAX_CLAMP = 10.0        # borne haute de sécurité générale

# -----------------------------------------------------------------------------
# Direction (servo) — valeurs calibrées
# -----------------------------------------------------------------------------

SERVO_DUTY_CENTER = 8.00     # centre calibré
SERVO_DUTY_MIN    = 6.40     # limite braquage gauche
SERVO_DUTY_MAX    = 9.60     # limite braquage droit

STEER_ANGLE_MAX_DEG = 18.0   # braquage physique max

# -----------------------------------------------------------------------------
# Propulsion (ESC) — valeurs calibrées
# -----------------------------------------------------------------------------

ESC_DUTY_NEUTRAL = 7.78      # neutre ESC calibré sur la voiture

PROP_POINT_MORT_PWM = 0.10   # seuil mini avant (offset sur le neutre)
PROP_DELTA_PWM_MAX  = 1.00   # delta PWM max propulsion

ESC_DUTY_FWD_START = ESC_DUTY_NEUTRAL + PROP_POINT_MORT_PWM  # seuil mini de marche avant

# -----------------------------------------------------------------------------
# ESC arrière — valeurs calibrées
# -----------------------------------------------------------------------------

ESC_DUTY_REV_START  = 7.20   # premier coup reverse (double-tap)
ESC_DUTY_REV_STABLE = 7.00   # recul stable après double-tap

# -----------------------------------------------------------------------------
# Lidar — RPLidar A2M12
# -----------------------------------------------------------------------------

PORT     = "/dev/ttyUSB0"    # port série lidar
BAUDRATE = 256000            # débit A2M12

LIDAR_FRESH_MAX_S   = 0.50   # scan considéré périmé au-delà de cette durée
LIDAR_TIMEOUT_TICKS = 15     # arrêt propulsion après N ticks sans scan frais
LIDAR_RATE_WIN_S    = 2.0    # fenêtre de calcul du taux de scan
LIDAR_REUSE_LAST_SCAN = True # réutilise le dernier scan frais en cas de timeout

# -----------------------------------------------------------------------------
# Navigation — direction
# -----------------------------------------------------------------------------

NAV_K   = 18.0               # gain de direction (sans logarithme)
NAV_EPS = 1.0                # seuil d'insensibilité latérale (mm)

DIR_LEFT_SECTOR  = (30, 60)  # secteur latéral gauche  — scan[30..60]
DIR_RIGHT_SECTOR = (300, 330) # secteur latéral droit  — scan[300..330]

# -----------------------------------------------------------------------------
# Navigation — vitesse latérale
# -----------------------------------------------------------------------------

SPEED_LEFT_SECTOR  = (35, 55)   # secteur gauche pour la loi de vitesse
SPEED_RIGHT_SECTOR = (305, 325) # secteur droit  pour la loi de vitesse
SPEED_EPS          = 1.0        # seuil d'insensibilité vitesse (mm)
SPEED_ALPHA        = 3.0        # exposant de la loi de vitesse exponentielle

VITESSE_CROISIERE = 0.50        # vitesse de croisière active
VITESSE_PLANCHER  = 0.25        # vitesse minimale en courbe serrée
VITESSE_MAX_MS    = 0.80        # vitesse avant maximale (échelle utilisateur)

# -----------------------------------------------------------------------------
# Navigation — terme frontal
# -----------------------------------------------------------------------------

FRONT_SECTOR_HALF = 10          # demi-angle du secteur frontal (degrés)
FRONT_D_REF_MM    = 600         # distance de référence frontale (mm)

# -----------------------------------------------------------------------------
# Boucle de contrôle
# -----------------------------------------------------------------------------

CONTROL_HZ = 50              # fréquence de boucle
DT_S = 1.0 / CONTROL_HZ     # période de boucle

# -----------------------------------------------------------------------------
# Recul et échappement
# -----------------------------------------------------------------------------

# Détection blocage
STUCK_DIST_MM        = 500   # distance frontale déclenchant l'incrément stuck
STUCK_TICKS          = 6     # ticks consécutifs avant recul
STUCK_COOLDOWN_TICKS = 50    # ticks d'immunité post-recul

# Séquence ESC double-tap
REVERSE_TAP_S   = 0.20       # durée de chaque étape du double-tap
REVERSE_DUREE_S = 1.0        # durée du recul stable après double-tap

# Secteurs et seuil pour choix de côté lors du recul
ESCAPE_LEFT_SECTOR  = (20, 80)    # secteur gauche pour choix d'angle de recul
ESCAPE_RIGHT_SECTOR = (280, 340)  # secteur droit  pour choix d'angle de recul
ESCAPE_MIN_PIVOT_MM = 200         # dégagement mini pour décider de pivoter
ESCAPE_SYMMETRY_MM  = 100         # écart minimal gauche/droite pour choisir un côté

# -----------------------------------------------------------------------------
# Capteurs additionnels
# -----------------------------------------------------------------------------

SONAR_MATERIEL_PRESENT = True             # présence physique du sonar
SONAR_ACTIF            = SONAR_MATERIEL_PRESENT  # sonar actif si matériel présent
SONAR_ARRIERE_SEUIL_CM = 25               # arrêt recul sous ce seuil

# -----------------------------------------------------------------------------
# Activation des logiques optionnelles
# -----------------------------------------------------------------------------

STUCK_RECOVERY_ACTIVE = True  # récupération de blocage active

# -----------------------------------------------------------------------------
# Validations de sécurité (résistantes à python3 -O)
# -----------------------------------------------------------------------------

if not (0.0 < VITESSE_PLANCHER < VITESSE_CROISIERE <= VITESSE_MAX_MS):
    raise ValueError(
        f"Incohérence vitesses : PLANCHER={VITESSE_PLANCHER} "
        f"CROISIERE={VITESSE_CROISIERE} MAX={VITESSE_MAX_MS}"
    )
if not (0.1 <= VITESSE_CROISIERE <= 0.8):
    raise ValueError(f"VITESSE_CROISIERE hors plage [0.1, 0.8] : {VITESSE_CROISIERE}")
if not (0.5 <= SPEED_ALPHA <= 6.0):
    raise ValueError(f"SPEED_ALPHA hors plage [0.5, 6.0] : {SPEED_ALPHA}")
if not (200 <= FRONT_D_REF_MM <= 2000):
    raise ValueError(f"FRONT_D_REF_MM hors plage [200, 2000] : {FRONT_D_REF_MM}")
if not (5 <= FRONT_SECTOR_HALF <= 45):
    raise ValueError(f"FRONT_SECTOR_HALF hors plage [5, 45] : {FRONT_SECTOR_HALF}")
if not (50 <= ESCAPE_MIN_PIVOT_MM <= 600):
    raise ValueError(f"ESCAPE_MIN_PIVOT_MM hors plage [50, 600] : {ESCAPE_MIN_PIVOT_MM}")
if not (ESC_DUTY_REV_START < ESC_DUTY_NEUTRAL):
    raise ValueError(f"REV_START ({ESC_DUTY_REV_START}) doit être < NEUTRAL ({ESC_DUTY_NEUTRAL})")
if CONTROL_HZ <= 0:
    raise ValueError(f"CONTROL_HZ doit être > 0 : {CONTROL_HZ}")
if not (0.10 <= REVERSE_TAP_S <= 0.50):
    raise ValueError(f"REVERSE_TAP_S hors plage : {REVERSE_TAP_S}")
if not (0.3 <= REVERSE_DUREE_S <= 3.0):
    raise ValueError(f"REVERSE_DUREE_S hors plage : {REVERSE_DUREE_S}")
if not (200 <= STUCK_DIST_MM <= 800):
    raise ValueError(f"STUCK_DIST_MM hors plage : {STUCK_DIST_MM}")

# =============================================================================
# config.py — Paramètres matériels et algorithmiques — CoVAPSy 2026
# =============================================================================
# COMMENT UTILISER CE FICHIER :
# Ce fichier est le seul à modifier le jour des essais.
# Chaque paramètre indique : valeur actuelle | plage valide | effet si on change.
# =============================================================================

# -----------------------------------------------------------------------------
# PWM — Général
# -----------------------------------------------------------------------------

PWM_HZ = 50                  # Fréquence PWM en Hz. Ne pas modifier (standard servo/ESC).

PWM_PROP_CHANNEL = 0         # Canal PWM propulsion — GPIO 12. Ne pas modifier.
PWM_DIR_CHANNEL  = 1         # Canal PWM direction  — GPIO 13. Ne pas modifier.

DUTY_MIN_CLAMP = 5.0         # Limite basse absolue de tout duty cycle envoyé.
DUTY_MAX_CLAMP = 10.0        # Limite haute absolue. Protège servo et ESC.

# -----------------------------------------------------------------------------
# Direction (servo) — valeurs calibrées
# -----------------------------------------------------------------------------

SERVO_DUTY_CENTER = 7.85    # Duty cycle (%) roues droites. Calibré terrain : 7.85.
                             # Si la voiture tire à gauche/droite en ligne droite :
                             # ajuster de ±0.05 jusqu'à rouler droit.

SERVO_DUTY_MIN = 6.65       # Duty cycle (%) butée gauche. Calibré terrain : 6.65.
                             # Ne pas descendre sous 6.00 (butée mécanique).

SERVO_DUTY_MAX = 8.80       # Duty cycle (%) butée droite. Calibré terrain : 8.80.
                             # Ne pas dépasser 10.00 (butée mécanique).

STEER_ANGLE_MAX_DEG = 18.0   # Débattement max en degrés de chaque côté.
                             # Correspond à SERVO_DUTY_MIN et SERVO_DUTY_MAX.
                             # Ne pas modifier sans re-calibrer les duty cycles.

# -----------------------------------------------------------------------------
# Propulsion (ESC) — valeurs calibrées
# -----------------------------------------------------------------------------

ESC_DUTY_NEUTRAL   = 7.78   # Point mort ESC. Calibré terrain : 7.78.
                             # Si le moteur tourne au repos : ajuster de ±0.05.

ESC_DUTY_FWD_START = 7.88   # Seuil minimal de marche avant. Calibré terrain : 7.88.
                             # FWD_START > NEUTRAL : avancer = augmenter le duty au-dessus du neutral.
                             # En dessous de FWD_START : moteur à l'arrêt (zone morte).

# -----------------------------------------------------------------------------
# Reverse — séquence double-tap (brake → neutral → brake → stable)
# -----------------------------------------------------------------------------
# Utilisé par actuators.reculer() lors de la manœuvre anti-blocage.

ESC_DUTY_REV_START  = 7.20  # Duty pour engager le reverse.
ESC_DUTY_REV_STABLE = 7.00  # Duty pour reculer stable.
T_REVERSE_S = 1.0            # Durée du recul en secondes.
REVERSE_ENGAGE_S = 0.10      # Temps entre chaque étape de la séquence reverse (s).
                             # Ne pas dépasser 0.10 : certains ESC hobby ont un timeout court
                             # entre les phases du double-tap et ne lancent pas le reverse avec 0.20.

# -----------------------------------------------------------------------------
# Lidar — RPLidar A2M12
# -----------------------------------------------------------------------------

PORT    = "/dev/ttyUSB0"     # Port série du lidar. Ne pas modifier.
BAUDRATE = 256000            # Vitesse série A2M12. Ne pas modifier (115200 pour A2M8).

LIDAR_FRESH_MAX_S = 0.50     # Scan ignoré si plus vieux que cette durée (secondes).
                             # Augmenter si beaucoup de scans sont rejetés.

LIDAR_RATE_WIN_S  = 2.0      # Fenêtre de calcul du taux de scan en secondes.
                             # Ne pas modifier.

# -----------------------------------------------------------------------------
# FTG — Follow The Gap
# -----------------------------------------------------------------------------

D_MIN_MM = 1500              # Distance de sécurité (mm). Tout point < D_MIN_MM
                             # est considéré comme obstacle et ferme le gap.
                             # Plage : 500–2000.
                             # Augmenter → plus prudent, gaps plus petits.
                             # Diminuer → prend plus de risques, gaps plus grands.

W_MIN_PTS = 20               # Largeur spatiale minimale d'un gap valide (points/degrés physiques du lidar).
                             # Plage : 10–40.
                             # Augmenter → ignore les petits gaps (plus sûr).
                             # Diminuer → accepte des passages étroits.

K_FTG = 1.6                  # Gain angulaire FTG : angle_commande = K_FTG * angle_gap_centre.
                             # Plage : 0.2–2.0.
                             # Augmenter → braquage plus agressif dans les virages.
                             # Diminuer → braquage plus doux, risque de rater les virages.

FTG_SECTOR_DEG = 150         # Demi-angle de la zone avant analysée par FTG (degrés).
                             # Plage : 60–180. Réduire si la voiture réagit à des
                             # obstacles latéraux non pertinents.

# -----------------------------------------------------------------------------
# Collision — seuil d'urgence
# -----------------------------------------------------------------------------

COLLISION_DIST_MM = 900      # Distance minimale avant ralentissement d'urgence (mm).
                             # Plage : 400–2000.
                             # Mesurée sur le secteur avant ±COLLISION_SECTOR_DEG.
                             # Diminuer → réagit plus tard (plus rapide mais risqué).
                             # Augmenter → réagit tôt (plus sûr mais peut freiner inutilement).

COLLISION_SECTOR_DEG = 15    # Demi-angle du secteur de détection collision (degrés).
                             # Plage : 10–45. Ne pas modifier sans tester.

# -----------------------------------------------------------------------------
# Boucle de contrôle
# -----------------------------------------------------------------------------

CONTROL_HZ = 50              # Fréquence de la boucle de conduite (Hz).
                             # Ne pas modifier.

DT_S = 1.0 / CONTROL_HZ     # Période de la boucle en secondes. Calculé automatiquement.

# -----------------------------------------------------------------------------
# Vitesse
# -----------------------------------------------------------------------------

VITESSE_MS = 0.30             # Vitesse de croisière (m/s). Plage : 0.3–0.8.
                             # Commencer à 0.3 le jour des essais, augmenter si stable.

VITESSE_MIN = 0.25          # Vitesse plancher (m/s). Ne jamais descendre en dessous.
                             # En dessous de ~0.25 le moteur n'a plus de couple.

VITESSE_MAX_MS = 2.0         # Limite logicielle absolue. Ne pas dépasser.

# -----------------------------------------------------------------------------
# Détection blocage et recul
# -----------------------------------------------------------------------------

STUCK_DIST_MM = 500          # Distance frontale (mm) en dessous de laquelle on
                             # incrémente le compteur de blocage.
                             # Plage : 200–800.

STUCK_TICKS = 6             # Nombre de ticks consécutifs avant de déclencher
                             # la manœuvre de recul. À 50 Hz, 6 = ~0.12 s.
                             # Plage : 5–50.

STUCK_AV_ZERO_TICKS = 15    # Nombre de ticks consécutifs avec av=0 (pas de
                             # mesure devant) ET murs latéraux proches → bloqué.
                             # À 50 Hz, 15 = 0.3 s. Plage : 10–30.

ESCAPE_TICKS = 40            # Durée de la phase avant post-recul (ticks). 25→40.

ESCAPE_COOLDOWN_TICKS = 50   # Ticks de cooldown après la phase escape avant que
                             # la détection de blocage ne se réactive. 50 = 1 s à 50 Hz.

ESCAPE_CANCEL_DIST_MM = 200  # Annuler la phase escape seulement si obstacle frontal < cette distance (mm).

# -----------------------------------------------------------------------------
# Capteurs Additionnels
# -----------------------------------------------------------------------------
SONAR_ACTIF = True            # mettre False si le sonar n'est pas branché
SONAR_ARRIERE_SEUIL_CM = 25   # distance minimale arrière avant d'arrêter le recul.

# -----------------------------------------------------------------------------
# Vitesse adaptative proportionnelle
# -----------------------------------------------------------------------------
# Vitesse adaptative. Si K_V > 0, la logique de ralentissement par COLLISION_DIST_MM dans main.py est inactive.
# Mettre K_V = 0.0 pour revenir à la logique COLLISION_DIST_MM.
K_V = 0.0                    # Gain pour mode de vitesse adaptatif basé sur la distance frontale

# -----------------------------------------------------------------------------
# Validations de sécurité (résistantes à python3 -O)
# -----------------------------------------------------------------------------
if not (0.1 <= VITESSE_MS <= 0.8):
    raise ValueError(f"VITESSE_MS hors plage [0.1, 0.8] : {VITESSE_MS}")
if not (400 <= COLLISION_DIST_MM <= 2000):
    raise ValueError(f"COLLISION_DIST_MM hors plage [400, 2000] : {COLLISION_DIST_MM}")
if not (200 <= STUCK_DIST_MM <= 800):
    raise ValueError(f"STUCK_DIST_MM hors plage [200, 800] : {STUCK_DIST_MM}")
if not (VITESSE_MIN < VITESSE_MS):
    raise ValueError(f"VITESSE_MIN ({VITESSE_MIN}) doit être < VITESSE_MS ({VITESSE_MS})")
if not (ESC_DUTY_REV_START < ESC_DUTY_NEUTRAL):
    raise ValueError(f"REV_START ({ESC_DUTY_REV_START}) doit être < NEUTRAL ({ESC_DUTY_NEUTRAL})")
if not (COLLISION_DIST_MM < D_MIN_MM):
    raise ValueError(f"COLLISION_DIST_MM ({COLLISION_DIST_MM}) doit être < D_MIN_MM ({D_MIN_MM}) — sinon ZeroDivisionError dans main.py")
if VITESSE_MIN < 0.0:
    raise ValueError(f"VITESSE_MIN doit être >= 0.0 : {VITESSE_MIN}")
if CONTROL_HZ <= 0:
    raise ValueError(f"CONTROL_HZ doit être > 0 : {CONTROL_HZ}")

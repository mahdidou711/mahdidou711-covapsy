# ~/covapsy/cal_lidar.py
# Calibrage lidar — validation connexion, distances et paramètres.
# Usage : python3 cal_lidar.py
# Ctrl+C pour quitter.

import time
import sys
from lidar_thread import Lidar360
import config as cfg

REFRESH_HZ = 4          # Rafraîchissement affichage (Hz)
REFRESH_S  = 1.0 / REFRESH_HZ

# Secteurs à afficher (degrés, convention : 0=avant, 90=gauche, 270=droite)
SECTORS = [
    ("AVANT",   list(range(350, 360)) + list(range(0, 11))),   # ±10°
    ("GAUCHE",  list(range(80, 101))),                          # 80°–100°
    ("DROITE",  list(range(260, 281))),                         # 260°–280°
]


def min_sector(scan, indices):
    vals = [scan[i] for i in indices if scan[i] > 0]
    return min(vals) if vals else 0


def scan_rate(timestamps, window_s):
    now = time.monotonic()
    recent = [t for t in timestamps if now - t <= window_s]
    return len(recent) / window_s if len(recent) > 1 else 0.0


print("=== Calibrage Lidar — RPLidar A2M12 ===")
print(f"Port     : {cfg.PORT}")
print(f"Baudrate : {cfg.BAUDRATE}")
print(f"LIDAR_FRESH_MAX_S : {cfg.LIDAR_FRESH_MAX_S} s")
print(f"D_MIN_MM          : {cfg.D_MIN_MM} mm  (seuil FTG obstacle)")
print(f"COLLISION_DIST_MM : {cfg.COLLISION_DIST_MM} mm  (seuil urgence)")
print()
print("Démarrage du lidar...")

lidar = Lidar360(port=cfg.PORT, baudrate=cfg.BAUDRATE)
lidar.start()

# Attendre le premier scan
t_wait = time.monotonic()
while True:
    scan_id, scan_ts, scan = lidar.get_latest_scan()
    if scan_id > 0:
        break
    if time.monotonic() - t_wait > 8.0:
        print("ERREUR : aucun scan reçu en 8s. Vérifier connexion USB et port.")
        lidar.stop()
        sys.exit(1)
    time.sleep(0.1)

print("Lidar connecté. Affichage en direct (Ctrl+C pour quitter).\n")

scan_timestamps = []
last_id = 0

try:
    while True:
        scan_id, scan_ts, scan = lidar.get_latest_scan()

        if scan_id != last_id:
            scan_timestamps.append(scan_ts)
            # Garder seulement la fenêtre utile
            cutoff = time.monotonic() - cfg.LIDAR_RATE_WIN_S - 1.0
            scan_timestamps = [t for t in scan_timestamps if t > cutoff]
            last_id = scan_id

        age_s = time.monotonic() - scan_ts
        fresh = age_s <= cfg.LIDAR_FRESH_MAX_S
        rate  = scan_rate(scan_timestamps, cfg.LIDAR_RATE_WIN_S)

        # Distances par secteur
        avant  = min_sector(scan, SECTORS[0][1])
        gauche = min_sector(scan, SECTORS[1][1])
        droite = min_sector(scan, SECTORS[2][1])

        # Collision urgence : secteur avant ±COLLISION_SECTOR_DEG
        coll_idx = list(range(360 - cfg.COLLISION_SECTOR_DEG, 360)) + \
                   list(range(0, cfg.COLLISION_SECTOR_DEG + 1))
        coll_dist = min_sector(scan, coll_idx)
        collision = coll_dist > 0 and coll_dist < cfg.COLLISION_DIST_MM

        print("\033[H\033[J", end="")  # clear terminal
        print("=== Calibrage Lidar — RPLidar A2M12 ===")
        print(f"Taux de scan : {rate:.1f} tours/s  (fenêtre {cfg.LIDAR_RATE_WIN_S}s)")
        print(f"Âge scan     : {age_s * 1000:.0f} ms  "
              f"{'[FRAIS]' if fresh else '[TROP VIEUX — augmenter LIDAR_FRESH_MAX_S]'}")
        print(f"Scan ID      : {scan_id}")
        print()
        print(f"  AVANT  (±10°)  : {avant:5d} mm", end="")
        if avant > 0 and avant < cfg.D_MIN_MM:
            print(f"  ← obstacle FTG (< D_MIN_MM={cfg.D_MIN_MM})", end="")
        print()
        print(f"  GAUCHE (±10°)  : {gauche:5d} mm")
        print(f"  DROITE (±10°)  : {droite:5d} mm")
        print()
        print(f"  Secteur collision ±{cfg.COLLISION_SECTOR_DEG}° : {coll_dist:5d} mm", end="")
        if collision:
            print(f"  *** COLLISION URGENCE *** (< {cfg.COLLISION_DIST_MM} mm)", end="")
        print()
        print()
        print("Paramètres à ajuster si nécessaire :")
        print(f"  D_MIN_MM={cfg.D_MIN_MM}          Augmenter si la voiture frôle les murs")
        print(f"  COLLISION_DIST_MM={cfg.COLLISION_DIST_MM}   Augmenter si réaction trop tardive")
        print(f"  LIDAR_FRESH_MAX_S={cfg.LIDAR_FRESH_MAX_S}  Augmenter si scans rejetés fréquemment")
        print()
        print("Ctrl+C pour quitter.")

        time.sleep(REFRESH_S)

except KeyboardInterrupt:
    pass
finally:
    print("\nArrêt lidar...")
    lidar.stop()
    print("Terminé.")

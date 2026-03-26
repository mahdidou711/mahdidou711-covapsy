# ~/covapsy/main.py
# Boucle de conduite principale — CoVAPSy 2026.
# Orchestre lidar, actuators et algorithme FTG.
#
# Machine à états :
#   AVANCE : conduite normale, FTG calcule l'angle, vitesse de croisière.
#   EVITE  : phase avant post-recul, braquage forcé vers le côté le plus dégagé.
#
# Ordre de démarrage :
#   1. Lidar360 (thread d'acquisition)
#   2. LidarConsumer (lecture non-bloquante)
#   3. Actuators (PWM servo + ESC)
#   4. Attente 2 s (lidar prêt)
#   5. Boucle principale à CONTROL_HZ

import time
import signal
import sys
import threading
import csv
import config
from lidar_thread import Lidar360
from lidar_consumer import LidarConsumer
from actuators import Actuators
from ftg import compute_ftg, detect_collision

sonar_module = None
if getattr(config, "SONAR_ACTIF", False):
    import sonar as sonar_module


def main():
    # ── Création et démarrage des composants ──────────────────────────
    lidar    = Lidar360(config.PORT, config.BAUDRATE)
    consumer = LidarConsumer(lidar)
    act      = Actuators()

    stop_event = threading.Event()
    lidar.start()

    if sonar_module:
        t_sonar = threading.Thread(
            target=sonar_module.sonar_thread_func,
            args=(stop_event,),
            daemon=True
        )
        t_sonar.start()

    # ── Gestionnaire de signal propre ─────────────────────────────────
    # SIGINT  = Ctrl+C depuis le terminal.
    # SIGTERM = kill depuis un autre process (ex: systemd, script de supervision).
    # Les deux doivent couper proprement les PWM et le lidar pour éviter
    # que le moteur reste alimenté ou que le servo reste braqué.
    def shutdown(signum, frame):
        stop_event.set()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    # ── try/finally global ─────────────────────────────────────────────
    # Garantit l'arrêt propre des actuateurs et du lidar quoi qu'il arrive
    # (Ctrl+C, signal, exception). Le signal handler reste en place pour
    # un arrêt immédiat, le finally couvre les cas restants.
    try:
        # Initialisation du log CSV — ouvert ici pour que le finally puisse
        # toujours appeler log_file.close(), même si une exception survient
        # pendant le time.sleep(2) qui suit.
        log_file = open("run_log.csv", "w", newline="")
        log_writer = csv.writer(log_file)
        log_writer.writerow(["t", "front_min", "angle", "vitesse", "etat", "stuck"])

        # ── Attente démarrage lidar ─────────────────────────────────────
        # Le lidar met ~1 s à démarrer le moteur et produire des scans valides.
        # On attend 2 s par sécurité pour ne pas démarrer la conduite sur des
        # scans vides ou incomplets.
        time.sleep(2)
        print("Lidar démarré, début de la conduite.")

        # ── Boucle principale ───────────────────────────────────────────
        etat = "AVANCE"
        ticks_sans_scan = 0
        # Compteur de blocage : incrémenté quand la voiture semble coincée.
        stuck_count = 0
        stuck_av_zero = 0
        # Après un recul, phase avant forcée avec braquage escape_angle.
        escape_ticks = 0
        escape_angle = 0.0
        # Après la phase avant, cooldown pour ne pas re-déclencher stuck.
        escape_cooldown = 0

        while not stop_event.is_set():
            t0 = time.monotonic()
            angle = 0.0
            vitesse = 0.0

            # ── Fail-safe perte lidar ───────────────────────────────────
            # Lecture non-bloquante du dernier scan disponible.
            fresh, sid, age, scan, rate = consumer.poll()

            if not fresh:
                ticks_sans_scan += 1
                # Seuil atteint : couper propulsion et recentrer le servo.
                # Le warning n'est affiché qu'une seule fois (== 15, pas >= 15)
                # pour ne pas inonder le terminal.
                if ticks_sans_scan == 15:
                    act.set_vitesse(0)
                    act.set_direction(0)
                    print("[SECURITE] Lidar périmé, arrêt propulsion.")
            else:
                ticks_sans_scan = 0

            # Décrémenter les ticks d'échappement indépendamment du lidar
            if escape_ticks > 0:
                escape_ticks -= 1

            if fresh and scan is not None:

                # Distance frontale minimale (secteur ±COLLISION_SECTOR_DEG).
                front_min = None
                for i in list(range(0, config.COLLISION_SECTOR_DEG + 1)) + \
                         list(range(360 - config.COLLISION_SECTOR_DEG, 360)):
                    d = scan[i]
                    if d > 0 and (front_min is None or d < front_min):
                        front_min = d

                # Bug fix : si front_min=None (av=0, lidar ne mesure rien
                # devant = trop proche), vérifier les murs latéraux via un secteur
                # au lieu d'un simple point (plus robuste au bruit).
                front_blocked = False
                if front_min is None:
                    left_sector = [scan[i] for i in range(85, 96) if scan[i] > 0]
                    right_sector = [scan[i] for i in range(265, 276) if scan[i] > 0]

                    left_d = min(left_sector) if left_sector else 100
                    right_d = min(right_sector) if right_sector else 100

                    if left_d < 1000 or right_d < 1000:
                        front_blocked = True

                # Détection d'état prolongé sans vision frontale utile
                if front_blocked:
                    stuck_av_zero += 1
                else:
                    stuck_av_zero = 0

                # Check de collision critique pendant escape.
                # On n'annule QUE si la distance frontale est CONFIRMÉE < cancel_dist.
                # front_blocked (av=0) = distance inconnue mais < 200 mm datasheet :
                # on ne sait pas si c'est 10 mm ou 190 mm → ne pas annuler,
                # mais ne pas avancer non plus (voir ci-dessous).
                cancel_dist = getattr(config, "ESCAPE_CANCEL_DIST_MM", 200)
                if escape_ticks > 0 and (front_min is not None and front_min < cancel_dist):
                    print(f"[SECURITE] Mur frontal ({front_min}mm) détecté pendant escape -> annulation escape.")
                    escape_ticks = 0

                # ── Phase avant forcée post-recul ─────────────────────
                # Après le recul, on avance avec l'angle INVERSE pour se recentrer dans la piste
                # sans refaire le même mur.
                if escape_ticks > 0:
                    escape_forward_angle = -escape_angle
                    print(f"av={scan[0]:4d} ga={scan[90]:4d} dr={scan[270]:4d} | ESCAPE {escape_ticks} {escape_forward_angle:+.0f}°")
                    act.set_direction(escape_forward_angle)
                    if front_blocked:
                        # Secteur avant aveugle (< 200 mm) : on braque mais on n'avance pas.
                        # Les ticks se décomptent quand même → sortie propre de l'escape.
                        act.set_vitesse(0.0)
                    else:
                        act.set_vitesse(config.VITESSE_MIN)
                else:
                    angle = compute_ftg(scan, config.D_MIN_MM, config.W_MIN_PTS, config.K_FTG,
                                        sector_deg=config.FTG_SECTOR_DEG,
                                        steer_limit_deg=config.STEER_ANGLE_MAX_DEG)

                    # Vitesse proportionnelle à la distance frontale.
                    if front_blocked:
                        vitesse = 0.0  # secteur avant entièrement aveugle (obstacles < 200 mm) → arrêt complet
                        print(f"av={scan[0]:4d} ga={scan[90]:4d} dr={scan[270]:4d} | BLOCKED v={vitesse:.2f}")
                    else:
                        # Vitesse adaptative expérimentale avec K_V si config.K_V est activé (> 0, différent de None)
                        k_v = getattr(config, "K_V", None)
                        if k_v is not None and k_v > 0.0:  # K_V > 0 : vitesse adaptative active — la logique COLLISION_DIST_MM ci-dessous est inactive tant que K_V > 0
                            dist_vitesse = front_min if front_min is not None else 0
                            v_calcule = k_v * (dist_vitesse / 1000.0)
                            vitesse = min(config.VITESSE_MS, max(config.VITESSE_MIN, v_calcule))
                            print(f"av={scan[0]:4d} ga={scan[90]:4d} dr={scan[270]:4d} | K_V v={vitesse:.2f}")
                        else:
                            # Logique originelle
                            if front_min is not None and front_min < config.D_MIN_MM:
                                ratio = max(0.0, (front_min - config.COLLISION_DIST_MM)) / \
                                        (config.D_MIN_MM - config.COLLISION_DIST_MM)
                                ratio = min(1.0, max(0.0, ratio))
                                vitesse = config.VITESSE_MIN + (config.VITESSE_MS - config.VITESSE_MIN) * ratio
                                print(f"av={scan[0]:4d} ga={scan[90]:4d} dr={scan[270]:4d} | ftg={angle:+.1f}° v={vitesse:.2f}")
                            else:
                                vitesse = config.VITESSE_MS
                                print(f"av={scan[0]:4d} ga={scan[90]:4d} dr={scan[270]:4d} | ftg={angle:+.1f}°")

                    act.set_direction(angle)
                    act.set_vitesse(vitesse)

                # ── Détection de blocage ──────────────────────────────
                # Cooldown après recul pour ne pas re-déclencher immédiatement.
                if escape_cooldown > 0:
                    escape_cooldown -= 1
                    stuck_count = 0
                    stuck_av_zero = 0
                elif front_min is not None and front_min < config.STUCK_DIST_MM:
                    stuck_count += 1
                else:
                    stuck_count = max(0, stuck_count - 1)  # Décroissance douce (hystérésis)

                if stuck_count >= config.STUCK_TICKS or stuck_av_zero >= config.STUCK_AV_ZERO_TICKS:
                    # Choisir le côté le plus dégagé pour le braquage EN RECULANT.
                    # On utilise à nouveau le min sur secteurs pour plus de robustesse.
                    left_sector = [scan[i] for i in range(85, 96) if scan[i] > 0]
                    right_sector = [scan[i] for i in range(265, 276) if scan[i] > 0]
                    left_d = min(left_sector) if left_sector else 100
                    right_d = min(right_sector) if right_sector else 100

                    if left_d > right_d:
                        # Si l'obstacle est principalement à droite, pour se redresser lors d'un virage:
                        # On braque vers l'obstacle (à droite) en reculant
                        # pour éloigner l'avant du mur
                        escape_angle = config.STEER_ANGLE_MAX_DEG
                    else:
                        escape_angle = -config.STEER_ANGLE_MAX_DEG

                    print(f"[RECUL] Bloqué → recul {config.T_REVERSE_S}s braquage {escape_angle:+.0f}°")
                    # Braquer AVANT de reculer → servo tourne pendant tout le recul.
                    act.set_vitesse(0)
                    act.reculer(config.T_REVERSE_S, force_angle_deg=escape_angle)

                    # Phase avant forcée avec même braquage
                    escape_ticks = config.ESCAPE_TICKS
                    escape_cooldown = config.ESCAPE_COOLDOWN_TICKS
                    stuck_count = 0
                    stuck_av_zero = 0

                # Ligne de log — etat est un label pour le CSV uniquement.
                etat = "EVITE" if escape_ticks > 0 else "AVANCE"
                current_angle = escape_angle if escape_ticks > 0 else angle
                current_vitesse = config.VITESSE_MIN if escape_ticks > 0 else vitesse
                log_writer.writerow([round(time.monotonic(), 3), front_min if front_min is not None else -1, round(current_angle, 1), round(current_vitesse, 3), etat, stuck_count])
                log_file.flush()

            # Régulation de fréquence : on calcule le temps restant avant le
            # prochain tick pour maintenir CONTROL_HZ (50 Hz = 20 ms par tick).
            # Si le traitement a pris plus que DT_S, on ne dort pas et on
            # enchaîne immédiatement le tick suivant.
            dt_ecoule = time.monotonic() - t0
            dt_restant = config.DT_S - dt_ecoule
            if dt_restant > 0:
                time.sleep(dt_restant)

    finally:
        stop_event.set()
        act.stop()
        lidar.stop()
        try:
            log_file.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()

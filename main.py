"""Boucle principale de conduite CoVAPSy."""

from __future__ import annotations  # annotations différées pour le typage

import csv  # journal de roulage
import signal  # arrêt propre sur SIGINT/SIGTERM
import threading  # thread sonar
import time  # horodatage et temporisation
from typing import Iterable, Optional  # types des helpers

import config  # paramètres actifs du runtime
from actuators import Actuators  # servo + ESC + recul bloquant
from lidar_consumer import LidarConsumer  # lecture non bloquante du scan
from lidar_thread import Lidar360  # thread d'acquisition lidar
from navigation import calculer_direction, calculer_vitesse, get_lateral_means  # navigation pure active

sonar_module = None  # import conditionnel du sonar arrière
if getattr(config, "SONAR_ACTIF", False):
    import sonar as sonar_module


def _iter_front_indices(half_angle_deg: int) -> Iterable[int]:
    """Indices du secteur frontal avec rebouclage 360°."""
    for idx in range(360 - half_angle_deg, 360):  # partie gauche du secteur frontal
        yield idx  # indices 360-N ... 359
    for idx in range(0, half_angle_deg + 1):  # partie droite du secteur frontal
        yield idx  # indices 0 ... N


def _min_valid_distance(scan: list[int], indices: Iterable[int]) -> Optional[int]:
    """Distance minimale > 0 sur une liste d'indices."""
    minimum = None  # aucune mesure valide trouvée au départ
    for idx in indices:  # parcours des indices utiles
        dist = scan[idx]  # distance lidar brute en mm
        if dist > 0 and (minimum is None or dist < minimum):
            minimum = dist  # conserve la plus petite distance valide
    return minimum  # None si tout est à 0


def _compute_front_min(scan: Optional[list[int]], half_angle_deg: Optional[int] = None) -> Optional[int]:
    """Distance minimale dans le secteur frontal."""
    if scan is None:
        return None  # pas de scan disponible
    if half_angle_deg is None:
        half_angle_deg = config.FRONT_SECTOR_HALF  # demi-angle frontal par défaut
    return _min_valid_distance(scan, _iter_front_indices(int(half_angle_deg)))  # min devant


def _write_log(
    log_writer: csv.writer,
    log_file,
    moyenne_gauche: Optional[float],
    moyenne_droite: Optional[float],
    fresh: bool,
    ticks_sans_scan: int,
    front_min: Optional[int],
    angle: float,
    vitesse: float,
):
    """Écrit une ligne CSV et force le flush."""
    log_writer.writerow(
        [
            round(time.monotonic(), 3),  # temps monotonic
            round(moyenne_gauche, 3) if moyenne_gauche is not None else -1,  # moyenne latérale gauche
            round(moyenne_droite, 3) if moyenne_droite is not None else -1,  # moyenne latérale droite
            round(angle, 3),  # angle commandé
            round(vitesse, 3),  # vitesse commandée
            front_min if front_min is not None else -1,  # -1 si avant vide
            int(fresh),  # 1 si le scan de ce tick est frais
            ticks_sans_scan,  # ticks depuis le dernier scan frais
        ]
    )
    log_file.flush()  # écrit immédiatement sur disque


def main():
    """Point d'entrée runtime."""
    lidar = Lidar360(config.PORT, config.BAUDRATE)  # thread lidar
    consumer = LidarConsumer(lidar)  # consommateur non bloquant
    act = Actuators()  # sorties servo/ESC
    stop_event = threading.Event()  # arrêt demandé

    lidar.start()  # démarrage acquisition

    if sonar_module:
        threading.Thread(
            target=sonar_module.sonar_thread_func,  # boucle sonar arrière
            args=(stop_event,),  # partage du stop_event
            daemon=True,  # ne bloque pas la sortie
        ).start()  # lancement du thread sonar

    def shutdown(signum, frame):
        stop_event.set()  # demande d'arrêt propre

    signal.signal(signal.SIGINT, shutdown)  # Ctrl+C
    signal.signal(signal.SIGTERM, shutdown)  # arrêt externe

    log_file = None  # défini avant le try pour le finally
    try:
        log_file = open("run_log.csv", "w", newline="")  # log simple par tick
        log_writer = csv.writer(log_file)  # writer CSV
        log_writer.writerow(
            ["t", "moyenne_gauche", "moyenne_droite", "angle", "vitesse", "front_min", "fresh", "ticks_sans_scan"]
        )  # en-tête CSV

        time.sleep(2.0)  # laisse le lidar publier ses premiers scans
        print("Lidar démarré, début de la conduite.")

        stuck_count = 0  # compteur actif uniquement si STUCK_RECOVERY_ACTIVE
        cooldown = 0  # immunité post-recul
        ticks_sans_scan = 0  # fail-safe lidar
        last_scan = None  # dernier scan exploitable
        angle_commande = 0.0  # angle loggé
        vitesse_commandee = 0.0  # vitesse loggée
        lidar_timeout_reported = False  # évite de spammer le message de sécurité

        while not stop_event.is_set():
            t0 = time.monotonic()  # début du tick courant
            fresh, _sid, _age, fresh_scan, _rate = consumer.poll()  # lecture du dernier scan

            if fresh and fresh_scan is not None:
                last_scan = fresh_scan  # mémorise le dernier scan frais
                ticks_sans_scan = 0  # reset du fail-safe
                lidar_timeout_reported = False  # un scan frais réarme le message de sécurité
            else:
                ticks_sans_scan += 1  # scan frais manquant

            scan = last_scan if config.LIDAR_REUSE_LAST_SCAN else fresh_scan  # stratégie de repli
            lidar_perime = scan is None or ticks_sans_scan >= config.LIDAR_TIMEOUT_TICKS  # plus de scan fiable
            front_min = _compute_front_min(scan)  # min utile devant, y compris sur scan mémorisé
            moyenne_gauche, moyenne_droite = get_lateral_means(scan)  # métriques de réglage terrain

            angle_commande = 0.0  # valeur par défaut de sécurité
            vitesse_commandee = 0.0  # valeur par défaut de sécurité

            if lidar_perime:
                act.set_vitesse(0.0)  # arrêt propulsion si lidar perdu
                act.set_direction(0.0)  # roues droites en sécurité
                if not lidar_timeout_reported:
                    print("[SECURITE] Lidar périmé, arrêt propulsion.")
                    lidar_timeout_reported = True
            else:
                recovery_triggered = False  # pas de recul sur ce tick par défaut

                if config.STUCK_RECOVERY_ACTIVE:
                    if cooldown > 0:
                        cooldown -= 1  # décrémente l'immunité post-recul
                        stuck_count = 0  # pas de détection pendant le cooldown
                    else:
                        if front_min is not None and front_min < config.STUCK_DIST_MM:
                            stuck_count += 1  # obstacle frontal persistant
                        else:
                            stuck_count = max(0, stuck_count - 1)  # décrémentation douce

                    if stuck_count >= config.STUCK_TICKS:
                        print("[STUCK] Recul déclenché")
                        act.set_vitesse(0.0)  # arrêt avant le recul
                        angle_recul = act.reculer(scan=last_scan)  # recul bloquant avec choix d'angle
                        print(f"[STUCK] Recul terminé, angle={angle_recul:+.1f}°")
                        cooldown = config.STUCK_COOLDOWN_TICKS  # immunité post-recul
                        stuck_count = 0  # reset du compteur
                        recovery_triggered = True  # aucune commande AV sur ce tick

                if not recovery_triggered:
                    angle_commande = calculer_direction(scan, config.NAV_K, config.NAV_EPS)  # direction active
                    vitesse_commandee = calculer_vitesse(scan, front_min)  # vitesse active
                    act.set_direction(angle_commande)  # applique le braquage
                    act.set_vitesse(vitesse_commandee)  # applique la propulsion

            _write_log(
                log_writer,
                log_file,
                moyenne_gauche,
                moyenne_droite,
                fresh,
                ticks_sans_scan,
                front_min,
                angle_commande,
                vitesse_commandee,
            )  # CSV

            dt_ecoule = time.monotonic() - t0  # durée réelle du tick
            dt_restant = config.DT_S - dt_ecoule  # régulation à 50 Hz
            if dt_restant > 0:
                time.sleep(dt_restant)  # attend le prochain tick

    finally:
        stop_event.set()  # demande l'arrêt aux threads
        act.stop()  # coupe servo et ESC
        lidar.stop()  # arrête le thread lidar
        if log_file:
            try:
                log_file.close()  # ferme le CSV
            except Exception:
                pass


if __name__ == "__main__":
    main()  # lancement direct en script

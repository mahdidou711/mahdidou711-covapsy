# ~/covapsy/lidar_thread.py
# Module d'acquisition lidar en thread dédié.
# Produit un tableau scan[360] (distances en mm, 1 case par degré).
#
# Convention de mapping angle → indice :
#   idx = int(round(angle)) % 360
#
# Ce que contient le tableau :
#   scan[0]   = avant    (angle lidar 0°)
#   scan[90]  = gauche   (angle lidar 90°)
#   scan[270] = droite   (angle lidar 270°)

import threading
import time
from typing import List, Optional, Tuple
from rplidar import RPLidar, RPLidarException


class Lidar360:
    """
    Thread acquisition basé sur la logique du code prof:
    - iter_scans() fournit des tours
    - mapping angle -> index: idx = int(round(angle)) % 360
    - publication scan stable à chaque tour

    Convention physique :
      scan[0]   = avant    (angle lidar 0°)
      scan[90]  = gauche   (angle lidar 90°)
      scan[270] = droite   (angle lidar 270°)
    """

    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 256000):
        self.port = port
        self.baudrate = baudrate

        self._th: Optional[threading.Thread] = None
        self._stop_evt = threading.Event()

        # _scan : tableau 360 cases, distances en mm (0 = pas de mesure).
        # _scan_id : compteur incrémenté à chaque nouveau tour publié,
        #            permet au consumer de détecter un nouveau scan.
        # _scan_ts : timestamp de publication, permet de calculer l'âge du scan.
        self._scan: List[int] = [0] * 360
        self._scan_id: int = 0
        self._scan_ts: float = 0.0

        # Lock protège _scan, _scan_id, _scan_ts, _running, _last_error
        # contre les accès concurrents thread lidar / thread principal.
        self._lock = threading.Lock()
        self._running: bool = False
        self._reconnects: int = 0
        self._last_error: str = ""

        self._sleep_on_error_s = 0.5

    def start(self):
        if self._th is not None:
            return
        self._stop_evt.clear()
        # daemon=True : le thread meurt automatiquement si le programme principal s'arrête,
        # pas besoin de join explicite en cas de crash.
        self._th = threading.Thread(target=self._run, daemon=True)
        self._th.start()

    def stop(self):
        self._stop_evt.set()
        if self._th is not None:
            # timeout=3.0 : évite de bloquer indéfiniment si le thread est coincé
            # dans une lecture série. Après 3s on considère le thread perdu.
            self._th.join(timeout=3.0)
        self._th = None

    def status(self) -> Tuple[bool, int, str]:
        with self._lock:
            return self._running, self._reconnects, self._last_error

    def get_latest_scan(self) -> Tuple[int, float, List[int]]:
        with self._lock:
            # list() crée une copie : le consumer peut lire/modifier le tableau
            # sans risque que le thread lidar l'écrase en pleine lecture.
            return self._scan_id, self._scan_ts, list(self._scan)

    def _publish(self, scan_mm: List[int]):
        with self._lock:
            self._scan = list(scan_mm)
            # Incrémenter scan_id permet au consumer de détecter qu'un nouveau
            # tour complet est disponible (comparaison avec son _last_id).
            self._scan_id += 1
            self._scan_ts = time.monotonic()

    def _set_err(self, msg: str):
        with self._lock:
            self._last_error = msg

    def _set_running(self, v: bool):
        with self._lock:
            self._running = v

    # Trois try/except séparés : si lidar.stop() plante, on veut quand même
    # tenter stop_motor() puis disconnect(). Un seul try/except sauterait
    # les étapes restantes à la première exception.
    def _cleanup(self, lidar: Optional[RPLidar]):
        try:
            if lidar is not None:
                lidar.stop()
        except Exception:
            pass
        try:
            if lidar is not None:
                lidar.stop_motor()
        except Exception:
            pass
        try:
            if lidar is not None:
                lidar.disconnect()
        except Exception:
            pass

    def _run_once(self):
        lidar = None
        try:
            lidar = RPLidar(self.port, baudrate=self.baudrate, timeout=2)

            # Comme dans le code prof: connect explicite (si dispo)
            try:
                lidar.connect()
            except Exception:
                pass

            lidar.start_motor()
            time.sleep(0.5)

            # iter_scans() regroupe les mesures par tour complet (vs iter_measures
            # qui donne les points un par un). max_buf_meas=500 évite le buffer
            # overflow, min_len=5 ignore les tours trop courts (lidar en démarrage).
            # scan_type='express' active le mode haute résolution de l'A2M12
            # (~16000 pts/s au lieu de ~5000 pts/s en mode standard).
            for scan in lidar.iter_scans(max_buf_meas=500, min_len=5, scan_type='express'):
                if self._stop_evt.is_set():
                    break

                scan_mm = [0] * 360
                for quality, angle, dist in scan:
                    # quality == 0 : point rejeté par la triangulation interne du lidar
                    # (bruit, réflexion parasite). dist <= 0 : hors portée ou erreur capteur.
                    if quality == 0 or dist <= 0:
                        continue
                    # Mapping angle lidar → indice tableau :
                    #   angle=0°   → idx=0   (avant)
                    #   angle=90°  → idx=90  (gauche)
                    #   angle=270° → idx=270 (droite)
                    idx = int(round(angle)) % 360
                    # Conserver la distance minimale quand deux mesures
                    # tombent sur le même degré (écrasement → min).
                    d = int(dist)
                    if scan_mm[idx] == 0 or d < scan_mm[idx]:
                        scan_mm[idx] = d

                self._publish(scan_mm)

        finally:
            self._cleanup(lidar)

    # Boucle de reconnexion automatique : si le lidar décroche (USB, erreur série),
    # _run_once lève une exception, on attend 0.5s puis on retente.
    def _run(self):
        self._set_running(True)
        self._set_err("")
        try:
            while not self._stop_evt.is_set():
                try:
                    self._run_once()
                # RPLidarException séparé : erreurs lidar connues (checksum, timeout série).
                except RPLidarException as e:
                    with self._lock:
                        self._reconnects += 1
                    self._set_err(f"RPLidarException: {e}")
                    time.sleep(self._sleep_on_error_s)
                # Exception générique : erreurs inattendues (USB arraché, permission, etc.).
                except Exception as e:
                    with self._lock:
                        self._reconnects += 1
                    self._set_err(f"{type(e).__name__}: {e}")
                    time.sleep(self._sleep_on_error_s)
        finally:
            self._set_running(False)

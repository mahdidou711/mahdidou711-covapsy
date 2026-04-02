"""Thread d'acquisition lidar 360."""

import threading  # thread d'acquisition
import time  # timestamps et temporisations
from typing import List, Optional, Tuple  # types internes
from rplidar import RPLidar, RPLidarException  # driver RPLidar


class Lidar360:
    """Acquisition continue avec publication d'un scan[360]."""

    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 256000):
        self.port = port  # port série du lidar
        self.baudrate = baudrate  # débit série du lidar

        self._th: Optional[threading.Thread] = None  # thread Python d'acquisition
        self._stop_evt = threading.Event()  # demande d'arrêt

        # Scan publié : 0=avant, 90=gauche, 270=droite.
        self._scan: List[int] = [0] * 360  # tableau complet des distances
        self._scan_id: int = 0  # compteur de scans publiés
        self._scan_ts: float = 0.0  # date de publication du dernier scan

        # Lock partagé entre le thread lidar et le consumer.
        self._lock = threading.Lock()  # protège les champs partagés
        self._running: bool = False  # état du thread
        self._reconnects: int = 0  # nombre de reconnexions
        self._last_error: str = ""  # dernier message d'erreur

        self._sleep_on_error_s = 0.5  # pause avant reconnexion

    def start(self):
        if self._th is not None:
            return  # déjà lancé
        self._stop_evt.clear()  # reset du flag d'arrêt
        # daemon=True pour ne pas bloquer la sortie du programme.
        self._th = threading.Thread(target=self._run, daemon=True)  # thread worker
        self._th.start()  # démarrage effectif

    def stop(self):
        self._stop_evt.set()  # demande la sortie de la boucle
        if self._th is not None:
            # Timeout pour éviter un arrêt bloqué sur le port série.
            self._th.join(timeout=3.0)  # attente bornée du thread
        self._th = None  # thread libéré

    def status(self) -> Tuple[bool, int, str]:
        with self._lock:
            return self._running, self._reconnects, self._last_error  # snapshot thread-safe

    def get_latest_scan(self) -> Tuple[int, float, List[int]]:
        with self._lock:
            # Copie défensive du scan publié.
            return self._scan_id, self._scan_ts, list(self._scan)  # copie du scan courant

    def _publish(self, scan_mm: List[int]):
        with self._lock:
            self._scan = list(scan_mm)  # remplace le scan courant
            # Incrémente à chaque tour complet publié.
            self._scan_id += 1  # nouveau scan disponible
            self._scan_ts = time.monotonic()  # timestamp de publication

    def _set_err(self, msg: str):
        with self._lock:
            self._last_error = msg  # mémorise la dernière erreur

    def _set_running(self, v: bool):
        with self._lock:
            self._running = v  # expose l'état du thread

    def _cleanup(self, lidar: Optional[RPLidar]):
        try:
            if lidar is not None:
                lidar.stop()  # stop des scans côté driver
        except Exception:
            pass
        try:
            if lidar is not None:
                lidar.stop_motor()  # arrêt moteur lidar
        except Exception:
            pass
        try:
            if lidar is not None:
                lidar.disconnect()  # fermeture du port
        except Exception:
            pass

    def _run_once(self):
        lidar = None  # instance driver locale à ce cycle
        try:
            lidar = RPLidar(self.port, baudrate=self.baudrate, timeout=2)  # ouverture du driver

            # Connexion explicite si le driver la demande.
            try:
                lidar.connect()  # certains drivers exigent connect()
            except Exception:
                pass  # tolère les drivers déjà connectés

            lidar.start_motor()  # démarre la rotation
            time.sleep(0.5)  # laisse le moteur monter en régime

            # Un tour complet est publié à chaque itération.
            for scan in lidar.iter_scans(max_buf_meas=500, min_len=5, scan_type='express'):
                if self._stop_evt.is_set():
                    break  # sortie demandée

                scan_mm = [0] * 360  # nouveau tour vide
                for quality, angle, dist in scan:
                    # Ignore les mesures invalides.
                    if quality == 0 or dist <= 0:
                        continue
                    # Miroir gauche/droite pour retrouver la convention runtime :
                    # 0 = avant, 90 = gauche, 270 = droite sur la voiture.
                    idx = (-int(round(angle))) % 360  # angle lidar -> indice scan
                    d = int(dist)  # distance entière en mm
                    if scan_mm[idx] == 0 or d < scan_mm[idx]:
                        scan_mm[idx] = d  # garde la plus petite mesure par degré

                self._publish(scan_mm)  # rend le scan visible au consumer

        finally:
            self._cleanup(lidar)  # nettoyage même en cas d'erreur

    def _run(self):
        self._set_running(True)  # thread déclaré actif
        self._set_err("")  # reset de l'erreur courante
        try:
            while not self._stop_evt.is_set():
                try:
                    self._run_once()  # tente un cycle complet d'acquisition
                except RPLidarException as e:
                    with self._lock:
                        self._reconnects += 1  # compteur de reconnect
                    self._set_err(f"RPLidarException: {e}")  # erreur driver connue
                    time.sleep(self._sleep_on_error_s)  # attend avant retry
                except Exception as e:
                    with self._lock:
                        self._reconnects += 1  # compteur de reconnect
                    self._set_err(f"{type(e).__name__}: {e}")  # erreur générique
                    time.sleep(self._sleep_on_error_s)  # attend avant retry
        finally:
            self._set_running(False)  # thread déclaré arrêté

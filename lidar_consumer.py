"""Lecture non bloquante des scans lidar."""

import time  # horodatage monotonic
from collections import deque  # fenêtre glissante du débit
from typing import Deque, List, Optional, Tuple  # types de retour
import config  # paramètres lidar
from lidar_thread import Lidar360  # source des scans publiés


class LidarConsumer:
    def __init__(self, lidar: Lidar360):
        self.lidar = lidar  # thread lidar source
        self._last_id: int = -1  # dernier scan déjà consommé

        self._hist: Deque[Tuple[float, int]] = deque()  # historique pour le débit
        self._rate_hz: float = 0.0  # tours/s estimés

        self._tw_s: float = float(getattr(config, "LIDAR_RATE_WIN_S", 2.0))  # fenêtre du débit

    def _update_rate(self, now: float, sid: int):
        self._hist.append((now, sid))  # ajoute le scan courant à l'historique

        t_min = now - self._tw_s  # borne basse de la fenêtre
        while self._hist and self._hist[0][0] < t_min:
            self._hist.popleft()  # enlève les points trop anciens

        if len(self._hist) >= 2:
            t0, id0 = self._hist[0]  # plus ancien point gardé
            t1, id1 = self._hist[-1]  # plus récent point gardé
            dt = t1 - t0  # durée de la fenêtre utile
            did = id1 - id0  # nombre de scans sur la fenêtre
            if dt > 0 and did >= 0:
                self._rate_hz = did / dt  # taux estimé en tours/s

    def poll(self) -> Tuple[bool, int, float, Optional[List[int]], float]:
        """Retourne fresh, id, âge, scan et taux estimé."""
        now = time.monotonic()  # temps courant
        sid, ts, scan = self.lidar.get_latest_scan()  # dernier scan publié

        if sid > 0:
            self._update_rate(now, sid)  # met à jour le débit si un scan existe

        age = now - ts if ts > 0 else 1e9  # âge infini tant qu'aucun scan n'existe

        is_new = (sid != self._last_id) and (sid > 0)  # nouveau tour disponible
        if is_new:
            self._last_id = sid  # mémorise le dernier scan vu

        fresh = is_new and (age <= config.LIDAR_FRESH_MAX_S)  # nouveau et assez récent
        if fresh:
            return True, sid, age, scan, self._rate_hz  # scan exploitable
        return False, sid, age, None, self._rate_hz  # pas d'usage de scan périmé

# ~/covapsy/lidar_consumer.py
# Interface de lecture non-bloquante des scans produits par Lidar360.
# Détecte les nouveaux scans, filtre les scans trop vieux, et estime
# le taux de scan (Hz) sur une fenêtre glissante.

import time
from collections import deque
from typing import Deque, List, Optional, Tuple
import config
from lidar_thread import Lidar360


class LidarConsumer:
    def __init__(self, lidar: Lidar360):
        self.lidar = lidar
        # _last_id : dernier scan_id vu, permet de détecter un nouveau tour
        # en comparant avec le scan_id courant de Lidar360.
        self._last_id: int = -1

        # _hist : fenêtre glissante de (timestamp, scan_id),
        # utilisée pour estimer combien de scans/s le lidar produit.
        # fenêtre glissante (time, scan_id)
        self._hist: Deque[Tuple[float, int]] = deque()
        # _rate_hz : taux de scan estimé (tours/s). Attendu ~10 Hz pour l'A2M12.
        self._rate_hz: float = 0.0

        # paramètres
        self._tw_s: float = float(getattr(config, "LIDAR_RATE_WIN_S", 2.0))

    # Fenêtre glissante : on garde les (timestamp, scan_id) des dernières _tw_s
    # secondes, on purge les plus vieux, puis on calcule le taux = nombre de
    # scans / durée de la fenêtre. Donne une estimation lissée du débit réel.
    def _update_rate(self, now: float, sid: int):
        self._hist.append((now, sid))

        # purge points trop vieux
        t_min = now - self._tw_s
        while self._hist and self._hist[0][0] < t_min:
            self._hist.popleft()

        # calc rate si fenêtre remplie
        if len(self._hist) >= 2:
            t0, id0 = self._hist[0]
            t1, id1 = self._hist[-1]
            dt = t1 - t0
            did = id1 - id0
            if dt > 0 and did >= 0:
                self._rate_hz = did / dt

    def poll(self) -> Tuple[bool, int, float, Optional[List[int]], float]:
        """
        Retourne:
        - fresh (bool): True si nouveau scan et suffisamment récent
        - scan_id
        - scan_age_s
        - scan_mm (copie) ou None
        - scan_rate_hz (estimé sur fenêtre)

        Le lidar A2M12 tourne à ~10 Hz → un scan toutes les ~100 ms.
        Un scan est considéré "frais" si age <= LIDAR_FRESH_MAX_S (0.50 s).
        """
        now = time.monotonic()
        sid, ts, scan = self.lidar.get_latest_scan()

        if sid > 0:
            self._update_rate(now, sid)

        # 1e9 si ts==0 : aucun scan n'a encore été publié par le thread lidar,
        # donc l'âge est "infini" → le scan ne sera jamais considéré frais.
        age = now - ts if ts > 0 else 1e9

        # Détection nouveau scan : scan_id différent du dernier vu ET non nul
        # (scan_id=0 signifie que le thread n'a encore rien publié).
        is_new = (sid != self._last_id) and (sid > 0)
        if is_new:
            self._last_id = sid

        # Double condition : le scan doit être à la fois NOUVEAU (pas déjà traité)
        # ET RÉCENT (age <= 0.50 s). Un vieux scan relu serait obsolète.
        fresh = is_new and (age <= config.LIDAR_FRESH_MAX_S)
        if fresh:
            return True, sid, age, scan, self._rate_hz
        # scan=None quand pas frais : évite que la boucle de contrôle utilise
        # des données périmées pour prendre des décisions de braquage/vitesse.
        return False, sid, age, None, self._rate_hz

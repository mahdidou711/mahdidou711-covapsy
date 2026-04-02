"""Navigation CoVAPSy : calcul pur de direction et de vitesse."""

from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

import config


# ---------------------------------------------------------------------------
# Utilitaires internes
# ---------------------------------------------------------------------------

def _clamp(value: float, lo: float, hi: float) -> float:
    return lo if value < lo else hi if value > hi else value


def _mean_valid(values: Sequence[float]) -> Optional[float]:
    valid_values = [float(v) for v in values if v > 0]  # ignore les zéros lidar
    if not valid_values:
        return None
    return float(sum(valid_values)) / float(len(valid_values))


def _sector_mean(scan: Sequence[float], bounds: Tuple[int, int]) -> Optional[float]:
    start_idx, end_idx = bounds  # bornes inclusives
    return _mean_valid([scan[idx] for idx in range(start_idx, end_idx + 1)])


# ---------------------------------------------------------------------------
# Direction
# ---------------------------------------------------------------------------

def calculer_direction(scan: Optional[Sequence[float]], k: float, eps: float) -> float:
    """Angle de direction normalisé en degrés.

    Loi : angle = k * (G - D) / (G + D + eps), puis saturation.
    Retourne 0.0 si les deux secteurs sont invalides.
    """
    if scan is None:
        return 0.0

    g = _sector_mean(scan, config.DIR_LEFT_SECTOR)   # secteur gauche scan[30..60]
    d = _sector_mean(scan, config.DIR_RIGHT_SECTOR)  # secteur droit  scan[300..330]

    if g is None and d is None:
        return 0.0  # aucun côté exploitable

    # Neutralisation si un seul côté est valide
    if g is None:
        g = d
    elif d is None:
        d = g

    angle = k * (g - d) / (g + d + eps)
    return _clamp(angle, -config.STEER_ANGLE_MAX_DEG, config.STEER_ANGLE_MAX_DEG)


# ---------------------------------------------------------------------------
# Vitesse
# ---------------------------------------------------------------------------

def calculer_vitesse(scan: Optional[Sequence[float]], front_min: Optional[float]) -> float:
    """Vitesse cible en m/s, loi exponentielle avec terme frontal.

    front_min : distance frontale minimale en mm, fournie par main.py.
    Retourne 0.0 si le scan est invalide.
    """
    if scan is None or not any(dist > 0 for dist in scan):
        return 0.0

    s_g = _sector_mean(scan, config.SPEED_LEFT_SECTOR)   # secteur gauche vitesse
    s_d = _sector_mean(scan, config.SPEED_RIGHT_SECTOR)  # secteur droit  vitesse

    # Terme latéral
    if s_g is None and s_d is None:
        v_lat = config.VITESSE_PLANCHER
    else:
        if s_g is None:
            s_g = s_d
        elif s_d is None:
            s_d = s_g

        contraste = abs(s_g - s_d)
        contraste_max = max(s_g, s_d)
        r = contraste / (contraste_max + config.SPEED_EPS)
        v_lat = (
            config.VITESSE_PLANCHER
            + (config.VITESSE_CROISIERE - config.VITESSE_PLANCHER)
            * math.exp(-config.SPEED_ALPHA * r)
        )

    # Terme frontal
    if front_min is None or front_min <= 0:
        f_front = 1.0
    else:
        f_front = min(1.0, front_min / config.FRONT_D_REF_MM)

    v = v_lat * f_front
    return _clamp(v, 0.0, config.VITESSE_CROISIERE)


# ---------------------------------------------------------------------------
# Utilitaire log / debug
# ---------------------------------------------------------------------------

def get_lateral_means(scan: Optional[Sequence[float]]) -> Tuple[Optional[float], Optional[float]]:
    """Retourne (g, d) sur les secteurs de direction, pour le log uniquement."""
    if scan is None:
        return None, None
    g = _sector_mean(scan, config.DIR_LEFT_SECTOR)
    d = _sector_mean(scan, config.DIR_RIGHT_SECTOR)
    return g, d

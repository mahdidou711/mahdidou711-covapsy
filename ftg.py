# ~/covapsy/ftg.py
# Algorithme Follow The Gap (FTG) — navigation réactive par lidar.
# Module purement algorithmique : aucun import hardware, aucun config.
#
# Convention lidar (issue de lidar_thread.py) :
#   scan[0]   = avant
#   scan[90]  = gauche
#   scan[270] = droite
#   0 = absence de mesure, PAS un obstacle à 0 mm
#
# Convention de sortie :
#   0°   = tout droit
#   +x°  = braquer à droite (vers scan[270])
#   -x°  = braquer à gauche (vers scan[90])
#
# Expose : compute_ftg() et detect_collision().


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def compute_ftg(
    scan: list,
    d_min_mm: float,
    w_min_pts: int,
    k_ftg: float,
    sector_deg: int = None,
    steer_limit_deg: float = 18.0,
) -> float:
    """
    Calcule un angle de direction à partir d'un scan lidar 360°.

    Paramètres :
        scan            : liste 360 entiers mm (convention ci-dessus)
        d_min_mm        : distance de sécurité en mm
        w_min_pts       : largeur minimale d'un gap valide en nombre de points
        k_ftg           : gain angulaire (angle_cmd = k_ftg × angle_gap)
        sector_deg      : demi-angle de la zone avant analysée (défaut 150°)
        steer_limit_deg : angle maximal de braquage en degrés (défaut 18.0°)

    Retourne :
        angle_cmd en degrés, clampé dans [-steer_limit_deg, +steer_limit_deg].
        0.0 si aucun gap trouvé (fallback tout droit).

    Étapes :
        1. Extraire la zone avant (±sector_deg)
        2. Seuillage : libre / occupé selon d_min_mm
        3. Trouver les gaps (séquences contiguës de points libres)
        4. Sélectionner le meilleur gap (distance moyenne maximale)
        5. Calculer l'angle cible depuis l'indice central du gap
        6. Appliquer le gain k_ftg et clamper à ±18°
    """

    # Valeur par défaut si non fourni par l'appelant.
    if sector_deg is None:
        sector_deg = 150

    # ── ÉTAPE 1 — Extraire la zone avant ──────────────────────────────
    # On veut les indices autour de 0 (avant), soit :
    #   droite : scan[360-sector_deg : 360]  → ex. indices 260–359
    #   avant  : scan[0 : sector_deg + 1]    → ex. indices 0–100
    # On concatène droite PUIS avant pour obtenir une liste contiguë
    # qui va de la droite lointaine (260) vers l'avant (0) puis vers
    # la gauche lointaine (100), sans discontinuité au passage par 0.
    points = []
    for i in range(360 - sector_deg, 360):
        if scan[i] != 0:
            points.append((i, scan[i]))
    for i in range(0, sector_deg + 1):
        if scan[i] != 0:
            points.append((i, scan[i]))

    if not points:
        return 0.0

    # ── ÉTAPE 2 — Seuillage ───────────────────────────────────────────
    # Chaque point est marqué libre (distance >= d_min_mm) ou occupé.
    # Les zéros ont déjà été exclus à l'étape 1 car 0 = absence de mesure,
    # pas un obstacle — les compter comme occupé fermerait les gaps à tort.

    # ── ÉTAPE 3 — Trouver les gaps ────────────────────────────────────
    # Un gap est une séquence contiguë de points libres dans la liste
    # "points". "Contigu" signifie que les points se suivent dans l'ordre
    # de parcours, MAIS ne doivent pas être séparés par de trop grands
    # espaces sans mesure (ex: > 3°).
    GAP_MAX_IDX_JUMP = 3
    gaps = []
    current_gap = []
    last_idx_unwrapped = None

    for idx, dist in points:
        # On calcule une position linéaire continue pour gérer le rebouclage
        # Les points sont ordonnés de 360-sector_deg vers 360, puis de 0 vers sector_deg
        linear_idx = idx if idx >= 180 else idx + 360

        if dist >= d_min_mm:
            # Si le dernier point du gap courant est trop loin numériquement (saut de >3°)
            # Cela veut dire qu'il y a des zéros au milieu qu'on a sautés. On coupe le gap.
            if current_gap and last_idx_unwrapped is not None and (linear_idx - last_idx_unwrapped > GAP_MAX_IDX_JUMP):
                if len(current_gap) >= w_min_pts:
                    gaps.append(current_gap)
                current_gap = []

            current_gap.append((idx, dist))
            last_idx_unwrapped = linear_idx
        else:
            if len(current_gap) >= w_min_pts:
                gaps.append(current_gap)
            current_gap = []
            last_idx_unwrapped = None

    if len(current_gap) >= w_min_pts:
        gaps.append(current_gap)

    # ── ÉTAPE 4 — Sélectionner le meilleur gap ───────────────────────
    # Critère : distance moyenne maximale → favorise l'espace libre le
    # plus profond (le plus sûr). En cas d'égalité, le plus large gagne
    # (plus de marge de manœuvre).
    if not gaps:
        if not points:
            return 0.0
        # Fallback de survie si TOUT est bloqué : braquer vers
        # le point le moins proche plutôt que de foncer tout droit
        best_point = max(points, key=lambda p: p[1])
        center_idx = best_point[0]
    else:
        best_gap = None
        best_avg = -1.0
        best_len = 0
        for gap in gaps:
            avg = sum(d for _, d in gap) / len(gap)
            if avg > best_avg or (avg == best_avg and len(gap) > best_len):
                best_avg = avg
                best_len = len(gap)
                best_gap = gap

        # ── ÉTAPE 5 — Calculer l'angle cible ─────────────────────────────
        # Prendre le point central du meilleur gap et récupérer son indice
        # original dans scan (0–359).
        mid = len(best_gap) // 2
        center_idx = best_gap[mid][0]

    # Conversion indice → angle physique :
    #   indice 0   → angle_brut = 0    → angle_physique = 0°    (avant)
    #   indice 90  → angle_brut = 90   → angle_physique = -90°  → clamp → gauche
    #   indice 270 → angle_brut = -90  → angle_physique = +90°  → clamp → droite
    #
    # Formule :
    #   si indice <= 180 : angle_brut = indice  (côté gauche + avant)
    #   si indice > 180  : angle_brut = indice - 360  (côté droite, valeur négative)
    #   angle_physique = -angle_brut  (inverser : indice positif = gauche = angle négatif)
    if center_idx <= 180:
        angle_brut = center_idx
    else:
        angle_brut = center_idx - 360
    angle_physique = -angle_brut

    # ── ÉTAPE 6 — Appliquer le gain et clamper ───────────────────────
    # k_ftg contrôle l'agressivité du braquage :
    #   k_ftg=0.2 → braquage doux, risque de rater les virages
    #   k_ftg=1.0 → braquage agressif, réactif mais peut osciller
    # À régler le jour des essais (valeur initiale : 1.0).
    angle_cmd = _clamp(k_ftg * angle_physique, -steer_limit_deg, steer_limit_deg)
    return angle_cmd


def detect_collision(scan: list, seuil_mm: float, sector_deg: int = 30) -> bool:
    """
    Détecte un obstacle dangereux dans le secteur avant ±sector_deg.

    sector_deg=30 par défaut : couvre ±30° devant la voiture, soit la largeur
    approximative du châssis projetée à distance. Plus large → plus prudent
    mais risque de faux positifs dans les virages.

    Retourne True si min(distances valides) < seuil_mm.
    Retourne False si aucune mesure valide dans le secteur.
    """
    # Secteur avant : scan[0:sector_deg+1] + scan[360-sector_deg:360]
    min_dist = None
    for i in list(range(0, sector_deg + 1)) + list(range(360 - sector_deg, 360)):
        d = scan[i]
        # Ignorer 0 = absence de mesure (pas un obstacle).
        if d == 0:
            continue
        if min_dist is None or d < min_dist:
            min_dist = d

    if min_dist is None:
        return False
    return min_dist < seuil_mm

## 2026-09-03 — S0 — TP 0.1 Logistique et fondations du dépôt — 2 h (prévu 4 h)

- **Objectif** : dépôt initialisé, build CMake/GoogleTest vert, PX4 figé, commande 2 planifiée.
- **Fait** :
  - arborescence §3.2 créée ;
  - CMake C++17 + GoogleTest via FetchContent, `ctest` vert (1 test bouchon `core::version()`) ;
  - PX4 cloné `--recursive`, branche `frozen-v1.17.0` sur le tag v1.17.0, sous-modules alignés ;
  - `docs/versions.md` : Ubuntu 24.04.4, PX4 v1.17.0 (`d6f12ad1`), gtest v1.18.0, cmake 3.28.3, gcc 13.3.0.
- **Bloqué sur** : header introuvable — fichier créé dans `core/include/` au lieu de `core/include/core/` (le chemin d'include est relatif au dossier déclaré dans `target_include_directories`). Notion de tag git revue.
- **Décision** : C++17 ; `-Wall -Wextra -Wpedantic` obligatoires sur `core/` ; GoogleTest figé v1.18.0 par FetchContent ; headers préfixés `core/` pour éviter les collisions dans PX4 (TP 4.2).
- **Prochaine session** : TP 0.2 étape 1 (toolchain PX4 + SITL). CMake compris comme outil, à approfondir avant TP 2.4 avec le C++.
- **Commande 2** : date visée à compléter (15-19 sept).
- **Jokers** : aucun.

## 2026-09-04 — S0 — TP 0.2 Environnement de dev et premier vol SITL — 3 h (prévu 8 h)

- **Objectif** : toolchain PX4 installée, vol SITL complet sur x500, ulog annoté, build cible fmu-v6c.
- **Fait** :
  - `ubuntu.sh` sur Ubuntu 24.04 natif (RTX 5070, rendu NVIDIA vérifié), Gazebo Harmonic, QGC v5.1.4 stable, venv Python + numpy/scipy/matplotlib/pyulog ;
  - `make px4_sitl gz_x500` ; décollage au joystick virtuel QGC (`COM_RC_IN_MODE = 1`) ; mission 3 WP à 5 m + Return, désarmement auto ;
  - ulog `09_35_29` → `flight-tests/sitl/2026-09-04_sitl_x500_mission3wp_5m.ulg` + `.md` annoté (topics, fréquences, `ref_*`, signe de `z` en NED, cadences IMU) ; Flight Review, PlotJuggler, `ulog_info` ;
  - `make px4_fmu-v6c_default` OK sans carte — **flash 97,97 %** en config default ;
  - `docs/versions.md` complété (Ubuntu 24.04.4, PX4 v1.17.0 `d6f12ad1`, gtest 1.18.0, cmake 3.28.3, gcc hôte 13.3.0, arm-none-eabi-gcc 13.2.1, QGC 5.1.4) ; `docs/liens.md` créé.
- **Bloqué sur** :
  - premier vol à 50 m d'altitude (défaut QGC non modifié) et fini en loiter (pas de Return) — réflexe : vérifier l'altitude de chaque item avant upload ;
  - coût de propagation d'un estimateur et rôle des deux cadences IMU compris en surface seulement — à reprendre aux TP 1.5 et 2.3.
- **Décision** : QGC stable (pas daily) ; doc PX4 **v1.17**, pas `main` ; Gazebo gz (pas Classic) ; deux compilateurs distincts dans `versions.md` ; risque **flash 98 %** à reporter en §10 (modules à retirer via `boardconfig` au TP 4.2) ; `SDLOG_MODE` à changer au TP 2.1 ; écart SITL/réel GNSS 30 Hz vs 5–10 Hz noté pour le TP 3.0.
- **Prochaine session** : TP 1.1 étape 1 (papier : NED/FRD, lacet 90°, quaternion Hamilton w-x-y-z). Lire avant : Solà §1–3, doc v1.17 « Flight Controller Orientation ».
- **Commande 2** : date toujours à fixer (15–19 sept).
- **Jokers** : aucun.

## 2026-09-04 — S0 — TP 1.1 étape 1 Rotations et conventions (papier) — 3 h (prévu ~3 h sur 10 h)

- **Objectif** : NED/FRD, R_nb du lacet 90°, convention quaternion figée, « q transforme du corps vers le monde » dans mes mots.
- **Fait** :
  - NED/FRD dessinés ; lacet positif = autour de +D, horaire vu du dessus ;
  - R_nb(90°) par la règle des colonnes (axes corps exprimés dans NED), généralisée en ψ ; RᵀR = I ⇐ colonnes orthonormées ⇒ R_bn = R_nbᵀ ;
  - notation figée : destination en premier, `R_nb`, `q_nb`, `v_n = R_nb v_b` ;
  - tableau des quatre choix de Solà §3 : (w,x,y,z), ij = k, passive, local→global — identique à PX4 (`VehicleAttitude.msg`) ;
  - q_nb(90°) = (√2/2, 0, 0, √2/2) ; rotation de (1,0,0) à la main par q ⊗ (0,v) ⊗ q* → (0,1,0), cohérent avec R_nb ;
  - q et −q : signe présent deux fois ⇒ même rotation, double recouvrement ;
  - Solà §1–3 lus (sauf §2.7–2.8) ; `docs/conventions.md` créé ;
  - correctif : GPU = RTX 5070 (pas Ti) dans l'entrée TP 0.2 et `versions.md`.
- **Bloqué sur** : notation vecteur/scalaire dans q = (cos θ/2, u·sin θ/2) ; plongement (0, v) du vecteur en quaternion pur ; erreur de signe en recopiant le produit par composantes et (√2/2)² pris pour 1 — attrapées par le test de norme.
- **Décision** : conventions figées dans `docs/conventions.md`, ne plus changer ; réflexes : norme conservée après rotation, ‖q‖ = 1 après tout quaternion, « deux repères nommés = passif ».
- **Prochaine session** : TP 1.1 étape 2 `sim/rotations.py` — fixer la séquence d'Euler avant de coder ; attention scipy x-y-z-w ; comparaison q / −q dans les tests.
- **Commande 2** : date toujours à fixer (15–19 sept).
- **Jokers** : aucun.

## 2026-09-05 — S0 — TP 1.1 étapes 2+3 `sim/rotations.py` (1/2) — ~3,5 h (prévu ~7 h sur 10 h pour les étapes 2–3)

- **Objectif** : produit, conjugué, rotation d'un vecteur, q → R, chacun avec ses tests ; API numérique fixée.
- **Fait** :
  - séquence d'Euler fixée : 3-2-1 intrinsèque, `R_nb = R_z(ψ)·R_y(θ)·R_x(φ)`, conforme à PX4 `src/lib/matrix/matrix/Euler.hpp` ; intrinsèque Z-Y-X ≡ extrinsèque X-Y-Z (même produit de matrices) ;
  - `sim/rotations.py` : `quaternion_product`, `quaternion_conjugate`, `rotate`, `skew`, `quaternion_to_matrix` (Solà éq. R(q) = (q_w² − q_vᵀq_v) I + 2 q_v q_vᵀ + 2 q_w [q_v]×) ;
  - `sim/tests/test_rotations.py` (pytest) : identité, norme conservée, non-commutativité, deux lacets de 45° = q_nb(90°), **i ⊗ j = k** (seul test qui verrouille Hamilton), q ⊗ q* = 1, rotation de (1,0,0) par q_nb(90°), norme après rotation, q* défait q, `skew` contre `np.cross`, R_nb(90°) en dur, R·Rᵀ = I et det R = 1, `R @ v` = `rotate` ;
  - étapes 2 et 3 menées ensemble : un test écrit avec chaque fonction, pas après ;
  - outillage : `pyproject.toml` (`pythonpath`, `testpaths`), `sim/__init__.py`, pytest installé dans le venv (le premier run tournait sur `/usr/bin/python3`).
- **Bloqué sur** :
  - `a @ b` entre deux `(3,)` = produit scalaire, pas produit extérieur (`np.outer`) ; `.T` sans effet sur un 1-D ;
  - `q[1:] = -q[1:]` modifiait l'argument de l'appelant (vue numpy, pas copie) ;
- **Décision** : quaternions `(4,)` et vecteurs `(3,)`, jamais `(n, 1)` ; pas de `quaternion_inverse` (q⁻¹ = q* ssi ‖q‖ = 1) ; renormalisation dans l'intégration seule (seule fonction rebouclée sur sa sortie) ; une fonction ne modifie jamais ses arguments ; réflexe : un test vert peut l'être pour une mauvaise raison (les deux lacets de 45° sont aveugles au signe du produit vectoriel, `test_rejects_bad_shape` passait par l'erreur de forme de l'identité).
- **Prochaine session** : R → q (dérivation depuis R(q) : trace et R − Rᵀ), Euler ⇄ q, intégration sous ω constante.
- **Commande 2** : date toujours à fixer (15–19 sept).
- **Jokers** : aucun.

## 2026-09-06 — S0 — TP 1.1 étapes 2+3 `sim/rotations.py` (2/2) + validation — ~3,5 h (TP 1.1 total ≈ 10 h, prévu 10 h)

- **Objectif** : R → q, Euler ⇄ q, intégration sous ω constante, validation orale du TP.
- **Fait** :
  - R → q dérivé de R(q) : tr R = 4 q_w² − 1 ⇒ q_w = √((tr R + 1)/4), racine positive ⇒ **q_w ≥ 0** ; R − Rᵀ = 4 q_w [q_v]× ⇒ q_x = (R₂₁ − R₁₂)/4q_w, etc. ; garde-fou `tr R + 1 < 4e-6` (|θ − 180°| < 0,11°) ; pas de Shepperd, domaine documenté ;
  - `build_quaternion(angle, u)` : axe-angle → q (Exp de Solà §2.4.3), lève sur axe nul ;
  - `euler_to_quaternion(phi, theta, psi)` = q_z(ψ) ⊗ q_y(θ) ⊗ q_x(φ), même ordre que les matrices car R(q₁ ⊗ q₂) = R(q₁)·R(q₂) (Solà §2.6) ;
  - `quaternion_to_euler` via R : θ = −asin(clip(R₂₀)), φ = atan2(R₂₁, R₂₂), ψ = atan2(R₁₀, R₀₀) ; R(φ, θ, ψ) développé à la main ;
  - `integrate_quaternion(q, ω, dt)` = q ⊗ q{ω·dt}, incrément à droite (ω mesurée dans b = perturbation locale, Solà §4.1), court-circuit si ‖ω‖dt = 0, renormalisation en sortie ; ordre zéro exact sous ω constante par pas (bloqueur d'ordre zéro), même noyau que `predictState` d'EKF2 ;
  - tests : aller-retour q → R → q à ±q près (`assert_same_rotation`), rejet 180° sur 20 axes aléatoires, oracle scipy `Rotation.from_euler("ZYX", [ψ, θ, φ]).as_matrix()`, aller-retour Euler (|θ| < π/2 − 0,1), lacet 90° puis roulis 90° sur x corps = Euler(π/2, 0, π/2), N pas = 1 pas, norme après 10⁴ pas, ω = 0 ⇒ q inchangé ;
  - validation orale : passif (deux repères nommés, la formule ne dit rien seule) ; accéléromètre = force spécifique f = a − g ⇒ posé à plat en FRD (0, 0, −9,81), chute libre ⇒ 0 ;
  - `docs/conventions.md` complété, commit.
- **Bloqué sur** :
  - `assert` dans une fonction (AssertionError, supprimé par `python -O`) au lieu de `raise ValueError` ;
  - formule au 1er ordre de Solà §4.6.2 (ω̄, ω_n × ω_{n+1}) comprise : moyenne = norme variable, produit vectoriel = axe qui tourne ; nulle sous ω constante.
- **Décision** : API Euler = trois scalaires (φ, θ, ψ) en rad, séquence figée non paramétrable ; `quaternion_to_euler` indéfinie à |θ| = 90° (cardan) ; 1er ordre = raffinement candidat TP 2.3, seulement si les logs le justifient ; chat unique gardé au-delà de 50 messages (choix assumé, à réévaluer si la recherche devient pénible).
- **Prochaine session** : TP 1.2 — Dynamique du quadrotor, à la main, paramètres X500.
- **Commande 2** : date toujours à fixer (15–19 sept).
- **Jokers** : aucun.

## 2026-09-07 — S0 — TP 1.2 étape 1 Newton-Euler 6 DDL (1/2) — ~3 h (TP prévu 9 h)

- **Objectif** : les quatre équations d'état (p, v, q, ω) dans mes conventions, dérivées et non recopiées.
- **Fait** :
  - Mahony-Kumar-Corke lu ; tableau de correspondance article ↔ mes conventions (ᴬR_B = R_nb, {A} = NED, {B} = FRD, Euler Z-X-Y chez eux → ne rien recopier en Euler) ;
  - `ṗ_n = v_n` ; `m v̇_n = m g z_n + R_nb F_b` avec `F_b = −T_Σ z_b` ; test hover à plat (`R_nb = I`, `T_Σ = mg` ⇒ `v̇_n = 0`) ;
  - `q̇_nb = ½ q_nb ⊗ (0, ω_b)` dérivé de `q ⊗ q{ω dt}` au 1er ordre en dt (`q{ω dt} ≈ (1, ω dt/2)`), `ω_b` à droite = perturbation locale ; vérifié sur le lacet ;
  - `J ω̇_b = τ_b − ω_b × (J ω_b)` dérivé en 5 lignes via `L_n = R_nb J ω_b`, `L̇_n = R_nb τ_b`, `Ṙ_nb = R_nb [ω_b]×` ; retrouve (1d) du papier.
- **Bloqué sur** :
  - première tentative : `−ω × v_b` de mavsim (éq. 3.15, écrite dans le corps pour `v_b`) collé dans une équation NED pour `v_n` ; `Ṙ·v̇` au lieu de `Ṙv + Rv̇` ; `m` à gauche et `1/m` sur la force ;
  - « attitude » : `R_nb` (colonnes = axes corps dans NED) *est* l'attitude, pas un outil qui la décrit ;
  - « pourquoi dériver q » : Newton-Euler = `f(x, u)` ; consommé par RK4 (TP 1.3), par la propagation et `F` de l'ESKF (TP 2.3-3.1), par la linéarisation LQR/MPC ;
  - `ω × Jω` : pas un couple, c'est le coût de dériver un vecteur exprimé dans un repère qui tourne (même origine que le `−ω × v_b` de mavsim).
- **Décision** : état en **`v_n`** (vitesse sol, NED), pas `(u, v, w)` — GNSS `velNED`, contrôleur de position et Newton direct en repère inertiel ; notation `F_b = −T_Σ z_b`, gravité `m g z_n`.
- **Prochaine session** : suite étape 1 — traînée linéaire + quadratique en vitesse air.
- **Commande 2** : date toujours à fixer (15–19 sept).
- **Jokers** : aucun.

## 2026-09-08 — S0 — TP 1.2 étape 1 Newton-Euler 6 DDL (2/2) — ~2,5 h (étape 1 ≈ 5 h 30)

- **Objectif** : sens physique d'Euler, couplage, traînées, vitesse air, ligne complète de translation.
- **Fait** :
  - fil conducteur posé : 2 cinématiques (`ṗ`, `q̇`) + 2 dynamiques (`v̇`, `ω̇`) ;
  - Euler : `J ω̇` = « m·a » de la rotation, `ω × Jω` = couple pour faire tourner `L` avec le corps (roue de vélo) ; nul si `ω ∥ Jω` (`J = λI` ou axe principal) ; composante x : `J_xx ṗ = τ_x + (J_yy − J_zz) q r` ⇒ couplage quadratique en ω, négligeable au hover (cascade PID axe par axe, TP 4.1) ;
  - vitesse air : `v_air,n = v_n − v_w,n` (drone / air), `v_air,b = R_bn v_air,n` ; trois cas limites vérifiés ;
  - traînée rotor (flapping / induced drag, Mahony) : `−T_Σ D v_air,b`, `D = A_flap + diag(d_x, d_y, 0)`, linéaire et ∝ T, disparaît à T = 0 ; traînée corps : `−α ‖v_air,n‖ v_air,n`, `α` en kg/m, porte tout en chute moteurs coupés ;
  - ligne complète : `m v̇_n = m g z_n + R_nb(−T_Σ z_b − T_Σ D v_air,b) − α ‖v_air,n‖ v_air,n` ; test hover air calme OK.
- **Bloqué sur** :
  - « en v² mais approximé en v à petite vitesse » : faux, ce sont deux mécanismes (rotor linéaire, fuselage quadratique) qui s'additionnent ;
  - « par rapport à » (quelle vitesse) ≠ « exprimé dans » (quel repère) — confusion récurrente, tableau 3 vitesses × 2 repères écrit ;
  - `v_b` (drone / sol dans le corps) resté dans la traînée rotor au lieu de `v_air,b` ;
  - vitesse air définie à l'envers (air / drone) — le TP prend drone / air, traînée `−(…)·v_air` ; `w_n` lu comme `ω_n` → noter le vent `v_w,n`.
- **Décision** : `v_w,n` = vitesse de l'air par rapport au sol en NED (pas la convention météo « d'où vient le vent », piège TP 3.8) ; `α` **scalaire** pour l'instant (option `diag` si le TP 3.8 le justifie) ; valeurs initiales de `D`, `α` dans `params-x500.md` avec incertitude « non mesuré » ; mise au propre des 3-4 pages en fin de TP, pas par étape.
- **Prochaine session** : TP 1.2 étape 2 actionneur (`f = k_f Ω²`, `m = k_m Ω²`, moteur 1er ordre, ordres de grandeur X500, inertie par cylindres/points masse) — nouveau chat « S0 · TP 1.2 · étape 2 actionneur ». Lire mavsim ch. 4 avant l'étape 4.
- **Commande 2** : à passer entre le 15 et le 19 septembre selon l'avancement (carte reçue le 12, flashée et vérifiée avant) ; le 19 est la butée, au-delà le montage glisse en S2.
- **Jokers** : aucun.

## 2026-09-12 → 14 — S0 — TP 1.2 étape 2 Actionneur et ordres de grandeur X500 — durée non comptée, largement au-delà (prévu ~2 h 15 sur 9 h ; TP 1.2 cumulé > ×2 du budget ⇒ arbitrage fait, voir Décision)

- **Objectif** : chaîne commande → régime → (poussée, couple) avec dynamique 1er ordre ; ordres de grandeur X500 ; `docs/params-x500.md`.
- **Fait** (deux chats, l'un avec un autre agent, l'autre avec le tuteur) :
  - bilan des couples sur l'axe du rotor `J_r Ω̇ = C_em − C_res` ; lois statiques `f = k_f Ω²`, `Q = k_m Ω²`, unités (`kg·m`, `kg·m²`), facteur 4 ;
  - 1er ordre `Ω̇ = (Ω_∞(u) − Ω)/τ_m`, solution exponentielle, sens de `τ_m` (36,8 % restant / 63,2 % de la **variation**), exercices numériques ;
  - stationnaire horizontal : `T_Σ = mg ≈ 13,7 N`, `f_i ≈ 3,4 N ≈ 350 gf`, indépendant du lacet (`R_nb z_b = z_n` pour tout ψ) ;
  - origine du carré par quantité de mouvement : `ṁ = ρπr²v_i`, `f = ṁ v_i`, `v_i = PΩ/2π` (vis sans glissement) ⇒ `k_f = ρr²P²/4π = 2,05e-5 kg·m` (1045) ;
  - couple par bilan de puissance `Q_i Ω_i = f_i v_i` ⇒ `k_m/k_f = P/2π = 0,018 m` ; signe : hélice horaire ⇒ `−k_m Ω²` sur le châssis ;
  - `τ_m = J_r/(2k_m Ω_0)` autour d'un point de fonctionnement (pas une constante universelle) ; asymétrie accélération/décélération (freinage par `Q` seul) ;
  - PX4 : `actuator_motors` ∈ [0,1], `THR_MDL_FAC` = φ (`f/f_max = (1−φ)u + φu²`, φ = 0 ⇒ `Ω_∞ ∝ √u`, φ = 1 ⇒ `∝ u`), `MPC_THR_HOVER` = poussée normalisée au stationnaire ⇒ `f_max ≈ 7,6 N ≈ 770 gf` ;
  - le banc TP 4.3 n'identifie que `f_∞(u)` ; `k_f` s'annule dans `f(t)` ⇒ paramètres = `f_∞(u)`, `τ_m`, `P/2π` ; état = `Ω_i` ; 1er ordre sur `Ω` et non sur `f` (40 % vs 63 % à `t = τ_m` sur grand échelon, confondus autour du stationnaire) ;
  - SITL v1.17 (`gz/models/x500{,_base}/model.sdf`) : `motorConstant 8,55e-6`, `momentConstant 0,016`, `τ` 12,5/25 ms, `maxRotVelocity 1000`, masse 2,06 kg ⇒ `Ω_hover ≈ 630 rad/s ≈ 6 000 tr/min` (idéal : 410 rad/s) ; `k_f` idéal 2,4× trop grand (glissement, pertes) ;
  - livrables tuteur : `docs/cours/cours-TP1.2-dynamique-quadrotor.{tex,pdf}` (tableau de notations + 4 étapes, exercices, réponses), `docs/params-x500.md` prérempli, instructions tuteur v2.
- **Bloqué sur** : amplitude de l'exponentielle (corrigée par le tuteur) ; % de la valeur finale pris pour % de la variation ; carré oublié dans le rapport `f(0,5)/f(1)` ; `Ω_∞ = f/k_f` sans racine (unités non vérifiées) ; « constante » de temps prise pour constante en tout point ; pente de `Q(Ω)` confondue avec le sens de la correction ; sens de `J_r` sur `τ_m` ; « stationnaire » = point de fonctionnement des rotors, pas immobilité du drone ; **méthode** : construction socratique par petites questions ⇒ fil perdu, motivation en baisse, temps hors budget.
- **Décision** :
  - instructions tuteur **v2** : cours complet d'abord → questions directes → 2-4 exercices d'application → implémentation par moi ; une question par message ; budget de temps tenu par le tuteur ; « STOP » ;
  - modèle actionneur : état `Ω_i`, 1er ordre sur `Ω`, un seul `τ_m = 20 ms` (non mesuré, entre les deux de Gazebo) ; interface `thrust_static(u)` idéale (`f_max u²`) / banc ; `Q_i = (P/2π) f_i`, `k_m` n'est pas un paramètre ; réaction inertielle `J_r Ω̇` négligée ;
  - notations : `Q_i` (couple aéro, ≠ `q`), `τ_m` (≠ `τ_b`), `v_w,n` (≠ `ω`) ;
  - étapes 3 (allocation) et 4 (vent) **prises en cours, non dérivées** : validation par les exercices du cours avant le TP 1.3 ; inertie `J ≈ diag(0,012, 0,012, 0,024) kg·m²` = exemple chiffré du cours, ±30 %, à refaire en exercice ; SITL = oracle d'ordre de grandeur, pas une référence ;
  - masse, `J`, `f_max`, `τ_m`, `D`, `α` : tous « non mesuré » dans `params-x500.md`.
- **Prochaine session** : relire le cours et le reprendre en notes (tablette) ; chat « Questions · cours TP 1.2 » ; exercices §3-§6 ; mavsim ch. 4 ; puis **TP 0.4 en parallèle** (carte reçue le 12, non démarrée — flash + M10/H-Flow conditionnent la commande 2) et TP 1.3.
- **Commande 2** : à passer entre le 15 et le 19 septembre selon l'avancement (carte reçue le 12, flashée et vérifiée avant) ; le 19 est la butée, au-delà le montage glisse en S2.
- **Jokers** : aucun (les étapes 3-4 données en cours relèvent du changement de méthode, pas d'un joker ; corrections locales sur l'amplitude exponentielle et le sens de `J_r`).

## 2026-09-14 — S0 — TP 0.4 étape 1 Flash et vérification des IMU — 0 h 40 (prévu 1 h 10 sur 6 h)

- **Objectif** : carte flashée sur v1.17.0 figé, deux IMU vues, état de référence (paramètres, identifiants) versionné.
- **Fait** :
  - `make px4_fmu-v6c_default upload` OK, USB seul ; `ver all` : hash `d6f12ad1`, branche `frozen-v1.17.0`, toolchain 13.2.1, conforme à `versions.md` ; HW type `V6C002002` ; `PX4GUID 0006000000003132343830345115001e002a` → `docs/versions.md` ;
  - `uorb top -1` : 2 instances `sensor_gyro` (667 / 810 Hz) et `sensor_accel` (815 / 810 Hz), `_fifo` idem, `#Q` = 8 ;
  - `listener` : IMU 0 = **BMI088** (`device_id` 6684682 gyro 0x66, 6946826 accel 0x6A ; deux dies, T gyro = nan, T accel 36,5 °C) ; IMU 1 = **ICM-42688-P** (`device_id` 2490378 partagé, 0x26 ; T 38,3 °C) ;
  - `icm42688p status` : FIFO 1250 µs (800 Hz) ; `bmi088 -A` 1250 µs, `-G` 1500 µs (666,7 Hz) ; 0 overflow partout ;
  - `samples` par message : ICM 10 (8 kHz interne), BMI088 gyro 3 (2 kHz), accel 2 (1,6 kHz) ;
  - paramètres → `px4/params/2026-09-14_6cmini_v1.17.0_stock.params` (export texte QGC) ; `param show -c` : 42 paramètres hors défaut = calibration usine Holybro (accel, gyro, mag interne IST8310, baro MS5611, niveau `SENS_BOARD_X/Y_OFF` −3,0°/+2,4°) + `SYS_AUTOSTART 4001` (Generic Quadcopter) — le fichier exporté est donc stock **+ calibration usine**.
- **Bloqué sur** : `bmi055 status` → « Not running » : la révision `V6C002002` embarque un BMI088, pas un BMI055 (`boards/px4/fmu-v6c/init/rc.board_sensors`, v1.17.0) ; `bmi088 status` exige `-A` ou `-G`.
- **Décision** :
  - le numéro d'instance uORB n'identifie pas le chip (ordre de démarrage des pilotes) : toujours `device_id` ;
  - calibration usine et cadre 4001 conservés, rien recalibré sur carte nue : tout est refait sur le drone monté au TP 3.4 ; « rien n'arme sans cadre » ne tient plus, ce sont les prechecks qui bloquent — log « from boot » maintenu ;
  - `sensor_gyro`/`sensor_accel` sont bruts, les `CAL_*` s'appliquent en aval dans `sensors` : Allan n'en dépend pas ;
  - pas de `param export` dans nsh (`param save <file>` = binaire) : l'export versionné est le texte QGC (Parameters → Tools → Save to file) ;
  - document maître à corriger (chat Outil) : TP 0.4 « `param export` » → export QGC ; TP 2.1 datasheet **BMI088** au lieu de BMI055 ;
  - pour TP 2.1 : `sensor_gyro` = moyenne de `samples` échantillons, bruit brut par échantillon dans `_fifo` → profils « high rate » + « sensor comparison » tous les deux.
- **Prochaine session** : TP 0.4 étape 2 (M10 sur GPS1 : `listener sensor_gps`, `sensor_mag`, log statique 20-30 min avec fix).
- **Commande 2** : conditionnée aux étapes 2-3, butée le 19.
- **Jokers** : aucun.

## 2026-09-15 — S0 — TP 0.4 étape 2 M10 sur GPS1 — 1 h 20 (prévu 1 h 10)

- **Objectif** : M10 reconnu sur GPS1, fix 3D lu, mag externe identifiée, premier nuage GNSS statique logué pour le TP 3.2a.
- **Fait** :
  - `SDLOG_PROFILE` 1 → 2051 (défaut + replay EKF2 + capteurs pleine cadence), reboot depuis le nsh ;
  - M10 : UBX sur `/dev/ttyS0` (GPS1), 115200 bauds, 10,01 Hz ; firmware SPG 5.10, protocole 34.10 ; `device_id` 10813445 (0xA5, série 0) → `versions.md` ;
  - fix 3D en chambre, fenêtre ouverte : 11-13 satellites, eph 0,39-0,42 m, epv 0,66-0,68 m, sAcc 0,16-0,18 m/s, hdop 1,2, jamming/spoofing OK, tous les seuils `EKF2_REQ_*` passés ; `altitude_ellipsoid − altitude_msl` = 45,868 m constant (ondulation du géoïde, modèle embarqué) ; entre deux captures à 42 s : ~0,9 m horizontal, 2,2 m vertical → erreur lente vue en direct ;
  - deux `sensor_mag` : instance 0 = IST8310 du M10 (I2C1, 0x0E, `device_id` 396809, rotation pilote `-R 10`) ; instance 1 = IST8310 interne 6C (I2C4, 0x0C, `device_id` 396321, démarré `-X` donc « externe » pour PX4) ; test de rotation du M10 seul : l'instance 0 bouge ; normes 0,46 / 0,51 G pour 0,49 attendu (WMM ≈ 50° N) ;
  - log statique `flight-tests/bench/2026-09-15_m10_static.ulg` + `.md` : 13:14:45 → 14:35:49 locale, 1 h 21 min 04 s, 0 dropout, `sensor_gps` 45 678 (9,39 Hz), `sensor_mag` 46,7 / 46,3 Hz, `sensor_combined` 202 Hz, EKF2 stock présent (innovations, `aid_src`) ;
  - exercices : R_pos / R_vel sans et avec planchers (ex. 1) ; 1/√(2N) = 0,58 % et N_eff = 25 → 14 % avec τ_c = 60 s, log réel 7,9 % (ex. 2) ; signes du mag selon montage, norme aveugle aux rotations (ex. 3) — prédiction (0,20 ; 0 ; +0,45) G et comparaison aux deux instances **données en synthèse, non dérivées**.
- **Bloqué sur** :
  - R non défini dans le cours v1 → v1.1 ; plancher vertical EKF2 = 1,5 × `EKF2_GPS_P_NOISE`, vitesse verticale × 1,5², plafond `EKF2_NOAID_NOISE` : absents de la v1, corrigés (terme d recalculé par le tuteur après tentative) ;
  - présentation : 0,152 pour 0,15 m² (chiffres significatifs), repère absent sur les matrices ;
  - `sensor_gps` logué à 9,39 Hz au lieu de 10,01 (~6 %) — hypothèse : polling du logger ; histogramme Δt au TP 3.2a ;
  - mag M10 : inclinaison apparente 54° et direction horizontale à ~90° de la 6C — hypothèses : fer dur non compensé et/ou `-R 10` inadaptée au M10 ; à trancher au TP 3.4.
- **Décision** : altitude MSL ou ellipsoïdale : choix au TP 3.2a, dans `conventions.md` ; `CAL_MAGn_PRIO` à régler à la main au TP 3.4 (les deux mags sont « externes ») ; profil 2051 gardé pour l'étape 3, profil Allan réglé à l'étape 4 (`sensor_gyro/accel` à 1 Hz dans 2051) ; bruit GNSS au TP 3.2a : partie rapide → R, partie lente → biais corrélé ou limite documentée ; `GPS_UBX_DYNMODEL` reste à 7.
- **Prochaine session** : TP 0.4 étape 3 — H-Flow, chat « S0 · TP 0.4 · étape 3 H-Flow » ; remplir les `___` du `.md` du log ; ajouter le cours v1.1 aux connaissances.
- **Commande 2** : à passer avant le 19 ; M10 validé, reste l'H-Flow.
- **Jokers** : aucun (ex. 3 conclue en synthèse pour budget ; terme vertical de l'ex. 1 corrigé après tentative, omission du cours).
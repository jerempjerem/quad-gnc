# Conventions — figées le 2026-09-04 (TP 1.1), complétées le 2026-09-06

## Repères
- Monde : NED (x = Nord, y = Est, z = Bas). Origine locale : `ref_lat/ref_lon/ref_alt` de `vehicle_local_position`.
- Corps : FRD (x = avant, y = droite, z = bas).
- Lacet positif : autour de +D, horaire vu du dessus (nez N → E).
- Gravité : `g_n = (0, 0, +9.81)`. Un accéléromètre mesure la force spécifique `f = a − g`, pas la gravité : posé à plat, immobile, il lit `(0, 0, −9.81)` en FRD ; en chute libre, `0`.

## Matrices de rotation
- Notation : destination en premier. `R_nb` transforme les composantes corps en composantes NED : `v_n = R_nb · v_b`.
- Composition : `R_nb · R_bc = R_nc` (les indices s'enchaînent).
- Inverse : `R_bn = R_nbᵀ` (colonnes orthonormées ⇒ RᵀR = I, det R = +1).
- Vérification de tête : les colonnes de `R_nb` sont x_b, y_b, z_b exprimés dans NED.
- Exemple validé : lacet +90° → `R_nb = [[0,-1,0],[1,0,0],[0,0,1]]`.
- Lacet ψ quelconque : `R_nb = [[cos ψ, −sin ψ, 0], [sin ψ, cos ψ, 0], [0, 0, 1]]` (ligne θ = φ = 0 du R_nb(φ, θ, ψ) développé le 6 sept ; ψ = 90° redonne l'exemple ci-dessus). L'ancienne version de cette ligne (sin ψ en haut à droite) était `R_bn = R_nbᵀ` — erreur corrigée le 6 sept.
- Forme générale (6 sept) : `R_nb = R_z(ψ)·R_y(θ)·R_x(φ)` =
  `[[cψcθ, cψsθsφ − sψcφ, cψsθcφ + sψsφ], [sψcθ, sψsθsφ + cψcφ, sψsθcφ − cψsφ], [−sθ, cθsφ, cθcφ]]`.
- Matrice antisymétrique : `[a]×` telle que `[a]× b = a × b` (`skew(a)`, testée contre `np.cross`).

## Quaternions
| Choix (Solà §3) | Moi | PX4 | Source |
|---|---|---|---|
| Ordre | (w, x, y, z) | (w, x, y, z) | `msg/versioned/VehicleAttitude.msg` @ v1.17.0 (chemin à vérifier) |
| Algèbre | ij = k (Hamilton) | Hamilton | idem |
| Fonction | passive (changement de repère) | passive | idem : « from the FRD body frame to the NED earth frame » |
| Direction | local → global | FRD → NED | idem |

- Notation : `q_nb`, même sens que `R_nb`.
- Action sur un vecteur : `v_n = q_nb ⊗ (0, v_b) ⊗ q_nb*` ; la partie scalaire du résultat est nulle. La formule est la même en actif et en passif ; c'est la notation `q_nb` qui dit ce qu'on fait. `q_nb* = q_bn` : exprime un vecteur monde dans le corps (prédiction des capteurs embarqués).
- Inverse et conjugué : `q⁻¹ = q* / ‖q‖²`, donc `q⁻¹ = q*` ssi `‖q‖ = 1`. Pas de fonction inverse : on ne manipule que des quaternions unitaires.
- q et −q : même rotation (double recouvrement, Solà §2.4.6). `(−1, 0, 0, 0)` est l'identité, pas 180°. 180° autour de u = `(0, u)`.
- Composition : `R(q₁ ⊗ q₂) = R(q₁)·R(q₂)` (Solà §2.6) ; les indices s'enchaînent comme pour les matrices ; **le premier appliqué au vecteur est à droite**.
- Axe-angle : `q = (cos α/2, u·sin α/2)`, ‖u‖ = 1 (`build_quaternion(angle, u)`, lève si u = 0).
- Test de convention dans la batterie : `i ⊗ j = k` (JPL donnerait −k). Les compositions autour d'un même axe n'y voient rien.

## Angles d'Euler
- Séquence : **3-2-1 intrinsèque (Z-Y-X)**, identique à PX4 `src/lib/matrix/matrix/Euler.hpp` : `R_nb = R_z(ψ)·R_y(θ)·R_x(φ)`, et `q_nb = q_z(ψ) ⊗ q_y(θ) ⊗ q_x(φ)`.
- Équivalence : intrinsèque Z-Y-X = extrinsèque X-Y-Z (même produit de matrices).
- API : `euler_to_quaternion(phi, theta, psi)` et `quaternion_to_euler(q) → [phi, theta, psi]`, trois scalaires en radians, ordre roulis-tangage-lacet ; la séquence n'est pas un paramètre.
- Plages : θ ∈ [−π/2, π/2], φ et ψ ∈ (−π, π]. `quaternion_to_euler` indéfinie à |θ| = 90° (blocage de cardan : φ et ψ confondus). Raison de garder q dans l'estimateur et les Euler pour l'affichage.
- Lecture sur R : θ = −asin(R₂₀) (clip à ±1 avant), φ = atan2(R₂₁, R₂₂), ψ = atan2(R₁₀, R₀₀). `atan2` sur un couple : quadrant conservé et pas de division par 0.

## Conversions et numérique (`sim/rotations.py`)
- Formes : quaternion `(4,)`, vecteur `(3,)`, matrice `(3, 3)`. Jamais `(n, 1)`. Avec des `(n,)` : `a @ b` = produit scalaire, `np.outer(a, b)` = a bᵀ, `M @ a` = M·a.
- Une fonction ne modifie jamais ses arguments ; elle renvoie un nouveau tableau (`q.copy()`).
- Erreurs : `raise ValueError` avec la cause ; jamais `assert` dans une fonction.
- `quaternion_to_matrix` : `R = (q_w² − q_vᵀq_v) I + 2 q_v q_vᵀ + 2 q_w [q_v]×` (Solà §2.5).
- `matrix_to_quaternion` : méthode trace, `q_w = √((tr R + 1)/4)` ⇒ **q_w ≥ 0 en sortie** (le double recouvrement est levé par convention ; les tests d'aller-retour comparent à ±q). Lève `ValueError` si `tr R + 1 < 4e-6`, soit |θ − 180°| < 0,11°. Pas de Shepperd : usage limité aux tests et conversions ponctuelles (PX4 livre des quaternions).
- `integrate_quaternion(q, ω, dt)` : `q ⊗ q{ω·dt}`, ω en rad/s dans b, hypothèse ω constante sur [t, t + dt] (bloqueur d'ordre zéro, exact par pas) ; renvoie `q` si ‖ω‖dt = 0 ; **seule fonction qui renormalise**, en sortie, car seule à être rebouclée sur son résultat. Raffinement 1er ordre (Solà §4.6.2, ω̄ et ω_n × ω_{n+1}) : candidat TP 2.3, seulement si les logs le justifient.

## Pièges
- Ne jamais mélanger w-x-y-z et x-y-z-w : scipy `Rotation.as_quat()` sort du x-y-z-w.
- scipy Euler : `from_euler("ZYX", [psi, theta, phi])` — majuscules = intrinsèque, angles **dans l'ordre des lettres** (ψ en premier, l'inverse de mon API). `"xyz"` minuscules donne la même matrice ; `"zyx"` non. Comparer via `.as_matrix()`, pas via `.as_quat()`.
- `np.sqrt` d'un négatif et `x / 0` ne lèvent pas : ils rendent `nan` avec un `RuntimeWarning`. Tester la quantité **avant** l'opération, avec un seuil choisi et documenté.
- Un test vert peut l'être pour une mauvaise raison ; se demander s'il passerait avec une fonction fausse.
- Doc et sources PX4 : v1.17, jamais `main`.
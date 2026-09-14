# Paramètres X500 — v1 (14 sept. 2026)
| Symbole | Valeur | Unité | Statut | Source / méthode | Remplacé par |
|---|---|---|---|---|---|
| m | 1,4 | kg | non mesuré | document maître | pesée TP 3.4 |
| J_xx, J_yy | 0,012 | kg m^2 | non mesuré ±30 % | cylindres + points masse (§4.7) | — |
| J_zz | 0,024 | kg m^2 | non mesuré ±30 % | idem | — |
| l | 0,25 | m | fiche Holybro (wheelbase 500) | — | mesure |
| f_max (u=1, 4S) | 7,6 | N | déduit (THR_HOVER 0,45) | cours §4.6 | banc TP 4.3 |
| f_inf(u) | f_max u^2 | N | idéal | φ = 1 | courbe banc TP 4.3 |
| k_f | 2,05e-5 idéal (SITL 8,55e-6) | kg m | idéal | rho r^2 P^2 / (4 pi) | — (s'annule) |
| c = k_m/k_f | 0,018 | m | idéal | P / 2π (1045) | — |
| tau_m | 0,020 | s | non mesuré | Gazebo 12,5 / 25 ms | logs TP 4.4 |
| D (d_x, d_y) | 0,02 ; 0,02 | s/m | non mesuré | hypothèse (cours §6.3) | TP 3.8 |
| alpha | 0,03 | kg/m | non mesuré | 0,5 rho C_D A, C_D~1, A~0,05 m^2 | TP 3.8 |
| Négligé | J_r dOmega/dt (réaction inertielle), traînée de profil, |
|         | frottements mécaniques, asymétrie up/down de tau_m |

Toutes les valeurs « non mesuré » ou « idéal » sont à remplacer : pesée (TP 3.4), banc de poussée (TP 4.3), traînée (TP 3.8), logs (TP 4.4). Détail des calculs : `docs/cours/cours-TP1.2-dynamique-quadrotor.pdf`, §4.6–4.7 et §6.3.

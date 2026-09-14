# Vol SITL — 2026-09-04 — x500, mission 3 WP à 5 m

Fichier : `2026-09-04_sitl_x500_mission3wp_5m.ulg` (copie de `build/px4_sitl_default/rootfs/log/2026-09-04/09_35_29.ulg`, 16 005 777 octets)

## Contexte

- PX4 v1.17.0 (`d6f12ad1`, branche `frozen-v1.17.0`), `make px4_sitl gz_x500`
- Gazebo Harmonic (gz), modèle x500, monde par défaut, origine Zurich (défaut SITL)
- QGC stable v5.1.4, joystick virtuel activé, `COM_RC_IN_MODE = 1`
- Mission QGC : Takeoff → 3 waypoints (~8m au sol) à 5 m relatif → Return
- Armement par commande externe (Start Mission), désarmement automatique à l'atterrissage
- Durée du log : 54 s

## Topics regardés (`ulog_info`)

| Topic | Échantillons | Fréquence loggée |
|---|---|---|
| `vehicle_attitude` | 13 570 | 250 Hz |
| `vehicle_local_position` | 6 786 | 125 Hz |
| `vehicle_gps_position` | 1 646 | 30 Hz |
| `actuator_motors` | 544 | 10 Hz |

Flight Review : max IMU rate 2000 Hz, max IMU integration rate 1000 Hz.

## Observations

- `actuator_motors` à 10 Hz = intervalle du **logger**, pas la cadence de publication (boucle de vitesse angulaire, plusieurs centaines de Hz). Loggé ≠ publié. À changer via `SDLOG_PROFILE` (high rate) pour le TP 4.4.
- `ref_lat` = 47.398, `ref_lon` = 8.546, `ref_alt` = 0.259 : constants sur tout le vol.
- `xy_global` : [à vérifier — passe de false à true quand `ref_*` deviennent valides ?]
- `vehicle_local_position.z` décroît quand le drone monte (≈ −5 à l'altitude de mission).
- Le logger démarre à l'armement et se ferme au désarmement (`SDLOG_MODE` par défaut). À modifier au TP 2.1 pour enregistrer l'IMU au sol.
- Raté instructif : premier vol (`09_31_44.ulg`) à 50 m d'altitude — altitude par défaut de QGC dans Plan non modifiée — et terminé en loiter faute de Return/Land. Réflexe à garder : vérifier l'altitude de chaque item avant l'upload.

## Écarts SITL / réel à retenir

- GNSS à 30 Hz en SITL ; le récepteur réel fera 5–10 Hz (TP 3.0).
- Origine locale et premier fix GNSS confondus en SITL ; pas forcément sur le terrain (TP 3.2).

## Trois questions

**1. Pourquoi deux cadences IMU (brute 2000 Hz pour le contrôleur, intégrée `IMU_INTEG_RATE` pour l'estimateur) ?**
Le contrôleur de vitesse angulaire calcule une erreur et une commande (PID, quelques multiplications) et ne tolère pas le retard : un retard dans cette boucle mange la marge de phase. L'estimateur propage un état de 16 variables et sa covariance (produits de matrices 16×16, ~10⁴ opérations par pas) : trop cher à 2000 Hz, et ce qu'il suit (biais, gravité, position) évolue lentement. Intégrer les échantillons sur une fenêtre réduit le bruit (≈ √N) au prix d'un retard acceptable pour lui. Note : PX4 n'effectue pas une moyenne mais une intégration en incréments d'angle et de vitesse (TP 2.3). Asymétrie à retenir : estimateur cher et tolérant au retard, contrôleur bon marché et intolérant au retard. Détail de la chaîne : TP 1.5, doc v1.17 « Filter/Control Latency Tuning ».

**2. Pourquoi `ref_lat`/`ref_lon`/`ref_alt` sont-ils constants ?**
Ils sont les coordonnées géographiques du point (0, 0, 0) du repère local ; `x`, `y`, `z` sont définis par rapport à lui. Si l'origine bougeait, la position mesurée sauterait sans mouvement réel et le contrôleur de position corrigerait un déplacement fictif. L'origine locale est fixée au démarrage d'EKF2 ; ses coordonnées géographiques (`ref_*`) n'existent qu'à partir du premier fix GNSS fusionné (`xy_global` = true).

**3. Pourquoi `z` est négatif quand le drone monte ?**
`msg/VehicleLocalPosition.msg` : « Fused local position in NED ». North-East-Down : D est aligné sur la verticale locale (la gravité), vers le bas. Monter = `z` qui décroît. Repère tangent local, pas géocentrique.

## À creuser plus tard

- TP 1.1 : conventions NED/FRD, quaternion Hamilton w-x-y-z.
- TP 1.5 : chaîne capteurs → `vehicle_imu` → EKF2 / contrôleur de rate, paramètres `IMU_GYRO_RATEMAX`, `IMU_INTEG_RATE`.
- TP 2.3 : intégration des incréments d'angle/vitesse.
- TP 3.2 : `xy_global`, démarrage sans GNSS, bascule de mode.
- TP 4.4 : profil de logging haute cadence pour les actionneurs.

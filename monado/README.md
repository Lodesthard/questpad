# QuestPad → Monado : ce que le dongle envoie

Guide pour écrire le driver Monado (fork Android) qui lit les manettes Touch via QuestPad.
Référence détaillée : [docs/vr.md](../docs/vr.md). Parser C prêt à l'emploi : [questpad_vr.h](questpad_vr.h).

## En bref
- Le dongle (nRF52840) est branché en USB OTG sur le téléphone, en **mode VR** (LED orange).
- Il présente **deux interfaces USB vendor**, une par manette : gauche et droite sont deux appareils séparés.
- Chaque interface envoie un **rapport de 64 octets** environ **500 fois par seconde** : orientation (quaternion déjà
  fusionné sur le dongle), vitesse angulaire, accélération, sticks, gâchette, grip, boutons, capteurs tactiles, batterie.
- Le runtime peut envoyer des **commandes** : vibration, recentrage, reset de l'orientation.
- **3DoF uniquement** : rotation, pas de position. À placer avec un modèle bras / cou.
- Repères **OpenXR** : le quaternion s'utilise tel quel dans un `xrt_quat` (même ordre x, y, z, w).

État : testé dans le simulateur (40 scénarios, dont l'orientation des deux manettes à travers toute la chaîne radio).
**Pas encore testé sur vraies manettes ni sur téléphone** : la latence réelle et la dérive réelle restent à mesurer.

## 1. Trouver le dongle
| | valeur |
|---|---|
| VID:PID | `1209:5151` (mêmes ids dans tous les modes) |
| bcdDevice | `0x0300` en mode VR (les autres modes ont une autre valeur) |
| produit | `QuestPad VR` |

Interfaces (les chercher par classe / sous-classe / protocole, **pas par numéro**) :

| classe / sous-classe / protocole | endpoints | |
|---|---|---|
| `FF / 51 / 01` | interrupt IN 64 o (intervalle 1 ms) + interrupt OUT 64 o | manette **gauche** |
| `FF / 51 / 02` | interrupt IN 64 o (intervalle 1 ms) + interrupt OUT 64 o | manette **droite** |
| CDC ACM (2 interfaces) | | port série du panneau de contrôle : à ignorer |

Sur Android : aucun driver noyau ne prend une interface vendor, donc pas besoin de root.
1. Côté Java/Kotlin : `UsbManager` et permission sur `1209:5151`. Un `device_filter.xml` avec ce VID/PID permet au
   système de proposer l'appli au branchement.
2. Ouvrir l'appareil et passer le file descriptor au code natif.
3. Côté natif : `libusb_set_option(NULL, LIBUSB_OPTION_NO_DEVICE_DISCOVERY)`, puis
   `libusb_wrap_sys_device(ctx, fd, &handle)`, puis claim des deux interfaces de sous-classe `0x51`.

Sur Linux desktop (pour développer) : `libusb_open_device_with_vid_pid` marche directement (règle udev pour
`1209:5151`, ou root).

## 2. Le rapport d'état (IN, 64 octets, little-endian)
| octets | type | contenu |
|---|---|---|
| 0 | u8 | type de rapport : `0x01` = état. Ignorer tout autre type |
| 1 | u8 | version du protocole : `1`. Refuser une autre version |
| 2 | u8 | flags (voir plus bas) |
| 3 | u8 | numéro de séquence, +1 à chaque nouveau rapport (repasse à 0 après 255) |
| 4..8 | u32 | horodatage de l'échantillon IMU, µs, **horloge de la manette** (déborde) |
| 8..12 | u32 | horodatage de construction du rapport, µs, **horloge du dongle** (déborde) |
| 12..28 | f32 × 4 | orientation, quaternion **x, y, z, w**, repère manette → monde |
| 28..40 | f32 × 3 | vitesse angulaire, rad/s, **repère manette**, biais du gyro déjà retiré |
| 40..52 | f32 × 3 | accélération (force spécifique), m/s², repère manette : +9,81 sur y à plat et immobile |
| 52..56 | i16 × 2 | stick x, y : +x droite, +y haut, ±32000 = à fond, 0, 0 au repos. Valeur brute de la manette : le dongle n'ajoute pas de zone morte |
| 56..58 | u16 | gâchette, 0 relâchée .. 4095 à fond |
| 58..60 | u16 | grip, 0 .. 4095 |
| 60 | u8 | boutons : `0x01` A / X, `0x02` B / Y, `0x04` clic stick, `0x08` Oculus (droite) / Menu (gauche) |
| 61 | u8 | tactile : `0x01` A / X, `0x02` B / Y, `0x04` stick, `0x08` gâchette ; `0x10`..`0x80` = doigt *proche* des quatre mêmes |
| 62 | u8 | batterie %, 255 = inconnue |
| 63 | u8 | réservé, 0 |

`questpad_vr_parse()` décode tout ça dans une `struct questpad_vr_state`. Il ramène aussi le stick à -1..1, la
gâchette et le grip à 0..1, et la batterie inconnue à -1.

### Flags (octet 2)
| bit | nom (`QUESTPAD_VR_FLAG_*`) | sens | quoi en faire |
|---|---|---|---|
| `0x01` | `CONNECTED` | la manette est connectée au dongle | sinon : appareil inactif, pose invalide |
| `0x02` | `RIGHT` | rapport de la manette droite | redondant avec l'interface, utile en UDP (voir §6) |
| `0x04` | `ORIENTATION` | orientation initialisée par la gravité | pose valide seulement si `CONNECTED` et `ORIENTATION` |
| `0x08` | `GYRO_CAL` | biais du gyro mesuré au moins une fois (manette posée immobile 1 s) | facultatif : afficher « posez les manettes une seconde » tant qu'il manque |
| `0x10` | `STANDBY` | manette en veille | traiter comme inactive |
| `0x20` | `DEAD_BATTERY` | batterie vide, la manette va s'éteindre | prévenir l'utilisateur |
| `0x40` | `IMU_FRESH` | ce rapport contient un nouvel échantillon IMU | utile seulement si vous faites votre propre fusion |
| `0x80` | `IMU_SCALE` | unités de l'IMU lues sur la manette | info de diagnostic |

### Cadence
- Un nouveau rapport par paquet radio de la manette : toutes les 2 ms (~500 Hz) tant qu'elle est connectée.
- S'il n'y a rien de nouveau (manette éteinte, hors de portée, en veille), le dongle **renvoie le dernier rapport
  toutes les 100 ms**, avec le **même numéro de séquence** et un horodatage dongle à jour.
- Donc : **séquence inchangée = pas de nouvelle donnée**, ne pas la repousser dans l'historique de pose.
- Au démarrage, avant toute manette : rapport « déconnecté » (flags = seulement `RIGHT` le cas échéant, quaternion
  identité, batterie 255).

## 3. Repères
Conventions **OpenXR**, donc directement celles de Monado :
- **Monde** : X droite, Y haut, Z vers l'arrière (vers l'utilisateur).
- **Manette** tenue naturellement : X droite, Y haut, −Z vers l'avant (direction où elle pointe).
- **Yaw 0** = la manette pointait vers −Z du monde au moment où elle s'est connectée ou au dernier recentrage.
- Le quaternion fait passer du repère manette au repère monde.
- La vitesse angulaire et l'accélération sont dans le **repère manette**. En monde : `ω_monde = q · ω · q⁻¹`.
- Contrôle rapide : après un recentrage, manette pointée devant soi, on lit yaw 0, pitch 0, roll 0. Tourner à
  gauche augmente le yaw, lever l'avant augmente le pitch, rouler vers la gauche augmente le roll.

Le roll et le pitch sont absolus (grâce à la gravité). Le yaw est **relatif** : les Touch n'ont pas de
magnétomètre, il peut dériver lentement. D'où le recentrage (§4).

## 4. Commandes (OUT, sur l'endpoint de la manette visée)
| octets | commande | fonction du header |
|---|---|---|
| `10 aa dd dd` | **vibration** : amplitude `aa` (0 = arrêt, 255 = max), durée `dddd` ms en u16 LE (0 = jusqu'à la prochaine commande vibration) | `questpad_vr_build_haptic(buf, amplitude_0_1, ms)` |
| `11` | **recentrage** : le cap actuel devient yaw 0 (pitch et roll gardés) | `questpad_vr_build_recenter(buf)` |
| `12` | **reset** : orientation repart de la gravité (recentre aussi) | `questpad_vr_build_reset(buf)` |

- Une commande par transfert OUT. Un octet de commande inconnu est ignoré.
- Les moteurs des Touch n'ont qu'une intensité, pas de fréquence : la fréquence demandée par OpenXR est ignorée.
- `XRT_MIN_HAPTIC_DURATION` (-1) de Monado : envoyer une impulsion courte, par exemple 20 ms.
- Les vibrations propres au dongle (connexion, geste d'extinction, batterie faible) passent brièvement devant.
- **Recentrer les deux manettes en même temps que la tête** (appui long sur Oculus, ou quand le runtime recentre) :
  envoyer `11` sur les deux OUT et remettre le yaw de la tête à zéro au même moment, pour que −Z du monde
  corresponde à la direction où regarde l'utilisateur.

## 5. Le driver, étape par étape
1. **Un `xrt_device` par interface** : protocole `0x01` → main gauche, `0x02` → main droite
   (`XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER` / `RIGHT_HAND_CONTROLLER`). Le profil Touch
   (`XRT_DEVICE_TOUCH_CONTROLLER`) correspond aux entrées une pour une.
2. **Un thread de lecture par interface** : `libusb_interrupt_transfer` sur l'IN (64 octets), puis
   `questpad_vr_parse`. Ignorer les rapports dont `seq` n'a pas changé.
3. **Pose** : pousser chaque nouvelle orientation dans un `m_relation_history`, horodatée à l'arrivée côté hôte moins
   la latence radio + USB (quelques ms, à mesurer). Orientation valide seulement si `CONNECTED` et `ORIENTATION`.
   Pour la prédiction, la vitesse angulaire est fournie (repère manette, la passer en monde).
4. **Entrées** :

   | rapport | OpenXR (profil Touch) |
   |---|---|
   | stick x, y | `/input/thumbstick` |
   | boutons `0x04` | `/input/thumbstick/click` |
   | tactile `0x04` | `/input/thumbstick/touch` |
   | gâchette | `/input/trigger/value` |
   | tactile `0x08` | `/input/trigger/touch` |
   | grip | `/input/squeeze/value` |
   | boutons `0x01` / `0x02` | droite `/input/a/click`, `/input/b/click` ; gauche `/input/x/click`, `/input/y/click` |
   | tactile `0x01` / `0x02` | les `/touch` des mêmes boutons |
   | boutons `0x08` | droite `/input/system/click` ; gauche `/input/menu/click` |
   | — | `/input/grip/pose`, `/input/aim/pose` : l'orientation, placée par le modèle de bras |
   | — | `/output/haptic` → `questpad_vr_build_haptic` |

5. **Placement** : helpers 3DoF de Monado, comme pour les autres manettes sans position : décalage fixe depuis la
   tête, tourné du yaw de la tête, puis orientation de la manette appliquée autour du poignet.
6. **Recentrage** : voir §4.

Variante : si le fork préfère sa propre fusion (`m_imu_3dof`), lui donner `angular_velocity` et `acceleration` (déjà
en repère manette et en unités SI) avec `imu_ts_us` comme temps d'échantillon, uniquement quand `IMU_FRESH` est
présent, et ignorer le quaternion.

### Horodatage
- `imu_ts_us` (horloge manette) et `dongle_ts_us` (horloge dongle) sont deux horloges différentes, en µs sur 32 bits :
  elles débordent toutes les ~71 minutes. Toujours calculer les écarts en `uint32_t` (soustraction modulo 2³²).
- Pour un horodatage hôte stable : suivre le décalage `temps_hôte − dongle_ts` et garder son minimum glissant (le
  minimum élimine la gigue USB), puis horodater chaque rapport à `dongle_ts + décalage`.

## 6. Développer sans matériel : le simulateur
Le dépôt contient un simulateur qui fait tourner le vrai firmware sur PC avec deux manettes virtuelles.

```sh
cd sim
cargo run --release -- live            # dongle virtuel en temps réel (Rust stable requis)
cargo run --release -- live --auto     # les manettes bougent toutes seules
python host/vrsim_view.py              # vue 3D (à lancer depuis la racine) : on oriente les manettes à la souris
```

- **Rapports** : envoyés en UDP sur `127.0.0.1:26770`, **octet pour octet comme sur USB**. Un driver peut donc lire
  un socket au lieu de l'endpoint pendant le développement.
  - Les deux manettes arrivent sur le même port : les distinguer par le flag `RIGHT` (octet 2, `0x02`).
  - Les datagrammes qui commencent par `0xF0` sont la vérité terrain pour la vue 3D : les ignorer (ou s'en servir
    pour mesurer l'erreur : `0xF0`, côté u8 0/1, quaternion f32 x y z w, amplitude moteur u8).
- **Commandes** : en texte sur UDP `127.0.0.1:26771`, une par datagramme : `L cmd=<hex>` ou `R cmd=<hex>`,
  par exemple `R cmd=10c86400` (vibration 200 pendant 100 ms) ou `L cmd=11` (recentrage).
- Le simulateur démarre avec un biais de gyro comme une vraie manette : 1 à 2° de dérive en yaw tant que la manette
  n'a pas été « posée » une seconde (ou appuyer sur `r` dans la vue 3D).

## 7. Vérifier sur PC avec le vrai dongle
```sh
pip install pyusb libusb-package
python host/vrtest.py               # orientation + entrées des deux manettes en direct
python host/vrtest.py --buzz right  # test vibration
python host/vrtest.py --recenter
```
`host/vrtest.py` lit les rapports comme le ferait le runtime : c'est une implémentation de référence courte (Python)
à comparer en cas de doute.

## 8. Compatibilité
- Le format du rapport est partagé par `firmware/src/vr.rs`, `monado/questpad_vr.h` et le simulateur.
- Toute modification du format **incrémente l'octet de version** (octet 1). `questpad_vr_parse` refuse une version
  inconnue : mettre à jour le header en même temps que le firmware.
- Exception : de nouveaux bits de flags dans des positions libres, ou l'usage de l'octet réservé, ne changent pas la
  version. Ignorer les bits inconnus.

## 9. Côté téléphone, à savoir
- Le dongle est alimenté par le téléphone via l'OTG.
- Si les manettes s'éteignent quand l'écran du téléphone s'éteint, c'est que le téléphone suspend le port USB.
  Décocher *Turn controllers off when the computer sleeps* dans le panneau de contrôle (PC, une fois : le dongle
  garde le réglage).
- Les manettes s'appairent et passent en mode VR depuis le PC (panneau de contrôle) ou depuis le dongle seul (voir
  le README). Une fois réglé, le dongle fonctionne seul.

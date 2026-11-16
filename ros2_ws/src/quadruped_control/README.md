# Commandes, IMU, caméra et visualisation LiDAR ROS

`serial_bridge` remplace les deux lecteurs série `command_recep` et `gyroscope`.
Il ouvre une seule liaison Arduino : les commandes ROS sont écrites sur ce port,
et un thread lit les réponses et les données IMU.

```text
command_emit -> command -> serial_bridge <-> Arduino
                               |
                               +-> imu/data (sensor_msgs/msg/Imu)

camera_ros -> camera/image_raw -> web_video_server -> navigateur du Mac
           -> camera/camera_info

pilote LiDAR -> scan -> reception_lidar (Matplotlib WebAgg) -> navigateur du Mac
```

`command_emit.py` et le firmware Arduino restent inchangés. Les anciens nœuds
`command_recep` et `gyroscope` sont supprimés : utiliser `serial_bridge`.
`reception_gyroscope` reste un observateur facultatif du topic ; il n'ouvre aucun
port série. La caméra utilise les packages existants `camera_ros` et
`web_video_server`, sans recopier leur code dans ce dépôt.

## Configuration et lancement commun

Deux fichiers organisent le démarrage :

- `config/robot.yaml` contient les réglages, regroupés par nom de nœud sous
  `ros__parameters`. ROS transmet à chaque nœud le bloc qui le concerne.
- `launch/robot.launch.py` démarre les trois programmes et leur passe ce fichier.
  Les noms des nœuds sont fixés pour correspondre au YAML et conserver les topics
  `/command`, `/imu/data`, `/camera/image_raw` et `/camera/camera_info`.

Le YAML reprend le port série `/dev/ttyUSB0` à 115200 bauds, le repère `imu_link`,
la conversion degrés/s vers radians/s, les images 640 × 480 et la rotation 180°.
Le mode capteur `"1640:1232"` reprend le réglage testé sur l'IMX219 pour éviter le
recadrage du mode 640 × 480. Il s'agit d'une chaîne de caractères, d'où les
guillemets. Le serveur web écoute sur le port 8080 et s'abonne avec le profil QoS
`sensor_data` déjà utilisé pour la caméra.

Les valeurs du YAML remplacent les valeurs par défaut déclarées dans les nœuds,
sans ajouter de lecteur YAML dans leur code. Ces réglages sont appliqués au
démarrage : modifier le fichier source, reconstruire le package, puis relancer.
Les valeurs par défaut du bridge restent utiles lorsqu'il est lancé seul sans
fichier de paramètres.

Références : [paramètres de camera_ros 0.7.0](https://github.com/christianrauch/camera_ros/tree/0.7.0#parameters),
[web_video_server](https://github.com/RobotWebTools/web_video_server/tree/ros2).

## Construire et lancer sur le Pi

Les commandes suivantes sont à exécuter sur le Pi après transfert des fichiers.
Elles supposent que le workspace caméra qui fonctionne est toujours installé
dans `~/camera_ws`. Dans une nouvelle session SSH :

```bash
source /opt/ros/jazzy/setup.bash
source ~/camera_ws/install/local_setup.bash
cd ~/Quad_nav/ros2_ws
rosdep install --from-paths src --ignore-src -r -y --skip-keys "camera_ros libcamera"
colcon build --symlink-install --packages-select quadruped_control
source install/local_setup.bash
ros2 pkg prefix quadruped_control
ros2 pkg prefix camera_ros
ros2 launch quadruped_control robot.launch.py
```

`rosdep` installe les dépendances déclarées dans `package.xml`. On exclut ici
`camera_ros` et `libcamera`, déjà construits dans `camera_ws` : ce lancement doit
réutiliser la version qui fonctionne. Il ne reconstruit pas le workspace caméra.

Le préfixe de `quadruped_control` doit désigner
`~/Quad_nav/ros2_ws/install/quadruped_control`, et celui de `camera_ros` doit être
dans `~/camera_ws/install`. Attention à une ancienne installation dans
`~/Quad_nav/install`. Les fichiers `local_setup.bash` ajoutent le workspace voulu
à l'environnement courant sans recharger ses anciens workspaces parents.

Pour les sessions suivantes, sans changement des fichiers :

```bash
source /opt/ros/jazzy/setup.bash
source ~/camera_ws/install/local_setup.bash
source ~/Quad_nav/ros2_ws/install/local_setup.bash
ros2 launch quadruped_control robot.launch.py
```

Le launch démarre les trois nœuds par défaut. Arrêter leurs anciennes instances
avant de l'utiliser, pour ne pas ouvrir deux fois la caméra ou le port série.

Pour tester uniquement la caméra et le serveur web :

```bash
ros2 launch quadruped_control robot.launch.py use_serial:=false
```

Pour tester uniquement le bridge :

```bash
ros2 launch quadruped_control robot.launch.py use_camera:=false use_web_video:=false
```

Ces trois options sont indépendantes. `use_web_video:=false` permet notamment
de publier les images dans ROS sans ouvrir de serveur web. Les arguments
disponibles peuvent être affichés sans démarrer le matériel :

```bash
ros2 launch quadruped_control robot.launch.py --show-args
```

Pour essayer un autre fichier de configuration, fournir son chemin complet :

```bash
ros2 launch quadruped_control robot.launch.py params_file:=/chemin/vers/essai.yaml
```

Le launch trouve le YAML installé grâce au nom du package : aucun chemin propre
au Mac ou au dossier personnel du Pi n'est inscrit dans le code de lancement.
`setup.py` installe les dossiers `launch/` et `config/` avec le package.

## Envoyer des commandes et visualiser

L'émetteur reste dans un deuxième terminal SSH, car il attend une saisie clavier :

```bash
source /opt/ros/jazzy/setup.bash
source ~/Quad_nav/ros2_ws/install/local_setup.bash
ros2 run quadruped_control command_emit
```

Sur le Mac, ouvrir dans le navigateur, en remplaçant `IP_DU_PI` par l'adresse du
Pi (par exemple son adresse Tailscale) :

```text
http://IP_DU_PI:8080/stream_viewer?topic=/camera/image_raw
```

Le navigateur reçoit la vidéo du serveur web sur le Pi ; ROS n'est pas nécessaire
sur le Mac. L'absence de fichier de calibration peut encore produire un
avertissement caméra ; ce lancement ne réalise pas de calibration.

Pour observer les données dans un autre terminal avec ROS chargé :

```bash
ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
# Ou, pour ouvrir le visualiseur IMU sur le port 8082 :
ros2 run quadruped_control reception_gyroscope
```

## Graphique LiDAR dans le navigateur

`reception_lidar` affiche les points de `/scan` dans le repère du LiDAR. Il reste
un observateur facultatif, lancé séparément du launch commun. Le pilote du
LiDAR doit déjà publier des messages `sensor_msgs/msg/LaserScan` sur `/scan`.

Après transfert des fichiers, sur le Pi :

```bash
source /opt/ros/jazzy/setup.bash
sudo apt install python3-matplotlib python3-tornado
cd ~/Quad_nav/ros2_ws
colcon build --symlink-install --packages-select quadruped_control
source install/local_setup.bash
ros2 run quadruped_control reception_lidar
```

Sur le Mac, ouvrir `http://IP_DU_PI:8081`, en remplaçant `IP_DU_PI` par l'adresse
du Pi, par exemple son IP Tailscale. La page se met à jour automatiquement ;
aucun bureau sur le Pi ni ROS sur le Mac n'est nécessaire. `Ctrl+C` dans le
terminal du nœud arrête le serveur. Fermer l'onglet laisse le nœud actif.

Trois constantes en haut de `reception_lidar.py` règlent le port (`WEB_PORT =
8081`), la vue initiale (`VIEW_RANGE = 5.0`, soit ±5 mètres) et la période
(`REFRESH_MS = 100`, soit au maximum 10 mises à jour/s). Le port 8080 reste
disponible pour la caméra. Si 8081 est occupé, le programme signale l'erreur sans
choisir silencieusement un autre port.

Les angles et distances sont convertis par `x = r cos(angle)` et
`y = r sin(angle)`. Les points non finis ou hors de la plage du capteur sont
ignorés. Chaque relevé remplace le précédent, y compris un relevé sans point
valide. L'abonnement conserve seulement le dernier scan et accepte le QoS
best effort utilisé par les pilotes de capteurs. La fréquence réelle dépend
du LiDAR, du Pi et du réseau. Si les messages cessent, le dernier relevé reste
affiché ; ce visualiseur ne surveille pas encore les interruptions du capteur.

La croix rouge marque le LiDAR, les axes sont en mètres, avec la même échelle.
Les commandes Matplotlib de la page permettent de zoomer et déplacer la vue.
Pour un pilote publiant sur un autre topic :

```bash
ros2 run quadruped_control reception_lidar --ros-args -r scan:=/autre_scan
```

Le graphique est calculé sur le Pi et transmis au navigateur par
[Matplotlib WebAgg](https://matplotlib.org/stable/users/explain/figure/backends.html#interactive-backends).

## Paramètres et messages

| Topic | Type | Producteur → consommateur |
|---|---|---|
| `/command` | `std_msgs/msg/String` | `command_emit` → `serial_bridge` |
| `/imu/data` | `sensor_msgs/msg/Imu` | `serial_bridge` → observateurs ROS |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | `camera` → `web_video_server` quand la vidéo est consultée |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | `camera` → outils de vision |
| `/scan` | `sensor_msgs/msg/LaserScan` | pilote LiDAR externe → `reception_lidar` |

| Paramètre | Valeur par défaut | Rôle |
|---|---|---|
| `serial_port` | `/dev/ttyUSB0` | Port de l'adaptateur CP2102 |
| `baud_rate` | `115200` | Débit de la liaison Arduino |
| `imu_frame_id` | `imu_link` | Repère des mesures, sans rotation des axes |
| `gyro_in_degrees` | `true` | Conversion des degrés/s reçus en radians/s |

Exemple de changement de port :

```bash
ros2 run quadruped_control serial_bridge --ros-args -p serial_port:=/dev/ttyUSB1
```

Cette commande lance le bridge seul : ne pas le laisser déjà actif dans le
launch. Avec le lancement commun, changer `serial_port` dans le YAML.

La trame attendue reste `IMU,ax,ay,az,gx,gy,gz`. Les six valeurs doivent être
finies. Les lignes incorrectes sont ignorées avec un avertissement ; les lignes
coupées par un timeout sont conservées jusqu'au prochain saut de ligne.

Le firmware présent dans ce dépôt utilise
`bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE)`. La bibliothèque Adafruit renvoie
des **degrés/s**, malgré le commentaire « rad/s » du firmware. Le bridge les
convertit donc en **rad/s**, unité attendue par ROS. Si le firmware réellement
chargé convertit déjà ces valeurs en radians/s, lancer avec
`--ros-args -p gyro_in_degrees:=false` pour éviter une double conversion.

Les accélérations restent en m/s², avec la gravité (VECTOR_ACCELEROMETER).
L'orientation n'est pas fournie : `orientation_covariance[0] = -1`.
Les autres covariances restent inconnues (zéro). L'horodatage correspond à la
réception sur le Pi, pas à l'instant d'acquisition sur l'Arduino. Les données ne
sont pas transformées dans le repère du corps.

Sources : [Adafruit BNO055](https://github.com/adafruit/Adafruit_BNO055/blob/master/Adafruit_BNO055.cpp),
[message ROS Imu](https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/Imu.msg).

Les lignes `STATE`, `STATUS`, `ERR`, etc. restent affichées dans le journal ROS.
Elles ne sont pas encore publiées dans un topic dédié. Les mesures IMU ne sont
pas imprimées systématiquement par le bridge.

## Commandes et arrêt

- `start`, `stand`, `stop`, `status` : même transmission texte qu'avant.
- `quit` : envoie `stand`, attend la fin de l'écriture avec `flush()`, puis ferme
  le bridge. Cela n'attend pas de confirmation Arduino ni la fin du mouvement.
- `Ctrl+C` : ferme le bridge sans envoyer de commande, comme auparavant.

Avec le launch, `quit` ferme seulement le bridge (et l'émetteur). La caméra et
le serveur web restent actifs. `Ctrl+C` dans le terminal du launch arrête tous
les processus lancés. Le launch n'envoie pas de commande moteur à leur place :
pour demander `stand` avant de fermer l'ensemble, saisir d'abord `quit` dans
l'émetteur. Aucun nœud n'est relancé automatiquement après sa fermeture.

Le thread de lecture est arrêté avant la fermeture du port et la destruction du
nœud. Une erreur de lecture ferme le bridge ; il n'y a pas encore de reconnexion
automatique. Le verrou série exclusif empêche deux bridges de prendre le même
port, mais ne remplace pas l'arrêt des anciens scripts série.

## Vérification

Tests de logique avec ROS et le port série simulés, exécutables aussi sur Mac
depuis le dossier `ros2_ws/src/quadruped_control` :

```bash
python3 -m unittest discover -s test -p 'test_serial_bridge.py' -v
```

Sur le Pi, vérifier les options avec `--show-args`, puis lancer séparément la
caméra et le bridge avant le lancement complet. Vérifier que `status` renvoie les
réponses Arduino pendant que `/imu/data` est publié, que la vidéo est visible et
que `quit` ferme le bridge après l'envoi de `stand`.

La syntaxe Python/XML/YAML et l'installation des fichiers peuvent être vérifiées
sur Mac, mais l'exécution du launch avec ROS Jazzy et les essais matériels restent
à faire sur le Pi. Les tests locaux ne valident pas le transport DDS, le câblage,
les unités du firmware chargé ou la cadence réelle.

## Visualisation IMU séparée

Après reconstruction, dans un nouveau terminal SSH avec Jazzy et le workspace
chargés : `ros2 run quadruped_control reception_gyroscope`. Ouvrir sur le Mac
`http://IP_DU_PI:8082`. `serial_bridge` doit déjà publier `/imu/data`
(par exemple via le launch du terminal A). Ne pas lancer un second bridge.

Deux graphiques montrent les accélérations (gravité incluse) et les vitesses
angulaires X/Y/Z sur les dix dernières secondes. Le Pi sert les dernières
valeurs en JSON ; le navigateur dessine les courbes, sans bibliothèque externe.
L'affichage est échantillonné à environ 10 Hz et ne constitue pas un enregistrement
de toutes les mesures ROS. Le temps affiché est celui de réception dans le
navigateur. Les interruptions sont signalées et les anciennes courbes défilent
hors de la fenêtre. Aucun calcul d'orientation n'est ajouté.

Ce nœud remplace l'ancien affichage texte de reception_gyroscope. Il n'ouvre
aucun port série et ne publie pas un deuxième topic IMU. Ctrl+C arrête le serveur.
Les réglages WEB_PORT et REFRESH_MS sont en haut du fichier Python.

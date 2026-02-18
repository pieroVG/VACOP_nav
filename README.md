# RTAB-Map + Nav2 (localisation et navigation locale)

Ce dépôt met en place une **localisation basée sur une carte existante** et une **navigation locale avec Nav2**

## 1. Prérequis

- Ubuntu 22.04  
- ROS 2 **Humble**  
- Colcon  

## 2. Installations nécessaires

```bash
sudo apt update
sudo apt-get install ros-humble-controller-manager
sudo apt-get install ros-humble-ros2-control
sudo apt-get install ros-humble-xacro

sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-rtabmap-ros
sudo apt install ros-humble-robot_localization
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

## 3. Compilation du workspace

Depuis la racine du workspace :

```bash
colcon build --symlink-install
source install/setup.bash
```

Source à refaire dans chaque nouveau terminal.

## 4. Lancement de l'urdf du VACOP et Rviz

```bash
ros2 launch vacop display.launch.py
```

## 5. Lancement de la carte 

La localisation utilise une **base carte existante** ou le topic /map si déjà publié

```bash
ros2 launch rtabmap_localization clean_map.launch.py
```

Fonctionnement :
- Charge la carte située dans le dossier /maps de `rtabmap_localization`
- Publie la frame `base_footprint`
- Publie la carte `/map` 
- Odométrie simulé avec `cmd_vel` pour la simulation

### Paramètres RViz (déjà configuré)

- Fixed Frame : base_footprint pour suivre le VACOP et map sinon
- Map > Topic > Durability Policy : Transient Local

Si la carte n’apparaît pas :
- relancer :
```bash
ros2 launch rtabmap_localization clean_map.launch.py
```

![Rviz](images/rviz_ok.png)

## 6. Lancement de Nav2 et MPPI

```bash
ros2 launch planif_locale nav2.launch.py
```
À utiliser avec `2D Goal Pose` de Rviz
- planification locale
- évitement d’obstacles

## 7. Odométrie

```bash
ros2 run odometry gps_odometry_node
```
Utilisation du GPS car problème avec les capteurs de vitesse des roues.

## 8. Frames TF 

Toute la navigation repose sur cette chaîne :

```
map → odom → base_footprint → base_link
```

Si une transformation manque ou est incorrecte,
Nav2 et RViz ne fonctionneront pas correctement.


**Carte / Localisation**

```
map → odom
```

Permet de positionner le robot dans la carte globale.

**Odométrie**

```
odom → base_footprint
```

Donne le déplacement continu du robot.

**VACOP (URDF)**

```
base_footprint → base_link
```

Définit la structure physique du robot.


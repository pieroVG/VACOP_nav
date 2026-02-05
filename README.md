# RTAB-Map + Nav2 (localisation et navigation locale)

Ce dépôt met en place une **localisation basée sur une carte RTAB-Map existante** et une **navigation locale avec Nav2**, sans odométrie réelle pour l’instant.

## 1. Prérequis

- Ubuntu 22.04  
- ROS 2 **Humble**  
- Colcon  

## 2. Installations de RTAB-Map et Nav2 (ROS2)

```bash
sudo apt update
sudo apt install ros-humble-rtabmap-ros
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

## 3. Compilation du workspace

Depuis la racine du workspace :

```bash
colcon build --symlink-install
source install/setup.bash
```

Source à refaire dans chaque nouveau terminal.

## 5. Lancement de la localisation RTAB-Map

La localisation utilise une **base RTAB-Map existante (`.db`)**

```bash
ros2 launch rtabmap_localization map.launch.py
```

Fonctionnement :
- Recharge la carte depuis `rtabmap.db`
- Publie la frame `map`
- Publie la carte `/map` (via `map_assembler`)
- **Pas d’odom pour l’instant**


## 6. Lancement de l'urdf du VACOP et Rviz

```bash
ros2 launch vacop display.launch.py
```

### Paramètres RViz

- Fixed Frame : map
- Map > Topic > Durability Policy : Transient Local


Si la carte n’apparaît pas :
- relancer :
```bash
ros2 launch rtabmap_localization map.launch.py
```

RTAB-Map publie la carte à la connexion d’un abonné.

![Rviz](images/rviz_ok.png)

## 7. Lancement de Nav2 (en cours)

```bash
ros2 launch planif_locale nav2.launch.py
```

Objectif :
- planification locale
- évitement d’obstacles

Limitations :
- pas d’odom réel

## 8. Frames TF (état actuel)

```
map → odom → base_footprint
```

- `map → odom` : RTAB-Map  
- `odom → base_footprint` : transform statique pour l'instant


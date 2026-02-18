# Objectif
Ce package contient :

- URDF du véhicule
- Publication des TF
- Visualisation RViz

# TF publiés
```
base_footprint → base_link
```
Rôle :
- fournir une structure cohérente à ROS2 et Nav2 
- permettre la visualisation correcte dans RViz

En effet, l'urdf et les TF de ce modèle du VACOP commencent par `base_footprint` qui est ensuite lié à `base_link` dans ce package.

Il est également important de désactiver Rviz dans le launch si l'on souhaite le lancer depuis un Rasberry pi.
# Objectif

Ce package permet de localiser le VACOP dans la carte locale à partir :
- du GPS
- de l’IMU
- d’une estimation d’odométrie

Il repose sur le package robot_localization pour fusionner les capteurs et produire une estimation cohérente de la position du véhicule.

# Limitation actuelle

- Carte locale non géo-référencée
- Alignement à faire manuellement
- Erreurs dues au bruit GPS
- Pas encore testé 

Une carte géo-référencée pourrait éviter l'utiliser de ce package entier.

# Prochaines étapes:
Avec les points géo-référencés, on pourrait localiser le VACOP dans la carte. Ce qui permettrait à nav2 de corriger la trajectoire avec une navigation autonome basée sur la position absolue.

Cela rendrait la planification et la génération de trajectoire autonome sur le véhicule.

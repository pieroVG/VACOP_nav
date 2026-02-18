# Objectif
Ce package utilise Nav2 avec le contrôleur MPPI pour :

- générer une trajectoire locale
- éviter les obstacles
- produire une trajectoire et le topic /cmd_vel

Il constitue le cœur décisionnel de la navigation.


# MPPI (Model Predictive Path Integral)
[text](https://docs.nav2.org/configuration/packages/configuring-mppic.html)

Utilisé via Nav2 :
- Génération de trajectoires candidates
- Simulation forward du modèle du véhicule
- Évaluation via fonction de coût
- Sélection de la meilleure commande

# Paramètres principal : `nav2_params.yaml`
Situé dans le dossier `config`.

La configuration de l'angle de braquage à déjà été calculée pour le VACOP : `min_turning_r: 3.02`. Cependant un tunning sur les autres paramètres pourraient permettre une meilleure trajectoire, en prenant plus large sur les virages par exemple, ou plus grande distance de sécurité.

## Points critiques

- Interaction forte avec la costmap
- Dépendance à la qualité de l’odométrie

# Prochaines étapes:
- Avec les points géo-référencés, on pourrait localiser le VACOP dans la carte. Ce qui permettrait à nav2 de corriger la trajectoire avec la position réelle du véhicule. Cela rendrait la planification et la génération de trajectoire autonome sur le véhicule.
- S'assurer que le MPPI prend bien en compte les obstacles soudains
- Prendre en compte le code de la route (plutôt grâce à une planification globale en amont)
- Ajouter des coûts spécifiques (zones interdites, priorités, etc.)

# Objectif
Ce package contient deux modes :
- `fake_odom`
Simule l’odométrie à partir de `/cmd_vel`
→ utilisé uniquement en simulation

- `gps_odometry_node`
Fusion GPS + IMU pour estimer le déplacement réel du véhicule

# Fonctionnement actuel de `gps_odometry_node`

Entrées :
- `vacop/gnss/fix`
- IMU à ajouter pour avoir la bonne orientation

Sortie :
- `/odom`
- TF `odom → base_footprint`

# Prochaines étapes:
- Implémenter odométrie roues (encodeurs effet Hall) une fois que les capteurs seront fonctionnels. Une piste est dans le dossier `/odometry/odom_motors`
- Vérifier cohérence angle IMU

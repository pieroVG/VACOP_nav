#!/bin/bash


# Terminal 1 : VACOP
gnome-terminal -- bash -c "
source install/setup.bash;
ros2 launch vacop display.launch.py;
exec bash"
sleep 2

# Terminal 2 : RTABMAP
gnome-terminal -- bash -c "
source install/setup.bash;
ros2 launch rtabmap_localization map.launch.py;
exec bash"
sleep 2

# Terminal 3 : NAV2
gnome-terminal -- bash -c "
source install/setup.bash;
ros2 launch planif_locale nav2.launch.py;
exec bash"

# Loads ROS in layers: ROS 2 -> CARLA bridge -> our workspace (if already built).
source /opt/ros/humble/setup.bash
source /opt/carla_bridge/install/setup.bash
[ -f /home/dev/acav/ros2_ws/install/setup.bash ] && source /home/dev/acav/ros2_ws/install/setup.bash
true

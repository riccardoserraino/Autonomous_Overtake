# ACAV image: Ubuntu 22.04 + ROS 2 Humble + CARLA client and ROS bridge + our tools.
# ONE image for every module; compose.yml starts it as several containers.
# Our code is NOT copied here: the acav folder is mounted, so code changes need no rebuild.
# CARLA itself (the simulator) is NOT here either: it runs on your PC (see docs/CARLA.md).

FROM osrf/ros:humble-desktop
ARG CARLA_VERSION=0.9.16
ARG USER_UID=1000
ARG USER_GID=1000
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# 1) Ubuntu + ROS packages
#    mesa-* = graphics drivers for Intel/AMD GPUs (NVIDIA brings its own, see compose.yml)
RUN apt-get update && apt-get install -y --no-install-recommends \
      git sudo nano python3-pip python3-colcon-common-extensions \
      python3-numpy python3-scipy python3-matplotlib \
      ros-humble-derived-object-msgs ros-humble-ackermann-msgs \
      ros-humble-rosbridge-server ros-humble-foxglove-bridge \
      ros-humble-rosbag2-storage-mcap ros-humble-plotjuggler-ros \
      mesa-utils libgl1-mesa-dri libegl1 libglx-mesa0 mesa-vulkan-drivers \
    && rm -rf /var/lib/apt/lists/*

# 2) Python packages: CARLA client (SAME version as the CARLA you downloaded),
#    pygame (keyboard-driving window), simple-pid + transforms3d (needed by the bridge;
#    rosdep cannot install transforms3d, its key "python-transforms3d-pip" no longer exists)
RUN pip3 install --no-cache-dir carla==${CARLA_VERSION} pygame simple-pid transforms3d

# 3) CARLA ROS bridge, built once in /opt/carla_bridge
#    Its CARLA_VERSION file must equal the client version, otherwise it refuses to start.
#    Skipped: pcl_recorder, rviz_carla_plugin (not needed) and carla_ad_demo (needs rviz_carla_plugin).
#    --skip-keys: transforms3d is already installed by pip above.
RUN git clone --recurse-submodules https://github.com/carla-simulator/ros-bridge.git /opt/carla_bridge/src/ros-bridge \
 && echo ${CARLA_VERSION} > /opt/carla_bridge/src/ros-bridge/carla_ros_bridge/src/carla_ros_bridge/CARLA_VERSION \
 && cd /opt/carla_bridge && source /opt/ros/humble/setup.bash \
 && apt-get update && rosdep update && rosdep install --from-paths src --ignore-src -r -y --skip-keys python-transforms3d-pip \
 && colcon build --packages-skip pcl_recorder rviz_carla_plugin carla_ad_demo \
 && rm -rf build log /var/lib/apt/lists/*

# 3b) The keyboard-driving window (carla_manual_control) is fixed at 800x600 in its code:
#     make it read KEYBOARD_WINDOW_SIZE from docker/.env instead (drive_keyboard.sh passes it)
RUN f=/opt/carla_bridge/install/carla_manual_control/lib/python3.10/site-packages/carla_manual_control/carla_manual_control.py \
 && sed -i 's/resolution = {"width": 800, "height": 600}/w, h = __import__("os").environ.get("KEYBOARD_WINDOW_SIZE", "800x600").split("x"); resolution = {"width": int(w), "height": int(h)}/' $f \
 && grep -q KEYBOARD_WINDOW_SIZE $f

# 4) Normal user "dev" with YOUR user id, so files created inside belong to you
RUN groupadd -f -g ${USER_GID} dev && useradd -m -u ${USER_UID} -g ${USER_GID} -s /bin/bash dev \
 && echo "dev ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/dev \
 && echo "source /entrypoint.sh" >> /home/dev/.bashrc
# --chmod: readable by user "dev", whatever the file permissions are on your PC
COPY --chmod=755 entrypoint.sh /entrypoint.sh

# Optional: Gazebo Fortress (~1 GB). Remove the # signs only if you need it, then ./scripts/build.sh
# RUN apt-get update && apt-get install -y --no-install-recommends ros-humble-ros-gz \
#  && rm -rf /var/lib/apt/lists/*

USER dev
WORKDIR /home/dev/acav/ros2_ws
ENTRYPOINT ["/bin/bash", "-c", "source /entrypoint.sh && exec \"$@\"", "--"]
CMD ["bash"]

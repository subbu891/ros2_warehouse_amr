FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Workspace with all dependencies resolved by rosdep
WORKDIR /ws
COPY . src/ros2_warehouse_amr
RUN apt-get update \
 && rosdep update \
 && rosdep install --from-paths src --ignore-src -y \
 && rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/jazzy/setup.bash \
 && colcon build --event-handlers console_cohesion+ \
 && rm -rf build log

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]

# Default: full stack without GUI, fleet interface enabled
CMD ["ros2", "launch", "amr_bringup", "bringup.launch.py", "headless:=true", "rviz:=false", "vda5050:=true"]

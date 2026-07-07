# Omokai mission pipeline - portable container
# Matches dev environment: ROS2 Jazzy + Gazebo Harmonic (Gz Sim 8.x)
FROM osrf/ros:jazzy-desktop-full

SHELL ["/bin/bash", "-c"]

# --- ROS2/Gazebo Harmonic deps ---
# ros-jazzy-ros-gz brings ros_gz_sim + ros_gz_bridge (gz_sim.launch.py,
# parameter_bridge) -- this is the Harmonic-generation bridge your nodes
# actually use, NOT the old gazebo_ros / turtlebot3-gazebo (Classic) stack.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-ros-gz \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-rviz2 \
    ros-jazzy-tf-transformations \
    python3-pip \
    curl \
    && rm -rf /var/lib/apt/lists/*

# --- Python deps: LLM planner + web UI ---
RUN pip install --break-system-packages --no-cache-dir \
    jsonschema \
    anthropic \
    flask

ENV ROS_DOMAIN_ID=0

WORKDIR /omokai_ws
COPY src/ /omokai_ws/src/

RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /omokai_ws/install/setup.bash" >> /root/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# mission_ui's Flask server
EXPOSE 5000

# ANTHROPIC_API_KEY should be passed at `docker run` time with -e, never baked in.
ENTRYPOINT ["/entrypoint.sh"]
# ROS 2 Humble + Gazebo Classic + TurtleBot3, as listed in the recruitment Tutorial.txt.
#
#   docker build -f docker/Dockerfile -t tsb-control .           # full image, runs the tests
#   docker build -f docker/Dockerfile --target deps -t tsb-control-deps .
#
# Lets the project run on any Linux distribution; on Ubuntu 22.04 / WSL with ROS 2
# Humble installed you can skip Docker and use colcon directly (see README.md).
FROM osrf/ros:humble-desktop AS deps

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      ros-humble-gazebo-ros-pkgs \
      ros-humble-gazebo-ros \
      ros-humble-gazebo-plugins \
      ros-humble-turtlebot3 \
      ros-humble-turtlebot3-simulations \
      ros-humble-ament-lint-auto \
      ros-humble-ament-lint-common \
      ros-humble-ament-cmake-gtest \
      ros-humble-launch-testing-ament-cmake \
      ros-humble-rosbag2-py \
      python3-matplotlib \
      python3-numpy \
 && rm -rf /var/lib/apt/lists/*

# Same environment as Tutorial.txt, so headless Gazebo starts without a GPU or
# a connection to the online model database.
ENV TURTLEBOT3_MODEL=burger \
    GAZEBO_MODEL_DATABASE_URI= \
    MESA_GL_VERSION_OVERRIDE=3.3 \
    LIBGL_ALWAYS_SOFTWARE=1 \
    GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib \
    GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models

FROM deps AS workspace

SHELL ["/bin/bash", "-c"]
WORKDIR /ws
COPY src src
RUN source /opt/ros/humble/setup.bash \
 && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && colcon test --event-handlers console_direct+ \
 && colcon test-result --verbose

# Make the workspace available to every command run in the container.
RUN sed -i 's|exec "\$@"|source /ws/install/setup.bash\nexec "$@"|' /ros_entrypoint.sh

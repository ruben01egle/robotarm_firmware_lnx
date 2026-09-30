FROM ros:jazzy-ros-base

# Things not declared in any package.xml: middleware choice and standalone tooling
# (numpy is used by robotarm_description/tools/workspace_envelope)
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-numpy \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

ENV PIP_BREAK_SYSTEM_PACKAGES=1

# Install all package dependencies declared in the package.xml files under src/.
# After adding a dependency, run inside the container instead of rebuilding:
#   rosdep install --from-paths src --ignore-src -y
COPY src /tmp/src
RUN apt-get update \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && rosdep install --from-paths /tmp/src --ignore-src -y -r --rosdistro ${ROS_DISTRO} \
    && rm -rf /var/lib/apt/lists/* /tmp/src

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WORKDIR /ros2_ws

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

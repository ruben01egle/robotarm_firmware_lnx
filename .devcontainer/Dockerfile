# syntax=docker/dockerfile:1

# Keep only package manifests and ignore markers, so code changes don't invalidate the rosdep layer.
FROM ros:jazzy-ros-base AS manifests
COPY src /tmp/src
RUN find /tmp/src ! -type d ! -name package.xml ! -name COLCON_IGNORE ! -name AMENT_IGNORE ! -name CATKIN_IGNORE -delete \
    && find /tmp/src -depth -type d -empty -delete

FROM ros:jazzy-ros-base

# PIP_BREAK_SYSTEM_PACKAGES is needed for pip rosdep keys (python3-mujoco-pip).
ENV PIP_BREAK_SYSTEM_PACKAGES=1 \
    PIP_NO_CACHE_DIR=1 \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Not declared in any package.xml (numpy is used by robotarm_description/tools/workspace_envelope).
RUN apt-get update \
    && apt-get install -y \
        python3-pip \
        python3-numpy \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

COPY --from=manifests /tmp/src /tmp/src
RUN apt-get update \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && rosdep install --from-paths /tmp/src --ignore-src -y --rosdistro ${ROS_DISTRO} \
    && rm -rf /var/lib/apt/lists/* /tmp/src

RUN echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> /root/.bashrc

WORKDIR /robotarm_firmware_lnx

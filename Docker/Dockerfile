FROM osrf/ros:foxy-desktop

SHELL ["/bin/bash", "-c"]

ARG DEBIAN_FRONTEND=noninteractive

## NVIDIA container runtime env
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    build-essential \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*

ENV WS=/root/ros2_ws
RUN mkdir -p ${WS}/src

WORKDIR ${WS}

# Always clone from the official repository and branch
ARG REPO_URL="https://github.com/OpenDroids-robot/R2D3_ros2.git"
ARG REPO_REF="main"
RUN git clone --depth 1 --branch "${REPO_REF}" "${REPO_URL}" "${WS}/src/R2D3_ros2"

# Keep image lean; run install scripts and build at runtime instead
RUN echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc && \
    echo "export ROS_WS=${WS}" >> /root/.bashrc && \
    echo 'if [ -f "$ROS_WS/install/setup.bash" ]; then source "$ROS_WS/install/setup.bash"; fi' >> /root/.bashrc

ENV ROS_DISTRO=foxy
ENV ROS_WS=${WS}

CMD ["bash"]


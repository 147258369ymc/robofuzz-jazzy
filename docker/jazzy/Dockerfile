FROM ros:jazzy-ros-base-noble

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV ROBOFUZZ_HOME=/robofuzz
ENV PX4_ROOT=/robofuzz/targets/PX4-Autopilot
ENV PX4_ROS2_WS=/robofuzz/px4_ros2_ws
ARG PX4_REF=v1.17.0
ARG PX4_MSGS_REF=release/1.17
ARG MICRO_XRCE_DDS_AGENT_REF=v2.4.3

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    ca-certificates \
    ccache \
    cmake \
    curl \
    gdb \
    git \
    gnupg \
    iproute2 \
    lcov \
    less \
    lsb-release \
    make \
    nano \
    net-tools \
    ninja-build \
    pkg-config \
    procps \
    python3-colcon-common-extensions \
    python3-dev \
    python3-numpy \
    python3-pandas \
    python3-pip \
    python3-pytest \
    python3-rosdep \
    python3-setuptools \
    python3-venv \
    python3-vcstool \
    python3-wheel \
    python3-yaml \
    rsync \
    tmux \
    unzip \
    vim \
    wget \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-action-msgs \
    ros-jazzy-control-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-irobot-create-msgs \
    ros-jazzy-irobot-create-nodes \
    ros-jazzy-moveit \
    ros-jazzy-moveit-msgs \
    ros-jazzy-moveit-resources-panda-description \
    ros-jazzy-moveit-resources-panda-moveit-config \
    ros-jazzy-nav-msgs \
    ros-jazzy-ros-gz \
    ros-jazzy-sensor-msgs \
    ros-jazzy-tf2-ros \
    ros-jazzy-trajectory-msgs \
    ros-jazzy-turtlebot4-simulator \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --break-system-packages --no-cache-dir \
    kconfiglib==14.1.0 \
    kinpy==0.5.6 \
    pymavlink \
    pyulog \
    sysv-ipc==1.1.0

RUN mkdir -p /robofuzz/targets

RUN git clone --branch "${PX4_REF}" --recursive https://github.com/PX4/PX4-Autopilot.git "${PX4_ROOT}" \
    && cd "${PX4_ROOT}" \
    && git checkout d6f12ad1c4f70ad3230afd7d86e971421e02fef4 \
    && git submodule update --init --recursive

RUN python3 -m pip install --break-system-packages --no-cache-dir \
    -r "${PX4_ROOT}/Tools/setup/requirements.txt"

RUN mkdir -p "${PX4_ROS2_WS}/src" \
    && git clone --branch "${PX4_MSGS_REF}" --depth 1 https://github.com/PX4/px4_msgs.git "${PX4_ROS2_WS}/src/px4_msgs" \
    && source /opt/ros/jazzy/setup.bash \
    && cd "${PX4_ROS2_WS}" \
    && colcon build --symlink-install --packages-select px4_msgs

RUN git clone --branch "${MICRO_XRCE_DDS_AGENT_REF}" --depth 1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/Micro-XRCE-DDS-Agent \
    && source /opt/ros/jazzy/setup.bash \
    && cmake -S /tmp/Micro-XRCE-DDS-Agent -B /tmp/Micro-XRCE-DDS-Agent/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DUAGENT_SUPERBUILD=OFF \
        -DUAGENT_USE_SYSTEM_FASTDDS=ON \
        -DUAGENT_USE_SYSTEM_FASTCDR=ON \
        -DUAGENT_LOGGER_PROFILE=OFF \
        -DUAGENT_P2P_PROFILE=OFF \
    && cmake --build /tmp/Micro-XRCE-DDS-Agent/build --target install -j"$(nproc)" \
    && ldconfig \
    && rm -rf /tmp/Micro-XRCE-DDS-Agent

RUN apt-get update && apt-get install -y --no-install-recommends \
    xvfb \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /robofuzz

COPY . /robofuzz

RUN chmod +x /robofuzz/run_target.sh

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc \
    && echo "source /robofuzz/px4_ros2_ws/install/setup.bash" >> /root/.bashrc \
    && echo "cd /robofuzz" >> /root/.bashrc

CMD ["/bin/bash"]

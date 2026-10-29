FROM ros:jazzy-ros-base
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz git cmake build-essential curl gnupg lsb-release \
    rapidjson-dev libopencv-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
    libgl1-mesa-dri mesa-utils xvfb \
    && rm -rf /var/lib/apt/lists/*

# Репозиторій Gazebo (для dev-пакетів, потрібних для збірки плагіна)
RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y libgz-sim8-dev && \
    rm -rf /var/lib/apt/lists/*

# Плагін ArduPilot для Gazebo (клонування з повторами, GitHub інколи обриває)
RUN for i in 1 2 3 4 5; do \
      git clone https://github.com/ArduPilot/ardupilot_gazebo /opt/ardupilot_gazebo && break \
      || (rm -rf /opt/ardupilot_gazebo; sleep 5); done
RUN cd /opt/ardupilot_gazebo && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && make -j4

ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ardupilot_gazebo/build
ENV GZ_SIM_RESOURCE_PATH=/opt/ardupilot_gazebo/models:/opt/ardupilot_gazebo/worlds
ENV LIBGL_ALWAYS_SOFTWARE=1
WORKDIR /work

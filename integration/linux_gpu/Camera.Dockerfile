# Source-rebuild recipe; no local images or ignored run outputs in this context.
ARG ROS_BASE=ros:jazzy-ros-base
FROM ${ROS_BASE}
USER root
ARG PLUGIN_COMMIT=f116b03552d05c8d0b0bed5ed67033a1183121d1
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz build-essential cmake git pkg-config rapidjson-dev \
    libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libgl1-mesa-dri libegl1 mesa-utils python3-numpy python3-opencv \
    python3-matplotlib linux-tools-generic && rm -rf /var/lib/apt/lists/*
RUN git clone https://github.com/ArduPilot/ardupilot_gazebo.git /opt/ardupilot_gazebo && \
    git -C /opt/ardupilot_gazebo checkout --detach ${PLUGIN_COMMIT} && \
    test "$(git -C /opt/ardupilot_gazebo rev-parse HEAD)" = "${PLUGIN_COMMIT}"
RUN . /opt/ros/jazzy/setup.sh && \
    cmake -S /opt/ardupilot_gazebo -B /opt/ardupilot_gazebo/build -DCMAKE_BUILD_TYPE=Release && \
    cmake --build /opt/ardupilot_gazebo/build --parallel 2
COPY linux_gpu/native_parts_render.cc /opt/phase1/native_parts_render.cc
COPY linux_gpu/build_native_probe.sh /opt/phase1/build_native_probe.sh
RUN bash /opt/phase1/build_native_probe.sh && \
    dpkg-query -W > /opt/phase1/packages.txt && \
    git -C /opt/ardupilot_gazebo rev-parse HEAD > /opt/phase1/plugin-commit.txt
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ardupilot_gazebo/build
ENV QT_QPA_PLATFORM=offscreen
ENTRYPOINT ["sleep"]
CMD ["infinity"]

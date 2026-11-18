FROM gz-sim:latest

USER root

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev && \
    rm -rf /var/lib/apt/lists/*

COPY source /opt/ardupilot_gazebo

RUN . /opt/ros/jazzy/setup.sh && \
    cmake -S /opt/ardupilot_gazebo \
          -B /opt/ardupilot_gazebo/build \
          -DCMAKE_BUILD_TYPE=Release && \
    cmake --build /opt/ardupilot_gazebo/build --parallel 2

ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ardupilot_gazebo/build
ENV GZ_SIM_RESOURCE_PATH=/opt/ardupilot_gazebo/models:/opt/ardupilot_gazebo/worlds


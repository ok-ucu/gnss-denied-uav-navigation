# OpenVINS baseline, built without ROS, plus a runner that reads the team's
# EuRoC-like recording folders directly (mav0/cam0 + mav0/imu0).
#
#   docker build -t gnss-vio-baseline:openvins-2.7 vio_offline/baseline
#
# Pinned: OpenVINS v2.7 (rpng/open_vins, GPLv3), Ubuntu 22.04 packages
# (Eigen 3.4.0, OpenCV 4.5.4, Ceres 2.0.0, Boost 1.74).
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git ca-certificates \
        libeigen3-dev libceres-dev libopencv-dev \
        libboost-system-dev libboost-filesystem-dev libboost-thread-dev libboost-date-time-dev \
    && rm -rf /var/lib/apt/lists/*

# v2.7 = 93adc241390d13e99232652cf05cbe18a93c7bea
ARG OPENVINS_COMMIT=93adc241390d13e99232652cf05cbe18a93c7bea
RUN git init -q /opt/open_vins && cd /opt/open_vins \
    && git remote add origin https://github.com/rpng/open_vins.git \
    && git fetch -q --depth 1 origin ${OPENVINS_COMMIT} \
    && git checkout -q FETCH_HEAD \
    && test "$(git rev-parse HEAD)" = "${OPENVINS_COMMIT}"

ARG BUILD_JOBS=3
RUN cmake -S /opt/open_vins/ov_msckf -B /opt/open_vins/build \
        -DENABLE_ROS=OFF -DENABLE_ARUCO_TAGS=OFF -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /opt/open_vins/build -j ${BUILD_JOBS} --target ov_msckf_lib

COPY run_asl_folder.cpp /opt/baseline/run_asl_folder.cpp
RUN g++ -O2 -std=c++17 -DROS_AVAILABLE=0 -DENABLE_ARUCO_TAGS=0 \
        -I/opt/open_vins/ov_msckf/src -I/opt/open_vins/ov_core/src -I/opt/open_vins/ov_init/src \
        -I/usr/include/eigen3 -I/usr/include/opencv4 \
        /opt/baseline/run_asl_folder.cpp -o /usr/local/bin/run_asl_folder \
        -L/opt/open_vins/build -lov_msckf_lib -Wl,-rpath,/opt/open_vins/build \
        -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lopencv_video -lopencv_features2d \
        -lopencv_calib3d -lopencv_highgui \
        -lceres -lglog -lboost_system -lboost_filesystem -lboost_thread -lboost_date_time -lpthread \
    && echo "${OPENVINS_COMMIT}" > /opt/baseline/OPENVINS_COMMIT

WORKDIR /work
ENTRYPOINT ["/usr/local/bin/run_asl_folder"]

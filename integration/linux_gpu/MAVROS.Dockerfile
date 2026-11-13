# NEW isolated transport image. Required public ROS digest; no cached team base.
ARG ROS_BASE
FROM ${ROS_BASE}
ARG ROS_BASE
ARG DEBIAN_FRONTEND=noninteractive
ARG MAVROS_DEB_VERSION
ARG MAVROS_EXTRAS_DEB_VERSION
ARG MAVROS_MSGS_DEB_VERSION
ARG LIBMAVCONN_DEB_VERSION
RUN case "${ROS_BASE}" in ros@sha256:*) ;; *) exit 1;; esac && \
    case "${MAVROS_DEB_VERSION}" in 2.16.0-*) ;; *) exit 1;; esac && \
    case "${MAVROS_EXTRAS_DEB_VERSION}" in 2.16.0-*) ;; *) exit 1;; esac && \
    case "${MAVROS_MSGS_DEB_VERSION}" in 2.16.0-*) ;; *) exit 1;; esac && \
    case "${LIBMAVCONN_DEB_VERSION}" in 2.16.0-*) ;; *) exit 1;; esac && \
    apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-mavros="${MAVROS_DEB_VERSION}" \
      ros-jazzy-mavros-extras="${MAVROS_EXTRAS_DEB_VERSION}" \
      ros-jazzy-mavros-msgs="${MAVROS_MSGS_DEB_VERSION}" \
      ros-jazzy-libmavconn="${LIBMAVCONN_DEB_VERSION}" \
      python3-colcon-common-extensions python3-numpy build-essential cmake \
      libeigen3-dev ros-jazzy-eigen3-cmake-module geographiclib-tools wget ca-certificates && \
    /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
COPY mavros_time_patch /tmp/mavros_time_patch
RUN bash -c 'source /opt/ros/jazzy/setup.bash && \
    colcon --log-base /tmp/build_logs build --base-paths /tmp/mavros_time_patch \
      --build-base /tmp/build --install-base /tmp/patch_install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release' && \
    mkdir -p /opt/phase1 && \
    uname -m > /opt/phase1/architecture.txt && \
    printf '%s\n' "${ROS_BASE}" > /opt/phase1/base-image.txt && \
    dpkg-query -W > /opt/phase1/packages.txt && \
    sha256sum /tmp/mavros_time_patch/src/sys_time.cpp > /opt/phase1/overlay-source.sha256 && \
    c++ --version > /opt/phase1/compiler.txt && \
    cp -r /tmp/build_logs /opt/phase1/build_logs && \
    python3 -c 'import xml.etree.ElementTree as E; from pathlib import Path; names=("mavros","mavros_msgs","libmavconn"); versions={n:E.parse(Path("/opt/ros/jazzy/share")/n/"package.xml").findtext("version") for n in names}; assert all(v=="2.16.0" for v in versions.values()),versions; import json; Path("/opt/phase1/ros-versions.json").write_text(json.dumps(versions,indent=2)+"\n")'
COPY SOURCE_HASHES.json /opt/phase1/source-hashes.json
ENV ROS_LOCALHOST_ONLY=1
ENTRYPOINT ["sleep"]
CMD ["infinity"]

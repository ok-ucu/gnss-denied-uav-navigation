# New isolated 4.4.4 build, never overwrite the historical ARM64 binary/pin.
ARG UBUNTU_BASE=ubuntu:22.04
FROM ${UBUNTU_BASE}
ARG DEBIAN_FRONTEND=noninteractive
ARG AP_COMMIT=9279df6191a772c098b2e4797c95ac954f258d88
RUN apt-get update && apt-get install -y --no-install-recommends \
    git sudo lsb-release tzdata ca-certificates python3-pip && \
    useradd -m -s /bin/bash docker && \
    echo 'docker ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/docker
USER docker
WORKDIR /home/docker
RUN git clone https://github.com/ArduPilot/ardupilot.git && \
    git -C ardupilot checkout --detach ${AP_COMMIT} && \
    test "$(git -C ardupilot rev-parse HEAD)" = "${AP_COMMIT}" && \
    git -C ardupilot submodule update --init --recursive
WORKDIR /home/docker/ardupilot
RUN USER=docker SKIP_AP_GRAPHIC_ENV=1 SKIP_AP_COV_ENV=1 SKIP_AP_EXT_ENV=1 \
    SKIP_AP_COMPLETION_ENV=1 Tools/environment_install/install-prereqs-ubuntu.sh -y
COPY --chown=docker:docker isolated_blockers/20261109_a/evidence/instrumentation.patch /tmp/instrumentation.patch
COPY --chown=docker:docker isolated_blockers/20261109_a/code/AP444_BOOLEAN_EXACT.patch /tmp/boolean.patch
COPY --chown=docker:docker isolated_blockers/20261109_a/code/test_sim_json.cpp libraries/SITL/tests/test_sim_json.cpp
RUN git apply --check /tmp/instrumentation.patch && git apply /tmp/instrumentation.patch && \
    git apply --check /tmp/boolean.patch && git apply /tmp/boolean.patch && \
    ./waf configure --board sitl --out build_experiment && ./waf copter -j2 && \
    ./waf --targets tests/test_sim_json -j2 && \
    build_experiment/sitl/tests/test_sim_json > /tmp/parser-tests.txt
USER root
RUN mkdir -p /opt/phase1 && cp build_experiment/sitl/bin/arducopter /opt/phase1/arducopter && \
    cp /tmp/parser-tests.txt /opt/phase1/parser-tests.txt && \
    sha256sum /opt/phase1/arducopter > /opt/phase1/firmware.sha256 && \
    git rev-parse HEAD > /opt/phase1/ardupilot-commit.txt && dpkg-query -W > /opt/phase1/packages.txt
WORKDIR /tmp
ENTRYPOINT ["sleep"]
CMD ["infinity"]

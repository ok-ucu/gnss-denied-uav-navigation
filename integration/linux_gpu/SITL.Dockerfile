# New isolated 4.4.4 build, never overwrite the historical ARM64 binary/pin.
ARG UBUNTU_BASE=ubuntu:22.04
FROM ${UBUNTU_BASE}
ARG UBUNTU_BASE
ARG DEBIAN_FRONTEND=noninteractive
ARG AP_COMMIT=9279df6191a772c098b2e4797c95ac954f258d88
ARG BUILD_JOBS=2
RUN test "${AP_COMMIT}" = 9279df6191a772c098b2e4797c95ac954f258d88 && \
    test "${BUILD_JOBS}" -ge 1 && test "${BUILD_JOBS}" -le 32
RUN apt-get update && apt-get install -y --no-install-recommends \
    git sudo lsb-release tzdata ca-certificates python3-pip && \
    useradd -m -s /bin/bash docker && \
    echo 'docker ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/docker
USER docker
WORKDIR /home/docker
RUN git init ardupilot && \
    git -C ardupilot remote add origin https://github.com/ArduPilot/ardupilot.git && \
    git -C ardupilot fetch --depth 1 origin ${AP_COMMIT} && \
    git -C ardupilot checkout --detach FETCH_HEAD && \
    test "$(git -C ardupilot rev-parse HEAD)" = "${AP_COMMIT}" && \
    (git -C ardupilot submodule update --init --recursive --depth 1 || \
     git -C ardupilot submodule update --init --recursive)
WORKDIR /home/docker/ardupilot
RUN USER=docker SKIP_AP_GRAPHIC_ENV=1 SKIP_AP_COV_ENV=1 SKIP_AP_EXT_ENV=1 \
    SKIP_AP_COMPLETION_ENV=1 DO_AP_STM_ENV=0 Tools/environment_install/install-prereqs-ubuntu.sh -y
COPY --chown=docker:docker isolated_blockers/20261109_a/evidence/instrumentation.patch /tmp/instrumentation.patch
COPY --chown=docker:docker isolated_blockers/20261109_a/code/AP444_BOOLEAN_EXACT.patch /tmp/boolean.patch
COPY --chown=docker:docker isolated_blockers/20261109_a/code/test_sim_json.cpp libraries/SITL/tests/test_sim_json.cpp
RUN git apply --check /tmp/instrumentation.patch && git apply /tmp/instrumentation.patch && \
    git apply --check /tmp/boolean.patch && git apply /tmp/boolean.patch && \
    ./waf configure --board sitl --out build_experiment && ./waf copter -j"${BUILD_JOBS}" && \
    ./waf --targets tests/test_sim_json -j"${BUILD_JOBS}" && \
    build_experiment/sitl/tests/test_sim_json > /tmp/parser-tests.txt && \
    git rev-parse HEAD > /tmp/ardupilot-commit.txt && \
    git submodule status --recursive > /tmp/submodules.txt && \
    git diff --binary > /tmp/applied-source.patch && \
    sha256sum /tmp/instrumentation.patch /tmp/boolean.patch libraries/SITL/tests/test_sim_json.cpp > /tmp/patches.sha256 && \
    c++ --version > /tmp/compiler.txt
USER root
RUN mkdir -p /opt/phase1 && cp build_experiment/sitl/bin/arducopter /opt/phase1/arducopter && \
    cp /tmp/parser-tests.txt /opt/phase1/parser-tests.txt && \
    sha256sum /opt/phase1/arducopter > /opt/phase1/firmware.sha256 && \
    cp /tmp/ardupilot-commit.txt /tmp/submodules.txt /tmp/applied-source.patch /tmp/patches.sha256 /tmp/compiler.txt /opt/phase1/ && \
    uname -m > /opt/phase1/architecture.txt && \
    printf '%s\n' "${UBUNTU_BASE}" > /opt/phase1/base-image.txt && \
    dpkg-query -W > /opt/phase1/packages.txt
COPY SOURCE_HASHES.json /opt/phase1/source-hashes.json
WORKDIR /tmp
ENTRYPOINT ["sleep"]
CMD ["infinity"]

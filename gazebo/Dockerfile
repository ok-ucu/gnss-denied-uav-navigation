
FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz \
    libgl1-mesa-dri \
    mesa-utils \
    xvfb \
    python3-pip \
    python3-venv \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m venv --system-site-packages /opt/venv

ENV PATH="/opt/venv/bin:${PATH}"

RUN python3 -m pip install --no-cache-dir pymavlink

RUN python3 -c "from pymavlink import mavutil; print('pymavlink OK')"

ENV LIBGL_ALWAYS_SOFTWARE=1

WORKDIR /work

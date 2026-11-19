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
# Phase 1 camera image: change only the pinned world's sun shadow flag.
RUN python3 - <<'NO_SHADOWS'
from pathlib import Path
import re
import xml.etree.ElementTree as ET

path = Path('/opt/ardupilot_gazebo/worlds/iris_runway.sdf')
source = path.read_text()

def check(text, expected):
    worlds = ET.fromstring(text).findall("./world[@name='iris_runway']")
    if len(worlds) != 1:
        raise RuntimeError('Expected exactly one iris_runway world')
    suns = worlds[0].findall("./light[@name='sun']")
    if len(suns) != 1 or suns[0].get('type') != 'directional':
        raise RuntimeError('Expected exactly one directional sun light')
    flags = suns[0].findall('cast_shadows')
    if len(flags) != 1 or (flags[0].text or '').strip() != expected:
        raise RuntimeError('Unexpected sun cast_shadows; expected exactly one '+expected)

check(source, 'true')
pattern = re.compile(
    r'''(<light\b(?=[^>]*\bname\s*=\s*["']sun["'])[^>]*>'''
    r'''(?:(?!</light\s*>).)*?<cast_shadows>\s*)true(\s*</cast_shadows>)''', re.S)
patched, replacements = pattern.subn(r'\g<1>false\g<2>', source)
if replacements != 1:
    raise RuntimeError('Expected exactly one sun shadow replacement, got '+str(replacements))
check(patched, 'false')
path.write_text(patched)
print('Phase 1 sun cast_shadows=false; replacements=1; '+str(path))
NO_SHADOWS
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

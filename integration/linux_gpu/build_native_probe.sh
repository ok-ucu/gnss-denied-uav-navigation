#!/usr/bin/env bash
set -euo pipefail
# Passive witness ABI must exist in the actually installed Gazebo/Ogre build.
export PKG_CONFIG_PATH
PKG_CONFIG_PATH=$(find /opt/ros/jazzy/opt -type d -name pkgconfig | paste -sd:)
engine=/opt/ros/jazzy/opt/gz_rendering_vendor/lib/libgz-rendering8-ogre2.so
nm -D "$engine" > /tmp/render-symbols.txt
grep -F '_ZTv0_n696_N2gz9rendering2v811Ogre2Camera6RenderEv' /tmp/render-symbols.txt
g++ -shared -fPIC -O2 /opt/phase1/native_parts_render.cc \
  -o /opt/phase1/native_parts_render.so $(pkg-config --cflags --libs gz-msgs10) -ldl

#!/usr/bin/env bash
set -eu
# Only a NEW disposable offline builder; never modify /opt/ros in working containers.
source_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
builder=${PATCH_BUILDER_NAME:-gnss-mavros-patch-build-new}
output=${PATCH_BUILD_OUTPUT:?Set PATCH_BUILD_OUTPUT to a new integration evidence directory}
mkdir "$output"
docker run -d --name "$builder" --network none --entrypoint sleep gnss-mavros-isolated:jazzy-2.16.0 infinity
trap 'docker stop "$builder" >/dev/null' EXIT
docker cp "$source_dir/mavros_time_patch" "$builder:/tmp/mavros_time_patch"
docker exec "$builder" bash -lc 'source /opt/ros/jazzy/setup.bash; colcon --log-base /tmp/build_logs build --base-paths /tmp/mavros_time_patch --build-base /tmp/build --install-base /tmp/patch_install --cmake-args -DCMAKE_BUILD_TYPE=Release'
docker cp "$builder:/tmp/patch_install" "$output/patch_install"
docker cp "$builder:/tmp/build_logs" "$output/build_logs"

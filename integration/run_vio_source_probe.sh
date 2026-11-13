#!/usr/bin/env bash
# Run INSIDE the existing Linux GPU Gazebo container. Source-only; no FCU writes.
# Usage: bash /opt/integration/run_vio_source_probe.sh RUN_ID /opt/integration/linux_gpu_runs/VIO_EXPORT
set -euo pipefail
[[ "$(uname -s)" == Linux ]] || { echo 'Linux GPU runtime required; no Mac simulation'; exit 2; }
[[ $# == 2 && "$1" =~ ^[a-zA-Z0-9][a-zA-Z0-9_-]{0,63}$ ]] || { echo 'Require RUN_ID and pinned VIO directory'; exit 2; }
[[ "${ROS_DOMAIN_ID:-}" =~ ^[0-9]+$ && "${ROS_DOMAIN_ID}" -gt 0 && "${ROS_DOMAIN_ID}" -le 232 ]] || { echo 'Require dedicated nonzero ROS_DOMAIN_ID'; exit 2; }
[[ -n "${GZ_PARTITION:-}" ]] || { echo 'Require existing Gazebo GZ_PARTITION'; exit 2; }
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
vio="$2"
run="/tmp/gnss-vio-source-$1"
mkdir "$run"
source /opt/ros/jazzy/setup.bash
command -v setsid > /dev/null
[[ -x /opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge ]]
export PYTHONDONTWRITEBYTECODE=1
helper="$here/vio_source_probe.py"
CAMERA=/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image
source_pid= observer_pid= witness_pid= camera_pid=
fixture_owned=0
cleanup() {
  local status=$?
  trap - EXIT INT TERM
  # Only process groups created by this script; never kill by a shared node name.
  for pid in "$source_pid" "$observer_pid" "$camera_pid" "$witness_pid"; do
    [[ -z "$pid" ]] || kill -TERM -- "-$pid" 2>/dev/null || true
  done
  for pid in "$source_pid" "$observer_pid" "$camera_pid" "$witness_pid"; do
    [[ -z "$pid" ]] && continue
    for unused in {1..150}; do
      kill -0 "$pid" 2>/dev/null || break
      sleep .1
    done
    if kill -0 "$pid" 2>/dev/null; then
      echo "Owned process $pid required forced shutdown" >> "$run/cleanup.log"
      kill -KILL -- "-$pid" 2>/dev/null || true
      status=1
    fi
    local child_status=0
    wait "$pid" 2>/dev/null || child_status=$?
    printf 'pid=%s exit=%s\n' "$pid" "$child_status" >> "$run/cleanup.log"
    [[ "$child_status" == 0 || "$child_status" == 143 ]] || status=1
  done
  if [[ "$fixture_owned" == 1 ]]; then
    python3 -B "$here/vio_ground_fixture.py" remove --directory "$run/fixture" >> "$run/fixture.log" 2>&1 || status=1
  fi
  python3 -B "$here/linux_gpu/disarmed.py" > "$run/disarmed-after.log" 2>&1 || status=1
  printf '%s\n' "$status" > "$run/collection.exit"
  # Missing evidence/failure/partial runs produce FAIL, never a fallback PASS.
  python3 -B "$helper" analyze --directory "$run" > "$run/analysis.log" 2>&1 || status=1
  printf '%s\n' "$status" > "$run/launcher.exit"
  echo "Evidence: $run; exit=$status; decision=HOLD; forwarding_enabled=false; execution_allowed=false"
  exit "$status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
python3 -B "$here/linux_gpu/disarmed.py" > "$run/disarmed-before.log" 2>&1
python3 -B "$helper" preflight --directory "$run" --vio-dir "$vio" > "$run/preflight.log" 2>&1
# Existing fixture/model is never removed on create failure. No rename fallback.
python3 -B "$here/vio_ground_fixture.py" create --directory "$run/fixture" --size-m 1 --tile-size-m .02 > "$run/fixture.log" 2>&1
fixture_owned=1
setsid python3 -B "$helper" witness --directory "$run" > "$run/witness.log" 2>&1 &
witness_pid=$!
for unused in {1..150}; do
  kill -0 "$witness_pid" || exit 1
  [[ ! -f "$run/witness.ready.json" ]] || break
  sleep .1
done
[[ -f "$run/witness.ready.json" ]] || exit 1
setsid /opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge "$CAMERA@sensor_msgs/msg/Image[gz.msgs.Image" > "$run/camera_bridge.log" 2>&1 &
camera_pid=$!
mark() { python3 -B "$helper" mark --directory "$run" --phase "$1"; }
healthy() {
  kill -0 "$witness_pid" && kill -0 "$camera_pid"
  [[ -z "$observer_pid" ]] || kill -0 "$observer_pid"
}
observe_for() {
  for unused in $(seq 1 "$1"); do healthy; sleep 1; done
}
mark before
observe_for 10
setsid python3 -B "$here/observe_frame_bridge.py" --directory "$run/observer" --duration 150 > "$run/observer.log" 2>&1 &
observer_pid=$!
mark on
setsid python3 -B "$here/run_vio_stream.py" --vio-dir "$vio" --output-dir "$run/source" --duration 60 --ground-z .001 --mount-calibration "$run/mount.json" --profile --async-diagnostics --reliable-camera --isolated-imu --min-odom-subscribers 1 > "$run/source.log" 2>&1 &
source_pid=$!
for unused in {1..1200}; do
  healthy
  kill -0 "$source_pid" 2>/dev/null || break
  sleep .1
done
kill -0 "$source_pid" 2>/dev/null && { echo 'Source exceeded120s wall bound'; exit 1; }
wait "$source_pid"
source_pid=
mark after
observe_for 10
mark end
# EXIT cleanup stops owned processes, removes only our fixture, checks DISARMED,
# and analyzes the full capture including final silence. No container lifecycle action.

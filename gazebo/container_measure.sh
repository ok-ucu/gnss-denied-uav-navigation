#!/usr/bin/env bash
# Запускається ВСЕРЕДИНІ контейнера gz-sim (його викликає run_linux_gpu.sh). Не запускати вручну на хості.
# cwd = /work (корінь репозиторію). Аргумент 1 = шлях до папки доказів у контейнері.
set -uo pipefail
OUT="${1:?потрібна папка доказів}"
STREAM_SECONDS="${STREAM_SECONDS:-60}"
RECORD_SECONDS="${RECORD_SECONDS:-120}"

unset LIBGL_ALWAYS_SOFTWARE          # головне: не форсувати CPU-рендер
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=47 GZ_PARTITION=gnss_gpu_run
mkdir -p "$OUT"

finish() {
  [[ -n "${GZ_PID:-}" ]] && kill "$GZ_PID" 2>/dev/null
  [[ -n "${BR_PID:-}" ]] && kill "$BR_PID" 2>/dev/null
  wait 2>/dev/null
  [[ -n "${HOST_UID:-}" ]] && chown -R "$HOST_UID:${HOST_GID:-$HOST_UID}" "$OUT" 2>/dev/null
  touch "$OUT/.done"
}
trap finish EXIT

{ echo "LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE-<unset>}"; echo "NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES-<unset>}"; } > "$OUT/env.txt"
nvidia-smi -L > "$OUT/container_nvidia-smi-L.txt" 2>&1 || echo "nvidia-smi у контейнері недоступний" >> "$OUT/container_nvidia-smi-L.txt"

python3 -c "import numpy, cv2, rclpy" 2>/dev/null || {
  echo "Не вистачає numpy/opencv — доставляю через apt..."
  apt-get update -qq && apt-get install -y -qq python3-numpy python3-opencv >> "$OUT/apt.log" 2>&1
}

echo "[1/5] Запуск Gazebo (headless, GPU)..."
gz sim -v4 -s -r --headless-rendering gazebo/worlds/camera_world.sdf > "$OUT/gazebo.log" 2>&1 &
GZ_PID=$!
for i in $(seq 1 120); do
  gz topic -l 2>/dev/null | grep -q "/camera/image_raw" && break
  kill -0 "$GZ_PID" 2>/dev/null || { echo "Gazebo впав, дивись gazebo.log"; exit 3; }
  sleep 1
done

echo "[2/5] Запуск ros_gz_bridge..."
ros2 run ros_gz_bridge parameter_bridge \
  "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image" \
  "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock" \
  "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU" \
  "/model/camera_drone/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry" \
  > "$OUT/bridge.log" 2>&1 &
BR_PID=$!
sleep 8

# best-effort: що Mesa/EGL вважає рендерером
{ echo "--- eglinfo ---"; EGL_PLATFORM=surfaceless timeout 20 eglinfo -B 2>&1 | head -40 || echo "eglinfo недоступний"; } > "$OUT/renderer_check.txt" 2>&1
{ echo "--- пошук software-рендерера в gazebo.log (порожньо = добре) ---"; grep -i -E "llvmpipe|softpipe|swrast" "$OUT/gazebo.log" || echo "(збігів немає)"; } >> "$OUT/renderer_check.txt"

touch "$OUT/.measuring"   # сигнал хосту: знімай nvidia-smi
echo "[3/5] Вимірювання потоку ${STREAM_SECONDS} с..."
python3 gazebo/check_gazebo_stream.py --duration "$STREAM_SECONDS" --json "$OUT/stream.json" 2>&1 | tee "$OUT/stream.txt"

echo "[4/5] Запис датасету ${RECORD_SECONDS} с..."
python3 gazebo/gazebo_recorder.py --out "$OUT/recording" --seconds "$RECORD_SECONDS" > "$OUT/recorder.log" 2>&1
python3 gazebo/check_gazebo_stream.py --analyze-recording "$OUT/recording" --json "$OUT/recording.json" 2>&1 | tee "$OUT/recording.txt"
# PNG-кадри великі й не потрібні як доказ — лишаємо тільки CSV і метадані
rm -rf "$OUT/recording/mav0/cam0/data"

echo "[5/5] Готово."

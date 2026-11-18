#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# run_linux_gpu.sh — ОДНА команда для Linux-машини з NVIDIA GPU.
#
#   З КОРЕНЯ РЕПОЗИТОРІЮ (там, де лежать папки gazebo/, camera/, dataset/):
#       bash gazebo/run_linux_gpu.sh
#
#   Опційно: STREAM_SECONDS=60 RECORD_SECONDS=120 bash gazebo/run_linux_gpu.sh
#
# Результат: папка evidence/linux_gpu_<час>/ + архів evidence/linux_gpu_<час>.tar.gz
# Архів треба надіслати Людині 2.
# Потрібно: драйвер NVIDIA, docker, nvidia-container-toolkit. sudo не потрібен, якщо docker працює без нього.
# ─────────────────────────────────────────────────────────────────────────────
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

STREAM_SECONDS="${STREAM_SECONDS:-60}"
RECORD_SECONDS="${RECORD_SECONDS:-120}"
TS="$(date -u +%Y%m%dT%H%M%SZ)"
EVD="evidence/linux_gpu_$TS"
CNAME="gz-gpu-$TS"
mkdir -p "$EVD"
SMI_PID=""

cleanup() {
  [[ -n "$SMI_PID" ]] && kill "$SMI_PID" 2>/dev/null || true
  docker logs "$CNAME" > "$EVD/container.log" 2>&1 || true
  docker rm -f "$CNAME" >/dev/null 2>&1 || true
}
trap cleanup EXIT

say() { echo -e "\n\033[1;32m[run_linux_gpu]\033[0m $*"; }
die() { echo -e "\033[1;31m[ПОМИЛКА]\033[0m $*" >&2; exit 1; }

say "Перевірка хоста..."
command -v docker >/dev/null    || die "docker не знайдено"
command -v nvidia-smi >/dev/null || die "nvidia-smi не знайдено — драйвер NVIDIA не встановлено"
nvidia-smi -L | tee "$EVD/host_nvidia-smi-L.txt"

if ! docker image inspect gz-sim >/dev/null 2>&1 || [[ "${FORCE_BUILD:-0}" == "1" ]]; then
  say "Збірка образу gz-sim (кілька хвилин)..."
  docker build -t gz-sim gazebo/
fi

say "Перевірка доступу Docker до GPU..."
docker run --rm --gpus all gz-sim nvidia-smi -L \
  || die "Docker не бачить GPU. Встановіть nvidia-container-toolkit: sudo apt install nvidia-container-toolkit && sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker"

say "Збір метаданих..."
{
  echo "date_utc: $TS"
  echo "git_commit: $(git rev-parse HEAD 2>/dev/null || echo unknown)"
  echo "git_branch: $(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)"
  echo "git_dirty_files:"; git status --short 2>/dev/null || true
  echo "kernel: $(uname -a)"
  echo "docker: $(docker --version)"
  echo "image_id: $(docker image inspect gz-sim --format '{{.Id}}')"
  echo "stream_seconds: $STREAM_SECONDS"
  echo "record_seconds: $RECORD_SECONDS"
} > "$EVD/meta.txt"
nvidia-smi > "$EVD/nvidia-smi_before.txt"

say "Запуск контейнера (Gazebo + bridge + вимірювання)..."
docker run -d --init --name "$CNAME" --gpus all --network host \
  -e NVIDIA_DRIVER_CAPABILITIES=all -e NVIDIA_VISIBLE_DEVICES=all \
  -e STREAM_SECONDS="$STREAM_SECONDS" -e RECORD_SECONDS="$RECORD_SECONDS" \
  -e HOST_UID="$(id -u)" -e HOST_GID="$(id -g)" \
  -v "$PWD":/work -w /work gz-sim \
  bash gazebo/container_measure.sh "/work/$EVD" >/dev/null

say "Чекаю на початок вимірювання (Gazebo стартує)..."
for _ in $(seq 1 300); do
  [[ -f "$EVD/.measuring" ]] && break
  [[ -f "$EVD/.done" ]] && break
  docker ps -q --filter "name=$CNAME" | grep -q . || break
  sleep 1
done

if [[ -f "$EVD/.measuring" ]]; then
  say "Знімаю стан GPU під навантаженням..."
  nvidia-smi --query-gpu=timestamp,utilization.gpu,memory.used,temperature.gpu,power.draw --format=csv -l 1 > "$EVD/gpu_util.csv" &
  SMI_PID=$!
  sleep 10
  nvidia-smi > "$EVD/nvidia-smi_during_run.txt"
  nvidia-smi pmon -c 3 > "$EVD/nvidia-smi_pmon.txt" 2>&1 || true
fi

say "Чекаю завершення вимірювань (~$((STREAM_SECONDS + RECORD_SECONDS + 60)) с)..."
docker wait "$CNAME" >/dev/null || true
[[ -n "$SMI_PID" ]] && kill "$SMI_PID" 2>/dev/null || true
SMI_PID=""
docker logs "$CNAME" > "$EVD/container.log" 2>&1 || true

say "Підсумок..."
python3 gazebo/summarize_gpu_run.py "$EVD" || echo "(summarize не вдався — надішліть усю папку $EVD)"

tar czf "$EVD.tar.gz" -C evidence "linux_gpu_$TS"
say "ГОТОВО. Надішліть файл: $PWD/$EVD.tar.gz"

#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# launch_all.sh — Єдиний скрипт запуску GNSS-Denied UAV Navigation (Gazebo + SITL)
#
# Вимога рев'ю: відтворюваний запуск одною командою (критерій #5)
#
# Використання:
#   ./launch_all.sh           → повний стек (Gazebo + SITL + ROS-міст)
#   ./launch_all.sh gazebo    → лише Gazebo (для перегляду топіків)
#   ./launch_all.sh mock      → mock-перевірка без Docker
#
# Режими запуску:
#   1. tmux (якщо встановлено) — відкриває 3 панелі автоматично
#   2. background процеси     — якщо tmux відсутній
# ─────────────────────────────────────────────────────────────────────────────
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
SITL_DIR="$REPO_DIR/../gnss-denied-uav-navigation-sitl-stand/sitl-stand"

MODE="${1:-full}"

# ─── Кольори для виводу ──────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'  # без кольору

log()  { echo -e "${GREEN}[launch_all]${NC} $*"; }
warn() { echo -e "${YELLOW}[УВАГА]${NC} $*"; }
err()  { echo -e "${RED}[ПОМИЛКА]${NC} $*" >&2; }

# ─── Mock-режим (без Docker) ──────────────────────────────────────────────────
if [[ "$MODE" == "mock" ]]; then
    log "Mock-режим: перевірка без Docker та SITL"
    cd "$REPO_DIR"
    python3 gazebo/verify_sitl_integration.py --mock
    exit $?
fi

# ─── Перевірка Docker ────────────────────────────────────────────────────────
if ! command -v docker &>/dev/null; then
    err "Docker не знайдено. Встановіть Docker Desktop."
    exit 1
fi

if ! docker image inspect gz-ardupilot &>/dev/null; then
    warn "Образ gz-ardupilot не знайдено. Збираємо..."
    if [[ -d "$SITL_DIR/gazebo-ardupilot" ]]; then
        docker build -t gz-ardupilot "$SITL_DIR/gazebo-ardupilot"
        log "Образ gz-ardupilot зібрано."
    else
        err "Папка gazebo-ardupilot не знайдена: $SITL_DIR/gazebo-ardupilot"
        err "Переконайтеся, що папка gnss-denied-uav-navigation-sitl-stand є поряд."
        exit 1
    fi
fi

# ─── Функції запуску окремих сервісів ────────────────────────────────────────
start_gazebo() {
    log "Запуск Gazebo (iris_runway.sdf, headless)..."
    docker run -d --rm \
        --name gz-sim \
        --network host \
        -e LIBGL_ALWAYS_SOFTWARE=1 \
        gz-ardupilot \
        gz sim -v3 -s -r --headless-rendering iris_runway.sdf
    log "Gazebo запущено (контейнер gz-sim). Очікуємо ініціалізації (10 с)..."
    sleep 10

    log "Топіки Gazebo:"
    docker exec gz-sim gz topic -l 2>/dev/null || warn "Не вдалося отримати список топіків"
}

start_ros_bridge() {
    log "Запуск ROS 2 мосту (Gazebo → ROS 2)..."
    # Знайти точний шлях до топіку камери
    CAM_TOPIC=$(docker exec gz-sim gz topic -l 2>/dev/null | grep -i image | head -1 || echo "")
    IMU_TOPIC=$(docker exec gz-sim gz topic -l 2>/dev/null | grep -i imu | head -1 || echo "")

    if [[ -z "$CAM_TOPIC" ]]; then
        warn "Не вдалося знайти gz топік камери. Використовуємо шлях за замовчуванням."
        CAM_TOPIC="/camera/image_raw"
    fi

    log "  Камера gz: $CAM_TOPIC → /camera/image_raw (ROS 2)"
    log "  IMU gz:    $IMU_TOPIC → /imu (ROS 2)"

    docker run -d --rm \
        --name gz-ros-bridge \
        --network host \
        -e LIBGL_ALWAYS_SOFTWARE=1 \
        gz-ardupilot \
        bash -c "
            source /opt/ros/jazzy/setup.bash
            ros2 run ros_gz_bridge parameter_bridge \
                /camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
                /camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
                /imu@sensor_msgs/msg/Imu[gz.msgs.IMU
        "
    log "ROS 2 міст запущено (контейнер gz-ros-bridge)."
}

start_sitl() {
    log "Запуск ArduPilot SITL (режим GPS для Фази 2)..."
    if [[ -d "$SITL_DIR" ]]; then
        cd "$SITL_DIR"
        bash run.sh gps &
        SITL_PID=$!
        log "SITL запущено (PID $SITL_PID)."
        cd "$SCRIPT_DIR"
    else
        warn "SITL директорія не знайдена: $SITL_DIR"
        warn "Запустіть SITL вручну: cd sitl-stand && ./run.sh gps"
    fi
}

stop_all() {
    log "Зупинка всіх контейнерів..."
    docker stop gz-sim gz-ros-bridge 2>/dev/null || true
    pkill -f "sim_vehicle.py" 2>/dev/null || true
    log "Зупинено."
}

# ─── Режим: лише Gazebo ──────────────────────────────────────────────────────
if [[ "$MODE" == "gazebo" ]]; then
    trap stop_all EXIT INT TERM
    start_gazebo
    log "Натисніть Ctrl+C для зупинки"
    wait
    exit 0
fi

# ─── Повний стек ─────────────────────────────────────────────────────────────
log "Запуск повного стеку: Gazebo + ROS 2 Міст + SITL"
echo ""
echo "  ┌─────────────────────────────────────────────────┐"
echo "  │  GNSS-Denied UAV Navigation — Full Stack        │"
echo "  │                                                   │"
echo "  │  Gazebo   : порт 9002 (ArduPilot плагін)        │"
echo "  │  SITL     : порт 14550 (QGC), 14551 (Python)   │"
echo "  │  ROS 2    : /camera/image_raw, /imu             │"
echo "  └─────────────────────────────────────────────────┘"
echo ""

trap stop_all EXIT INT TERM

# Перевірка tmux
if command -v tmux &>/dev/null; then
    log "Знайдено tmux — відкриваємо 3 панелі..."
    SESSION="gnss-uav"
    tmux new-session -d -s "$SESSION" -n main 2>/dev/null || true

    # Панель 0: Gazebo
    tmux send-keys -t "$SESSION:main" \
        "docker run --rm --name gz-sim --network host -e LIBGL_ALWAYS_SOFTWARE=1 gz-ardupilot gz sim -v3 -s -r --headless-rendering iris_runway.sdf" Enter

    # Панель 1: SITL
    tmux split-window -h -t "$SESSION:main"
    if [[ -d "$SITL_DIR" ]]; then
        tmux send-keys -t "$SESSION:main" \
            "sleep 8 && cd '$SITL_DIR' && bash run.sh gps" Enter
    else
        tmux send-keys -t "$SESSION:main" \
            "echo 'SITL: запустіть вручну з ardupilot: sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON'" Enter
    fi

    # Панель 2: ROS 2 міст
    tmux split-window -v -t "$SESSION:main"
    tmux send-keys -t "$SESSION:main" \
        "sleep 15 && docker run --rm --name gz-ros-bridge --network host gz-ardupilot bash -c 'source /opt/ros/jazzy/setup.bash && ros2 run ros_gz_bridge parameter_bridge /camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image /imu@sensor_msgs/msg/Imu[gz.msgs.IMU'" Enter

    tmux attach-session -t "$SESSION"
else
    # Без tmux — послідовний запуск у фоні
    warn "tmux не знайдено. Запуск у фоні (логи: /tmp/gz-*.log)."
    start_gazebo 2>&1 | tee /tmp/gz-gazebo.log &
    sleep 12
    start_ros_bridge 2>&1 | tee /tmp/gz-bridge.log &
    sleep 3
    start_sitl 2>&1 | tee /tmp/gz-sitl.log &

    log "Всі сервіси запущені. Натисніть Ctrl+C для зупинки."
    log "Логи: /tmp/gz-gazebo.log, /tmp/gz-bridge.log, /tmp/gz-sitl.log"
    wait
fi

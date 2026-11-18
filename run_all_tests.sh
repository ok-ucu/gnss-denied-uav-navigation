#!/usr/bin/env bash
# Автоматичний запуск усіх перевірок і тестів для Людини 2 (Gazebo, Камера, IMU, Датасет)
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR"

echo "================================================================"
echo "    ЗАПУСК ПЕРЕВІРОК ТА ТЕСТІВ (Людина 2: Gazebo + Сенсори)     "
echo "================================================================"

PYTHON_BIN="${PYTHON_BIN:-python3}"
if ! "$PYTHON_BIN" -B -c 'import cv2, numpy, pytest, yaml; from pymavlink import mavutil'; then
    echo "Install requirements.txt into the environment selected by PYTHON_BIN." >&2
    exit 1
fi
output_dir="$(mktemp -d)"
trap 'rm -r -- "$output_dir"' EXIT

echo "[1/7] Тестування модулів через Pytest (camera, calibration, dataset, gazebo)..."
"$PYTHON_BIN" -B -m pytest -p no:cacheprovider camera/ calibration/ dataset/ gazebo/ -v

echo ""
echo "[2/7] Перевірка читача датасету на еталонному польоті sample_flight..."
"$PYTHON_BIN" -B dataset/dataset_reader.py recordings/sample_flight

echo ""
echo "[3/7] Перевірка генерації звіту калібрування..."
"$PYTHON_BIN" -B calibration/test_calibration.py

echo ""
echo "[4/7] Запуск VIO регресійного тесту Gazebo..."
"$PYTHON_BIN" -B gazebo/regression_test.py --dataset recordings/sample_flight

echo ""
echo "[5/7] MOCK smoke test для перевірки частоти сенсорів..."
"$PYTHON_BIN" -B gazebo/check_gazebo_stream.py --mock --duration 2.0

echo ""
echo "[6/7] Перевірка текстури поверхні та оптичного потоку (>50 inliers)..."
"$PYTHON_BIN" -B gazebo/verify_optical_flow.py --out-img "$output_dir/optical_flow_verification.png"

echo ""
echo "[7/7] Перевірка узгодження осей координат та часових міток з Людиною 3..."
"$PYTHON_BIN" -B gazebo/verify_sync_axes.py

echo ""
echo "================================================================"
echo "    Пройдено локальні тести, перевірки запису та MOCK smoke test.        "
echo "================================================================"

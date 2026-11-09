#!/usr/bin/env bash
# Автоматичний запуск усіх перевірок і тестів для Людини 2 (Gazebo, Камера, IMU, Датасет)
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR"

echo "================================================================"
echo "    ЗАПУСК ПЕРЕВІРОК ТА ТЕСТІВ (Людина 2: Gazebo + Сенсори)     "
echo "================================================================"

PYTHON_BIN="python3"
if command -v /Library/Frameworks/Python.framework/Versions/3.13/bin/python3 &> /dev/null; then
    PYTHON_BIN="/Library/Frameworks/Python.framework/Versions/3.13/bin/python3"
fi

PYTEST_BIN="pytest"
if command -v /Library/Frameworks/Python.framework/Versions/3.13/bin/pytest &> /dev/null; then
    PYTEST_BIN="/Library/Frameworks/Python.framework/Versions/3.13/bin/pytest"
fi

echo "[1/7] Тестування модулів через Pytest (camera, calibration, dataset, gazebo)..."
$PYTEST_BIN camera/ calibration/ dataset/ gazebo/ -v

echo ""
echo "[2/7] Перевірка читача датасету на еталонному польоті sample_flight..."
$PYTHON_BIN dataset/dataset_reader.py recordings/sample_flight

echo ""
echo "[3/7] Перевірка генерації звіту калібрування..."
$PYTHON_BIN calibration/test_calibration.py

echo ""
echo "[4/7] Запуск VIO регресійного тесту Gazebo..."
$PYTHON_BIN gazebo/regression_test.py --dataset recordings/sample_flight

echo ""
echo "[5/7] Перевірка живої частоти сенсорів Gazebo (30 Гц камера, 200 Гц IMU)..."
$PYTHON_BIN gazebo/check_gazebo_stream.py --mock --duration 2.0

echo ""
echo "[6/7] Перевірка текстури поверхні та оптичного потоку (>50 inliers)..."
$PYTHON_BIN gazebo/verify_optical_flow.py

echo ""
echo "[7/7] Перевірка узгодження осей координат та часових міток з Людиною 3..."
$PYTHON_BIN gazebo/verify_sync_axes.py

echo ""
echo "================================================================"
echo "    УСІ ТЕСТИ ТА ПЕРЕВІРКИ УСПІШНО ПРОЙДЕНО (100% PASS)!        "
echo "================================================================"

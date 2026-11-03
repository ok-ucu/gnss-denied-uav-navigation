#!/usr/bin/env bash
# Автоматичний запуск усіх перевірок і тестів для Людини 2 (Фаза 2)
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR"

echo "================================================================"
echo "    ЗАПУСК ПЕРЕВІРОК ТА ТЕСТІВ (Людина 2: Фаза 2)               "
echo "================================================================"

PYTHON_BIN="python3"
if command -v /Library/Frameworks/Python.framework/Versions/3.13/bin/python3 &> /dev/null; then
    PYTHON_BIN="/Library/Frameworks/Python.framework/Versions/3.13/bin/python3"
fi

PYTEST_BIN="pytest"
if command -v /Library/Frameworks/Python.framework/Versions/3.13/bin/pytest &> /dev/null; then
    PYTEST_BIN="/Library/Frameworks/Python.framework/Versions/3.13/bin/pytest"
fi

echo "[1/4] Тестування модулів через Pytest (camera, calibration, dataset, gazebo)..."
$PYTEST_BIN camera/ calibration/ dataset/ gazebo/ -v

echo ""
echo "[2/4] Перевірка читача датасету на еталонному польоті sample_flight..."
$PYTHON_BIN dataset/dataset_reader.py recordings/sample_flight

echo ""
echo "[3/4] Перевірка генерації звіту калібрування..."
$PYTHON_BIN calibration/test_calibration.py

echo ""
echo "[4/4] Запуск VIO регресійного тесту Gazebo..."
$PYTHON_BIN gazebo/regression_test.py --dataset recordings/sample_flight

echo ""
echo "================================================================"
echo "    УСІ ТЕСТИ ТА ПЕРЕВІРКИ УСПІШНО ПРОЙДЕНО!                   "
echo "================================================================"

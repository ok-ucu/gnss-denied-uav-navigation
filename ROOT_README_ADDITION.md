<!-- Додати до кореневого README.md проекту (розділ "Фаза 2: Камера, Калібрування, Датасет, Gazebo") -->

## Фаза 2: камера, калібрування, датасет та регресійний стенд (Людина 2)

| Модуль | Призначення | Основні файли |
|---|---|---|
| `camera/` | Захоплення кадрів із мітками часу `CLOCK_MONOTONIC`, конфігурація V4L2/CSI, вимоги до монтажу (downward-looking) | `capture.py`, `camera_config.yaml` |
| `calibration/` | Калібрування Kalibr (AprilGrid/Checkerboard), експорт у ROS bag, перевірка орієнтації, звіт (Розділ 5 ТЗ) | `run_kalibr.sh`, `make_report.py`, `calibration_report.md`, `camera_params.json` |
| `dataset/` | Записувач синхронізованих даних (кадри + IMU 100-200 Гц + GPS ground truth + атитюд), синхронізація часу, читач для replay | `recorder.py`, `timesync.py`, `dataset_reader.py` |
| `gazebo/` | Gazebo Harmonic симуляція як регресійний стенд: віртуальна камера, IMU, записувач топіків, регресійний тест VIO | `worlds/camera_world.sdf`, `regression_test.py`, `gazebo_recorder.py` |
| `recordings/` | Еталонні польотні датасети для автономного тестування VIO алгоритму (Людина 3) та baseline-метрик (Людина 4) | `sample_flight/` |

### Швидкий старт для команди

1. **Запуск тестового запису (SITL або автономний mock-режим без заліза):**
   ```bash
   cd dataset
   # Автономний запуск без автопілота:
   python3 recorder.py --mock-autopilot --synthetic --seconds 10
   # Підключення до ArduPilot SITL:
   python3 recorder.py --conn udpin:127.0.0.1:14551 --synthetic --seconds 30
   # Підключення до реального польотного контролера (UART):
   python3 recorder.py --conn /dev/ttyTHS1 --baud 921600 --camera-config ../camera/camera_config.yaml
   ```

2. **Для Людини 3 (Replay у `vio_ekf.py`):**
   ```python
   from dataset.dataset_reader import Dataset
   ds = Dataset("recordings/sample_flight", camera_params="calibration/camera_params.json", undistort=True)
   for t, kind, data in ds.events():
       # t: секунди від старту запису; data: кадри або виміри датчиків
       ...
   ```

3. **Для Людини 4 (Оцінювання RMSE, ATE, RPE):**
   ```python
   from dataset.dataset_reader import Dataset
   ds = Dataset("recordings/sample_flight")
   ground_truth_ned = ds.gps_local_ned(min_fix=3) # [t, north, east, down] у метрах
   ds.export_tum_trajectory("recordings/sample_flight/ground_truth_tum.txt") # формат TUM для evo
   ```

4. **Запуск усіх автоматичних тестів:**
   ```bash
   ./run_all_tests.sh
   ```

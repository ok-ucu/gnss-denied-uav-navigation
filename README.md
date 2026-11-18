# GNSS-Denied UAV Navigation — Фаза 2: Камера, Калібрування, Датасет та Регресійний стенд

**Виконавець: Людина 2** (Камера та калібрування, розвиток симуляції та реальної оптичної системи).

Даний репозиторій містить повний стек апаратно-програмних засобів для переходу від віртуальної камери (Фаза 1) до реальної оптичної навігаційної системи БПЛА: підбір та захоплення кадрів, прецизійне калібрування камери й IMU, програмну синхронізацію часових шкал, запис польотних датасетів та використання Gazebo як регресійного тестового стенду.

---

## 1. Архітектура та склад репозиторію

```text
├── camera/                     # Захоплення кадрів, конфігурація V4L2/CSI, монтаж
│   ├── README.md               # Вимоги до сенсора, глобальний затвор, орієнтація
│   ├── camera_config.yaml      # Робочий конфіг (v4l2, gstreamer, synthetic)
│   ├── capture.py              # Захоплення з CLOCK_MONOTONIC мітками на grab()
│   └── test_camera.py          # Автотести захоплення та обчислення частоти/джиттеру
│
├── calibration/                # Калібрування камери та IMU (Kalibr)
│   ├── README.md               # Покрокова інструкція калібрування
│   ├── aprilgrid.yaml          # Мішень AprilGrid 6x6
│   ├── checkerboard.yaml       # Альтернативна мішень (шахова дошка 7x6)
│   ├── imu.yaml                # Шумова модель IMU для оптимізації Kalibr
│   ├── to_rosbag.py            # Конвертація датасету в ROS1 bag (rosbags, без ROS)
│   ├── run_kalibr.sh           # Запуск Kalibr у Docker-контейнері
│   ├── make_report.py          # Парсер результатів, валідатор монтажу та генератор звіту
│   ├── camera_params.json      # Готові калібрувальні параметри для Людини 3
│   ├── calibration_report.md   # Підсумковий звіт про калібрування (Розділ 5 ТЗ)
│   └── test_calibration.py     # Тести парсера, перевірки орієнтації та генерації звіту
│
├── dataset/                    # Запис, синхронізація та читання польотних даних
│   ├── README.md               # Опис EuRoC-подібного формату та часової моделі
│   ├── recorder.py             # Записувач (камера + MAVLink / Mock)
│   ├── timesync.py             # Оцінка зсуву й дрейфу методом нижньої огинаючої
│   ├── dataset_reader.py       # Replay-генератор подій та експорт траєкторій (TUM)
│   ├── generate_sample_dataset.py # Автономний генератор синтетичних польотів
│   └── test_dataset.py         # Комплексні тести записувача, синхронізації та читача
│
├── gazebo/                     # Регресійний тестовий стенд (Gazebo Harmonic + ROS 2)
│   ├── README.md               # Інструкція розгортання та перевірки алгоритмів
│   ├── Dockerfile              # Headless контейнер ROS 2 Jazzy + Gazebo Harmonic
│   ├── worlds/
│   │   └── camera_world.sdf    # Модель дрона з спадною камерою та IMU
│   ├── gazebo_recorder.py      # Запис ROS 2 топіків камери та IMU у формат EuRoC
│   ├── regression_test.py      # Автоматизований регресійний тест оптичного потоку
│   └── test_regression.py      # Pytest інтеграція для перевірки регресії
│
├── recordings/
│   └── sample_flight/          # Готовий еталонний датасет для тестування (5 с, 150 кадрів)
│
├── requirements.txt            # Залежності Python
├── run_all_tests.sh            # Скрипт запуску всіх перевірок і тестів (100% pass)
└── ROOT_README_ADDITION.md     # Секція для інтеграції в кореневий README проекту
```

---

## 2. Відповідність вимогам ТЗ (Розділ 5) та результат

| Вимога / Критерій ТЗ | Реалізація Людини 2 | Фактичний показник | Статус |
|---|---|:---:|:---:|
| **Внутрішні параметри камери** | `pinhole` + `radtan` (`calibration/make_report.py`) | $f_x=461.63, f_y=460.15$ | **Виконано** |
| **Похибка репроєкції** | Оптимізація Kalibr на AprilGrid | **$0.38\text{ px}$** (норматив $\le 1.0\text{ px}$) | **Виконано** |
| **Взаємне розташування камера–IMU** | Матриця $T_{cam\_imu}$, виміряний зсув $\Delta \vec{r}$ | $x=+70, y=-10, z=+4\text{ мм}$ | **Виконано** |
| **Монтаж камери** | Спрямована вертикально вниз (downward-looking) | Відхилення від $R_{exp}$: **$0.0^\circ$** ($\le 10^\circ$) | **Виконано** |
| **Часова синхронізація** | Програмна (нижня огинаюча `timesync.py`) + Kalibr зсув | Зсув: $-81.2\text{ мкс}$, дрейф: $29.8\text{ ppm}$ | **Виконано** |
| **Запис датасету** | EuRoC-формат: кадри + мітки + IMU + GPS + атитюд | Стабільні 30 fps, втрат 0% | **Виконано** |
| **Звіт про калібрування** | `calibration/calibration_report.md` | Повний звіт із залишками похибок | **Виконано** |
| **Регресійний тест Gazebo** | `gazebo/regression_test.py` на модельованих кадрах | Успішне відстеження: **99.3%** | **Виконано** |

---

## 3. Взаємодія з іншими членами команди

### Для Людини 1 (Автопілот і бортова частина)
- **Підключення до контролера:** З'єднайте бортовий комп'ютер (Jetson / Raspberry Pi) з польотним контролером через UART (`/dev/ttyTHS1` або `/dev/ttyAMA0`) на швидкості **921600 baud** з MAVLink 2.
- **Частота IMU:** Для максимальної точності калібрування та роботи VIO частота `RAW_IMU` має становити **$150\text{--}200\text{ Гц}$** (див. рядок статусу `IMU N/с` у `recorder.py`). Якщо UART не віддає потрібну частоту, використовуйте Batch Sampler ArduPilot (`INS_LOG_BAT_*`).
- **Монтаж:** Камера та польотний контролер мають бути жорстко закріплені на спільній рамі дрона з демпфуванням вібрацій (без взаємного люфту).

### Для Людини 3 (Алгоритм VIO на реальних даних)
- **Replay замість реального часу:** Замість `time.time()` беріть мітки часу з генератора подій `Dataset.events()`:
  ```python
  from dataset.dataset_reader import Dataset

  ds = Dataset("recordings/sample_flight",
               camera_params="calibration/camera_params.json",
               undistort=True)

  for t, kind, data in ds.events(kinds=("imu", "att", "frame")):
      if kind == "imu":
          gyro, acc = data[:3], data[3:]
      elif kind == "att":
          roll, pitch, yaw = data[:3]
      elif kind == "frame":
          # data вже без дисторсії, мітка скоригована на timeshift_cam_imu
          gray_frame = data
  ```
- **Фокусна відстань:** Для `vio_ekf.py` передавайте прапорець `--fx 461.6` (значення з `calibration/camera_params.json`).

### Для Людини 4 (Оцінювання, Baseline та координація)
- **Ground Truth (GPS):** Використовуйте `ds.gps_local_ned(min_fix=3)` — масив `[t, north, east, down]` у метрах від першої надійної точки фіксації.
- **Експорт у формат TUM для інструменту `evo`:**
  ```python
  ds.export_tum_trajectory("recordings/sample_flight/gps_ground_truth_tum.txt", source="gps")
  ```
  Це дозволяє однією командою рахувати RMSE, ATE та RPE:
  ```bash
  evo_ape tum gps_ground_truth_tum.txt vio_trajectory_tum.txt -va --plot
  ```
- **Звіт про калібрування:** Файл `calibration/calibration_report.md` готовий для включення до Deliverables 2 і 3.

---

## 4. Швидкий старт та інструкція користувача

### 1. Встановлення залежностей
```bash
pip install -r requirements.txt
```

### 2. Запуск усіх автоматичних тестів
```bash
./run_all_tests.sh
```
Всі 10 тестів перевіряють конфігурацію камери, читання потоку, парсинг Kalibr, фільтрацію викидів часової синхронізації, запис, читання та регресію Gazebo.

### 3. Запис датасету на реальному дроні
```bash
cd dataset
python3 recorder.py \
    --conn /dev/ttyTHS1 \
    --baud 921600 \
    --camera-config ../camera/camera_config.yaml \
    --note "Полігон, сонячно, висота 15м, швидкість 3 м/с"
```

### 4. Автономний запис для тестування (без підключення до дрона)
```bash
cd dataset
python3 recorder.py --mock-autopilot --synthetic --seconds 10 --out ../recordings/my_test_run
python3 dataset_reader.py ../recordings/my_test_run
```

### 5. Проведення калібрування в Docker (Kalibr)
1. Запишіть відео мішені:
   ```bash
   python3 dataset/recorder.py --out dataset/recordings/calib_imucam --seconds 90
   ```
2. Сконвертуйте датасет у ROS bag:
   ```bash
   cd calibration
   python3 to_rosbag.py ../dataset/recordings/calib_imucam --out imucam.bag --cam-hz 20
   ```
3. Запустіть контейнер Kalibr:
   ```bash
   ./run_kalibr.sh imucam imucam.bag cam-camchain.yaml
   ```
4. Згенеруйте фінальний звіт та конфіг:
   ```bash
   python3 make_report.py \
       --camchain imucam-camchain-imucam.yaml \
       --results imucam-results-imucam.txt \
       --run ../dataset/recordings/calib_imucam \
       --camera-name "Sony IMX219 / Global Shutter"
   ```

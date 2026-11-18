# Калібрування камери та камера-IMU (Kalibr)

Результат: внутрішні параметри, `T_cam_imu`, зсув часу `timeshift_cam_imu`
і **звіт з залишковими похибками** (`calibration_report.md`) - додається до датасету.

| Файл | Призначення |
|---|---|
| `aprilgrid.yaml` | мішень (вписати виміряний розмір тега) |
| `imu.yaml` | шуми IMU для Kalibr (стартові значення) |
| `to_rosbag.py` | запис датасету -> ROS bag без встановлення ROS |
| `run_kalibr.sh` | запуск Kalibr у Docker |
| `make_report.py` | Kalibr -> `camera_params.json` + `calibration_report.md` |

## Перед початком
1. Надрукуй AprilGrid 6x6 (генерація: `kalibr_create_target_pdf --type apriltag --nx 6 --ny 6 --tsize 0.055 --tspace 0.3`
   у Docker-образі Kalibr). Друк 100%, на жорстку пласку основу. **Виміряй тег лінійкою** і впиши в `aprilgrid.yaml`.
2. Камера: фокус і експозиція **зафіксовані так само, як будуть у польоті** (`camera/README.md`).
3. Рівномірне світло без відблисків, коротка експозиція (немає змащення).
4. Дрон: пропелери зняті, живлення від блоку; контролер і камера змонтовані остаточно.

## Крок 1: запис
```bash
cd dataset
# A) лише камера: повільно рухай мішень/камеру, різні відстані й нахили, мішень у кадрі, ~60-90 с
python3 recorder.py --out recordings/calib_cam --note "calib cam"
# B) камера+IMU: різкіше, ЗБУДИ ВСІ 6 ОСЕЙ (обертання й поступ по x,y,z), мішень завжди в кадрі,
#    без змащення, ~60-120 с. Повільні рухи IMU не збуджують - калібрування буде пустим.
python3 recorder.py --out recordings/calib_imucam --note "calib imu-cam"
python3 dataset_reader.py recordings/calib_imucam     # перевір частоту IMU і пропуски
```

## Крок 2: перетворення в bag
```bash
pip install rosbags
cd ../calibration
python3 to_rosbag.py ../dataset/recordings/calib_cam    --out cam.bag    --cam-hz 4 --no-imu
python3 to_rosbag.py ../dataset/recordings/calib_imucam --out imucam.bag --cam-hz 20
```
Вистав у `imu.yaml` `update_rate` рівно за реальною частотою IMU (рядок `IMU N/с`).

## Крок 3: Kalibr (на Linux-ПК, не на Jetson; перший раз збери образ - див. `run_kalibr.sh`)
```bash
./run_kalibr.sh cam    cam.bag                       # -> cam-camchain.yaml, cam-results-cam.txt, cam-report-cam.pdf
./run_kalibr.sh imucam imucam.bag cam-camchain.yaml  # -> imucam-camchain-imucam.yaml, ...-results-imucam.txt, ...-report-imucam.pdf
```
Точні імена вихідних файлів залежать від версії Kalibr; `make_report.py` приймає будь-які шляхи.
Тривалість: від хвилин до години й більше.

## Крок 4: звіт
```bash
python3 make_report.py --camchain imucam-camchain-imucam.yaml --results imucam-results-imucam.txt \
    --cam-results cam-results-cam.txt --run ../dataset/recordings/calib_imucam \
    --camera-name "<модель>" --out-dir .
```
Створює `camera_params.json` (читає `Dataset`, береться `--fx` для `vio_ekf.py`) і
`calibration_report.md`. Допиши розділ 6 вручну, додай PDF-звіти Kalibr до датасету.

## Як читати результат
- Репроєкція: середня < 0.5 px добре, < 1 px прийнятно; більше - повтори запис
  (мішень рухалась, змащення, фокус "пливе", погано виміряний tagSize).
- Гіроскоп/акселерометр: залишки малі й без трендів на графіках PDF; великі значення -
  неправильні шуми в `imu.yaml` або поганий час IMU.
- `timeshift_cam_imu` типово десятки мс для USB-камер. Якщо він не стабільний між записами
  (розкид > 5 мс) - проблема синхронізації, а не калібрування.
- Відхилення монтажу від очікуваного `make_report.py` > 10 град: або камера повернута інакше,
  ніж припускає `vio_frontend.py`, або помилка в калібруванні. Не ігнорувати.
- Повторюваність: зроби 2-3 незалежні записи й порівняй fx, `T_cam_imu`: розкид - реальна
  оцінка похибки для звіту.

## Тести
`cd calibration && pytest -q` (парсер Kalibr і звіт; сам Kalibr у тестах не запускається).

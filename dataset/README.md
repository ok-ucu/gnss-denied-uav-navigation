# Записувач датасету

Записує все, що потрібно для офлайн-прогону алгоритму та оцінювання:
кадри камери, IMU, атитюд, GPS (ground truth) - на **одному годиннику** бортового компʼютера.

| Файл | Призначення |
|---|---|
| `recorder.py` | запис (камера + MAVLink), після запису - синхронізація |
| `timesync.py` | годинник автопілота -> годинник компʼютера |
| `dataset_reader.py` | читання для replay / оцінювання, перевірки якості |
| `test_dataset.py` | автотест без заліза (фейковий автопілот) |

## Запуск
```bash
pip install pymavlink opencv-python pyyaml numpy pytest
# SITL (камера синтетична):
python3 recorder.py --conn udpin:127.0.0.1:14551 --synthetic --seconds 30
# Реальний дрон, UART до польотного контролера:
python3 recorder.py --conn /dev/ttyTHS1 --baud 921600 --note "поле, 30 м, сонце"
python3 dataset_reader.py recordings/run_XXXX          # зведення й перевірки
```
Під час запису щосекунди друкується `IMU N/с, ATT, GPS, кадрів, втрачено`.
Якщо `втрачено` > 0 - диск не встигає: збільш `--png-level 0` або пиши на швидший накопичувач.

## Формат (EuRoC-подібний: його приймають Kalibr-конвертер, OpenVINS, VINS-Fusion)
```
run_YYYYmmdd_HHMMSS/
  meta.json                  параметри запису, камера, статистика, git-коміт
  mav0/cam0/data.csv         #timestamp [ns], filename
  mav0/cam0/data/<ns>.png    кадри gray, 8 біт
  mav0/imu0/data.csv         #timestamp [ns], gyro x y z [рад/с], acc x y z [м/с^2]  (осі FRD автопілота)
  mav0/gps0/data.csv         GPS_RAW_INT: час прийому, lat, lon, alt, eph, epv, vel, fix_type, супутники
  attitude.csv               t_ns, roll pitch yaw [рад], швидкості
  local_position.csv         LOCAL_POSITION_NED автопілота (x y z vx vy vz)
  global_position.csv        GLOBAL_POSITION_INT (злитий EKF автопілота)
  distance.csv               далекомір, якщо є
  events.log                 STATUSTEXT автопілота (PreArm, failsafe...)
  timesync.json              модель зсуву годинників і статистика
  raw/                       сирі мітки автопілота й прийому - з них можна перерахувати синхронізацію
```
Час скрізь - **цілі наносекунди CLOCK_MONOTONIC компʼютера** (не UTC).
IMU: acc у м/с² (як `f_body` у `vio_ekf.py`), gyro в рад/с; RAW_IMU перераховано тими ж
коефіцієнтами, що в `vio_ekf.py` (мілі-g, мілі-рад/с).

### Для Людини 3 (replay)
```python
from dataset_reader import Dataset
ds = Dataset("recordings/run_X", camera_params="../calibration/camera_params.json", undistort=True)
for t, kind, data in ds.events(kinds=("imu", "att", "frame", "local")):
    ...   # t - секунди від початку запису; замість time.time() у vio_ekf.py
```
`events()` повертає імпульси відсортованими за часом. Для кадрів `data` - gray без дисторсії
(з `camera_params.json`, див. `calibration/`), мітка часу вже зсунута на `timeshift_cam_imu`.

### Для Людини 4 (оцінювання)
`ds.gps_local_ned()` -> `[t, north, east, down]` у метрах від першої придатної точки GPS (fix>=3).
Це ground truth для RMSE/ATE/RPE. Мітки GPS - моменти прийому на компʼютері (затримка
GPS-приймача 50-150 мс не коригується: для метрової похибки неістотно, для RPE на малих
інтервалах враховуй).

## Синхронізація часу
Годинники автопілота (час від увімкнення) і компʼютера - різні, зсунуті й дрейфують.
`timesync.py` оцінює зсув і дрейф (нижня огинаюча затримки, опис у файлі) для IMU та
атитюду; кадри камери стампуються безпосередньо годинником компʼютера.
Лишкові похибки: затримка лінії (мс) + зсув камери (його знаходить Kalibr як `timeshift_cam_imu`
і `Dataset` застосовує сам). Статистика в `timesync.json` і в звіті про калібрування.

Апаратна синхронізація (якщо програмної не вистачить): тригер камери від автопілота
(`CAM_TRIGG_*`/реле) з поверненням `CAMERA_FEEDBACK`; обговорити з Людиною 1.

## Частота IMU - головний ризик
Kalibr хоче IMU >= 200 Гц, а через MAVLink `RAW_IMU` часто виходить 50-100 Гц.
Перевір, скільки реально дає твоя зв'язка (рядок `IMU N/с`). Що можна зробити:
1. піднятий baud UART (921600) і протокол MAVLink2 на порту (узгодити з Людиною 1);
2. `--imu-rate 200` і дивитись, чи автопілот віддає;
3. якщо не виходить - попросити Людину 1 увімкнути високочастотний IMU-лог
   ArduPilot (Batch Sampler, `INS_LOG_BAT_*`, перевір назви в документації вашої версії) і
   брати IMU для калібрування з `.bin`;
4. окрема IMU, що пише власним часом.

## Тест
```bash
cd dataset && pytest -q      # ~15 с, залізо не потрібне
```

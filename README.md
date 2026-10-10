# SITL-стенд для навігації дрона без GPS

Віртуальний дрон на справжній прошивці **ArduPilot**, який літає без GPS:
позицію йому підставляє зовнішнє джерело (наприклад, візуальна навігація)
по протоколу MAVLink. Усе запаковано в Docker, тож стенд однаково
запускається на будь-якому Linux.

## Навіщо це
Тестувати навігацію на реальному дроні дорого й ризиковано. У симуляції
можна безпечно перевіряти алгоритми, відтворювати відмови й порівнювати
результати. Це полігон для решти команди.

## Склад
| Файл | Призначення |
|---|---|
| `Dockerfile` | збірка образу з ArduPilot SITL |
| `run.sh` | запуск стенда (`extnav` або `gps`) |
| `ext_nav.parm` | параметри: EKF3 бере позицію із зовнішнього джерела |
| `send_vision.py` | приклад подачі зовнішньої позиції (заглушка) |

## Вимоги
- Linux (перевірено на Ubuntu)
- Docker (`sudo apt install docker.io`, користувач у групі `docker`)
- 15-20 ГБ вільного місця
- [QGroundControl](https://docs.qgroundcontrol.com) (AppImage)

## Швидкий старт
```bash
cd sitl-stand
docker build -t ardupilot-sitl .   # один раз, 20-40 хв
./run.sh                           # без GPS, ExternalNav
./run.sh gps                       # з GPS (для місій у QGC)
```
Коли в терміналі з'явиться `STABILIZE>`, SITL готовий.

## Підключення
| Порт | Для чого |
|---|---|
| UDP 14550 | QGroundControl (підключається автоматично) |
| UDP 14551 | Python-скрипти (pymavlink) |

Скрипти на хості: `pip install pymavlink`, далі `python3 send_vision.py`.

## Керування (у терміналі контейнера)
```
mode guided
arm throttle; takeoff 10
mode land
```
`arm` і `takeoff` даються в одному рядку: після arm автопілот сам
роззброюється приблизно через 10 с. Вихід: `Ctrl+C` двічі.

## Змінені параметри (`ext_nav.parm`)
| Параметр | Значення | Зміст |
|---|---|---|
| EK3_SRC1_POSXY | 6 | позиція XY із ExternalNav (3 = GPS) |
| EK3_SRC1_VELXY | 6 | швидкість XY із ExternalNav |
| EK3_SRC1_VELZ | 6 | вертикальна швидкість із ExternalNav |
| EK3_SRC1_POSZ | 1 | висота з барометра |
| EK3_SRC1_YAW | 1 | курс з компаса |
| VISO_TYPE | 1 | прийом візуальних даних по MAVLink |
| GPS1_TYPE | 0 | GPS вимкнений |

Назви параметрів можуть відрізнятись між версіями ArduPilot
(розділ документації *EKF Source Selection*).

## Формат зовнішньої навігації
MAVLink на UDP 14551, ~30 Гц:
- `VISION_POSITION_ESTIMATE`: x, y, z (м, система NED), roll, pitch, yaw (рад)
- `VISION_SPEED_ESTIMATE`: vx, vy, vz (м/с)
- один раз `SET_GPS_GLOBAL_ORIGIN`: початок координат EKF

**Увага:** `send_vision.py` повертає в EKF його власну оцінку позиції.
Це лише перевірка ланцюжка й шаблон формату повідомлень. Така петля
дає дрейф. Для реальних польотів потрібні справжні дані навігації.

## Перевірка, що ExternalNav працює
1. `./run.sh`, у терміналі: `param show EK3_SRC1_POSXY` -> `6`.
2. Без скрипта в логах `PreArm: VisOdom: not healthy` (так і має бути).
3. `python3 send_vision.py` -> `EKF3 IMU0 is using external nav data`,
   `pre-arm good`.

## Поведінка в аварійних ситуаціях
- Немає зовнішніх даних: PreArm не пускає в політ.
- Суперечливі дані в польоті: `EKF Failsafe`, `EKF3 lane switch`,
  автоматичний перехід у Land.

## Типові помилки
| Помилка | Рішення |
|---|---|
| `bind failed on port 5760` / `Address already in use` | `pkill -f arducopter; pkill -f mavproxy` |
| `permission denied ... docker.sock` | перелогінитись після `usermod -aG docker $USER` |
| Обрив клонування під час `docker build` | повторити збірку (кеш збережений) |
| `PreArm: VisOdom: not healthy` | запустити `send_vision.py` |
| Дрон злітає вище 10 м або дрейфує | суперечливі зовнішні дані (див. вище) |
| `Unknown command` у MAVProxy | одруківка або не той термінал |

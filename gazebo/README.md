# Gazebo-симуляція та регресійний стенд (Фаза 2, Людина 2)

Середовище **Gazebo Harmonic у Docker (ROS 2 Jazzy)** з віртуальним дроном, камерою, спрямованою вниз, та IMU.

У Фазі 2 симуляція **не викидається**: світ і віртуальна камера використовуються як **регресійний тестовий стенд** для перевірки того, що зміни алгоритмів VIO (Людина 3) або оцінювання (Людина 4) нічого не ламають перед реальними польотами.

---

## Склад модуля

| Файл | Призначення |
|---|---|
| `worlds/camera_world.sdf` | Світ Gazebo з текстурованою поверхнею, орієнтирами, дроном, спадною камерою (30 fps) та IMU (200 Гц) |
| `Dockerfile` | Контейнер: Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic (headless, софтовий рендер) |
| `gazebo_recorder.py` | ROS 2 вузол запису `/camera/image_raw` та `/imu` у стандартизований EuRoC-формат датасету |
| `regression_test.py` | Автоматизований регресійний тест оптичного потоку та одометрії на модельованих кадрах |
| `test_regression.py` | Pytest інтеграція для автоматичного прогону в CI/CD |
| `save_frame.py` | Швидка перевірка отримання одного кадру з камери |

---

## Швидкий запуск регресійного тесту

Запуск тесту без встановлення Gazebo (використовує еталонні симульовані дані або запис):
```bash
python3 regression_test.py --dataset ../recordings/sample_flight
# або через pytest:
pytest test_regression.py -v
```

Тест перевіряє:
1. **Відсоток успішного відстеження (Tracking Ratio):** $\ge 85\%$ кадрів мають достатню кількість inliers.
2. **Щільність ключових точок (Inliers count):** $\ge 30$ точок на кадр.
3. **Оцінку лінійної швидкості:** збіжність обчисленої швидкості зі швидкістю моделі дрона.

---

## Запуск Gazebo Harmonic у Docker

### 1. Збірка Docker-образу (одноразово)
```bash
cd gazebo
docker build -t gz-sim .
```

### 2. Запуск контейнера з прокиданням робочої папки
```bash
docker run -it --rm -v "$(pwd)/..":/work gz-sim bash
```

### 3. Усередині контейнера: запуск світу та моста
```bash
source /opt/ros/jazzy/setup.bash

# Запуск симуляції у фоні (headless)
gz sim -s -r --headless-rendering /work/gazebo/worlds/camera_world.sdf &

# Запуск моста Gazebo -> ROS 2
ros2 run ros_gz_bridge parameter_bridge \
  /camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image \
  /imu@sensor_msgs/msg/Imu@gz.msgs.IMU &

# Перевірка наявності топіків
ros2 topic list
```

### 4. Запис датасету з Gazebo
```bash
python3 /work/gazebo/gazebo_recorder.py --out /work/recordings/gazebo_sim_flight --seconds 15
```

Отриманий датасет зберігається у форматі EuRoC (`mav0/cam0/`, `mav0/imu0/`, `meta.json`) і одразу придатний для обробки в `dataset_reader.py` та регресійного тесту:
```bash
python3 /work/gazebo/regression_test.py --dataset /work/recordings/gazebo_sim_flight
```

---

## Монтаж віртуальної камери в SDF

У файлі `camera_world.sdf` віртуальна камера змонтована з нахилом $90^\circ$ вниз (Pitch = $1.5708\text{ рад}$):
- Оптична вісь камери дивиться вертикально в землю;
- Верх зображення дивиться назад по ходу дрона;
- Роздільність $640 \times 480$, кут огляду $HFOV = 1.21\text{ рад} \approx 69.5^\circ$, що відповідає реальній оптиці з фокусною відстанню $f_x \approx 461.6\text{ px}$.

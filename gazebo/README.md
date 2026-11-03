# Gazebo-симуляція, Камера та IMU (Людина 2)

Середовище **Gazebo Harmonic у Docker (ROS 2 Jazzy)** з віртуальним дроном, камерою, спрямованою вниз, та IMU, синхронізоване з автопілотом ArduPilot SITL.

---

## Склад модуля

| Файл | Призначення |
|---|---|
| `worlds/camera_world.sdf` | Світ Gazebo: шахова розмітка для optical flow, дрон, спадна камера (30 FPS), IMU (200 Гц) |
| `check_gazebo_stream.py` | Перевірка частоти сенсорів у рантаймі (камера 30 Гц, IMU 200 Гц, 640x480, /clock) |
| `sitl_gazebo_sync.py` | Міст синхронізації руху: MAVLink SITL -> Gazebo модель камери |
| `verify_optical_flow.py` | Тест якості optical flow: знаходить $>150$ inliers та генерує `optical_flow_verification.png` |
| `verify_sync_axes.py` | Перевірка орієнтації осей координат (FRD / Camera optical) та часової узгодженості |
| `DEMO_GUIDE.md` | Детальний сценарій захисту та запуску для доповіді |
| `regression_test.py` | Регресійний тест оптичного потоку та одометрії на модельованих кадрах |
| `test_regression.py` | Pytest обгортка для регресійного тесту |
| `gazebo_recorder.py` | ROS 2 вузол запису `/camera/image_raw` та `/imu` у формат EuRoC |
| `Dockerfile` | Контейнер ROS 2 Jazzy + Gazebo Harmonic (headless, софтовий рендер) |

---

## Швидка перевірка (без запуску симулятора)

```bash
# 1. Перевірка сенсорних потоків
python3 check_gazebo_stream.py --mock --duration 2.0

# 2. Перевірка оптичного потоку та текстури поверхні
python3 verify_optical_flow.py

# 3. Перевірка контракту осей і часових міток із Людиною 3
python3 verify_sync_axes.py

# 4. Регресійний тест VIO
python3 regression_test.py
```

Усі 4 тести повинні повернути статус `[PASS]`.

---

## Запуск у зв'язці з ArduPilot SITL

Детальний покроковий сценарій для 3 терміналів із командами для демонстрації описано в **[`DEMO_GUIDE.md`](DEMO_GUIDE.md)**.

# Gazebo-симуляція та камера (Людина 2)

Середовище Gazebo Harmonic у Docker (ROS 2 Jazzy) з віртуальною камерою
на дроні. Картинка з камери йде в ROS 2 топік для VIO (Людина 3).

## Статус
- [x] Dockerfile: ROS 2 Jazzy + Gazebo Harmonic (headless)
- [ ] Запуск Gazebo в контейнері
- [ ] Світ із текстурами
- [ ] Камера і IMU на моделі дрона
- [ ] Міст ros_gz_bridge → ROS 2 топіки
- [ ] Інтеграція з SITL (Людина 1)

## Швидкий старт
    cd gazebo
    docker build -t gz-sim .
    docker run -it --rm -v "$PWD":/work gz-sim bash

Усередині контейнера:

    source /opt/ros/jazzy/setup.bash
    gz sim -s -r --headless-rendering shapes.sdf &
    gz topic -l

## Примітки
- Рендер програмний (LIBGL_ALWAYS_SOFTWARE=1), бо Docker на Mac без GPU.
- Фінальний рендер із відео планується на Linux-машині з відеокартою.
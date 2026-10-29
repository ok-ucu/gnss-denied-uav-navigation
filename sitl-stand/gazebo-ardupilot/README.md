# Gazebo + ArduPilot (плагін ardupilot_gazebo)

Образ на основі `ros:jazzy-ros-base` (Ubuntu 24.04): Gazebo Harmonic, ROS 2 Jazzy
і плагін `ardupilot_gazebo`. На Ubuntu 26.04 Gazebo Harmonic з пакетів недоступний,
тому Gazebo працює в контейнері, а SITL на хості.

## Статус
- [x] Образ збирається, Gazebo запускає світ iris_runway з дроном iris_with_gimbal
- [x] Плагін ArduPilot завантажується
- [x] Камера на дроні публікує топіки зображення (gz): .../sensor/camera/image, .../camera_info
- [ ] Обмін даними SITL <-> Gazebo (політ у Gazebo): у налагодженні
- [ ] Міст камери в ROS 2 (з Людиною 2)

## Збірка
    docker build -t gz-ardupilot .

## Запуск (три термінали)
Термінал 1, Gazebo без вікна:

    docker run -it --rm --network host --name gz gz-ardupilot \
      gz sim -v3 -s -r --headless-rendering iris_runway.sdf

Термінал 2, SITL на хості:

    cd ~/ardupilot/ArduCopter
    sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console

Перевірка топіків камери:

    docker exec gz bash -c "gz topic -l | grep -i -E 'image|camera'"

## Відомі проблеми
- `PreArm: Motors: Check frame class and type` і відсутність орієнтації в консолі
  SITL: причину з'ясовують.

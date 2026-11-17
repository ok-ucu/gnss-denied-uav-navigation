#!/bin/bash
# Запуск Kalibr у Docker (без ROS на хості). Виконувати на Linux x86-64 ПК, не на Jetson:
# Kalibr важкий (години CPU). На Mac з Apple Silicon образ x86 працює лише під емуляцією
# (повільно, інколи падає) - краще звичайний Linux-ПК.
#
# Одноразово зібрати образ:
#   git clone https://github.com/ethz-asl/kalibr && cd kalibr
#   docker build -t kalibr -f Dockerfile_ros1_20_04 .
#
# Використання (у папці calibration/, де лежать bag-файли й yaml):
#   ./run_kalibr.sh cam    cam.bag            # крок 1: внутрішні параметри
#   ./run_kalibr.sh imucam imucam.bag cam-camchain.yaml   # крок 2: камера + IMU
set -e
STEP="$1"; BAG="$2"; CHAIN="$3"
[ -z "$STEP" ] || [ -z "$BAG" ] && { grep '^#' "$0" | sed 's/^# \?//'; exit 1; }
KAL='source /catkin_ws/devel/setup.bash && cd /data && '
RUN="docker run --rm -v $(pwd):/data kalibr bash -c"
case "$STEP" in
  cam)
    $RUN "$KAL rosrun kalibr kalibr_calibrate_cameras --bag $BAG --topics /cam0/image_raw \
      --models pinhole-radtan --target aprilgrid.yaml --dont-show-report" ;;
  imucam)
    [ -z "$CHAIN" ] && { echo "потрібен camchain.yaml з кроку 1"; exit 1; }
    $RUN "$KAL rosrun kalibr kalibr_calibrate_imu_camera --bag $BAG --cam $CHAIN \
      --imu imu.yaml --target aprilgrid.yaml --dont-show-report --timeoffset-padding 0.1" ;;
  *) echo "cam | imucam"; exit 1 ;;
esac
echo "Готово. Результати поруч із bag: *camchain*.yaml, *results*.txt, *report*.pdf"

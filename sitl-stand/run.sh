#!/bin/bash
# Використання: ./run.sh [extnav|gps]
#   extnav (за замовчуванням) - без GPS, позиція із зовнішнього джерела
#   gps                       - звичайний режим з GPS (для місій у QGC)
MODE="${1:-extnav}"

pkill -f arducopter 2>/dev/null
pkill -f mavproxy 2>/dev/null

case "$MODE" in
  extnav)
    docker run -it --rm --network host ardupilot-sitl
    ;;
  gps)
    docker run -it --rm --network host ardupilot-sitl \
      sim_vehicle.py -v ArduCopter --no-rebuild --out=udp:127.0.0.1:14550
    ;;
  *)
    echo "Використання: ./run.sh [extnav|gps]"
    exit 1
    ;;
esac

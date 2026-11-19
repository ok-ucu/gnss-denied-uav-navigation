#!/usr/bin/env bash
# DISARMED loss-of-VIO demo on ArduCopter 4.6.3 SITL. See RUN.md.
# Usage: ./run_demo.sh build | A | B | B-nohold | dry-run
set -euo pipefail
cd "$(dirname "$0")"
IMAGE=ardupilot-sitl-4.6.3
NAME=gnss-sitl-demo
PY=${PYTHON:-python3}
HOLD=(--gate-closed-hold-max-sim-s 0.5 --gate-closed-hold-max-host-s 1.2
      --gate-closed-hold-budget-evidence ../integration/gate_hold_budget_simulation.json)

start_sitl() {
  docker rm -f "$NAME" >/dev/null 2>&1 || true
  mkdir -p "$OUT/logs" && chmod 777 "$OUT/logs"
  docker run -d --rm --name "$NAME" --network host \
    -v "$OUT/logs:/home/dev/ardupilot/ArduCopter/logs" "$IMAGE" \
    bash -c 'cd /home/dev/ardupilot/ArduCopter && exec ../build/sitl/bin/arducopter --model + --speedup 1 \
      --home -35.363261,149.165230,584,353 -I0 \
      --defaults ../Tools/autotest/default_params/copter.parm,/home/dev/ext_nav.parm' >/dev/null
  trap 'docker logs "$NAME" >"$OUT/sitl_console.txt" 2>&1 || true; docker stop "$NAME" >/dev/null 2>&1 || true' EXIT
}

case "${1:-}" in
  build)
    docker build -t "$IMAGE" . ;;
  A|B|B-nohold)
    OUT="$HOME/sitl-demo/$(date +%Y%m%d-%H%M%S)-$1"
    echo "Results will be in: $OUT"
    start_sitl
    case "$1" in
      A)        "$PY" loss_of_vio_demo.py --mode A --out "$OUT" ;;
      B)        "$PY" loss_of_vio_demo.py --mode B --out "$OUT" "${HOLD[@]}" ;;
      B-nohold) "$PY" loss_of_vio_demo.py --mode B --no-hold --out "$OUT" ;;
    esac ;;
  dry-run)
    OUT="$HOME/sitl-demo/$(date +%Y%m%d-%H%M%S)-dry-run"
    "$PY" loss_of_vio_demo.py --mode B --dry-run --out "$OUT/hold" "${HOLD[@]}"
    "$PY" loss_of_vio_demo.py --mode B --dry-run --no-hold --out "$OUT/nohold" ;;
  *)
    echo "Usage: $0 build | A | B | B-nohold | dry-run" >&2; exit 2 ;;
esac

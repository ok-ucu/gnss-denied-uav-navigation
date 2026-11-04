#!/usr/bin/env bash
set -euo pipefail

# The pinned image contains the compiled binary and MAVProxy. Never build here.
: "${SITL_HOST:=host.docker.internal}"
: "${COVARIANCE_PORT:=14556}"
: "${MONITOR_PORT:=14552}"
if [[ "$COVARIANCE_PORT" == "$MONITOR_PORT" ]]; then
    echo 'Covariance and monitor must use different UDP ports' >&2
    exit 2
fi
for port in "$COVARIANCE_PORT" "$MONITOR_PORT"; do
    if [[ ! "$port" =~ ^[0-9]+$ ]] || (( port < 1024 || port > 65535 )); then
        echo "Invalid UDP port: $port" >&2
        exit 2
    fi
done

# Image layout is checked before execution; allow explicit override for other images.
ARDUPILOT_DIR="${ARDUPILOT_DIR:-/home/docker/ardupilot}"
if [[ ! -x "$ARDUPILOT_DIR/build/sitl/bin/arducopter" ]]; then
    echo "Missing prebuilt arducopter at $ARDUPILOT_DIR; no automatic rebuild" >&2
    exit 1
fi
command -v mavproxy.py >/dev/null
run_dir="/results/$(date -u +%Y%m%dT%H%M%SZ)-$$"
mkdir "$run_dir"
cd "$run_dir"
printf '%s\n' "$ARDUPILOT_DIR" > image-source-path.txt
git -C "$ARDUPILOT_DIR" rev-parse HEAD > firmware-commit.txt
uname -m > architecture.txt

sitl_pid=''
proxy_pid=''
cleanup() {
    trap - EXIT INT TERM
    [[ -z "$proxy_pid" ]] || kill -TERM "$proxy_pid" 2>/dev/null || true
    [[ -z "$sitl_pid" ]] || kill -TERM "$sitl_pid" 2>/dev/null || true
    [[ -z "$proxy_pid" ]] || wait "$proxy_pid" 2>/dev/null || true
    [[ -z "$sitl_pid" ]] || wait "$sitl_pid" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

"$ARDUPILOT_DIR/build/sitl/bin/arducopter" \
    -S --model + --speedup 1 --home=-35.363261,149.165230,584,0 \
    --defaults "$ARDUPILOT_DIR/Tools/autotest/default_params/copter.parm,/integration/sitl_ext_nav.parm,/integration/sitl_transport.parm" &
sitl_pid=$!
mavproxy.py --master=tcp:127.0.0.1:5760 --mav20 --non-interactive \
    --source-system=254 --streamrate=10 \
    --out="udp:$SITL_HOST:$COVARIANCE_PORT" \
    --out="udp:$SITL_HOST:$MONITOR_PORT" \
    --logfile="$run_dir/mav.tlog" &
proxy_pid=$!
# If either process exits, stop the other. No arm, takeoff, or flight commands.
set +e
wait -n "$sitl_pid" "$proxy_pid"
status=$?
exit "$status"

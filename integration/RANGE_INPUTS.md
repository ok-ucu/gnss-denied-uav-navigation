# Opt-in SITL range and MAVLink input probe

This checks MAVLink inputs, not camera/SITL physical consistency. Range comes
from ArduCopter's internal simulation, not Gazebo geometry. No arm/takeoff is sent.
The existing covariance profile remains unchanged by default (`SITL_RANGE=0`).

## Start and stop

From the repository root, using the existing pinned ARM64 image (no build):

```bash
RUN_DIR="$PWD/integration/input_probe_runs/$(date -u +%Y%m%dT%H%M%SZ)_manual"
mkdir -p "$RUN_DIR"
CONTAINER="gnss-range-probe-$(date -u +%Y%m%dT%H%M%SZ)"

# A subshell trap also stops the container when the strict probe returns FAIL.
(
trap 'docker stop --timeout 15 "$CONTAINER" >/dev/null 2>&1 || true' EXIT
docker compose -p gnss-range-probe -f integration/sitl.compose.yaml run -d \
  --name "$CONTAINER" \
  -e SITL_RANGE=1 -e SITL_STREAMRATE=-1 -e COVARIANCE_PORT=14553 -e MONITOR_PORT=14554 \
  -v "$RUN_DIR:/results" sitl

probe_status=0
.venv/bin/python -B integration/probe_vio_inputs.py \
  --conn udpin:0.0.0.0:14553 --duration 12 --heartbeat-timeout 20 \
  --request-streams --output "$RUN_DIR/probe.json" || probe_status=$?

# Always stop, including when probe exits 1:
docker stop --timeout 15 "$CONTAINER"
exit "$probe_status"
)
```

Use a fresh results directory/container name for each repeat. Do not run another
listener on 14553. Port 14554 is available for a separate monitor/readback client.
Docker sends to `host.docker.internal`; no host networking is assumed.
Default existing outputs 14556/14552 remain unchanged when overrides are absent.

`--request-streams` explicitly requests 20 Hz range and 50 Hz IMU/attitude/local
position using MAV_CMD_SET_MESSAGE_INTERVAL (511), leaving margin above thresholds.
Without this flag the probe only observes. Requests are not proof of delivered rates.
Other clients/MAVProxy stream settings can override rates on the shared FCU channel.
Use `SITL_STREAMRATE=-1` for explicit per-message requests: this disables MAVProxy's
periodic stream-rate requests. Its default stays 10 for existing experiments.
The probe serializes interval requests and records each command ACK result (0 means
accepted; a rejected/missing ACK fails the requested-stream check). ACK does not
prove a message was generated. Command 511 ACK has no message ID; serialization
and target filtering reduce ambiguity. Avoid concurrent interval setters.

Probe exit 0 means PASS; exit 1 means FAIL. It requires an autopilot heartbeat,
MAVLink 2 framing on that heartbeat and all counted input messages, range orientation
25, min/max 20/5000 cm, current within those bounds, range >=10 Hz and the other three
streams >=30 Hz. Only messages from the heartbeat's system/component count.
Rates use the whole observation duration. Maximum arrival gaps include both window
boundaries. Arrival gaps are not sensor acquisition latency. JSON records range
values, stream counts/rates/gaps, source, armed observation and failure reasons.
The heartbeat has a separate timeout; socket errors fail and close the connection.

## Parameters checked against 4.4.4

The pinned image's firmware commit is
`9279df6191a772c098b2e4797c95ac954f258d88`. The overlay contains only simulated
analog-sonar configuration, no frame/precision-landing parameters:

- `libraries/SITL/SITL.cpp:81,97`: SONAR_ROT and SONAR_SCALE.
- `Tools/autotest/default_params/gazebo-iris.parm:9-13`: existing analog sonar example.
- `libraries/SITL/SIM_Aircraft.cpp:486-558`: downward surface distance, tilt and noise.
- `libraries/GCS_MAVLink/GCS_Common.cpp:387-415`: cm fields and orientation in id 132.

The live readback independently confirmed all nine overlay values. The range
backend is analog (type 1), linear (function 0), pin 0; 25 means pitch 270/downward.
The 5000 cm advertised maximum is not proof of useful range throughout that interval.

A grounded disarmed drone may report 0 cm, outside the required minimum. EKF without
ExternalNav initialization may not publish LOCAL_POSITION_NED. Both are legitimate
probe FAILs; this tool does not fake range, initialize navigation or fly to obtain PASS.

## Verification and actual result

```bash
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B -m unittest discover -s integration/tests -v
bash -n integration/start_sitl.sh
.venv/bin/python -B -c 'import ast,pathlib; [ast.parse(p.read_text()) for p in pathlib.Path("integration").rglob("*.py")]'
git diff --check
```

Offline tests use explicitly fake transports and do not establish real stream delivery.
Actual disarmed live result: **FAIL**, with partial stream checks passing. See
[the portable range/input report](reports/sitl_range_inputs.md).
Gazebo/camera correspondence, VIO fusion, flight and capture latency are **NOT TESTED**.

Follow-up [disarmed diagnosis](reports/sitl_range_inputs.md)
confirmed accepted interval requests, EKF constant-position gating even after origin
was set, and `OutOfRangeLow` for grounded range. Disabling MAVProxy streamrate fixed
the interval override, not the absent local-position solution or zero range.

## Dependencies and related tools

Run commands from the repository root. Docker Desktop/Linux ARM64 and the pinned
SITL image are required for this profile; no Gazebo or ROS dependency is needed
for the MAVLink probe. Host Python needs pymavlink; a project-local environment:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install pymavlink
```

The probe never starts SITL itself and never sends flight commands. Its default UDP
port is 14553. Do not bind another client to that port. Without --request-streams
it observes only. Reusing the container name requires the previous container to be
absent or choosing a fresh name; no command here deletes an existing container.
Raw results remain local under input_probe_runs/; only the self-contained
[report](reports/sitl_range_inputs.md) is versioned.

See [integration tools and camera commands](README.md),
[static camera result](reports/arm64_camera.md),
[camera motion result](reports/gazebo_motion.md) and
[blocked motion bridge](reports/motion_bridge.md).

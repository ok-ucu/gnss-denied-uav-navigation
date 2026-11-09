#!/usr/bin/env bash
set -euo pipefail
TASK_ROOT=/integration/isolated_blockers/20261109_a
SRC=$TASK_ROOT/work/ardupilot
test ! -e "$SRC"
cp -a /home/docker/ardupilot "$SRC"
chown -R root:root "$SRC"
cd "$SRC"
test "$(git rev-parse HEAD)" = 9279df6191a772c098b2e4797c95ac954f258d88
git status --short > "$TASK_ROOT/evidence/source_initial_status.txt"
git submodule status > "$TASK_ROOT/evidence/source_submodules.txt"
cp libraries/SITL/SIM_Aircraft.cpp "$TASK_ROOT/evidence/SIM_Aircraft_original.cpp"
cp libraries/SITL/SIM_JSON.cpp "$TASK_ROOT/evidence/SIM_JSON_original.cpp"
cp libraries/SITL/SIM_JSON.h "$TASK_ROOT/evidence/SIM_JSON_original.h"
python3 "$TASK_ROOT/code/prepare_source.py" "$SRC" instrument
git diff > "$TASK_ROOT/evidence/instrumentation.patch"
./waf configure --board sitl --out build_experiment > "$TASK_ROOT/evidence/configure.log" 2>&1
./waf copter -j1 > "$TASK_ROOT/evidence/build_baseline.log" 2>&1
cp build_experiment/sitl/bin/arducopter "$TASK_ROOT/evidence/arducopter_baseline"
python3 "$TASK_ROOT/code/prepare_source.py" "$SRC" fix
cp "$TASK_ROOT/code/test_sim_json.cpp" libraries/SITL/tests/test_sim_json.cpp
./waf copter -j1 > "$TASK_ROOT/evidence/build_patched.log" 2>&1
cp build_experiment/sitl/bin/arducopter "$TASK_ROOT/evidence/arducopter_patched"
./waf --targets tests/test_sim_json -j1 > "$TASK_ROOT/evidence/build_parser_test.log" 2>&1
build_experiment/sitl/tests/test_sim_json > "$TASK_ROOT/evidence/parser_tests.log" 2>&1
git diff > "$TASK_ROOT/evidence/experimental_source.patch"
cp libraries/SITL/SIM_JSON.cpp "$TASK_ROOT/evidence/SIM_JSON_patched.cpp"
sha256sum "$TASK_ROOT/evidence/arducopter_"* libraries/SITL/SIM_JSON.cpp > "$TASK_ROOT/evidence/build_sha256.txt"
echo BUILD_AND_ACTUAL_PARSER_TESTS_COMPLETE

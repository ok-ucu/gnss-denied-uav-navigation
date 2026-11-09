#!/usr/bin/env bash
set -euo pipefail
set -C
TASK_ROOT=/integration/isolated_blockers/20261109_a
cd "$TASK_ROOT/work/ardupilot"
# First -j1 attempt was interrupted deliberately, not erased, for -j2 scheduling.
./waf copter -j2 > "$TASK_ROOT/evidence/build_baseline_attempt2.log" 2>&1
cp --no-clobber build_experiment/sitl/bin/arducopter "$TASK_ROOT/evidence/arducopter_baseline"
python3 "$TASK_ROOT/code/prepare_source.py" "$PWD" fix
cp "$TASK_ROOT/code/test_sim_json.cpp" libraries/SITL/tests/test_sim_json.cpp
./waf copter -j2 > "$TASK_ROOT/evidence/build_patched.log" 2>&1
cp --no-clobber build_experiment/sitl/bin/arducopter "$TASK_ROOT/evidence/arducopter_patched"
./waf --targets tests/test_sim_json -j2 > "$TASK_ROOT/evidence/build_parser_test.log" 2>&1
build_experiment/sitl/tests/test_sim_json > "$TASK_ROOT/evidence/parser_tests.log" 2>&1
git diff > "$TASK_ROOT/evidence/experimental_source.patch"
cp libraries/SITL/SIM_JSON.cpp "$TASK_ROOT/evidence/SIM_JSON_patched.cpp"
sha256sum "$TASK_ROOT/evidence/arducopter_"* libraries/SITL/SIM_JSON.cpp > "$TASK_ROOT/evidence/build_sha256.txt"
echo BUILD_AND_ACTUAL_PARSER_TESTS_COMPLETE

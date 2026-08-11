#!/usr/bin/env bash
# =============================================================================
# autotest.sh : run the whole walking test by itself and print a verdict.
#
# WHAT THIS REPLACES
# Four terminals, a person watching a Gazebo window, and a description sent
# back over chat. That loop cost days, because "it wiggles", "it flies away"
# and "it moves randomly" are the same sentence for several unrelated faults,
# and each round trip needed a rebuild, a relaunch, and a human's attention.
#
# This runs headless (no GUI, so it is fast), drives the robot itself,
# measures /odom, and prints a table with a one-line verdict. Nothing to
# watch, nothing to time, nothing to transcribe.
#
# WHY IT TESTS BOTH CONTROL MODES
# The open question is not "does this configuration work" but "which of the
# two approaches works". Testing them one at a time, a day apart, with other
# things changing in between, is how we ended up unable to attribute any
# result to any cause. Running both back to back against an identical robot
# makes the comparison mean something.
#
# USAGE
#   bash ~/hexapod_ws/tools/autotest.sh              # both modes
#   bash ~/hexapod_ws/tools/autotest.sh effort       # one mode
#   bash ~/hexapod_ws/tools/autotest.sh position
#
# Takes about 90 seconds per mode. Safe to run repeatedly.
# =============================================================================
set -o pipefail

WS="${HOME}/hexapod_ws"
TOOLS="${WS}/tools"

if [ ! -f "${WS}/install/setup.bash" ]; then
  echo "ERROR: ${WS}/install/setup.bash not found. Run: hexbuild" >&2
  exit 1
fi

# shellcheck disable=SC1091
source "${WS}/install/setup.bash"

# DEFAULT IS EFFORT ONLY.
#
# It used to be "position effort". Position mode has since been shown, by
# measurement rather than argument, to be incapable of walking a free-base
# robot in Gazebo:
#
#   coxa swing  9.2 deg   the legs reach correctly
#   femur swing 16.6 deg  the legs lift correctly
#   body travel 2.9 mm over 20 s
#   tilt        0.0 deg, height steady to 0.3 mm
#
# The gait is executed perfectly and the body does not move. Gazebo realises
# position commands with SetPosition(), which relocates a joint without
# giving it a matching velocity. Contact friction is computed from sliding
# VELOCITY, so the solver sees a stationary foot and generates no propulsive
# force. The feet pass through the floor rather than pushing off it. No
# parameter changes this.
#
# That result is deterministic: it reproduces digit for digit on every run.
# Re-running it costs 90 seconds and teaches nothing, so it is no longer the
# default. Still available explicitly:  autotest.sh position
MODES="${1:-effort}"

SPEED=0.03
BASELINE=8
DRIVE=20
SETTLE=3

# Every background pid we start, so teardown is total. A surviving gzserver
# holds port 11345 and silently breaks the NEXT run, which has already cost
# this project one full debugging cycle.
PIDS=()

cleanup() {
  for p in "${PIDS[@]:-}"; do
    kill -INT "${p}" 2>/dev/null
  done
  sleep 2
  for p in "${PIDS[@]:-}"; do
    kill -9 "${p}" 2>/dev/null
  done
  pkill -9 -f gzserver  2>/dev/null
  pkill -9 -f gzclient  2>/dev/null
  pkill -9 -f gait_node 2>/dev/null
  sleep 1
  PIDS=()
}
trap cleanup EXIT INT TERM

wait_for_topic() {
  # $1 topic, $2 timeout seconds
  local topic="$1" limit="$2" i=0
  while [ "${i}" -lt "${limit}" ]; do
    if timeout 3 ros2 topic echo "${topic}" --once >/dev/null 2>&1; then
      return 0
    fi
    i=$((i + 3))
  done
  return 1
}

wait_for_controller_active() {
  # $1 controller name, $2 timeout seconds
  #
  # STRIP ANSI COLOUR BEFORE MATCHING.
  #
  # `ros2 control list_controllers` colourises the state word, so the raw
  # bytes are:
  #     joint_state_broadcaster  ...  <ESC>[92mactive<ESC>[0m
  # The obvious pattern "name.* active" then never matches, because what
  # immediately precedes "active" is an escape sequence, not a space. The
  # controllers were active within two seconds and this function waited
  # ninety, reported failure, and sent us hunting a startup bug that did not
  # exist.
  #
  # Lesson worth keeping: never pattern-match the output of a CLI tool
  # without stripping formatting first. What you see in a terminal and what
  # arrives in a pipe are not the same bytes.
  local name="$1" limit="$2" i=0
  while [ "${i}" -lt "${limit}" ]; do
    if timeout 10 ros2 control list_controllers 2>/dev/null \
         | sed 's/\x1b\[[0-9;]*[a-zA-Z]//g' \
         | grep -qE "^[[:space:]]*${name}[[:space:]].*[[:space:]]active[[:space:]]*$"; then
      return 0
    fi
    sleep 2
    i=$((i + 2))
  done
  return 1
}

run_one_mode() {
  local mode="$1"
  local controller topic

  if [ "${mode}" = "effort" ]; then
    controller="leg_trajectory_controller"
  else
    controller="leg_position_controller"
  fi
  topic="/${controller}/joint_trajectory"

  echo ""
  echo "############################################################"
  echo "#  MODE: ${mode}"
  echo "############################################################"

  cleanup
  bash "${TOOLS}/reset_sim.sh" >/dev/null 2>&1

  echo "-- launching simulation (headless) ..."
  ros2 launch hexapod_bringup hexapod_sim.launch.py \
      control_mode:="${mode}" \
      fix_base:=false \
      gui:=false \
      rviz:=false \
      > "/tmp/autotest_sim_${mode}.log" 2>&1 &
  PIDS+=($!)

  # On failure, PRINT the evidence rather than pointing at a file. A message
  # that says "see the log" costs a whole round trip when the person running
  # this is reporting results to someone else.
  dump_failure() {
    echo ""
    echo "   ---- last 40 lines of the simulation log ----"
    tail -40 "/tmp/autotest_sim_${mode}.log" 2>/dev/null | sed 's/^/   | /'
    echo "   ---- controllers as seen right now ----"
    timeout 10 ros2 control list_controllers 2>&1 | sed 's/^/   | /'
    echo "   ---- nodes ----"
    timeout 10 ros2 node list 2>&1 | sed 's/^/   | /'
    echo "   ---------------------------------------------"
  }

  if ! wait_for_controller_active "joint_state_broadcaster" 90; then
    echo "   FAILED: joint_state_broadcaster never became active."
    dump_failure
    return 20
  fi
  if ! wait_for_controller_active "${controller}" 90; then
    echo "   FAILED: ${controller} never became active."
    dump_failure
    return 21
  fi
  if ! wait_for_topic /joint_states 30; then
    echo "   FAILED: no /joint_states."
    return 22
  fi
  echo "   controllers active, joint states flowing"

  echo "-- launching gait node ..."
  ros2 launch hexapod_bringup walk.launch.py \
      command_type:=trajectory \
      command_topic:="${topic}" \
      cycle_time:=3.0 \
      step_height:=0.025 \
      max_stride:=0.05 \
      max_linear_speed:=0.04 \
      max_angular_speed:=0.25 \
      startup_ramp:=4.0 \
      > "/tmp/autotest_gait_${mode}.log" 2>&1 &
  PIDS+=($!)

  # The gait node blocks until it has a start pose, then ramps. Give the ramp
  # time to finish before measuring, or the baseline captures the ramp motion
  # and reports it as drift.
  echo "-- waiting for startup ramp ..."
  sleep 12

  if ! grep -q "Ramp complete" "/tmp/autotest_gait_${mode}.log" 2>/dev/null; then
    echo "   WARNING: ramp did not report completion. Continuing anyway."
    tail -5 "/tmp/autotest_gait_${mode}.log" | sed 's/^/     /'
  fi

  echo "-- measuring ..."
  python3 "${TOOLS}/measure_walk.py" \
      --speed "${SPEED}" \
      --baseline "${BASELINE}" \
      --drive "${DRIVE}" \
      --settle "${SETTLE}"
  local rc=$?

  # Codes 17 (robot absent) and 10 (ejected) mean the failure is in the
  # SIMULATION, not the gait, so the simulator's own log is the evidence.
  # Printing it here saves a round trip; asking someone to go and fetch a
  # file costs an entire exchange.
  if [ "${rc}" = "17" ] || [ "${rc}" = "10" ]; then
    echo ""
    echo "   ---- simulation log, last 50 lines ----"
    tail -50 "/tmp/autotest_sim_${mode}.log" 2>/dev/null | sed 's/^/   | /'
    echo "   ---- gait node log, last 25 lines ----"
    tail -25 "/tmp/autotest_gait_${mode}.log" 2>/dev/null | sed 's/^/   | /'
    echo "   ---- who publishes /odom, and what does it say ----"
    timeout 10 ros2 topic info /odom --verbose 2>&1 \
      | grep -E "Publisher count|Node name|Reliability" | sed 's/^/   | /'
    timeout 10 ros2 topic echo /odom --once --field pose.pose 2>&1 \
      | head -20 | sed 's/^/   | /'
    echo "   ---- p3d plugin loaded? ----"
    grep -iE "p3d|ground_truth" "/tmp/autotest_sim_${mode}.log" 2>/dev/null \
      | head -5 | sed 's/^/   | /'
    echo "   ---- models actually present in gazebo ----"
    timeout 10 ros2 service call /get_model_list gazebo_msgs/srv/GetModelList \
      2>&1 | tail -5 | sed 's/^/   | /'
    echo "   ---------------------------------------"
  fi

  cleanup
  return ${rc}
}

echo "============================================================"
echo " AUTOMATED WALK TEST"
echo " speed ${SPEED} m/s   baseline ${BASELINE}s   drive ${DRIVE}s"
echo "============================================================"

declare -A RESULTS
for m in ${MODES}; do
  run_one_mode "${m}"
  RESULTS[${m}]=$?
done

echo ""
echo "============================================================"
echo " SUMMARY"
echo "============================================================"
for m in ${MODES}; do
  rc="${RESULTS[${m}]}"
  case "${rc}" in
    0)  msg="WALKING -- this mode works" ;;
    10) msg="ejected / tipped over" ;;
    11) msg="drifts while standing still" ;;
    12) msg="walks but feet slip" ;;
    13) msg="marches in place, body does not travel" ;;
    14) msg="wanders, motion not coordinated" ;;
    15) msg="legs not moving at all -- command never reached the gait" ;;
    16) msg="steps on the spot -- no stride commanded" ;;
    17) msg="ROBOT NOT IN THE WORLD -- model missing or destroyed" ;;
    2)  msg="no odometry -- sim did not come up" ;;
    3)  msg="simulation time frozen" ;;
    2[0-2]) msg="startup failed, see /tmp/autotest_sim_${m}.log" ;;
    *)  msg="unknown result (code ${rc})" ;;
  esac
  printf '  %-10s %s\n' "${m}" "${msg}"
done
echo "============================================================"
echo ""
echo "Logs: /tmp/autotest_sim_*.log  /tmp/autotest_gait_*.log"

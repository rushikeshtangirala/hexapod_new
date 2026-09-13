#!/usr/bin/env bash
# =============================================================================
# check_vision.sh : verify checkpoint 8 in ONE command, from ONE terminal.
#
# USAGE
#   bash ~/hexapod_ws/tools/check_vision.sh
#
# Gazebo must already be running with the camera enabled:
#   ros2 launch hexapod_bringup hexapod_sim.launch.py enable_camera:=true
#
# WHY THIS SCRIPT EXISTS
# Checking the detector by hand needs three terminals held in the right
# states at the same time: one running Gazebo, one running the node, one
# reading topics. `ros2 run` BLOCKS the terminal it is in, and while it
# blocks the shell is not reading input, so anything typed there is buffered
# and then executed as one mangled line the moment the node exits. That has
# now eaten three debugging rounds and told us nothing about the robot.
#
# So: this starts the node, waits for it, reads the topics, prints a verdict
# and cleans up after itself. Nothing is left running and there is nothing
# to type while it works.
# =============================================================================
set -o pipefail

WS="${HOME}/hexapod_ws"
LOG=/tmp/stem_detector.log

# Ground truth from hexapod.world. stem_a is the nearest target and therefore
# the one the detector should pick: apparent size falls with range, and the
# node selects the largest qualifying blob.
TRUE_X=1.00
TRUE_Y=0.00
TOL=0.15

pass() { echo "  PASS  $1"; }
fail() { echo "  FAIL  $1"; FAILED=1; }
FAILED=0

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${WS}/install/setup.bash"

echo "=============================================================="
echo " CHECKPOINT 8 : STEM DETECTION"
echo "=============================================================="

# ---------------------------------------------------------------------------
echo ""
echo "1. clearing any detector left over from a previous run"
# ---------------------------------------------------------------------------
# Several copies of a node with the same name is not harmless. Parameter
# calls cannot resolve an ambiguous name, which produces a confusing
# "Node not found" while the node is plainly running.
pkill -f stem_detector 2>/dev/null
sleep 1
echo "  clear"

# ---------------------------------------------------------------------------
echo ""
echo "2. camera feed"
# ---------------------------------------------------------------------------
# Checked BEFORE starting the detector, deliberately. If this fails there is
# no point looking at anything downstream, and the fault is in the launch,
# not in the vision code.
if timeout 8 ros2 topic echo /camera/image_raw --once >/dev/null 2>&1; then
  pass "/camera/image_raw is publishing"
else
  fail "/camera/image_raw is silent"
  echo ""
  echo "  Gazebo is not running, or it was launched without the camera."
  echo "  In another terminal:"
  echo "    bash ~/hexapod_ws/tools/reset_sim.sh"
  echo "    ros2 launch hexapod_bringup hexapod_sim.launch.py enable_camera:=true"
  exit 1
fi

# ---------------------------------------------------------------------------
echo ""
echo "3. starting the detector"
# ---------------------------------------------------------------------------
ros2 run hexapod_vision stem_detector > "${LOG}" 2>&1 &
PID=$!
sleep 4

if ! kill -0 "${PID}" 2>/dev/null; then
  fail "the detector exited immediately"
  echo ""
  echo "  its output was:"
  sed 's/^/    /' "${LOG}"
  exit 1
fi
pass "running as pid ${PID}"

if grep -q "intrinsics:" "${LOG}"; then
  pass "camera_info received"
  grep "intrinsics:" "${LOG}" | tail -1 | sed 's/^/        /'
else
  fail "no camera_info; the detector cannot compute range without it"
fi

# ---------------------------------------------------------------------------
echo ""
echo "4. detection"
# ---------------------------------------------------------------------------
DET=$(timeout 8 ros2 topic echo /stem/detected --once 2>/dev/null \
      | grep -oE 'true|false' | head -1)

if [ "${DET}" = "true" ]; then
  pass "a stem is detected"
elif [ "${DET}" = "false" ]; then
  fail "no stem detected; the colour window is the thing to move"
  echo "        with this script finished, and Gazebo still up, try:"
  echo "        ros2 run hexapod_vision stem_detector &"
  echo "        ros2 param set /stem_detector h_min 35"
  echo "        ros2 param set /stem_detector s_min 60"
else
  fail "/stem/detected published nothing"
fi

# ---------------------------------------------------------------------------
echo ""
echo "5. position against ground truth"
# ---------------------------------------------------------------------------
# This is the real test. One pair of numbers validates the intrinsics, the
# optical to body rotation, the mount pitch and the ground plane solve all at
# once, because a mistake in any of them moves the answer.
POS=$(timeout 8 ros2 topic echo /stem/position --once 2>/dev/null)
# Plain space class, not \s. \s is a GNU grep extension and this script has
# to survive being run under a different grep without silently matching
# nothing, which would look exactly like "the topic published nothing".
X=$(echo "${POS}" | grep -E '^ +x:' | head -1 | awk '{print $2}')
Y=$(echo "${POS}" | grep -E '^ +y:' | head -1 | awk '{print $2}')

if [ -z "${X}" ]; then
  fail "/stem/position published nothing"
  echo "        The stem is seen but its BASE is out of frame, so no ground"
  echo "        solution exists. Bearing is still published. Back the robot"
  echo "        away from the target and re-run."
else
  echo "        measured   x = ${X}   y = ${Y}"
  echo "        truth      x = ${TRUE_X}   y = ${TRUE_Y}   tolerance ${TOL} m"
  OK=$(awk -v x="${X}" -v y="${Y}" -v tx="${TRUE_X}" -v ty="${TRUE_Y}" \
           -v t="${TOL}" \
           'BEGIN{dx=x-tx; dy=y-ty; if(dx<0)dx=-dx; if(dy<0)dy=-dy;
                  print (dx<t && dy<t) ? "yes" : "no"}')
  if [ "${OK}" = "yes" ]; then
    pass "within tolerance of the true stem position"
  else
    fail "outside tolerance"
    echo "        x wrong by a constant FACTOR  -> camera_height parameter"
    echo "        y sign reversed               -> bearing negation"
    echo "        both wildly wrong             -> camera_pitch parameter"
  fi
fi

# ---------------------------------------------------------------------------
echo ""
echo "6. stopping the detector"
# ---------------------------------------------------------------------------
kill "${PID}" 2>/dev/null
wait "${PID}" 2>/dev/null
echo "  stopped, nothing left running"

echo ""
echo "=============================================================="
if [ "${FAILED}" -eq 0 ]; then
  echo " CHECKPOINT 8 PASSED"
  echo ""
  echo " Next: checkpoint 9 needs the robot to translate, so Gate 4"
  echo " (free base walking) is the only thing still blocking."
else
  echo " CHECKPOINT 8 FAILED. Full detector log: ${LOG}"
fi
echo "=============================================================="

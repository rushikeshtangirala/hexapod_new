#!/usr/bin/env bash
# =============================================================================
# diagnose_runtime.sh : capture the state of a RUNNING simulation.
#
# diagnose.sh checks the build. This checks the live system, which is where
# "the joints move randomly" has to be diagnosed, because that symptom has
# several unrelated causes that look identical:
#
#   a) two robot_state_publishers fighting over /tf
#   b) clock disagreement between the gait node and Gazebo
#   c) the robot physically falling over in Gazebo
#   d) commanded joint angles not being tracked by the controller
#   e) nothing wrong at all: RViz cannot show body translation because its
#      fixed frame IS the body, so a correct walk looks like flailing legs
#
# Guessing between these wastes more time than measuring. Run this with the
# simulation and the gait node both up, and paste the whole output.
#
# USAGE
#   bash tools/diagnose_runtime.sh
# =============================================================================
set -uo pipefail

echo "============================================================"
echo " RUNTIME DIAGNOSTICS"
echo " $(date)"
echo "============================================================"

echo ""
echo "--- 1. Nodes (expect EXACTLY ONE robot_state_publisher) ---"
ros2 node list 2>&1 | sort

echo ""
echo "--- 2. Duplicate robot_state_publisher check ---"
N_RSP=$(ros2 node list 2>/dev/null | grep -c robot_state_publisher || true)
echo "robot_state_publisher count: ${N_RSP}"
if [ "${N_RSP}" -gt 1 ]; then
  echo ">> PROBLEM: more than one. They fight over /tf and /robot_description."
  echo ">> Kill any leftover display.launch.py."
fi

echo ""
echo "--- 3. Controllers ---"
timeout 15 ros2 control list_controllers 2>&1 || echo ">> controller_manager did not answer"

echo ""
echo "--- 4. use_sim_time on every node (ALL must be true) ---"
for n in $(ros2 node list 2>/dev/null); do
  v=$(timeout 5 ros2 param get "${n}" use_sim_time 2>/dev/null | tail -1)
  printf '  %-45s %s\n' "${n}" "${v:-<no param>}"
done

echo ""
echo "--- 5. Clock ---"
echo "/clock rate (should be non-zero if Gazebo is running):"
timeout 5 ros2 topic hz /clock 2>&1 | head -3

echo ""
echo "--- 6. Command and feedback rates ---"
echo "commands:"
timeout 5 ros2 topic hz /leg_position_controller/commands 2>&1 | head -3
echo "joint_states:"
timeout 5 ros2 topic hz /joint_states 2>&1 | head -3

echo ""
echo "--- 7. Commanded joint vector (one sample) ---"
timeout 5 ros2 topic echo /leg_position_controller/commands --once 2>&1 | head -25

echo ""
echo "--- 8. Actual joint positions (one sample) ---"
timeout 5 ros2 topic echo /joint_states --once 2>&1 | head -30

echo ""
echo "--- 9. BODY POSE: is the robot upright, and is it moving? ---"
echo "Two samples of ground-truth odometry, ~3 s apart."
echo "  position.z should be roughly 0.05 to 0.12 and STEADY if standing."
echo "  orientation should stay near identity (x,y,z small, w near 1)."
echo "  position.x/y should CHANGE between samples if walking forward."
echo ""
echo "sample 1:"
timeout 5 ros2 topic echo /odom --once 2>&1 | head -22
echo ""
sleep 3
echo "sample 2:"
timeout 5 ros2 topic echo /odom --once 2>&1 | head -22

echo ""
echo "============================================================"
echo " Paste this entire output."
echo "============================================================"

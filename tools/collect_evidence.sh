#!/usr/bin/env bash
# =============================================================================
# collect_evidence.sh : one command, one file, everything needed to diagnose
#                       "it still does not work" without another round trip.
#
# WHY THIS EXISTS
# "It is not working" describes at least eight different faults in this stack,
# and they need opposite fixes. Asking one question per round trip costs a day.
# This captures every discriminating signal in about 40 seconds.
#
# It answers, in order:
#   1. Is the model the one you think it is?      (masses tell you the profile)
#   2. Did the controllers actually activate?
#   3. Is anything subscribed to the command topic?
#   4. Are commands being SENT?
#   5. Are they being TRACKED?                    (commanded vs measured)
#   6. Does the BODY move?                        (ground truth odometry)
#   7. If not, is the body sliding or are the joints wandering?
#
# USAGE
#   Run with the simulation AND the gait node up, then drive it:
#     bash tools/collect_evidence.sh
#   It sends its own velocity command, so you do not need teleop running.
#   Paste the whole of the file it writes.
# =============================================================================
set -uo pipefail

OUT="/tmp/hexapod_evidence_$(date +%H%M%S).txt"
DRIVE_SPEED=0.03
DRIVE_TIME=12

exec > >(tee "${OUT}") 2>&1

echo "============================================================"
echo " HEXAPOD EVIDENCE   $(date)"
echo "============================================================"

echo ""
echo "=== 1. WHICH MODEL IS LOADED ==============================="
# Link masses identify the sim_profile with no ambiguity: 1e-5 is repo mode,
# 1.31 / 0.201 / 0.101 / 0.035 is the measured model. If this disagrees with
# the flag you passed, the build did not pick up your change and nothing else
# in this file matters.
timeout 10 ros2 param get /robot_state_publisher robot_description 2>/dev/null \
  | grep -o 'mass value="[^"]*"' | sort | uniq -c | head
echo "(1e-05 everywhere = sim_profile:=repo.  1.31/0.201/0.101/0.035 = physical)"

echo ""
echo "=== 2. NODES ==============================================="
timeout 10 ros2 node list 2>/dev/null

echo ""
echo "=== 3. CONTROLLERS (every one must say 'active') ==========="
timeout 15 ros2 control list_controllers 2>/dev/null \
  || echo "controller_manager did not answer. Nothing below will work."

echo ""
echo "=== 4. HARDWARE INTERFACES ================================="
timeout 15 ros2 control list_hardware_interfaces 2>/dev/null | head -60

echo ""
echo "=== 5. IS ANYONE LISTENING TO THE GAIT NODE ================"
for t in /leg_trajectory_controller/joint_trajectory \
         /leg_position_controller/joint_trajectory \
         /leg_position_controller/commands; do
  echo "--- ${t}"
  timeout 5 ros2 topic info "${t}" 2>/dev/null || echo "    (no such topic)"
done
echo "Publisher count 1 and subscriber count 0 is THE silent failure:"
echo "the gait runs, the robot never moves, and nothing logs an error."

echo ""
echo "=== 6. COMMAND RATE ========================================"
echo "--- cmd_vel"
timeout 6 ros2 topic hz /cmd_vel 2>&1 | head -3
echo "--- joint_states"
timeout 6 ros2 topic hz /joint_states 2>&1 | head -3

echo ""
echo "=== 7. BASELINE BODY POSE (should be still) ================"
timeout 8 ros2 topic echo /odom --field pose.pose.position --once 2>/dev/null \
  || echo "NO /odom. The p3d plugin is not running, so nothing can tell"
echo "   whether the body moves. Check hexapod.gazebo.xacro is included."

echo ""
echo "=== 8. DRIVE FORWARD ${DRIVE_SPEED} m/s FOR ${DRIVE_TIME}s ============="
timeout $((DRIVE_TIME + 2)) ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: ${DRIVE_SPEED}, y: 0.0, z: 0.0}, angular: {z: 0.0}}" \
  > /dev/null 2>&1 &
PUB=$!
sleep "${DRIVE_TIME}"

echo "--- body pose AFTER driving"
timeout 8 ros2 topic echo /odom --field pose.pose.position --once 2>/dev/null
echo "Expected travel: about $(python3 -c "print(${DRIVE_SPEED}*${DRIVE_TIME})") m in x."
echo "Near zero        = legs cycling, no traction or no translation."
echo "Large and random = thrown by the solver."
echo "Negative x       = walking backwards, a sign error somewhere."

echo ""
echo "--- commanded vs measured joints, while still driving"
echo "COMMANDED (first trajectory point):"
timeout 8 ros2 topic echo /leg_trajectory_controller/joint_trajectory \
  --field points --once 2>/dev/null | head -8 \
  || timeout 8 ros2 topic echo /leg_position_controller/joint_trajectory \
     --field points --once 2>/dev/null | head -8 \
     || echo "   nothing being published on either trajectory topic"
echo "MEASURED:"
timeout 8 ros2 topic echo /joint_states --field position --once 2>/dev/null
echo "If MEASURED barely changes while COMMANDED sweeps, the controller is"
echo "not tracking: gains too low for the mass, or the wrong interface."

kill "${PUB}" 2>/dev/null
wait "${PUB}" 2>/dev/null

echo ""
echo "=== 9. AFTER STOPPING, DOES IT STAY PUT ===================="
sleep 4
timeout 8 ros2 topic echo /odom --field pose.pose.position --once 2>/dev/null
echo "Changing while stopped = drift. Run tools/check_drift.sh to split"
echo "'body sliding on its feet' from 'joints moving on their own'."

echo ""
echo "=== 10. RECENT ERRORS ======================================"
timeout 6 ros2 topic echo /rosout --once 2>/dev/null | head -20

echo ""
echo "============================================================"
echo " Written to ${OUT}"
echo " Paste the WHOLE file."
echo "============================================================"

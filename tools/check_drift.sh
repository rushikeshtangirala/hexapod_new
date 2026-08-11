#!/usr/bin/env bash
# =============================================================================
# check_drift.sh : measure whether a standing robot stays put, and if it does
#                  not, say WHICH of the two possible causes it is.
#
# THE BISECT
# A standing robot that wanders has exactly two possible causes, and they need
# opposite fixes:
#
#   A) THE BODY IS SLIDING ON ITS FEET.
#      Joint angles hold steady, body pose changes.
#      Cause: contact instability, friction, or solver error.
#      Fix:   physics (kp, kd, mu, timestep, solver iterations).
#
#   B) THE JOINTS ARE MOVING.
#      Joint angles change even though no command was sent.
#      Cause: control (commands being written, PID, interface behaviour).
#      Fix:   controller configuration.
#
# Watching Gazebo cannot distinguish these; both look like a wandering robot.
# Comparing the two numeric streams settles it in 20 seconds.
#
# USAGE
#   Run with the simulation up and NO gait node running.
#     bash tools/check_drift.sh
# =============================================================================
set -uo pipefail

DUR=20

echo "============================================================"
echo " DRIFT TEST : ${DUR} s, robot should be standing still"
echo "============================================================"

echo ""
echo "--- Joint positions at t=0 ---"
J0=$(timeout 5 ros2 topic echo /joint_states --field position --once 2>/dev/null)
echo "${J0}"

echo ""
echo "--- Body pose at t=0 ---"
P0=$(timeout 5 ros2 topic echo /odom --field pose.pose --once 2>/dev/null)
echo "${P0}"

echo ""
echo "waiting ${DUR} s ..."
sleep "${DUR}"

echo ""
echo "--- Joint positions at t=${DUR} ---"
J1=$(timeout 5 ros2 topic echo /joint_states --field position --once 2>/dev/null)
echo "${J1}"

echo ""
echo "--- Body pose at t=${DUR} ---"
P1=$(timeout 5 ros2 topic echo /odom --field pose.pose --once 2>/dev/null)
echo "${P1}"

echo ""
echo "============================================================"
echo " HOW TO READ THIS"
echo ""
echo " Joints IDENTICAL, body pose CHANGED"
echo "   -> cause A: the body is sliding on its feet."
echo "      Physics problem. Lower kp further (5e4 -> 2e4), or raise"
echo "      solver <iters> to 150, in that order."
echo ""
echo " Joints CHANGED"
echo "   -> cause B: something is commanding the joints."
echo "      Control problem. Check nothing is publishing:"
echo "        ros2 topic info /leg_position_controller/commands --verbose"
echo "      (expect zero publishers with no gait node running)"
echo ""
echo " BOTH steady"
echo "   -> fixed. Proceed to the gait."
echo ""
echo " Body orientation drifting but position steady"
echo "   -> asymmetric contact. Check all six feet touch at the same height:"
echo "        ros2 run tf2_ros tf2_echo base_link lf_foot_link"
echo "      and compare against rf, lm, rm, lr, rr."
echo "============================================================"

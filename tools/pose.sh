#!/usr/bin/env bash
# =============================================================================
# pose.sh : drive all 18 joints to a named pose, for Phase 6 verification.
#
# USAGE
#   bash tools/pose.sh zero
#   bash tools/pose.sh stand
#   bash tools/pose.sh crouch
#   bash tools/pose.sh wave        # moves lf leg only, proves per-leg control
#
# THE COMMAND ARRAY IS ORDER SENSITIVE. Index order is fixed by the `joints:`
# list in hexapod_control/config/controllers.yaml:
#
#     0  1  2   lf coxa femur tibia
#     3  4  5   lm
#     6  7  8   lr
#     9 10 11   rf
#    12 13 14   rm
#    15 16 17   rr
#
# Publishing in any other order silently drives the wrong joints, and the
# result looks exactly like a kinematics bug. It is not. Check the order first.
#
# WHY THESE ANGLES
# Frame convention from leg_macro.xacro: each segment extends along its own
# +X, femur and tibia rotate about +Y, and POSITIVE angle moves the foot
# DOWNWARD. So a standing pose needs positive femur (drop the knee) and
# negative tibia (bring the shin back out towards horizontal).
#
# Forward kinematics for the chosen stand pose, in the coxa frame:
#     L1 = 0.150  L2 = 0.1166  L3 = 0.150      (metres)
#     femur = +60 deg, tibia = -30 deg
#     foot_x = L1 + L2*cos(60) + L3*cos(30) = 0.338 m   radial reach
#     foot_z = -L2*sin(60) - L3*sin(30)     = -0.176 m  below the coxa axis
# Body underside sits at -0.0575 m, so ground clearance is about 0.118 m.
#
# *** WARNING: THIS TOOL SENDS A STEP COMMAND ***
#
# ros2 topic pub --once publishes ONE message, so all 18 joints jump to the
# new angles in a single physics step. gazebo_ros2_control realises position
# commands with SetPosition(), i.e. by TELEPORTING the joint. Teleporting a
# loaded leg drives the foot through the ground plane, and ODE removes that
# penetration with an impulse large enough to throw the robot out of the
# world. That is the "robot stands, then flies away" failure.
#
# Use this tool for:
#   - small pose changes from an already-standing robot (stand -> crouch)
#   - the 'wave' index-mapping check
#   - a robot in mid-air
#
# Do NOT use it to lift a collapsed robot into stance. The launch file now
# spawns the robot already standing, and the gait node ramps smoothly from
# wherever the joints are, so neither needs this.
# =============================================================================
set -euo pipefail

TOPIC="/leg_position_controller/commands"
MSG="std_msgs/msg/Float64MultiArray"

pose="${1:-stand}"

# per-leg triplets: coxa femur tibia
case "${pose}" in
  zero)
    C=0.0;   F=0.0;    T=0.0    ;;
  stand)
    C=0.0;   F=1.619;  T=-1.183 ;;   # +92.8 deg, -67.8 deg : foot 0.28 out, 0.18 down
  crouch)
    C=0.0;   F=1.850;  T=-1.500 ;;   # femur further past vertical : lower body
  tall)
    C=0.0;   F=1.400;  T=-0.950 ;;   # femur nearer vertical : higher body
  wave)
    # Front-left leg lifted, the rest standing. Proves the index mapping is
    # correct: exactly ONE leg should move, and it should be the front left.
    ros2 topic pub --once "${TOPIC}" "${MSG}" \
      "{data: [0.0, -0.6, -0.6,
               0.0,  1.047, -0.524,
               0.0,  1.047, -0.524,
               0.0,  1.047, -0.524,
               0.0,  1.047, -0.524,
               0.0,  1.047, -0.524]}"
    echo "Sent 'wave'. ONLY the front-left leg should have lifted."
    exit 0
    ;;
  *)
    echo "Unknown pose: ${pose}" >&2
    echo "Valid: zero | stand | crouch | tall | wave" >&2
    exit 1
    ;;
esac

ros2 topic pub --once "${TOPIC}" "${MSG}" \
  "{data: [${C}, ${F}, ${T},
           ${C}, ${F}, ${T},
           ${C}, ${F}, ${T},
           ${C}, ${F}, ${T},
           ${C}, ${F}, ${T},
           ${C}, ${F}, ${T}]}"

echo "Sent pose '${pose}'  (coxa=${C}, femur=${F}, tibia=${T} for all six legs)"

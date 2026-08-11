#!/usr/bin/env bash
# =============================================================================
# walk_free.sh : free-base walking attempt with the gentlest possible gait.
#
# THE IDEA
# The floating-base drift comes from momentum injected when Gazebo teleports
# joints to satisfy position commands. How MUCH momentum depends on how far
# and how fast the joints are teleported each cycle. That is not fixed by the
# control mode alone; it is set by the gait.
#
# This runs the same validated position-control stack, free-base, with every
# gait parameter turned down:
#
#   cycle_time  1.4  -> 3.0 s   legs move less than half as fast
#   step_height 0.045 -> 0.025  smaller vertical excursion
#   max_stride  0.10  -> 0.05   shorter steps
#   speed       0.06  -> 0.03   half the commanded body velocity
#
# It also benefits from the corrected stance. A nearly straight leg is
# effectively rigid; the bent-knee stance now in use has real vertical
# compliance, so touchdown is absorbed by the leg rather than transmitted
# into the body.
#
# This is worth trying before spending time on PID tuning, because it is a
# parameter change with no code risk, and if it works the problem is solved.
#
# USAGE, three terminals
#   1)  bash tools/walk_free.sh sim
#   2)  bash tools/walk_free.sh gait
#   3)  bash tools/walk_free.sh drive
#   4)  bash tools/walk_free.sh watch      (optional, measures progress)
#
# DIAGNOSTICS -- run these from a spare terminal with 'sim' already up
#   bash tools/walk_free.sh status    is physics stepping, are controllers up
#   bash tools/walk_free.sh unpause   manually release a stuck paused Gazebo
#   bash tools/walk_free.sh hold      publish stance only, gait clock frozen
#
# ALWAYS run 'status' before starting terminal 2. A paused Gazebo and a
# correctly standing robot look IDENTICAL in the GUI, and starting the gait
# node against a frozen sim wastes a full debugging cycle.
# =============================================================================
# NOTE: deliberately NOT using `set -u`.
#
# The previous version had `set -uo pipefail` and then sourced ROS's
# setup.bash with stderr sent to /dev/null. ROS's setup scripts reference
# variables that may be unset; under `set -u` that is a FATAL error, so the
# shell exited on the source line, and the redirect discarded the only clue.
# The symptom was a script that ran and exited instantly with no output at
# all, which is close to the least debuggable failure possible.
#
# Two lessons worth keeping: never combine `set -u` with sourcing a script
# you did not write, and never send stderr to /dev/null on a line that can
# kill the shell.
set -o pipefail

WS="${HOME}/hexapod_ws"

if [ ! -f "${WS}/install/setup.bash" ]; then
  echo "ERROR: ${WS}/install/setup.bash not found." >&2
  echo "Build the workspace first:  hexbuild" >&2
  exit 1
fi

# shellcheck disable=SC1091
source "${WS}/install/setup.bash"

echo "Workspace sourced. ROS_DISTRO=${ROS_DISTRO:-unset}"

case "${1:-help}" in

  sim)
    echo ">>> Free base, position control, Gazebo GUI."
    ros2 launch hexapod_bringup hexapod_sim.launch.py \
      control_mode:=position \
      fix_base:=false \
      gui:=true
    ;;

  sim-effort)
    # WITH the GUI, torque control, free base. This is the configuration that
    # actually walks -- 1.1 m in 20 s, measured. Use it to watch and to
    # record a demo; use autotest.sh when you want numbers instead.
    echo ">>> Free base, TORQUE control, Gazebo GUI."
    echo ">>> This is the mode that walks. Terminal 2: walk_free.sh gait-effort"
    ros2 launch hexapod_bringup hexapod_sim.launch.py \
      control_mode:=effort \
      fix_base:=false \
      gui:=true
    ;;

  gait-effort)
    # Same gait, published to the EFFORT controller's topic. Sending it to
    # the position controller's topic instead is silent: the robot simply
    # never moves, with no error anywhere.
    echo ">>> Gentle gait -> torque controller."
    ros2 launch hexapod_bringup walk.launch.py \
      command_type:=trajectory \
      command_topic:=/leg_trajectory_controller/joint_trajectory \
      cycle_time:=3.0 \
      step_height:=0.025 \
      max_stride:=0.05 \
      max_linear_speed:=0.04 \
      max_angular_speed:=0.25 \
      startup_ramp:=4.0
    ;;

  hold)
    echo ">>> DIAGNOSTIC: stance pose only, gait clock frozen."
    echo ">>> If the robot is ejected in this mode, the FIRST COMMAND is the"
    echo ">>> problem, not the gait. Watch the delta table it prints."
    ros2 launch hexapod_bringup walk.launch.py \
      command_type:=trajectory \
      command_topic:=/leg_position_controller/joint_trajectory \
      hold_only:=true \
      startup_ramp:=4.0
    ;;

  gait)
    echo ">>> Gentle gait: 3 s cycle, 25 mm lift, 50 mm stride."
    ros2 launch hexapod_bringup walk.launch.py \
      command_type:=trajectory \
      command_topic:=/leg_position_controller/joint_trajectory \
      cycle_time:=3.0 \
      step_height:=0.025 \
      max_stride:=0.05 \
      max_linear_speed:=0.04 \
      max_angular_speed:=0.25
    ;;

  drive)
    echo ">>> Click THIS window, then press i to walk forward, k to stop."
    ros2 run teleop_twist_keyboard teleop_twist_keyboard \
      --ros-args -p speed:=0.03 -p turn:=0.2
    ;;

  watch)
    echo ">>> Body position. x should climb steadily while walking forward."
    ros2 topic echo /odom --field pose.pose.position
    ;;

  status)
    # -------------------------------------------------------------------
    # Compact liveness check. Run this with terminal 1 up, BEFORE terminal 2.
    #
    # It answers the only question that matters at startup: is the simulation
    # actually stepping, and are the controllers actually active. Both must be
    # true before the gait node can safely command anything.
    #
    # WHY THIS EXISTS
    # We start Gazebo paused so the robot cannot collapse before the
    # controllers take hold. But a paused Gazebo does not call
    # controller_manager::update(), and a controller switch is APPLIED inside
    # that update. So if the unpause never fires, controllers can sit forever
    # in 'unconfigured' or 'inactive', nothing publishes /joint_states, and
    # every downstream node waits on a robot that is frozen in time.
    #
    # That failure is invisible in the Gazebo window -- a paused robot and a
    # correctly standing robot look identical.
    # -------------------------------------------------------------------
    echo ">>> SIMULATION STATUS"
    echo ""

    echo "--- gzserver process ---"
    if pgrep -x gzserver >/dev/null 2>&1; then
      echo "  running (pid $(pgrep -x gzserver | tr '\n' ' '))"
    else
      echo "  NOT RUNNING  -> terminal 1 is not up. Start it first."
      exit 1
    fi

    echo ""
    echo "--- /clock advancing? (is physics stepping) ---"
    C1=$(timeout 3 ros2 topic echo /clock --once 2>/dev/null \
         | grep -E '^\s+sec:' | head -1 | tr -dc '0-9')
    sleep 2
    C2=$(timeout 3 ros2 topic echo /clock --once 2>/dev/null \
         | grep -E '^\s+sec:' | head -1 | tr -dc '0-9')
    echo "  sim time sample 1: ${C1:-<none>}"
    echo "  sim time sample 2: ${C2:-<none>}"
    if [ -z "${C1}" ] || [ -z "${C2}" ]; then
      echo "  >> NO /clock. Gazebo is not publishing time at all."
      PAUSED=unknown
    elif [ "${C1}" = "${C2}" ]; then
      echo "  >> SIM TIME IS FROZEN. Gazebo is PAUSED."
      echo "  >> Fix now with:  bash ~/hexapod_ws/tools/walk_free.sh unpause"
      PAUSED=yes
    else
      echo "  >> advancing, physics is running"
      PAUSED=no
    fi

    echo ""
    echo "--- controllers (both must say 'active') ---"
    timeout 15 ros2 control list_controllers 2>&1 \
      || echo "  >> controller_manager did not answer"

    echo ""
    echo "--- /joint_states publishing? (gait node needs this) ---"
    if timeout 6 ros2 topic echo /joint_states --once >/dev/null 2>&1; then
      echo "  yes -- safe to start terminal 2"
    else
      echo "  >> NO /joint_states in 6s."
      if [ "${PAUSED}" = "yes" ]; then
        echo "  >> Cause is the pause above. Unpause, then re-run status."
      else
        echo "  >> joint_state_broadcaster is not active. See controller list."
      fi
    fi
    ;;

  activate)
    # Escape hatch: bring up a controller that failed to activate, without
    # relaunching the simulation. Chiefly for joint_state_broadcaster, whose
    # activation is silently dropped if it is requested while Gazebo is
    # paused (the switch is applied inside controller_manager::update(), and
    # a paused world never calls it).
    echo ">>> Forcing joint_state_broadcaster active..."
    ros2 control set_controller_state joint_state_broadcaster active
    echo ""
    echo ">>> Controllers now:"
    ros2 control list_controllers
    ;;

  unpause)
    # Manual release, for when the launch file's automatic unpause did not
    # fire. Harmless if physics is already running.
    echo ">>> Releasing physics..."
    ros2 service call /unpause_physics std_srvs/srv/Empty
    echo ">>> Now re-run:  bash ~/hexapod_ws/tools/walk_free.sh status"
    ;;

  *)
    sed -n '2,40p' "$0"
    ;;
esac

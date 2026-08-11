#!/usr/bin/env bash
# =============================================================================
# autotest.sh : run the walking test by itself, against every candidate
#               configuration, and print one table with a verdict for each.
#
# WHAT THIS REPLACES
# Four terminals, a person watching a Gazebo window, and a description typed
# back over chat. That loop cost days, because "it wiggles", "it flies away"
# and "it moves randomly" are the same sentence for several unrelated faults,
# and each round trip needed a rebuild, a relaunch, and a human's attention.
# Worse, a mistyped or half-run command produces output that looks like a
# result and is not one.
#
# Nothing here needs watching, timing or transcribing. It runs headless,
# drives the robot itself, measures ground-truth odometry, tears everything
# down, and writes a single file to paste.
#
# WHY IT TESTS SEVERAL CONFIGURATIONS
# The open question is not "does this configuration work" but "WHICH of them
# works". Testing one at a time, hours apart, with other things changing in
# between, is exactly how this project ended up unable to attribute any
# result to any cause. Running them back to back against an identical robot
# is what makes the comparison mean anything.
#
# USAGE
#   bash ~/hexapod_ws/tools/autotest.sh            # all configurations
#   bash ~/hexapod_ws/tools/autotest.sh B          # just one, by letter
#   bash ~/hexapod_ws/tools/autotest.sh B C
#
# About 2 minutes per configuration. Safe to re-run. Safe to interrupt.
# =============================================================================
set -o pipefail

WS="${HOME}/hexapod_ws"
TOOLS="${WS}/tools"
REPORT="/tmp/autotest_report_$(date +%Y%m%d_%H%M%S).txt"

# Everything this script prints also lands in the report file, so there is
# exactly one thing to paste and no chance of a partial copy.
exec > >(tee "${REPORT}") 2>&1

# =============================================================================
# CONFIGURATIONS
#
#   name | control_mode | fix_base | sim_profile | why it is in the list
# -----------------------------------------------------------------------------
#   B    | position     | false    | physical    | Never actually tried with a
#          free base and a CORRECT gait. The earlier verdict against it
#          ("body travels 2.9 mm in 20 s") was measured while the stance swept
#          every foot the wrong way, dragging them over the ground at twice
#          body speed. That measurement is void. It may simply work now.
#
#   C    | effort       | false    | repo        | KevinOchs/hexapod_ros
#          assumptions: 1e-5 kg links, identity inertias, p=100. Non-physical
#          and stable because of it.
#
#   D    | effort       | false    | physical    | Our measured model on the
#          interface that is physically correct. The one we actually want.
#          Expected to need gain work; this tells us how much.
#
# A (fix_base:=true) is deliberately absent. It welds the body to the world,
# so it CANNOT translate, and "the legs move but the robot does not" is the
# guaranteed result rather than a finding. Use it by hand to inspect leg
# motion, never as a walking test.
# =============================================================================
config_mode()    { case "$1" in B) echo position ;; C|D) echo effort ;; esac; }
config_profile() { case "$1" in B|D) echo physical ;; C) echo repo ;; esac; }
config_desc() {
  case "$1" in
    B) echo "position interface, measured model" ;;
    C) echo "effort interface, hexapod_ros model (non-physical)" ;;
    D) echo "effort interface, measured model" ;;
  esac
}

CONFIGS="${*:-B C D}"

SPEED=0.03
BASELINE=8
DRIVE=20
SETTLE=3

PIDS=()

cleanup() {
  for p in "${PIDS[@]:-}"; do kill -INT "${p}" 2>/dev/null; done
  sleep 2
  for p in "${PIDS[@]:-}"; do kill -9 "${p}" 2>/dev/null; done
  pkill -9 -f gzserver   2>/dev/null
  pkill -9 -f gzclient   2>/dev/null
  pkill -9 -f gait_node  2>/dev/null
  pkill -9 -f "topic pub" 2>/dev/null
  sleep 1
  PIDS=()
}
trap cleanup EXIT INT TERM

hr() { echo "------------------------------------------------------------"; }

# =============================================================================
# PREFLIGHT
#
# Every check here has cost this project at least one full debugging cycle,
# and every one of them is invisible at runtime: the symptom is always some
# variant of "it launches and nothing happens".
# =============================================================================
preflight() {
  local fail=0
  echo "============================================================"
  echo " PREFLIGHT"
  echo "============================================================"

  if [ ! -f "${WS}/install/setup.bash" ]; then
    echo "  FAIL  no ${WS}/install/setup.bash. Run: hexbuild"
    return 1
  fi
  # shellcheck disable=SC1091
  source "${WS}/install/setup.bash"
  echo "  ok    workspace sourced"

  # ROS 2 talks over DDS multicast. Two shells with different ROS_DOMAIN_ID
  # see completely separate graphs, which looks exactly like "the service
  # never appears".
  echo "  info  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}  RMW=${RMW_IMPLEMENTATION:-default}"

  local so
  so=$(find /opt/ros -name "libgazebo_ros2_control.so" 2>/dev/null | head -1)
  if [ -z "${so}" ]; then
    echo "  FAIL  libgazebo_ros2_control.so not found."
    echo "        sudo apt install ros-humble-gazebo-ros2-control"
    fail=1
  else
    echo "  ok    libgazebo_ros2_control.so present"
  fi

  if pgrep -f "[g]zserver" >/dev/null 2>&1; then
    echo "  warn  a gzserver is already running; killing it (it would hold"
    echo "        port 11345 and break this run in a confusing way)"
    cleanup
    bash "${TOOLS}/reset_sim.sh" >/dev/null 2>&1
  else
    echo "  ok    no stale gzserver"
  fi

  # ---- per configuration: does the model build, and does the controller
  # ---- YAML it references actually exist in the INSTALL space?
  #
  # This is the check that would have caught the current failure immediately.
  # gazebo_ros2_control is given a <parameters> path at plugin load. If that
  # file is missing, the plugin throws, gzserver carries on running quite
  # happily, and no controller_manager is ever created. There is no error in
  # the launch terminal. `ros2 control list_controllers` then blocks forever
  # on a service that will never exist.
  local desc_share
  desc_share=$(ros2 pkg prefix hexapod_description 2>/dev/null)/share/hexapod_description
  for c in ${CONFIGS}; do
    local prof urdf params
    prof=$(config_profile "${c}")
    urdf="/tmp/autotest_model_${prof}.urdf"

    if ! ros2 run xacro xacro "${desc_share}/urdf/hexapod.urdf.xacro" \
          sim:=true fix_base:=false \
          control_mode:="$(config_mode "${c}")" \
          sim_profile:="${prof}" > "${urdf}" 2>"/tmp/autotest_xacro_${prof}.err"; then
      echo "  FAIL  [${c}] xacro failed for sim_profile:=${prof}"
      sed 's/^/        | /' "/tmp/autotest_xacro_${prof}.err" | tail -15
      fail=1
      continue
    fi

    params=$(grep -o '<parameters>[^<]*</parameters>' "${urdf}" \
             | sed 's|</\?parameters>||g' | head -1)
    if [ -z "${params}" ]; then
      echo "  FAIL  [${c}] no <parameters> in the URDF: the ros2_control"
      echo "        plugin block is not reaching the model."
      fail=1
    elif [ ! -f "${params}" ]; then
      echo "  FAIL  [${c}] controller YAML MISSING:"
      echo "        ${params}"
      echo "        The plugin will throw on load and no controller_manager"
      echo "        will ever appear. Fix: hexsync && colcon build"
      fail=1
    else
      echo "  ok    [${c}] ${prof} model builds, controller YAML present"
    fi

    # Masses confirm the profile actually took effect rather than silently
    # falling through to the default.
    local m
    m=$(grep -o 'mass value="[^"]*"' "${urdf}" | sort -u | head -4 | tr '\n' ' ')
    echo "        masses: ${m}"
  done

  hr
  return ${fail}
}

wait_for_topic() {
  local topic="$1" limit="$2" i=0
  while [ "${i}" -lt "${limit}" ]; do
    if timeout 3 ros2 topic echo "${topic}" --once >/dev/null 2>&1; then return 0; fi
    i=$((i + 3))
  done
  return 1
}

wait_for_controller_manager() {
  # Separated from the controller check on purpose. "No controller_manager"
  # and "controller_manager exists but a controller failed to activate" are
  # different faults with different fixes, and the CLI reports both as an
  # endless wait followed by "No controllers are currently loaded!".
  local limit="$1" i=0
  while [ "${i}" -lt "${limit}" ]; do
    if timeout 3 ros2 service list 2>/dev/null \
         | grep -q "/controller_manager/list_controllers"; then
      return 0
    fi
    sleep 3
    i=$((i + 3))
  done
  return 1
}

wait_for_controller_active() {
  # STRIP ANSI COLOUR BEFORE MATCHING.
  # `ros2 control list_controllers` colourises the state word, so the bytes
  # are  name ... <ESC>[92mactive<ESC>[0m . A pattern of "name.* active" never
  # matches, because what precedes "active" is an escape sequence, not a
  # space. That cost ninety wasted seconds per run and a hunt for a startup
  # bug that did not exist. Never pattern-match CLI output without stripping
  # formatting: what you see in a terminal and what arrives in a pipe are not
  # the same bytes.
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

dump_sim_failure() {
  local tag="$1"
  echo ""
  echo "   ---- simulation launch log, last 40 lines ----"
  tail -40 "/tmp/autotest_sim_${tag}.log" 2>/dev/null | sed 's/^/   | /'
  echo "   ---- gazebo server log (where PLUGIN failures are recorded) ----"
  # Plugin load exceptions do not appear in the launch terminal. They go
  # here. This is the single most useful file when nothing comes up.
  tail -30 ~/.gazebo/server-11345/default.log 2>/dev/null | sed 's/^/   | /' \
    || echo "   | (no gazebo server log found)"
  echo "   ---- anything mentioning control or plugin ----"
  grep -iE "plugin|controller_manager|ros2_control|exception|error" \
    "/tmp/autotest_sim_${tag}.log" 2>/dev/null | tail -20 | sed 's/^/   | /'
  echo "   ---- nodes / services present ----"
  timeout 10 ros2 node list 2>&1 | sed 's/^/   | /'
  timeout 10 ros2 service list 2>&1 | grep -i controller | sed 's/^/   | /' \
    || echo "   | no controller_manager services at all"

  # ---- THE ALL-ZEROS CASE ------------------------------------------------
  # If /odom and /joint_states BOTH report exactly zero while the gait node
  # logged a robot standing at stance, the robot has not collapsed: a
  # collapsed robot has non-zero angles, and a blown-up one has huge ones or
  # NaN. Identity everywhere means the world got reset, or we are reading a
  # different world from the one the gait is driving.
  #
  # These four probes separate those:
  echo "   ---- is the model actually in the world ----"
  timeout 10 ros2 service call /get_model_list gazebo_msgs/srv/GetModelList \
    2>&1 | tail -6 | sed 's/^/   | /'
  echo "   ---- raw /odom, one message ----"
  timeout 8 ros2 topic echo /odom --once --field pose.pose 2>&1 \
    | head -12 | sed 's/^/   | /'
  echo "   ---- who publishes /odom and /joint_states ----"
  # More than one publisher on either topic means a survivor from the
  # previous configuration is still alive and we are averaging two worlds.
  timeout 8 ros2 topic info /odom --verbose 2>&1 \
    | grep -E "Publisher count|Node name" | sed 's/^/   | /'
  timeout 8 ros2 topic info /joint_states --verbose 2>&1 \
    | grep -E "Publisher count|Node name" | sed 's/^/   | /'
  echo "   ---- how many gzserver processes are alive ----"
  pgrep -af "[g]zserver" 2>&1 | sed 's/^/   | /' \
    || echo "   | none: gzserver DIED during the run"
  echo "   ---- did anything reset or eject the world ----"
  grep -iE "reset|eject|nan|inf|destroy|removed" \
    "/tmp/autotest_sim_${tag}.log" 2>/dev/null | tail -10 | sed 's/^/   | /'
  echo "   ----------------------------------------------"
}

run_one() {
  local c="$1"
  local mode profile controller topic tag
  mode=$(config_mode "${c}")
  profile=$(config_profile "${c}")
  tag="${c}_${mode}_${profile}"

  if [ "${mode}" = "effort" ]; then
    controller="leg_trajectory_controller"
  else
    controller="leg_position_controller"
  fi
  topic="/${controller}/joint_trajectory"

  echo ""
  echo "############################################################"
  echo "#  CONFIG ${c} : $(config_desc "${c}")"
  echo "#  control_mode=${mode}  fix_base=false  sim_profile=${profile}"
  echo "############################################################"

  cleanup
  bash "${TOOLS}/reset_sim.sh" >/dev/null 2>&1

  # LONGER SETTLE BETWEEN CONFIGURATIONS.
  #
  # Config D failed on 2026-08-12 with /odom and /joint_states both reporting
  # exactly zero, while the gait node's own log showed the robot standing
  # correctly at stance. Identity on both streams is not a collapsed robot;
  # it is the wrong world being read. The most likely cause is a survivor
  # from the previous configuration: DDS discovery keeps stale endpoints for
  # several seconds after a process dies, and gzserver can take longer than
  # `pkill` returns to actually release port 11345 and tear its world down.
  #
  # Two seconds of teardown was not enough. This costs 6 seconds per run and
  # removes a whole class of false result.
  sleep 6
  if pgrep -f "[g]zserver" >/dev/null 2>&1; then
    echo "   WARNING: gzserver still alive after cleanup; waiting longer"
    sleep 8
    pkill -9 -f gzserver 2>/dev/null
    sleep 3
  fi

  # WATCH=1 runs with the Gazebo window open and leaves everything running at
  # the end. Human eyes settle "is it standing, collapsed, or gone" in one
  # second, and no amount of topic archaeology substitutes for looking.
  local gui="false"
  if [ "${WATCH:-0}" = "1" ]; then gui="true"; fi

  echo "-- launching simulation (gui=${gui}) ..."
  ros2 launch hexapod_bringup hexapod_sim.launch.py \
      control_mode:="${mode}" \
      sim_profile:="${profile}" \
      fix_base:=false \
      gui:="${gui}" \
      rviz:=false \
      > "/tmp/autotest_sim_${tag}.log" 2>&1 &
  PIDS+=($!)

  if ! wait_for_controller_manager 60; then
    echo "   FAILED: no controller_manager service after 60 s."
    echo "   This is a PLUGIN LOAD failure, not a controller failure:"
    echo "   gazebo_ros2_control never instantiated. Usual causes are a"
    echo "   missing controller YAML, a missing .so, or the robot never"
    echo "   being spawned into the world."
    dump_sim_failure "${tag}"
    return 30
  fi
  echo "   controller_manager is up"

  if ! wait_for_controller_active "joint_state_broadcaster" 60; then
    echo "   FAILED: joint_state_broadcaster never became active."
    dump_sim_failure "${tag}"
    return 20
  fi
  if ! wait_for_controller_active "${controller}" 60; then
    echo "   FAILED: ${controller} never became active."
    dump_sim_failure "${tag}"
    return 21
  fi
  if ! wait_for_topic /joint_states 30; then
    echo "   FAILED: no /joint_states."
    return 22
  fi
  echo "   controllers active, joint states flowing"

  echo "-- launching gait node ..."
  # control_mode is passed so walk.launch.py derives the matching topic.
  # command_topic is passed too, belt and braces: publishing to a topic with
  # no subscriber is silent, and it has already cost one full test cycle.
  ros2 launch hexapod_bringup walk.launch.py \
      control_mode:="${mode}" \
      command_topic:="${topic}" \
      cycle_time:=3.0 \
      step_height:=0.025 \
      max_stride:=0.05 \
      max_linear_speed:=0.04 \
      max_angular_speed:=0.25 \
      startup_ramp:=4.0 \
      > "/tmp/autotest_gait_${tag}.log" 2>&1 &
  PIDS+=($!)

  echo "-- waiting for startup ramp ..."
  sleep 12

  if grep -q "NOTHING IS SUBSCRIBED" "/tmp/autotest_gait_${tag}.log" 2>/dev/null; then
    echo "   FAILED: gait node reports nobody is listening to ${topic}."
    grep -A6 "NOTHING IS SUBSCRIBED" "/tmp/autotest_gait_${tag}.log" | sed 's/^/   | /'
    return 23
  fi
  if ! grep -q "Ramp complete" "/tmp/autotest_gait_${tag}.log" 2>/dev/null; then
    echo "   WARNING: ramp did not report completion. Continuing anyway."
    tail -8 "/tmp/autotest_gait_${tag}.log" | sed 's/^/     /'
  fi

  # ---- TIMELINE SAMPLER ---------------------------------------------------
  # Config D reports the robot in the world, physics running, the gait node
  # reading real joint angles at startup, and the measurement reading exactly
  # zero for both /odom and /joint_states. Those cannot all be true at the
  # same instant, so the missing datum is WHEN it changed.
  #
  # A snapshot every 2 s answers it directly:
  #   zero from the first sample  -> plumbing. The measurement is reading
  #                                  something other than this robot.
  #   real, then zero at sample N -> the robot did something at a knowable
  #                                  moment, and N tells us whether it was
  #                                  the ramp finishing or the drive starting.
  local tl="/tmp/autotest_timeline_${tag}.txt"
  : > "${tl}"
  (
    for i in $(seq 1 16); do
      od=$(timeout 3 ros2 topic echo /odom --once --field pose.pose.position 2>/dev/null \
           | tr '\n' ' ' | sed 's/  */ /g')
      js=$(timeout 3 ros2 topic echo /joint_states --once --field position 2>/dev/null \
           | tr '\n' ' ' | cut -c1-70)
      printf '  t+%-3ss  odom[%s]  joints[%s]\n' "$((i * 2))" "${od}" "${js}" >> "${tl}"
      sleep 2
    done
  ) &
  PIDS+=($!)

  echo "-- measuring ..."
  python3 "${TOOLS}/measure_walk.py" \
      --speed "${SPEED}" --baseline "${BASELINE}" \
      --drive "${DRIVE}" --settle "${SETTLE}"
  local rc=$?

  echo ""
  echo "   ---- timeline: /odom and /joint_states every 2 s ----"
  cat "${tl}" 2>/dev/null | sed 's/^/   /'
  echo "   -----------------------------------------------------"

  if [ "${rc}" = "17" ] || [ "${rc}" = "10" ]; then
    dump_sim_failure "${tag}"
    echo "   ---- gait node log, last 25 lines ----"
    tail -25 "/tmp/autotest_gait_${tag}.log" 2>/dev/null | sed 's/^/   | /'
  fi

  if [ "${WATCH:-0}" = "1" ]; then
    echo ""
    echo "   WATCH=1: leaving the simulation running so you can look at it."
    echo "   Drive it by hand:"
    echo "     ros2 run teleop_twist_keyboard teleop_twist_keyboard"
    echo "   Stop everything when done:"
    echo "     pkill -9 -f gzserver; pkill -9 -f gzclient; pkill -9 -f gait_node"
    trap - EXIT INT TERM        # do not tear down on exit
    return ${rc}
  fi

  cleanup
  return ${rc}
}

echo "============================================================"
echo " AUTOMATED WALK TEST   $(date)"
echo " configurations: ${CONFIGS}"
echo " speed ${SPEED} m/s   baseline ${BASELINE}s   drive ${DRIVE}s"
echo "============================================================"

if ! preflight; then
  echo ""
  echo "PREFLIGHT FAILED. Nothing was launched, because every check above"
  echo "fails in a way that looks identical at runtime. Fix the above first."
  exit 1
fi

declare -A RESULTS
for c in ${CONFIGS}; do
  run_one "${c}"
  RESULTS[${c}]=$?
done

echo ""
echo "============================================================"
echo " SUMMARY"
echo "============================================================"
for c in ${CONFIGS}; do
  rc="${RESULTS[${c}]}"
  case "${rc}" in
    0)  msg="WALKING" ;;
    10) msg="ejected / tipped over" ;;
    11) msg="drifts while standing still" ;;
    12) msg="walks but feet slip" ;;
    13) msg="marches in place, body does not travel" ;;
    14) msg="wanders, motion not coordinated" ;;
    15) msg="legs not moving -- command never reached the gait" ;;
    16) msg="steps on the spot -- no stride commanded" ;;
    17) msg="ROBOT NOT IN THE WORLD -- model missing or destroyed" ;;
    2)  msg="no odometry -- sim did not come up" ;;
    3)  msg="simulation time frozen" ;;
    20) msg="joint_state_broadcaster never activated" ;;
    21) msg="leg controller never activated" ;;
    22) msg="no /joint_states" ;;
    23) msg="nothing subscribed to the command topic" ;;
    30) msg="NO controller_manager -- gazebo_ros2_control failed to load" ;;
    *)  msg="unknown result (code ${rc})" ;;
  esac
  printf '  %-3s %-46s %s\n' "${c}" "$(config_desc "${c}")" "${msg}"
done
echo "============================================================"
echo ""
echo " Report written to: ${REPORT}"
echo " Paste that one file. Per-run logs, if they are needed:"
echo "   /tmp/autotest_sim_*.log   /tmp/autotest_gait_*.log"
echo "============================================================"

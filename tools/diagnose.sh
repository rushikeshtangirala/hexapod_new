#!/usr/bin/env bash
# =============================================================================
# diagnose.sh : check every prerequisite for gazebo_ros2_control before
#               blaming the launch file.
#
# WHY THIS EXISTS
# "Could not contact service /controller_manager/list_controllers" is a
# symptom with at least six distinct causes, and the message distinguishes
# none of them. Rather than guessing, this checks each link in the chain in
# the order the system builds it, so the FIRST failure is the real one.
#
# USAGE (from inside WSL, after sourcing the workspace)
#   bash tools/diagnose.sh
# =============================================================================
set -uo pipefail

WS="${HOME}/hexapod_ws"
PASS=0
FAIL=0

ok()   { echo "  [ OK ] $1"; PASS=$((PASS+1)); }
bad()  { echo "  [FAIL] $1"; FAIL=$((FAIL+1)); }
info() { echo "         $1"; }

echo "============================================================"
echo " HEXAPOD SIM DIAGNOSTICS"
echo "============================================================"

# --- 1. workspace sourced ---------------------------------------------------
echo ""
echo "1. Environment"
if [ -n "${AMENT_PREFIX_PATH:-}" ] && [[ "${AMENT_PREFIX_PATH}" == *"hexapod_ws"* ]]; then
  ok "workspace overlay is sourced"
else
  bad "workspace NOT sourced"
  info "run: source ${WS}/install/setup.bash"
fi

# --- 2. required packages ---------------------------------------------------
echo ""
echo "2. Required packages"
for p in gazebo_ros gazebo_ros2_control controller_manager position_controllers \
         joint_state_broadcaster; do
  if ros2 pkg prefix "${p}" >/dev/null 2>&1; then
    ok "${p}"
  else
    bad "${p} MISSING"
    info "sudo apt install ros-humble-${p//_/-}"
  fi
done

# --- 3. the plugin shared object --------------------------------------------
echo ""
echo "3. gazebo_ros2_control plugin binary"
SO_PATH="$(find /opt/ros/humble -name 'libgazebo_ros2_control.so' 2>/dev/null | head -1)"
if [ -n "${SO_PATH}" ]; then
  ok "found ${SO_PATH}"
else
  bad "libgazebo_ros2_control.so NOT FOUND"
  info "sudo apt install ros-humble-gazebo-ros2-control"
fi

# --- 4. xacro expansion ------------------------------------------------------
echo ""
echo "4. URDF expansion"
URDF_RAW="/tmp/hexapod_diag_raw.urdf"
URDF="/tmp/hexapod_diag.urdf"

if xacro "${WS}/src/hexapod_description/urdf/hexapod.urdf.xacro" sim:=true \
     > "${URDF_RAW}" 2>/tmp/hexapod_diag.err; then
  ok "xacro expanded cleanly"
else
  bad "xacro FAILED"
  sed 's/^/         /' /tmp/hexapod_diag.err
  echo ""
  echo "Stopping: nothing downstream can work."
  exit 1
fi

# Strip comments before ANY further inspection.
#
# This is not cosmetic. Every check below greps for XML tags, and the source
# comments in hexapod.ros2_control.xacro discuss those very tags by name
# ("<parameters> points at the controller YAML..."). Grepping the raw output
# therefore matches PROSE ABOUT the tag before the tag itself, and reports a
# confident, completely wrong diagnosis.
#
# It also means we inspect exactly the artefact the launch file feeds to
# gazebo_ros2_control, rather than a different one. A diagnostic that tests
# something other than what runs is worse than no diagnostic.
python3 - "${URDF_RAW}" "${URDF}" <<'PYEOF'
import re, sys
src, dst = sys.argv[1], sys.argv[2]
text = open(src, encoding="utf-8").read()
text = re.sub(r"<!--.*?-->", "", text, flags=re.DOTALL)
text = re.sub(r"\n\s*\n+", "\n", text)
open(dst, "w", encoding="utf-8").write(text)
PYEOF

if grep -q '<!' "${URDF}"; then
  bad "comment stripping did not fully succeed"
else
  ok "comments stripped (this is what gazebo_ros2_control will receive)"
fi

RAW_BYTES=$(wc -c < "${URDF_RAW}")
CLEAN_BYTES=$(wc -c < "${URDF}")
info "URDF size: ${RAW_BYTES} bytes raw -> ${CLEAN_BYTES} bytes stripped"

# --- 5. contents of the expanded URDF ---------------------------------------
echo ""
echo "5. Expanded URDF contents"

N_REV=$(grep -c 'type="revolute"' "${URDF}" || true)
[ "${N_REV}" = "18" ] && ok "18 revolute joints" || bad "expected 18 revolute joints, got ${N_REV}"

if grep -q '<ros2_control' "${URDF}"; then
  ok "<ros2_control> block present"
else
  bad "<ros2_control> block MISSING"
  info "hexapod.ros2_control.xacro was not included. Check the sim arg."
fi

N_CMD=$(grep -c '<command_interface name="position"' "${URDF}" || true)
[ "${N_CMD}" = "18" ] && ok "18 position command interfaces" \
                      || bad "expected 18 command interfaces, got ${N_CMD}"

if grep -q 'libgazebo_ros2_control.so' "${URDF}"; then
  ok "gazebo_ros2_control plugin tag present"
else
  bad "plugin tag MISSING from URDF"
fi

# --- 6. the controllers.yaml path baked into the URDF -----------------------
echo ""
echo "6. Controller parameter file"
# Grepping the COMMENT-STRIPPED urdf, so this cannot match prose.
YAML_PATH=$(grep -oP '(?<=<parameters>)[^<]+' "${URDF}" | head -1 | xargs || true)
if [ -z "${YAML_PATH}" ]; then
  bad "no <parameters> path found in the URDF"
else
  info "URDF points at: ${YAML_PATH}"
  if [ -f "${YAML_PATH}" ]; then
    ok "file exists"
    if grep -q 'leg_position_controller' "${YAML_PATH}"; then
      ok "leg_position_controller declared"
    else
      bad "leg_position_controller not declared in YAML"
    fi
  else
    bad "file DOES NOT EXIST"
    info "This is why the controller manager never starts. The plugin"
    info "aborts silently when its parameter file is missing."
    info "Did you rebuild after adding config/controllers.yaml?"
  fi
fi

# --- 7. mesh resolution ------------------------------------------------------
echo ""
echo "7. Meshes and GAZEBO_MODEL_PATH"
DESC_SHARE="${WS}/install/hexapod_description/share"
for m in body coxa femur_lower femur_upper tibia; do
  if [ -f "${DESC_SHARE}/hexapod_description/meshes/visual/${m}.stl" ]; then
    ok "${m}.stl installed"
  else
    bad "${m}.stl NOT in install space"
    info "run tools/reorigin_meshes.py, copy results to the Windows side, rebuild"
  fi
done

if [[ "${GAZEBO_MODEL_PATH:-}" == *"${DESC_SHARE}"* ]]; then
  ok "GAZEBO_MODEL_PATH contains the workspace share dir"
else
  info "GAZEBO_MODEL_PATH does not yet contain ${DESC_SHARE}"
  info "The launch file appends it at runtime, so this is only a problem"
  info "if you are running gazebo manually."
fi

echo ""
echo "============================================================"
echo " ${PASS} passed, ${FAIL} failed"
echo "============================================================"
[ "${FAIL}" -eq 0 ] || exit 1

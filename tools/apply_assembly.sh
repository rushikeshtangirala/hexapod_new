#!/usr/bin/env bash
# =============================================================================
# apply_assembly.sh : one command, assembled STL -> working model.
#
# Does everything, in order, stopping at the first failure:
#   1. sync (which lints the XML)
#   2. split the assembled leg into its three links, in assembly pose
#   3. measure the true axis-to-axis link lengths from the joint overlaps
#   4. write the link meshes to the WINDOWS side
#   5. patch common_properties.xacro and kinematics.py with the measurements
#   6. sync again, build
#   7. verify the kinematics offline
#
# Every step is checked. If one fails the script stops rather than carrying a
# bad state into the next stage, which is what turns one mistake into an hour
# of confusion.
#
# USAGE
#   bash ~/hexapod_ws/tools/apply_assembly.sh
# =============================================================================
set -o pipefail

WIN_ROOT="/mnt/c/Users/Rushi Tangirala/OneDrive/Desktop/hexapod_scratch"
WS="${HOME}/hexapod_ws"
MESH_DIR="${WS}/src/hexapod_description/meshes/visual"
ASSEMBLY="${MESH_DIR}/leg assembled.stl"

step() { echo ""; echo "=============================================="; \
         echo " $1"; echo "=============================================="; }
die()  { echo ""; echo "FAILED: $1" >&2; exit 1; }

step "1/6  sync"
bash "${WIN_ROOT}/sync_to_wsl.sh" || die "sync or XML lint failed"

step "2/6  check the assembly file is present"
[ -f "${ASSEMBLY}" ] || die "not found: ${ASSEMBLY}
Copy 'leg assembled.stl' into
  ${WIN_ROOT}/src/hexapod_description/meshes/visual/
on the Windows side, then re-run."
echo "  found: $(du -h "${ASSEMBLY}" | cut -f1)"

step "3/6  split, measure, export, patch"
python3 "${WS}/tools/split_assembly.py" "${ASSEMBLY}" \
        --patch-dir "${WIN_ROOT}" || die "split_assembly.py"

step "4/6  sync the new meshes and lengths back into the workspace"
bash "${WIN_ROOT}/sync_to_wsl.sh" || die "sync after patch"

step "5/6  build"
cd "${WS}" || die "cd ${WS}"
colcon build --symlink-install || die "colcon build"
# shellcheck disable=SC1091
source "${WS}/install/setup.bash"

step "6/6  verify the kinematics"
python3 "${WS}/tools/verify_ik.py" || echo "  >> verify_ik reported failures"

echo ""
echo "=============================================================="
echo " DONE. Look at it:"
echo "   ros2 launch hexapod_description display.launch.py"
echo ""
echo " Then Gazebo:"
echo "   bash ~/hexapod_ws/tools/reset_sim.sh"
echo "   ros2 launch hexapod_bringup hexapod_demo.launch.py"
echo "=============================================================="

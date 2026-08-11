#!/usr/bin/env bash
# =============================================================================
# test_gazebo_gui.sh : get gzclient rendering, one variable at a time.
#
# WHY A DEDICATED SCRIPT
# gzclient failing inside a 6-node launch file is nearly impossible to
# diagnose: the output is interleaved, the failure is non-fatal so the launch
# continues, and half the messages are unrelated. This runs gazebo ALONE,
# with an empty world, so anything that appears is about rendering and
# nothing else.
#
# If the window opens here, the GUI works and any remaining problem is in the
# launch file. If it does not, the fixes below are ordered by likelihood.
#
# USAGE
#   bash tools/test_gazebo_gui.sh
# =============================================================================
set -uo pipefail

echo "============================================================"
echo " GAZEBO GUI DIAGNOSTICS"
echo "============================================================"

echo ""
echo "--- 1. Is there a display? ---"
echo "DISPLAY        = ${DISPLAY:-<unset>}"
echo "WAYLAND_DISPLAY= ${WAYLAND_DISPLAY:-<unset>}"
if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
  echo ">> No display at all. WSLg is not running. Restart WSL from PowerShell:"
  echo ">>     wsl --shutdown"
fi

echo ""
echo "--- 2. What OpenGL do we actually have? ---"
if command -v glxinfo >/dev/null 2>&1; then
  glxinfo -B 2>/dev/null | grep -E "OpenGL renderer|OpenGL version|OpenGL core"
  echo ""
  echo "  Interpretation:"
  echo "    'D3D12 (...)'  = GPU accelerated through WSLg. Best case."
  echo "    'llvmpipe'     = software rasteriser. Works, but slow."
  echo "    nothing        = no GL at all; nothing graphical will run."
else
  echo ">> glxinfo not installed. Run:  sudo apt install -y mesa-utils"
fi

echo ""
echo "--- 3. Gazebo resource paths ---"
echo "GAZEBO_RESOURCE_PATH = ${GAZEBO_RESOURCE_PATH:-<unset>}"
if [ -z "${GAZEBO_RESOURCE_PATH:-}" ]; then
  echo ">> UNSET. This is the direct cause of:"
  echo ">>     [Err] [RTShaderSystem.cc:480] Unable to find shader lib"
  echo ">>     [Err] [RenderEngine.cc:197] Failed to initialize scene"
  echo ">> Fix:  source /usr/share/gazebo/setup.sh"
fi

echo ""
echo "--- 4. Is the OGRE shader library actually on disk? ---"
FOUND=""
for d in /usr/share/gazebo-11/media/rtshaderlib150 \
         /usr/share/gazebo/media/rtshaderlib150; do
  if [ -d "$d" ]; then
    echo "  found: $d"
    FOUND=1
  fi
done
if [ -z "$FOUND" ]; then
  echo ">> Shader library MISSING from disk. Reinstall:"
  echo ">>     sudo apt install --reinstall gazebo11-common gazebo11"
fi

echo ""
echo "--- 5. Launching gzclient against an empty world ---"
echo "    A Gazebo window with a grey ground plane should appear."
echo "    Close it, or press Ctrl-C here, when you have seen it."
echo ""
sleep 2

gazebo --verbose /usr/share/gazebo-11/worlds/empty.world 2>&1 | \
  grep -vE "model.config|Fuel|Publicized" | head -40

echo ""
echo "============================================================"
echo " IF NO WINDOW APPEARED, try these IN ORDER, one at a time:"
echo ""
echo "  a) source /usr/share/gazebo/setup.sh && gazebo"
echo "     (missing resource path is the most common cause)"
echo ""
echo "  b) export LIBGL_ALWAYS_SOFTWARE=1 && gazebo"
echo "     (forces llvmpipe; slow but avoids driver negotiation)"
echo ""
echo "  c) export QT_QPA_PLATFORM=xcb && gazebo"
echo "     (forces X11 instead of Wayland; OGRE 1.9 predates Wayland)"
echo ""
echo "  d) In PowerShell:  wsl --shutdown   then reopen"
echo "     (WSLg's compositor can wedge; a restart clears it)"
echo ""
echo "  e) In PowerShell:  wsl --update"
echo "     (older WSL builds have materially worse GL support)"
echo ""
echo " Change ONE variable at a time and re-run. Changing several at once"
echo " means you will not know which one worked, and you will need to know"
echo " that on review day."
echo "============================================================"

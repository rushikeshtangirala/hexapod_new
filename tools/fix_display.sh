#!/usr/bin/env bash
# =============================================================================
# fix_display.sh : get GUI windows appearing again under WSLg.
#
# WHEN TO USE THIS
# gzclient starts, prints "Connected to gazebo master", and no window appears.
# No RenderEngine or OGRE error. That combination means the problem is not
# Gazebo at all: the process is running fine and simply has nowhere to draw.
#
# The test below proves it in five seconds by trying a trivial X application.
# If xeyes does not appear, nothing graphical will, and debugging Gazebo is
# wasted effort.
#
# THE USUAL CAUSE
# WSLg exposes its X server socket at /mnt/wslg/.X11-unix and expects
# /tmp/.X11-unix to be a symlink to it. Various things replace that symlink
# with an ordinary empty directory: a package that writes to /tmp, a snap, or
# an unclean shutdown. X clients then connect to a socket that is not there,
# and fail silently.
#
# USAGE
#   bash tools/fix_display.sh
# =============================================================================
set -uo pipefail

echo "============================================================"
echo " WSLg DISPLAY REPAIR"
echo "============================================================"

echo ""
echo "--- 1. Display variables ---"
echo "DISPLAY         = ${DISPLAY:-<unset>}"
echo "WAYLAND_DISPLAY = ${WAYLAND_DISPLAY:-<unset>}"
echo "XDG_RUNTIME_DIR = ${XDG_RUNTIME_DIR:-<unset>}"

if [ -z "${DISPLAY:-}" ]; then
  echo ">> DISPLAY unset. Setting it for this shell."
  export DISPLAY=:0
fi

echo ""
echo "--- 2. Is WSLg actually mounted? ---"
if [ -d /mnt/wslg ]; then
  echo "  /mnt/wslg present"
  ls -d /mnt/wslg/.X11-unix 2>/dev/null && echo "  X11 socket dir present" \
    || echo "  >> /mnt/wslg/.X11-unix MISSING"
else
  echo "  >> /mnt/wslg NOT PRESENT."
  echo "  >> WSLg is not running. From PowerShell:"
  echo "  >>     wsl --update"
  echo "  >>     wsl --shutdown"
  echo "  >> then reopen Ubuntu. Nothing graphical can work until this exists."
  exit 1
fi

echo ""
echo "--- 3. The /tmp/.X11-unix symlink ---"
if [ -L /tmp/.X11-unix ]; then
  echo "  OK, symlink -> $(readlink /tmp/.X11-unix)"
elif [ -d /tmp/.X11-unix ]; then
  echo "  >> It is a real DIRECTORY, not a symlink. This is the usual cause."
  echo "  >> Repairing..."
  sudo rm -rf /tmp/.X11-unix
  sudo ln -sf /mnt/wslg/.X11-unix /tmp/.X11-unix
  echo "  >> Repaired: $(readlink /tmp/.X11-unix)"
else
  echo "  >> Missing entirely. Creating..."
  sudo ln -sf /mnt/wslg/.X11-unix /tmp/.X11-unix
  echo "  >> Created: $(readlink /tmp/.X11-unix)"
fi

echo ""
echo "--- 4. Minimal GUI test ---"
if ! command -v xeyes >/dev/null 2>&1; then
  echo "  installing x11-apps..."
  sudo apt install -y x11-apps >/dev/null 2>&1
fi

echo "  Launching xeyes for 5 seconds."
echo "  A small window with a pair of eyes should appear."
timeout 5 xeyes 2>&1 | head -5
echo "  (xeyes closed)"

echo ""
echo "--- 5. OpenGL test ---"
if command -v glxinfo >/dev/null 2>&1; then
  glxinfo -B 2>&1 | grep -E "OpenGL renderer|OpenGL version" || \
    echo "  >> No GL. Graphics will not work."
else
  echo "  glxinfo missing: sudo apt install -y mesa-utils"
fi

echo ""
echo "============================================================"
echo " IF xeyes DID NOT APPEAR"
echo ""
echo " The problem is WSLg, not ROS and not Gazebo. In PowerShell:"
echo "     wsl --shutdown"
echo " wait 10 seconds, reopen Ubuntu, and run this script again."
echo ""
echo " If it still fails:"
echo "     wsl --update"
echo "     wsl --shutdown"
echo ""
echo " IF xeyes APPEARED BUT GAZEBO STILL DOES NOT"
echo ""
echo " gzclient is a Qt5 application and may be choosing a Wayland backend"
echo " that OGRE 1.9 cannot render into. Force X11:"
echo "     export QT_QPA_PLATFORM=xcb"
echo "     ros2 launch hexapod_bringup hexapod_sim.launch.py ..."
echo " If that works, add it to ~/.bashrc."
echo "============================================================"

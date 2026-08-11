#!/usr/bin/env bash
# =============================================================================
# wsl_env_setup.sh : one-time environment fixes for running Gazebo under WSL2.
#
# Run ONCE:
#     bash tools/wsl_env_setup.sh
#     source ~/.bashrc
#
# Each block below corresponds to a specific error you have already seen or
# will see. None of them are hexapod-specific; they are WSL2 and Gazebo
# Classic packaging issues.
# =============================================================================
set -euo pipefail

BASHRC="${HOME}/.bashrc"
MARKER="# ---- hexapod project environment ----"

if grep -qF "${MARKER}" "${BASHRC}"; then
  echo "Environment block already present in ${BASHRC}. Nothing to do."
  exit 0
fi

cat >> "${BASHRC}" <<'EOF'

# ---- hexapod project environment ----

# 1. Gazebo's own resource paths.
#    Fixes: [Err] [RTShaderSystem.cc:480] Unable to find shader lib.
#           Shader generating will fail. Your GAZEBO_RESOURCE_PATH is
#           probably improperly set.
#    Gazebo ships a setup script that exports GAZEBO_RESOURCE_PATH,
#    GAZEBO_PLUGIN_PATH and GAZEBO_MODEL_PATH. The ROS 2 apt packages do not
#    source it for you.
if [ -f /usr/share/gazebo/setup.sh ]; then
  source /usr/share/gazebo/setup.sh
fi

# 2. Rendering.
#
#    LIBGL_ALWAYS_SOFTWARE IS DELIBERATELY *NOT* SET.
#
#    It was set here at one point as a guess at fixing
#        [Err] [RenderEngine.cc:197] Failed to initialize scene
#    but that error's actual cause was the line immediately above it in the
#    log:
#        [Err] [RTShaderSystem.cc:480] Unable to find shader lib.
#              Your GAZEBO_RESOURCE_PATH is probably improperly set.
#    OGRE could not load its shader library, so it could not build a scene,
#    so the GL widget had nothing to draw. Sourcing gazebo's setup.sh in
#    block 1 above fixes that properly.
#
#    Forcing software rendering is also counterproductive on WSLg, which
#    provides GPU-accelerated GL through Mesa's d3d12 driver. OGRE 1.9
#    generally negotiates a context against that more successfully than
#    against llvmpipe.
#
#    If gzclient still fails after this, uncomment the line below and
#    compare. Change ONE variable at a time.
# export LIBGL_ALWAYS_SOFTWARE=1

# 3. Qt platform.
#    gzclient and rviz2 are Qt5 applications. Under WSLg, Qt may select a
#    Wayland backend that OGRE 1.9 cannot create a GL context inside, and the
#    result is a process that starts, connects, logs nothing unusual, and
#    never shows a window. Forcing the X11 (xcb) backend routes it through
#    WSLg's X server instead, which OGRE handles correctly.
export QT_QPA_PLATFORM=xcb
export QT_X11_NO_MITSHM=1

#    WSLg publishes its X socket at /mnt/wslg/.X11-unix and expects
#    /tmp/.X11-unix to be a symlink to it. If something replaces that with a
#    real directory, every GUI application fails silently. Repair it on each
#    shell start; it is a no-op when already correct.
if [ -d /mnt/wslg/.X11-unix ] && [ ! -L /tmp/.X11-unix ]; then
  sudo rm -rf /tmp/.X11-unix 2>/dev/null
  sudo ln -sf /mnt/wslg/.X11-unix /tmp/.X11-unix 2>/dev/null
fi
export DISPLAY=${DISPLAY:-:0}

# 3b. DDS transport, and this one matters.
#     Fixes: services that are DISCOVERABLE but never respond, e.g.
#            "Failed getting a result from calling
#             /controller_manager/list_controllers in 10.0"
#     as distinct from "waiting for service to become available", which
#     means the service does not exist at all. Learn to tell these apart:
#     the first is a transport problem, the second is a startup problem.
#
#     Why it happens: FastDDS (the ROS 2 Humble default) discovers peers
#     over shared memory, which works fine inside one WSL2 VM, but carries
#     request/response traffic over UDP including multicast. WSL2's virtual
#     NIC handles multicast unreliably. So nodes SEE each other and then
#     cannot talk.
#
#     ROS_LOCALHOST_ONLY confines all traffic to the loopback interface,
#     which sidesteps the virtual NIC entirely. Everything here runs on one
#     machine, so nothing is lost.
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=42

# 4. ROS 2 + workspace
source /opt/ros/humble/setup.bash
if [ -f "${HOME}/hexapod_ws/install/setup.bash" ]; then
  source "${HOME}/hexapod_ws/install/setup.bash"
fi

# 5. Project aliases
alias hexsync="bash '/mnt/c/Users/Rushi Tangirala/OneDrive/Desktop/hexapod_scratch/sync_to_wsl.sh'"
alias hexbuild="cd ~/hexapod_ws && hexsync && colcon build --symlink-install && source install/setup.bash"
alias hexclean="cd ~/hexapod_ws && rm -rf build install log && hexbuild"
alias hexdiag="bash '/mnt/c/Users/Rushi Tangirala/OneDrive/Desktop/hexapod_scratch/tools/diagnose.sh'"
alias hexpose="bash '/mnt/c/Users/Rushi Tangirala/OneDrive/Desktop/hexapod_scratch/tools/pose.sh'"
# ---- end hexapod project environment ----
EOF

echo "Appended environment block to ${BASHRC}."
echo ""
echo "Now run:  source ~/.bashrc"
echo ""
echo "New aliases available:"
echo "  hexsync   mirror Windows -> WSL, with XML lint gate"
echo "  hexbuild  sync + colcon build + source"
echo "  hexclean  wipe build/install/log, then hexbuild"
echo "  hexdiag   run the simulation diagnostics"
echo "  hexpose   send a named pose, e.g. hexpose stand"

#!/usr/bin/env bash
# =============================================================================
# reset_sim.sh : clear every stale process and lock before relaunching.
#
# WHY THIS IS NEEDED
# Gazebo Classic runs a master on TCP port 11345. If gzserver is killed
# abruptly (Ctrl-C during startup, a WSL restart, a crashed launch) the
# process frequently survives, or the port stays in TIME_WAIT. The next
# launch cannot bind, and the failure is nearly silent: gzclient prints
# "Waiting for master" forever, or the window never appears at all.
#
# ros2 daemon caches the node graph as well. After a hard kill it can serve
# stale entries, which makes `ros2 node list` and `ros2 control
# list_controllers` report things that no longer exist.
#
# Run this whenever the simulation "just will not start". It is safe: it
# only touches simulation processes.
# =============================================================================
set -uo pipefail

echo "Stopping stale simulation processes..."

for p in gzserver gzclient gazebo rviz2 spawn_entity.py robot_state_publisher; do
  if pgrep -f "${p}" >/dev/null 2>&1; then
    echo "  killing ${p}"
    pkill -9 -f "${p}" 2>/dev/null || true
  fi
done

# Controller spawners and the gait node
pkill -9 -f "controller_manager" 2>/dev/null || true
pkill -9 -f "gait_node" 2>/dev/null || true
pkill -9 -f "teleop_twist_keyboard" 2>/dev/null || true

echo "Restarting the ROS 2 daemon (clears the cached node graph)..."
ros2 daemon stop >/dev/null 2>&1 || true
sleep 1
ros2 daemon start >/dev/null 2>&1 || true

echo "Checking the Gazebo master port (11345)..."
if command -v ss >/dev/null 2>&1; then
  if ss -tlnp 2>/dev/null | grep -q 11345; then
    echo "  >> STILL IN USE. Wait ~30 s for TIME_WAIT to clear, or reboot WSL:"
    echo "  >>   (from PowerShell)  wsl --shutdown"
  else
    echo "  free"
  fi
fi

sleep 2
echo ""
echo "Reset complete. Relaunch now."

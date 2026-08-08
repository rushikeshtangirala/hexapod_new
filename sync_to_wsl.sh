#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# sync_to_wsl.sh  --  one-way mirror:  Windows authoring folder  ->  WSL workspace
#
# WHY THIS EXISTS
#   The ROS 2 workspace MUST live on the native WSL ext4 filesystem
#   (~/hexapod_ws), not on /mnt/c. Two hard reasons:
#     1. Build speed. /mnt/c is exposed to WSL2 over the 9p protocol. colcon
#        performs tens of thousands of small stat()/open() calls; on 9p this is
#        roughly 5-20x slower. An 18-DOF description + control stack that
#        builds in 25 s natively can take 5+ minutes on /mnt/c.
#     2. Correctness. `colcon build --symlink-install` creates POSIX symlinks.
#        DrvFs support for them is conditional on metadata mount options and
#        Windows Developer Mode. OneDrive additionally rewrites/locks files it
#        is syncing, which corrupts build/ and install/ artefacts.
#
#   So: source of truth for TEXT lives on Windows (editable, OneDrive-backed,
#   writable by tooling), the BUILD happens natively in WSL. build/, install/
#   and log/ never touch Windows.
#
# USAGE (from inside WSL)
#   chmod +x /mnt/c/Users/'Rushi Tangirala'/OneDrive/Desktop/hexapod_scratch/sync_to_wsl.sh
#   ~/hexapod_ws/sync.sh          # after adding the alias below
#
# RECOMMENDED ALIAS  (append to ~/.bashrc, then `source ~/.bashrc`)
#   alias hexsync="bash '/mnt/c/Users/Rushi Tangirala/OneDrive/Desktop/hexapod_scratch/sync_to_wsl.sh'"
#   alias hexbuild="cd ~/hexapod_ws && hexsync && colcon build --symlink-install && source install/setup.bash"
# ---------------------------------------------------------------------------
set -euo pipefail

WIN_SRC="/mnt/c/Users/Rushi Tangirala/OneDrive/Desktop/hexapod_scratch/src/"
WS_ROOT="${HOME}/hexapod_ws"
WS_SRC="${WS_ROOT}/src/"

if [ ! -d "${WIN_SRC}" ]; then
  echo "ERROR: Windows source folder not found: ${WIN_SRC}" >&2
  echo "Check the path (spaces in 'Rushi Tangirala' are significant)." >&2
  exit 1
fi

mkdir -p "${WS_SRC}"

# --delete makes this a true mirror: files removed on the Windows side are
# removed in the workspace. This prevents the classic failure mode where a
# renamed xacro file leaves a stale copy behind that colcon keeps installing.
rsync -av --delete \
  --exclude '__pycache__/' \
  --exclude '*.pyc' \
  --exclude '.git/' \
  --exclude 'build/' \
  --exclude 'install/' \
  --exclude 'log/' \
  "${WIN_SRC}" "${WS_SRC}"

echo ""
echo "Synced -> ${WS_SRC}"
echo "Next:  cd ${WS_ROOT} && colcon build --symlink-install && source install/setup.bash"

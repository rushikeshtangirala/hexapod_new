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

# ---------------------------------------------------------------------------
# Mirror tools/ as well.
#
# The Windows path contains a space ("Rushi Tangirala"). Every command that
# references it therefore needs quoting, and an unquoted use word-splits into
# nonsense like:
#     bash: /walk_free.sh: No such file or directory
# That is a persistent source of friction for no benefit. Copying the scripts
# to ~/hexapod_ws/tools/ gives them a space-free path, so every command is
# simply:
#     bash ~/hexapod_ws/tools/<script>.sh
# ---------------------------------------------------------------------------
WIN_TOOLS="$(dirname "${BASH_SOURCE[0]}")/tools/"
WS_TOOLS="${WS_ROOT}/tools/"

if [ -d "${WIN_TOOLS}" ]; then
  mkdir -p "${WS_TOOLS}"
  rsync -a --delete \
    --exclude '__pycache__/' \
    --exclude '*.pyc' \
    "${WIN_TOOLS}" "${WS_TOOLS}"
  chmod +x "${WS_TOOLS}"*.sh 2>/dev/null || true
  echo "Tools  -> ${WS_TOOLS}"
fi

# ---------------------------------------------------------------------------
# XML LINT GATE
#
# Run automatically, every sync, because the failure it catches is cheap to
# make and expensive to diagnose. XML forbids a double hyphen inside a comment
# body, and xacro reports the error against the FULLY EXPANDED output, which
# for a six-leg macro bears no resemblance to the file you edited.
#
# Enforcing this in the tooling rather than in a checklist is the point: a
# rule that depends on remembering to run it is not a rule, it is a hope.
# ---------------------------------------------------------------------------
LINT="${WS_TOOLS}check_xml.py"

if [ -f "${LINT}" ]; then
  echo ""
  if ! python3 "${LINT}" "${WS_SRC}"; then
    echo ""
    echo "=============================================================" >&2
    echo " SYNC COMPLETED, BUT XML LINT FAILED. Do not build yet." >&2
    echo " Fix the files listed above on the WINDOWS side, then re-run." >&2
    echo "=============================================================" >&2
    exit 2
  fi
else
  echo "NOTE: ${LINT} not found, skipping XML lint."
fi

echo ""
echo "Next:  cd ${WS_ROOT} && colcon build --symlink-install && source install/setup.bash"

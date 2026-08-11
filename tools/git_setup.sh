#!/usr/bin/env bash
# =============================================================================
# git_setup.sh : commit the project and prepare it for sharing.
#
# WHY A SCRIPT RATHER THAN COMMANDS TO PASTE
# The project lives at a Windows path containing a SPACE ("Rushi Tangirala").
# Unquoted, that path splits into two arguments and every command fails with
# a confusing "No such file or directory". That has already cost this project
# one debugging cycle. The path is quoted exactly once, here.
#
# RUN THIS FROM UBUNTU (WSL):
#     bash ~/hexapod_ws/tools/git_setup.sh
# =============================================================================
set -o pipefail

HEX="/mnt/c/Users/Rushi Tangirala/OneDrive/Desktop/hexapod_scratch"

if [ ! -d "${HEX}" ]; then
  echo "ERROR: project folder not found at:" >&2
  echo "  ${HEX}" >&2
  exit 1
fi

cd "${HEX}" || exit 1
echo "Working in: $(pwd)"
echo ""

# A crashed or interrupted git leaves this behind and blocks every later
# command with "Another git process seems to be running".
if [ -f .git/index.lock ]; then
  echo "Removing stale git lock..."
  rm -f .git/index.lock
fi

if [ ! -d .git ]; then
  echo "Initialising repository..."
  git init -q
fi

git config user.email "hexapod69420@gmail.com"
git config user.name  "Varun"

echo "Staging..."
git add -A

if git diff --cached --quiet; then
  echo "Nothing to commit; working tree already clean."
else
  git commit -q -F - <<'MSG'
Knee-down IK branch matching CAD, servo friction model, automated walk test

GEOMETRY / KINEMATICS
- Inverse kinematics now selects the POSITIVE law-of-cosines root, so the
  femur reaches outward (15.4 deg) and the tibia hangs down (+67.8 deg).
  This matches the CAD assembly; the previous negative root folded the leg
  the opposite way, with the femur vertical and the tibia nearly horizontal.
- Consequence: the horizontal lever from knee to foot falls from 136 mm to
  18 mm, so knee torque falls from 1.48 to 0.19 N*m for the same stance.
  The measured 24.3 deg tibia sag disappears and the body holds its design
  height of 0.194 m.
- femur limit returns to 90 deg (was 120). The offset servo-horn mounting
  previously required at assembly is no longer needed.
- tibia limits set to -60..130 deg to permit the correct branch.
- Joint limits were duplicated in kinematics.py and common_properties.xacro
  and had drifted apart. Both corrected, with a cross-reference note.

PHYSICS MODEL
- joint_friction 0.01 -> 1.5 N*m, modelling geared-servo stiction. A real
  ~200:1 geared servo is close to non-backdriveable and holds position when
  unpowered; the model previously used frictionless pivots, so the legs
  folded in the window before the controllers activate. Measured sag fell
  from 10.3 deg to 0.4 deg.
- joint_effort 5.0 -> 7.0 N*m to cover friction plus stance load plus the
  torque needed to accelerate a swinging leg.

STARTUP ORDERING
- Gazebo no longer starts paused. A paused world never calls
  controller_manager::update(), and controller switches are applied inside
  that update, so activations requested during the pause were silently
  discarded. joint_state_broadcaster stayed inactive, /joint_states never
  published, and the gait node had no start pose.
- gait_node now blocks until a complete /joint_states arrives and refuses to
  publish without one, rather than issuing a step command from an unknown
  pose and catapulting the robot.

TOOLING
- tools/autotest.sh + tools/measure_walk.py: headless end-to-end walk test.
  Drives the robot itself and scores odometry, leg swing, stride and
  per-joint tracking, then prints a verdict. Replaces a human watching a
  Gazebo window and describing what they saw, which could not distinguish
  drift, slip, ejection and a correct slow walk.
- tools/walk_free.sh: added status, unpause, activate, sim-effort and
  gait-effort subcommands.

MEASURED STATUS
- position mode: holds design height 0.194 m, zero tilt, exact joint
  tracking (shortfall 0.0000 rad), but cannot translate. Gazebo realises
  position commands with SetPosition(), which relocates a joint without a
  matching velocity; contact friction is computed from sliding velocity, so
  the solver sees a stationary foot and produces no propulsion. Structural,
  not a tuning problem.
- effort mode: has walked 1.1 m in 20 s, but is not yet stable. Gains are
  still those tuned for the previous leg geometry.
MSG
  echo "Committed."
fi

echo ""
echo "History:"
git log --oneline | head -5
echo ""
echo "Tracked files: $(git ls-files | wc -l)"
echo ""
echo "============================================================"
echo " TO SHARE WITH YOUR FRIEND"
echo "============================================================"
echo "1. Create an EMPTY repository on github.com (no README, no"
echo "   .gitignore -- this project already has both)."
echo ""
echo "2. Then run, replacing USER and REPO:"
echo ""
echo "     cd \"${HEX}\""
echo "     git remote add origin https://github.com/USER/REPO.git"
echo "     git branch -M main"
echo "     git push -u origin main"
echo ""
echo "3. Add your friend under Settings -> Collaborators."
echo ""
echo "They clone it, then build with:"
echo "     colcon build --symlink-install && source install/setup.bash"
echo "============================================================"

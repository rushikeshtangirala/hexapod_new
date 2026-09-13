#!/usr/bin/env bash
# =============================================================================
# git_setup.sh : commit the project, and optionally push it to GitHub.
#
# USAGE
#   bash ~/hexapod_ws/tools/git_setup.sh
#       Stage and commit everything. Safe to run repeatedly.
#
#   bash ~/hexapod_ws/tools/git_setup.sh https://github.com/USER/REPO.git
#       Commit, attach that remote, and push to main.
#
#   bash ~/hexapod_ws/tools/git_setup.sh https://github.com/USER/REPO.git "my message"
#       As above with your own commit message.
#
# WHY A SCRIPT RATHER THAN COMMANDS TO PASTE
# The project lives at a Windows path containing a SPACE ("Rushi Tangirala").
# Unquoted, that path splits into two arguments and every command fails with a
# confusing "No such file or directory". That has already cost this project one
# debugging cycle. The path is quoted exactly once, here.
#
# WHY THE REPOSITORY LIVES ON THE WINDOWS SIDE
# sync_to_wsl.sh mirrors Windows to WSL with rsync --delete. The WSL copy is
# DERIVED and anything committed there would be destroyed by the next sync.
# The Windows folder is the source of truth, so it is the only correct place
# for the repository.
# =============================================================================
set -o pipefail

HEX="/mnt/c/Users/Rushi Tangirala/OneDrive/Desktop/hexapod_scratch"
REMOTE_URL="${1:-}"
MESSAGE="${2:-}"

say()  { echo ""; echo "=============================================="; \
         echo " $1"; echo "=============================================="; }
die()  { echo ""; echo "FAILED: $1" >&2; exit 1; }

[ -d "${HEX}" ] || die "project folder not found at:
  ${HEX}"

cd "${HEX}" || die "cd into the project folder"
echo "Working in: $(pwd)"

# ---------------------------------------------------------------------------
say "1/5  clearing anything a previous crash left behind"
# ---------------------------------------------------------------------------
# This folder is inside OneDrive. OneDrive can hold a file open mid-sync while
# git is writing it, which aborts the write and leaves a partial object or a
# lock behind. Every later git command then fails with a message that has
# nothing to do with the real cause. Clearing these first costs nothing and
# removes a whole category of confusing failure.
#
# If git errors persist, pause OneDrive syncing, run this again, then resume.
if [ -f .git/index.lock ]; then
  echo "  removing stale index.lock"
  rm -f .git/index.lock
fi
find .git -name 'tmp_obj_*' -delete 2>/dev/null
rm -f .git/testwrite 2>/dev/null
echo "  clean"

# ---------------------------------------------------------------------------
say "2/5  repository and identity"
# ---------------------------------------------------------------------------
if [ ! -d .git ]; then
  echo "  initialising"
  git init -q || die "git init"
else
  echo "  already a repository"
fi

git config user.email "hexapod69420@gmail.com"
git config user.name  "Varun"

# Windows and Linux disagree about line endings, and this tree is edited from
# both. Without this, every file looks modified to whichever side did not
# write it last, and diffs become unreadable.
git config core.autocrlf input

# ---------------------------------------------------------------------------
say "3/5  checking for files GitHub will reject"
# ---------------------------------------------------------------------------
# GitHub warns above 50 MB and refuses above 100 MB. Meshes are the only
# plausible offenders here. Better to find out now than halfway through a push
# that then has to be unwound with a history rewrite.
BIG=0
while IFS= read -r -d '' f; do
  sz=$(stat -c%s "$f" 2>/dev/null || echo 0)
  mb=$(( sz / 1048576 ))
  if [ "${sz}" -gt 104857600 ]; then
    echo "  TOO LARGE (${mb} MB, hard limit 100): $f"
    BIG=1
  elif [ "${sz}" -gt 52428800 ]; then
    echo "  large (${mb} MB, GitHub will warn): $f"
  fi
done < <(find . -path ./.git -prune -o -type f -print0)

if [ "${BIG}" -eq 1 ]; then
  die "at least one file exceeds GitHub's 100 MB hard limit.
Either remove it, or install Git LFS and track it:
  git lfs install
  git lfs track '*.stl'
  git add .gitattributes"
fi
echo "  nothing over the limit"

# ---------------------------------------------------------------------------
say "4/5  commit"
# ---------------------------------------------------------------------------
git add -A || die "git add"

if git diff --cached --quiet; then
  echo "  nothing to commit, working tree already clean"
else
  if [ -n "${MESSAGE}" ]; then
    git commit -q -m "${MESSAGE}" || die "git commit"
  else
    git commit -q -m "Project state $(date +%Y-%m-%d)" -m \
"Checkpoints 1 to 6 and 8 complete.

MODEL
- Visual meshes sit on their correct links. The earlier coxa/tibia
  visual exchange in leg_macro.xacro has been removed: coxa.stl is
  the forked clevis at the body, tibia.stl the truss at the foot.
- hexapod.sensors.xacro adds IMU, forward camera and lidar, each
  behind its own flag. Camera and lidar default off because
  rendering roughly halves the real time factor.

BUG: SPAWN HEIGHT AND WORLD ANCHOR WERE ADDING
- With fix_base true, spawn_entity -z placed the whole model and the
  world weld then applied on top, so the body sat at 0.16 + 0.132 =
  0.292 m with the feet 0.16 m clear of the ground. Every anchored
  run so far was cycling its legs in mid air.
- Found by the stem detector: it projected a target of known
  position onto the ground and reported 0.51 m against a true
  1.00 m, which solves back to a camera height of 0.295 m.
- Fixed by spawning at zero whenever the base is welded.

CONTROL
- Effort gains reduced, not raised. Swing inertia at the femur is
  about 2.5e-3 kg m^2, so p = 1500 put the closed loop at 123 Hz
  against a 200 Hz control rate: under two samples per period. p is
  now 120 to 160 and the integral term carries the constant gravity
  load instead.

PERCEPTION
- New package hexapod_vision. stem_detector finds green stems by
  hue and aspect ratio and recovers range by intersecting the pixel
  ray with the ground plane, which needs only the stem base to be
  visible.
- hexapod.world gains three green stems at known positions plus a
  red decoy that tests hue discrimination. The ten side markers are
  removed.

VERIFIED
- verify_ik.py 32/32, verify_gait.py 498/498.
- tools/check_vision.sh checks checkpoint 8 end to end in one
  command and compares against ground truth.

OPEN
- Free base walking under effort control (Gate 4).
- Checkpoints 7, 9, 10, 11." || die "git commit"
  fi
  echo "  committed"
fi

echo ""
echo "  history:"
git log --oneline | head -5 | sed 's/^/    /'
echo "  tracked files: $(git ls-files | wc -l)"

# ---------------------------------------------------------------------------
say "5/5  remote"
# ---------------------------------------------------------------------------
# If no URL was given but origin already exists, push to it. Requiring the
# URL every single time turns routine "share my progress" into a trip to the
# browser to copy a string git already knows.
if [ -z "${REMOTE_URL}" ] && git remote | grep -qx origin; then
  REMOTE_URL=$(git remote get-url origin)
  echo "  using the existing origin: ${REMOTE_URL}"
fi

if [ -z "${REMOTE_URL}" ]; then
  cat <<'EOF'
  No remote URL given and no origin is set, so nothing was pushed.

  TO PUT THIS ON GITHUB
  ---------------------
  1. Sign in at github.com and create a NEW EMPTY repository.
     Do NOT tick "Add a README" or "Add .gitignore": this project
     already has both, and an initialised remote causes a rejected
     push that then needs a merge to untangle.

  2. Copy the HTTPS URL it shows you, then run this script again
     with that URL as the argument:

       bash ~/hexapod_ws/tools/git_setup.sh https://github.com/USER/REPO.git

  3. Settings -> Collaborators -> add your friend, so they can push
     as well as read.
EOF
  exit 0
fi

if git remote | grep -qx origin; then
  git remote set-url origin "${REMOTE_URL}"
  echo "  origin updated to ${REMOTE_URL}"
else
  git remote add origin "${REMOTE_URL}"
  echo "  origin added: ${REMOTE_URL}"
fi

git branch -M main
echo ""
echo "  pushing. GitHub will ask for your username and a PERSONAL ACCESS"
echo "  TOKEN. Your account password will NOT work: GitHub stopped"
echo "  accepting passwords over HTTPS in 2021. Create a token at"
echo "  github.com -> Settings -> Developer settings -> Personal access"
echo "  tokens -> Tokens (classic), with the 'repo' scope ticked."
echo ""

git push -u origin main || die "git push.
If it was rejected as non-fast-forward, the GitHub repository was created
with a README. Either delete and recreate it empty, or run:
  git pull --rebase origin main && git push -u origin main"

say "DONE"
echo "Your friend clones and builds with:"
echo ""
echo "  git clone ${REMOTE_URL} hexapod_ws"
echo "  cd hexapod_ws"
echo "  colcon build --symlink-install"
echo "  source install/setup.bash"
echo "  python3 tools/verify_ik.py && python3 tools/verify_gait.py"
echo ""
echo "Those two verifiers should print 32/32 and 498/498 on their machine"
echo "before they change anything. If they do not, the clone is at fault,"
echo "not the edit they are about to make."

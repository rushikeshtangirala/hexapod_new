# Hexapod — 18-DOF Autonomous Walking Robot

Final-year Mechanical Engineering capstone. ROS 2 Humble / Gazebo Classic simulation stack,
built from our own CAD, targeting later hardware integration.

---

## 1. Environment

| Item | Value |
|---|---|
| Host OS | Windows 11 |
| Dev OS | Ubuntu 22.04 LTS under WSL2 |
| ROS 2 | Humble Hawksbill |
| Simulator | Gazebo Classic 11 |
| Control | ros2_control / gazebo_ros2_control |
| Languages | Python 3.10, C++17 |
| CAD source | AutoCAD → STL, one file per link |

## 2. Where the code lives

This is deliberately split across two filesystems.

```
Windows (this folder)                      WSL2 (native ext4)
C:\...\Desktop\hexapod_scratch\            ~/hexapod_ws/
├── src/            ── rsync ──────────>   ├── src/          (mirror, never edit here)
├── sync_to_wsl.sh                         ├── build/        (never leaves WSL)
├── .gitignore                             ├── install/      (never leaves WSL)
└── README.md                              └── log/          (never leaves WSL)
```

**Edit only on the Windows side. Build only on the WSL side.**

Reason: `/mnt/c` is exposed to WSL2 over the 9p protocol. colcon issues tens of thousands
of small filesystem calls, which is 5–20× slower there. Separately, `--symlink-install`
creates POSIX symlinks that DrvFs handles inconsistently, and OneDrive actively rewrites
files it is syncing — which corrupts `build/` and `install/`.

### Workflow

```bash
# one-time, in ~/.bashrc
alias hexsync="bash '/mnt/c/Users/Rushi Tangirala/OneDrive/Desktop/hexapod_scratch/sync_to_wsl.sh'"
alias hexbuild="cd ~/hexapod_ws && hexsync && colcon build --symlink-install && source install/setup.bash"

# every edit cycle
hexbuild
```

## 3. Package architecture

Six packages. The split is not bureaucracy — each boundary exists to keep one kind of
change from forcing a rebuild or a redesign of something unrelated.

| Package | Build type | Contains | Why it is separate |
|---|---|---|---|
| `hexapod_description` | ament_cmake | URDF/Xacro, meshes, RViz configs | The robot model must be usable by sim, hardware, MoveIt and visualisation **without dragging in a simulator dependency**. Data only, no code. |
| `hexapod_gazebo` | ament_cmake | Worlds, physics params, spawn launch | Simulation-only. Deleting this package must leave a stack that still runs on real hardware. |
| `hexapod_control` | ament_cmake | controller_manager YAML, controller launch | Controller configuration is swapped wholesale between sim and hardware; isolating it makes that a one-line change. |
| `hexapod_bringup` | ament_cmake | Composed top-level launch files | Single entry point. Nothing depends on it, so it can change freely. |
| `hexapod_teleop` | ament_python | Keyboard teleop node | *Created in Phase 7.* |
| `hexapod_gait` | ament_python | IK + tripod gait generator | *Created in Phase 8.* |

Dependency direction (arrows point at dependencies — note there are no cycles):

```
        hexapod_bringup
        /      |       \
       v       v        v
 hexapod_  hexapod_  hexapod_
 gazebo    control    gait
       \       |       /
        v      v      v
      hexapod_description
```

`hexapod_description` depends on nothing. That is the property that makes the whole
stack portable to hardware in Phase 9.

## 4. Roadmap — 10 working days

| Day | Phase | Milestone | Gate to pass |
|---|---|---|---|
| 1 | 0–1 | Toolchain verified, workspace + 4 packages build clean | `colcon build` returns 4/4, `ros2 pkg list` shows all |
| 2 | 2a | STL audit: units, origins, triangle count, watertightness | Every mesh loads, scale is known, origins documented |
| 3 | 2b | Parametric leg macro + 6-leg body, 18 joints | TF tree complete in RViz2, sliders move all 18 joints |
| 4 | 3 | Mass / COM / inertia tensors, collision primitives, friction | `check_urdf` clean, no negative or non-physical inertias |
| 5 | 4 | Robot spawns and stands in Gazebo | Stands ≥ 60 s without drift, jitter or explosion |
| 6 | 5 | ros2_control + gazebo_ros2_control, controllers spawn | `ros2 control list_controllers` all **active** |
| 7 | 6 | All 18 joints individually commanded and verified | Sign/axis map table filled in, `/joint_states` tracks commands |
| 8 | 7 | Analytic leg IK + keyboard teleop | IK↔FK round-trip error < 1e-6 m; foot reaches commanded points |
| 9 | 8 | Tripod gait walking in Gazebo | Body translates ≥ 0.5 m/s command tracking, no leg collisions |
| 10 | 9 | Hardware interface stub, docs, demo recording | Sim/real launch switch works; report material complete |

**Deadline discipline:** Days 1–7 are the critical path to a *controllable robot*.
Days 8–10 are the demo. If a day slips, cut scope from Phase 9, never from Phase 3 —
bad inertias will silently poison every phase after them.

## 5. Verification gates

No phase is "done" because the code was written. A phase is done when its gate in the
table above passes and the result is committed. Each phase ends with:

1. A stated verification command and its expected output
2. A git commit with a conventional-commit message
3. A note in `docs/` recording anything non-obvious that was decided

## 6. Git

```bash
git init
git add .
git commit -m "chore: scaffold ROS 2 workspace and package architecture"
```

`build/`, `install/` and `log/` are gitignored. They live in WSL and are never committed.

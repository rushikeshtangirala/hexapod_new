# Runbook

Backup branch before the repo-mirror work: `pre-repo-mirror-2026-08-12`.
`git checkout pre-repo-mirror-2026-08-12` returns everything.

---

## The three combinations that matter

| | `control_mode` | `fix_base` | `sim_profile` | What it is for |
|---|---|---|---|---|
| A | position | true | physical | Leg motion only. Body welded, so it CANNOT travel. Use it to check leg direction and IK, nothing else. |
| B | position | **false** | physical | **Try this first.** Free base on the position interface, with the stance sign bug now fixed. |
| C | effort | false | **repo** | The hexapod_ros mirror. Non-physical model, stable by construction. |

---

## Try B first. It costs nothing and the old conclusion about it is void.

`ARCHITECTURE.md` used to record that position + free base drifts and does not
travel. That conclusion was reached **while the stance was sweeping the feet the
wrong way**, dragging every planted foot across the ground at twice body speed
in the wrong direction. Any conclusion drawn from that run is worthless,
including this one.

Position commands do teleport the joint, which is a real objection and still
true. But a teleport is only violent in proportion to how far the joint has to
move, and with a correct gait the per-tick movement is small and smooth. It may
simply work now.

```bash
cd ~/hexapod_ws && hexbuild
ros2 launch hexapod_bringup hexapod_sim.launch.py control_mode:=position fix_base:=false
# second terminal
ros2 launch hexapod_bringup walk.launch.py control_mode:=position
# third terminal
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Tap `i` once or twice. Do not hold it.

---

## C: the hexapod_ros mirror

```bash
cd ~/hexapod_ws && hexbuild
ros2 launch hexapod_bringup hexapod_sim.launch.py \
    control_mode:=effort fix_base:=false sim_profile:=repo
# second terminal
ros2 launch hexapod_bringup walk.launch.py control_mode:=effort
# third terminal
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

`sim_profile:=repo` changes, and only changes:

* every link mass to 1e-5 kg
* every inertia tensor to identity
* joint limits to plus/minus pi
* joint effort 6.0 N m, velocity 0.524 rad/s
* joint damping and friction to 0
* controller gains to p=100, i=0, d=0 (`controllers_repo.yaml`, selected
  automatically — the gains and the masses are one decision, not two)

Leave `sim_profile` off and the measured model is back, byte for byte.

### What repo mode is and is not

It is a way to get a hexapod walking on screen. It is stable because a 1e-5 kg
link with a 1.0 kg m^2 inertia tensor is roughly 100,000 times too reluctant to
rotate for anything to go wrong, and because gravity torque at that mass is
negligible so p=100 has thousands of times the authority it needs.

It is not a simulation you can quote. No torque, contact force, servo
requirement, stability margin or power figure from repo mode means anything
about the real machine. Keep every such number in the report sourced from
`sim_profile:=physical`, and say in the write-up which figures came from which
profile. That distinction is itself a good result to report.

---

## Checks, in the order worth running them

```bash
# maths, no ROS needed, fast
python3 tools/verify_ik.py            # expect 32 passed, 0 failed
python3 tools/verify_gait.py          # expect 498 passed, 0 failed

# is the model what you think it is
ros2 param get /robot_state_publisher robot_description | grep -m3 "mass value"

# is anything actually listening
ros2 control list_controllers
ros2 topic info /leg_trajectory_controller/joint_trajectory     # effort mode
ros2 topic info /leg_position_controller/joint_trajectory       # position mode

# does it travel
python3 tools/measure_walk.py --speed 0.03 --drive 20

# if it wanders while standing, with the gait node stopped
bash tools/check_drift.sh
```

The gait node now logs `Controller is listening on <topic>` at startup, and a
loud error naming the topic if nothing is subscribed after 5 seconds. If you see
that error, the controller did not spawn or the modes disagree; nothing else in
the system will tell you.

---

## Reading the outcomes

**It walks.** Record which profile. If it was C, the open question is what
`physical` needs to match it, and that is a gains problem with a known-good
reference to compare against — much easier than tuning blind.

**It walks but veers or crawls.** Gait geometry or gains, not structure. Send
`measure_walk.py` output.

**It flies off.** Almost always the first command being large: the gait node
prints a measured-pose vs stance-target table at startup and flags any joint
delta over 0.05 rad. Look there before anything else.

**It sits still with a subscriber connected.** The commands are being sent and
ignored. Check `ros2 control list_controllers` shows `active`, not `inactive`.

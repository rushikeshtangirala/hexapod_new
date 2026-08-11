# How this project works

Written to be read top to bottom. Start with the data flow; everything else
is detail hanging off it.

---

## 1. The one diagram that matters

```
  YOU  (keyboard)
   |
   |  teleop_twist_keyboard
   v
  /cmd_vel                        geometry_msgs/Twist
   |                              "move forward at 0.06 m/s, turn at 0.2 rad/s"
   v
  hexapod_gait_node               50 Hz timer
   |
   |   1. advance the gait phase clock by the measured dt
   |   2. for each of 6 legs, work out where its foot should be NOW
   |   3. run inverse kinematics: foot position -> 3 joint angles
   |
   v
  /leg_position_controller/commands      18 numbers, radians
   |
   v
  controller_manager               200 Hz, INSIDE gzserver
   |
   v
  hardware interface  (gazebo_ros2_control)
   |
   v
  GAZEBO PHYSICS                   1000 Hz
   |
   +--> joint_state_broadcaster --> /joint_states --> robot_state_publisher
   |                                                        |
   |                                                        v
   |                                                       /tf  (RViz, TF lookups)
   |
   +--> p3d plugin --------------> /odom     ground-truth body pose
```

Two things to notice.

**The velocity command is the only input.** Everything downstream is derived.
There is no scripted animation and no recorded motion; ask for a different
velocity and the whole leg trajectory changes.

**Feedback exists but is not used for control.** `/joint_states` drives
visualisation, `/odom` is for measurement. The gait is open loop. That is
normal for a first hexapod and it is why the gait had to be provably correct
before it ever ran.

---

## 2. Packages, and why the split exists

| Package | Contains | Why separate |
|---|---|---|
| `hexapod_description` | URDF/Xacro, meshes, RViz config | The robot model must be usable by simulation, hardware and visualisation **without depending on a simulator**. Data only, no code. |
| `hexapod_gazebo` | World file, physics tuning | Simulation only. Delete it and the rest still runs on real hardware. |
| `hexapod_control` | Controller YAML | Controller configuration is swapped wholesale between sim and hardware. |
| `hexapod_gait` | Kinematics, gait, the ROS node | The mathematics, deliberately separable from ROS. |
| `hexapod_bringup` | Composed launch files | Single entry point. Nothing depends on it, so it can change freely. |

The dependency graph has no cycles, and `hexapod_description` depends on
nothing. That single property is what makes Phase 9 hardware integration a
configuration change rather than a rewrite.

---

## 3. The robot model

### `common_properties.xacro`
Every dimension, mass, limit and gain, declared once.

Why it exists: in a naive URDF the femur length appears four times per leg
across six legs. That is 24 places to edit and 24 chances to miss one, and a
miss does not error, it silently makes the kinematics disagree with the
appearance.

Also holds the closed-form inertia macros. A box and a cylinder have
analytically exact inertia tensors, so the physics is exact rather than
estimated.

### `leg_macro.xacro`
One 3-DOF leg, instantiated six times.

```
base_link
  -> coxa joint   (revolute about +Z)  yaw: swings the leg
    -> coxa link   0.150 m
      -> femur joint (revolute about +Y)  pitch: lifts and lowers
        -> femur link  0.117 m
          -> tibia joint (revolute about +Y)  pitch: extends
            -> tibia link  0.150 m
              -> foot (fixed)   sphere, r = 0.015
```

**Every segment extends along its own local +X.** One rule, no special cases.

**Positive femur and tibia angles move the foot DOWN.** Rotation about +Y maps
+X toward -Z. We kept the mathematically clean right-handed axis and
documented the consequence rather than flipping the axis, because a
left-handed frame breaks every standard rotation formula.

**There is no mirroring.** All six legs use the same macro; the mount yaw
rotates each chain into place. Left and right differ only in the sign of `y`
and `yaw`. Mirroring joint axes instead is the classic hexapod mistake that
makes three legs bend backwards.

**Visual is the CAD mesh; collision and inertia are primitives.** Contact is
evaluated pairwise every physics step across 25 links, so a 9,572-triangle
body mesh there would cost three orders of magnitude more than a box and buy
nothing. The body mesh is not watertight either, so its inertia is undefined,
which makes the box formula not an approximation but the better number.

### `hexapod.ros2_control.xacro`
Declares what each joint can accept and report. The `<plugin>` line is the
only thing Phase 9 changes.

### `hexapod.gazebo.xacro`
Ground-truth odometry. Without it there is no way to tell a robot that is
walking from one that is cycling its legs in place, because RViz's fixed
frame is the body itself.

---

## 4. The control pipeline

**ros2_control** defines one interface between "something that can move
joints" and "something that decides where joints go":

```
CONTROLLERS  (position, trajectory, gait)
      |   command interfaces / state interfaces
      v
RESOURCE MANAGER
      |
HARDWARE INTERFACE     <-- the only swappable part
      |
  +---+---+
Gazebo    Real servos
```

* **command interface** — what a controller may write. Position (an angle)
  or effort (a torque).
* **state interface** — what it may read. Position, velocity, effort.
* **controller_manager** — the real-time loop. Reads all state, runs each
  active controller's `update()`, writes all commands. Nothing else touches
  the hardware.

In simulation the controller_manager runs **inside gzserver**, created by the
`gazebo_ros2_control` plugin. That is deliberate: the control loop must run
in lockstep with physics, not asynchronously over the network. It is also why
you never run `ros2 run controller_manager ros2_control_node` in simulation.

### Position versus effort, and the one real limitation

`position` commands are implemented with Gazebo's `SetPosition()`, which
**teleports** the joint rather than applying the torque that would move it.
Fine for a bolted-down arm. For a floating base standing on six frictional
contacts, each control cycle hands the solver a configuration it did not
produce, and the residual becomes a small impulse on the free body. At 200 Hz
that integrates into slow drift.

This was established experimentally, not assumed:

* base anchored, gait running, feet loaded -> completely stable
* no controller writing to any joint -> robot collapses and stays put

So contact, friction, inertia and the solver are all sound. Only "floating
base + position commands" drifts.

`effort` commands are torques. The engine integrates them and produces
contact reactions consistent by construction, so no spurious momentum
appears. It is also the more honest model of a real servo, which has finite
torque, droops under load, and takes time to converge. That path is
implemented (`control_mode:=effort`) and its PID gains are still being tuned.

---

## 5. The mathematics

`kinematics.py` and `gait.py` have **no ROS dependency**. That is the single
most useful decision in the project: it let both be proven correct with a
plain Python script while the simulator was still misbehaving.

### Inverse kinematics

Because the femur and tibia rotate about the same axis direction, everything
past the coxa lies in one vertical plane, and the coxa yaw only selects which
plane. A 3-D problem collapses to 2-D, which is the only reason a closed-form
solution exists.

1. **Coxa** — the only joint producing lateral motion: `t1 = atan2(y, x)`
2. **Reduce** — move the origin to the femur joint:
   `r' = sqrt(x^2+y^2) - L1`, `D = sqrt(r'^2 + z'^2)`
3. **Tibia** — law of cosines, interior angle at the knee is `pi - t3`:
   `cos(t3) = (D^2 - L2^2 - L3^2) / (2*L2*L3)`
4. **Femur** — `t2 = atan2(-z', r') + acos((L2^2 + D^2 - L3^2)/(2*L2*D))`

`acos` returns `[0, pi]`, so there are two solutions: elbow-up and
elbow-down, both reaching the same point. We take the **negative** root, the
insect-like knee-up configuration. The other is mechanically valid but the
knee collides with the ground during swing.

Verified by `tools/verify_ik.py`: round-trip FK -> IK -> FK over a
joint-space grid, comparing **foot positions**, not angles, because the
two-solution ambiguity means correct code can legitimately return different
angles. Worst error 3.5e-16 m, i.e. floating-point noise.

### The tripod gait

Two constraints, simultaneously.

**Static stability.** The centre of mass must project inside the polygon
formed by the feet on the ground. Three non-collinear points is the minimum,
so a hexapod can lift three legs at once. Stability is a *geometric* property
you can check, not a dynamic one you must control — which is why hexapods are
far easier than bipeds.

```
Tripod A = { lf, rm, lr }        Tripod B = { rf, lm, rr }
```

Alternating sides, exactly half a cycle apart, duty factor 0.5, so exactly
three feet are down at all times.

**Non-slip.** A foot in contact must not move relative to the *ground*. But we
command positions in the *body* frame, and the body is moving, so a
stationary foot must travel backwards through the body frame at exactly the
body's speed. Get this wrong and the legs cycle while the robot goes nowhere.

This makes stride length **derived, not tuned**:

```
stride_i = -(vx - w*p_iy,  vy + w*p_ix) * stance_duration
```

Each leg gets a different stride when turning, because a point further from
the turn centre travels further. That one expression makes forward walking,
strafing and turning the same code path.

**Swing profile.** Horizontal is a cycloid, vertical a raised cosine. Both
have zero derivative at both endpoints, so the foot lifts without jerking the
body and **touches down with zero horizontal velocity**. A naive linear swing
lands at full stride speed: the foot skids, the body yaws randomly, and no
gait tuning fixes it because the problem is the profile. Contact is the
expensive event in legged locomotion.

Verified by `tools/verify_gait.py`: exactly three feet down throughout,
trajectory continuity, near-zero touchdown velocity, and stance travel
matching body travel.

---

## 6. The stance, and why the numbers are what they are

Foot 0.28 m out, 0.18 m below the coxa axis. Femur +92.8 degrees, tibia -67.8.

The femur passes **vertical**, which is why `femur_upper` is 120 degrees and
not 90. With a 0.150 m coxa the femur joint sits well outboard, so putting a
foot underneath the robot requires swinging past vertical. Capped at 90, the
leg extends almost straight sideways: the tibia ends up horizontal, the feet
stick out, and ground clearance is minimal.

**This implies a build requirement**: the femur servo horn must be mounted
with an offset so its 180 degrees of travel maps to roughly -60 to +120 in
this frame. That is a simulation result driving a mechanical decision, which
is exactly the kind of finding the report wants.

---

## 7. Tooling, and why each exists

| Tool | Catches |
|---|---|
| `check_xml.py` | Double hyphens inside XML comments, which are illegal and produce a parse error against the *expanded* output. Runs automatically inside `hexsync`. |
| `mesh_audit.py` | STL units, mesh origins, watertightness. STL carries no unit metadata at all. |
| `compute_inertia.py` | Inertia tensors from mesh geometry, validated for positive-definiteness and the triangle inequality. |
| `reorigin_meshes.py` | Bakes the layout-sheet offset into the geometry, so URDF origins are zero. |
| `verify_ik.py` | IK correctness, offline, no ROS. |
| `verify_gait.py` | Gait stability and non-slip, offline, no ROS. |
| `diagnose.sh` | Build-time prerequisites, in dependency order. |
| `diagnose_runtime.sh` | Live system state. |
| `check_drift.sh` | Separates "body sliding on feet" from "joints moving". |
| `reset_sim.sh` | Stale gzserver holding port 11345. |
| `fix_display.sh` | WSLg X socket, the cause of GUI windows never appearing. |

The pattern throughout: **make the failure mode measurable rather than
guessable.** Most of the time lost on this project was spent guessing between
causes that produce identical symptoms.

---

## 8. Current status

| Objective | State |
|---|---|
| Import CAD into ROS 2 | done |
| URDF/Xacro from own CAD | done |
| Joints and kinematics | done |
| Collision, mass, COM, inertia, friction, damping | done |
| Spawn in Gazebo | done |
| ros2_control | done |
| Controllers | done |
| All 18 joints verified | done |
| Keyboard teleoperation | done |
| Tripod gait | done, verified offline and running under load |
| Hardware preparation | architecture in place, plugin stub outstanding |

Open item: body translation with a free base. Cause identified and proven,
fix implemented (`control_mode:=effort`), gains still to tune.

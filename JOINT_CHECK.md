# Joint verification checklist

A static screenshot cannot tell you whether a joint is correct. A leg can look
right in its stance pose and still rotate about the wrong axis, in the wrong
direction, or drag the wrong child links with it. The only reliable test is to
move **one joint at a time** and watch what happens.

Ten minutes, and it settles every orientation question.

## Setup

```bash
hexsync && hexbuild
ros2 launch hexapod_description display.launch.py
```

RViz plus the Joint State Publisher slider panel. No physics, so nothing falls
over while you look. Set RViz's Fixed Frame to `base_link` and tick **TF** with
**Show Axes** on, so you can see each joint frame.

Work on the **front-left leg (`lf`)** only. If it is right, all six are: they
come from one macro.

---

## 1. Coxa

Move `lf_coxa_joint` from its lower limit to its upper limit.

| Check | Expected |
|---|---|
| Axis of rotation | Vertical. The leg sweeps **horizontally**, like a door. |
| What moves | The whole leg: coxa, femur, tibia and foot together. |
| Positive direction | Foot swings **counter-clockwise seen from above** (towards the robot's front on the left side). |
| Foot height | **Unchanged.** If the foot rises or falls, the axis is wrong. |
| Body | Does not move. |

Foot height staying constant is the sharpest test here: a vertical axis cannot
change it. Any change means the axis is not `0 0 1`.

## 2. Femur

Return the coxa to 0. Move `lf_femur_joint`.

| Check | Expected |
|---|---|
| Axis of rotation | Horizontal, perpendicular to the leg. The leg swings in a **vertical plane**. |
| What moves | Femur, tibia and foot. The **coxa stays still**. |
| Positive direction | Foot moves **downward**. |
| Sideways motion | **None.** The foot stays in the same vertical plane. |

Positive-is-down follows from the `+Y` axis and the right-hand rule. It is
counter-intuitive and it is the number one source of sign errors in the IK. If
positive moves the foot **up**, the axis sign is flipped.

## 3. Tibia

Return the femur to its stance value (about 0.087 rad). Move `lf_tibia_joint`.

| Check | Expected |
|---|---|
| Axis of rotation | Same direction as the femur axis, parallel to it. |
| What moves | **Only** the tibia and the foot. Coxa and femur stay still. |
| Positive direction | Foot moves **downward and inward**. |
| Knee | Bends like a knee, not like a hyperextension. |

## 4. Stance pose

Set all three to the stance values from `common_properties.xacro`:

```
coxa 0.0    femur 0.0865    tibia 1.3979
```

| Check | Expected |
|---|---|
| Foot position | 0.19 m out from the coxa axis, 0.12 m below it |
| Femur | Roughly **horizontal** (about 5 degrees) |
| Tibia | Dropping **steeply** to the ground (about 80 degrees) |
| Overall | Insect posture, not sprawled or straight-legged |

Confirm numerically rather than by eye:

```bash
ros2 run tf2_ros tf2_echo lf_coxa_link lf_foot_link
```

Expect translation about `[0.129, 0.000, -0.120]` — that is 0.19 minus the
0.061 coxa, and 0.12 down.

## 5. All six legs

Set every joint to 0 and look from directly above.

| Check | Expected |
|---|---|
| Symmetry | Left and right mirror images |
| Splay | Front legs 45 deg forward-out, middle 90 deg straight out, rear 135 deg |
| Bending | All six knees bend the **same way**. If three bend opposite, the mount yaw signs are wrong. |

---

## If something fails

| Symptom | Cause | Fix |
|---|---|---|
| Foot height changes with coxa | Coxa axis not vertical | `<axis xyz="0 0 1"/>` in `leg_macro.xacro` |
| Positive femur moves foot up | Femur axis sign | `<axis xyz="0 1 0"/>` |
| Moving femur also moves coxa | Parent/child reversed | Check the joint's `<parent>` and `<child>` |
| Three legs bend backwards | Mount yaw signs | Right legs need negative `y` **and** negative `yaw` |
| Part sits at the wrong angle | Mesh roll only | `*_mesh_rpy` — cosmetic, no effect on physics |

The last row matters: a mesh orientation error changes **only what you see**.
Collision and inertia use primitives. So if the five checks above pass, the
robot is kinematically correct even if a bracket still looks rotated.

---

## The stronger check

The sliders test what you can see. These test the mathematics, and they run in
two seconds with no ROS and no Gazebo:

```bash
python3 ~/hexapod_ws/tools/verify_ik.py
python3 ~/hexapod_ws/tools/verify_gait.py
```

`verify_ik.py` round-trips forward and inverse kinematics over a grid of poses
and compares foot **positions**, so it catches sign errors, quadrant errors and
branch errors that no amount of looking would reveal. It also cross-checks the
stance against a separately derived number, which catches a consistent
convention flip that a round trip alone cannot.

# Crib sheet

One page to read the night before. Everything you need to explain the system
and answer questions about it.

---

## 1. The whole system in five sentences

1. We designed the robot in CAD and turned it into a **URDF**, a file that
   describes every link, joint, mass and inertia in a form ROS understands.
2. **Gazebo** loads that description, creates the robot in a world with
   gravity and friction, and simulates the physics.
3. **ros2_control** sits between the robot and the software, and gives one
   fixed interface for "move these joints", whether the joints are simulated
   or real.
4. Our **gait node** takes a velocity command, works out where all six feet
   should be, converts that to eighteen joint angles with inverse kinematics,
   and publishes them.
5. Everything is driven by **one input**, a velocity. Nothing is animated.

---

## 2. The five packages, and why they are separate

| Package | What is in it | Why it is its own package |
|---|---|---|
| `hexapod_description` | URDF/Xacro, meshes, RViz config | Data only, no code. Must be usable by the simulator, by RViz, and later by the real robot. **Depends on nothing.** |
| `hexapod_gazebo` | World file, physics settings | Simulation only. Delete it and the stack still works on hardware. |
| `hexapod_control` | Controller YAML files | Swapped wholesale between simulation and hardware. Isolating it makes that a one-line change. |
| `hexapod_gait` | Kinematics, gait, the ROS node | The mathematics. Written with no ROS dependency so it can be tested offline. |
| `hexapod_bringup` | Launch files | One entry point. Nothing depends on it, so it can change freely. |

**The line to say:** *"The robot description depends on nothing else. That is
what makes moving to hardware a settings change rather than a rewrite."*

If someone asks why not one package: because then the model could not be
loaded without pulling in Gazebo, and the gait could not be tested without
launching a simulator.

---

## 3. What happens when you run the launch command

This is the answer to "how does the robot get into Gazebo". Seven steps, in
this order, and the order matters.

```
ros2 launch hexapod_bringup hexapod_sim.launch.py
```

**Step 1. Xacro expands the model.**
`hexapod.urdf.xacro` is a template. Xacro processes it into plain URDF,
substituting the arguments you passed. One leg macro is expanded six times.

```
python3 xacro_nocomment.py hexapod.urdf.xacro \
        sim:=true fix_base:=false control_mode:=position sim_profile:=demo
```

**Step 2. Gazebo server starts** and loads our world file, which sets gravity,
the ground plane, the physics timestep of 1 ms and the solver settings.

**Step 3. `robot_state_publisher` starts.** It holds the expanded URDF as a
parameter and publishes it on the `/robot_description` topic. It also converts
joint angles into coordinate frames for RViz.

**Step 4. The robot is spawned.** This is the command you were asking about:

```
ros2 run gazebo_ros spawn_entity.py \
    -topic robot_description \
    -entity hexapod \
    -z 0.195
```

- `-topic robot_description` — read the model from that topic, not from a
  file. This is why step 3 must come first.
- `-entity hexapod` — the name the model gets in Gazebo. It is the name
  `/model_states` reports and the name our body driver addresses.
- `-z 0.195` — spawn height. **Not a free number.** It is the stance depth
  0.180 plus the foot sphere radius 0.015. Too low and the feet start inside
  the floor, and the solver throws the robot across the world on the first
  step.

**Step 5. `gazebo_ros2_control` loads inside the Gazebo process.** It reads
the `<ros2_control>` block in the URDF, registers all eighteen joints, and
starts a `controller_manager`. It runs inside Gazebo rather than as its own
node because the control loop must step with the physics.

**Step 6. Controller spawners activate the controllers.**

```
ros2 run controller_manager spawner joint_state_broadcaster \
     --controller-manager /controller_manager
ros2 run controller_manager spawner leg_trajectory_controller \
     --controller-manager /controller_manager
```

`joint_state_broadcaster` publishes what the joints are doing.
`leg_trajectory_controller` accepts our commands and drives the joints.

**Step 7. RViz**, if enabled.

Then separately:

```
ros2 launch hexapod_bringup walk.launch.py        # the gait node
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 4. Data flow, once running

```
  /cmd_vel  (geometry_msgs/Twist)
      |
      v
  hexapod_gait_node                     50 Hz
      |   advances a phase clock
      |   foot_target()  -> where each foot should be now
      |   inverse_kinematics()  -> 18 joint angles
      v
  /leg_trajectory_controller/joint_trajectory
      |
      v
  controller_manager                   200 Hz, inside gzserver
      |
      v
  Gazebo physics                      1000 Hz
      |
      v
  /joint_states  ->  robot_state_publisher  ->  /tf  ->  RViz
```

Three rates and why: the gait at 50 Hz is smooth enough for legs, the
controller at 200 Hz so it corrects errors faster than the gait creates them,
the physics at 1000 Hz because contact forces need small timesteps to stay
stable.

---

## 5. Commands you might actually type tomorrow

```bash
# build
cd ~/hexapod_ws && hexbuild

# THE DEMO. This is the one to use.
ros2 launch hexapod_bringup demo.launch.py
ros2 run hexapod_gait demo_sequence            # second terminal, after ~20 s

# manual driving instead of the script
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# the offline proofs, if they ask to see the tests
python3 tools/verify_ik.py                     # 32 passed
python3 tools/verify_gait.py                   # 498 passed

# if something looks wrong
ros2 control list_controllers                  # all must say 'active'
ros2 topic hz /joint_states                    # should be ~50
bash tools/reset_sim.sh                        # kill a stuck gzserver
```

**If the demo will not start:** almost always a leftover `gzserver` holding
port 11345. Run `bash tools/reset_sim.sh` and launch again.

---

## 6. The numbers, with their sources

| Figure | Value | Where it comes from |
|---|---|---|
| Joints | 18 | 6 legs x 3 |
| Links | 25 | chassis + 6 x (coxa, femur, tibia, foot) |
| Total mass | 3.39 kg | measured mesh volumes at PLA density |
| Link lengths | 150 / 116.6 / 150 mm | CAD |
| IK accuracy | 3.5e-16 m | `verify_ik.py` round trip |
| Gait checks | 498 passing | `verify_gait.py` |
| Foot load standing | 10.9 N | 3.39 kg on three legs |
| Knee torque, chosen elbow | 0.19 N m | vs 1.48 N m for the other branch |
| Swing clearance | 45 mm | gait parameter, verified |
| Gait cycle | 1.4 s normal, 2.0 s in the demo | slower reads better on video |

---

## 7. Questions, with short answers

**Is the demo real or animated?**
It is a kinematic simulation. The legs run the real gait and the real inverse
kinematics. The body moves by the amount those legs geometrically require,
because the gait is proven non-slip. The dynamic simulation is a separate
result and here is what we measured.

**Why doesn't it walk with full physics?**
Position control cannot, and we can explain that one: Gazebo moves the joint
without giving it a speed, friction is computed from sliding speed, so the
simulator sees a motionless foot and produces no push. Effort control with our
real masses leaves the body at the origin and we have not found the cause yet.

**How do you know the leg maths is right?**
We compute the foot position from joint angles, feed it back through the
inverse kinematics, and compute it again. Worst disagreement is 3.5e-16 m,
which is the computer's precision limit.

**Why six legs?**
Three feet is the minimum to form a support triangle, so a hexapod can lift
three legs and stay stable. Stability becomes something you check, not
something you continuously control.

**What did the simulation tell you about the hardware?**
Two things. The elbow branch choice cuts knee torque by a factor of eight. And
the femur has to swing past vertical to place a foot under the robot, which
means the servo horn needs an offset at assembly.

**Why Gazebo Classic when it is end of life?**
ROS 2 Humble has the mature integration for it, and migrating was not on the
critical path. It is noted as future work.

**What is a URDF?**
A file describing the robot as links and joints, with shapes, masses and
inertias. It is what lets the simulator, RViz and the controllers all agree on
what the robot is.

**What is ros2_control?**
A standard layer between "decide where the joints go" and "actually move
them". Because the interface is fixed, swapping the simulator for real servos
does not change anything above it.

---

## 8. If you only remember three things

1. **One velocity command in, everything else derived.** No animation.
2. **We proved the maths before trusting the simulator**, which is how we
   found a sign error that visual inspection never would have.
3. **The robot description depends on nothing**, which is why hardware is a
   settings change and not a rewrite.

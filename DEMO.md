# Review demo — exact commands

Two configurations. **Configuration A is the one to present.** It is known to
work end to end. Configuration B is better but still being tuned; try it only
once A is safely rehearsed.

Before anything, always:

```bash
bash /mnt/c/Users/'Rushi Tangirala'/OneDrive/Desktop/hexapod_scratch/tools/reset_sim.sh
```

Gazebo Classic leaves `gzserver` holding TCP port 11345 after any abrupt exit.
The next launch then cannot bind and fails almost silently: gzclient prints
"Waiting for master" forever, or no window appears. Reset first, every time.

---

## CONFIGURATION A — fixed base, position control (PRESENT THIS)

Terminal 1:

```bash
cd ~/hexapod_ws && source install/setup.bash
ros2 launch hexapod_bringup hexapod_sim.launch.py \
    control_mode:=position fix_base:=true fix_base_height:=0.125
```

Wait for `Configured and activated leg_position_controller`.

Terminal 2:

```bash
cd ~/hexapod_ws && source install/setup.bash
ros2 launch hexapod_bringup walk.launch.py command_type:=position
```

Wait for `Ramp complete, gait active.`

Terminal 3:

```bash
cd ~/hexapod_ws && source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -p speed:=0.06 -p turn:=0.3
```

Click terminal 3 so it has keyboard focus, then:

```
   u  i  o        i  forward        ,  backward
   j  k  l        j  turn left      l  turn right
   m  ,  .        k  STOP
   Shift+J / L    strafe
```

### What this demonstrates

* the 18 DOF robot built from your own CAD, in a physics simulation
* ros2_control with a hardware abstraction layer
* all 18 joints commanded and tracked through a controller
* analytic inverse kinematics placing each foot at a computed position
* a tripod gait: three legs swing while three support, correctly phased,
  making and breaking real ground contact

### What to say about the fixed base

Say it plainly, because it is a legitimate engineering answer:

> The body is anchored in this configuration. gazebo_ros2_control implements
> position commands by setting joint positions directly rather than by
> applying torque, which is fine for a fixed-base manipulator but injects
> momentum into a floating base standing on frictional contacts. We verified
> this experimentally: with no controller writing to the joints the robot is
> completely stable, and with the base anchored the gait runs cleanly under
> load, so the drift is neither a contact problem nor a gait problem. The
> resolution is effort control with a PID per joint, which we have
> implemented and are tuning, and which is also what the real hardware
> interface will use.

That answer shows you found the cause, proved it, and know the fix. It is
stronger than a demo that happens to work for reasons you cannot explain.

### Supporting evidence to have open

```bash
python3 tools/verify_ik.py      # 32 passed: IK exact to machine precision
python3 tools/verify_gait.py    # tripod stability and non-slip proven offline
```

These prove the gait's correctness independently of any simulator: three feet
always down, zero touchdown velocity, stance travel matching body travel.

---

## CONFIGURATION B — free base, effort control (walks, still tuning)

```bash
# terminal 1
ros2 launch hexapod_bringup hexapod_sim.launch.py

# terminal 2
ros2 launch hexapod_bringup walk.launch.py

# terminal 3
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -p speed:=0.06 -p turn:=0.3
```

Measure rather than eyeball:

```bash
ros2 topic echo /odom --field pose.pose.position
```

`x` rising at about 0.06 m/s is a walking robot.

### If it sags or wobbles

Edit `src/hexapod_control/config/controllers.yaml`:

| symptom | change |
|---|---|
| body sits low, legs feel soft | raise femur and tibia `p` by 50% |
| visible oscillation or buzzing | raise `d` by 50%, then lower `p` |
| settles low but steady | raise `i` |

One change at a time, `hexbuild`, relaunch.

---

## If Gazebo will not start at all

```bash
bash tools/reset_sim.sh
bash tools/test_gazebo_gui.sh      # isolates rendering from everything else
```

Still nothing, from PowerShell:

```
wsl --shutdown
```

then reopen Ubuntu. WSLg's compositor wedges occasionally and a restart is
the only cure.

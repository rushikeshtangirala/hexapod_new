# Review pack

Everything needed to present this project: the argument, the slides, the
numbers, the demo runbook, and the questions you will be asked.

---

## 1. The argument, in one paragraph

Read this until you can say it without notes. Everything else supports it.

> We built an 18-DOF hexapod in ROS 2 from our own CAD, with measured masses
> and closed-form inertia tensors. The kinematics and gait were written with
> no ROS dependency and proven correct offline before the simulator was
> trusted — analytic inverse kinematics with a round-trip error of 3.5e-16 m,
> and a tripod gait verified by 498 assertions including static stability and
> non-slip. That verification-first approach found a sign error in the stance
> phase that had survived every visual inspection, and we then characterised
> three simulation strategies against each other and can say precisely why
> each behaves as it does.

The thing that distinguishes this from a hexapod that walks in a video is the
**method**: making failure modes measurable rather than guessable. Lead with
that. It is also the honest description of where the effort actually went.

---

## 2. Slide plan (14 slides, ~15 minutes)

| # | Slide | The one thing it must land |
|---|---|---|
| 1 | Title | Project, name, supervisor |
| 2 | Objective and scope | 18-DOF hexapod, ROS 2, sim-first, hardware-ready architecture |
| 3 | The machine | CAD renders, 6 legs x 3 DOF, 25 links, 3.39 kg |
| 4 | CAD to URDF | Mesh audit: units, origins, watertightness. Meshes for visual, primitives for collision, and why |
| 5 | Physical model | Masses from measured mesh volumes at PLA density; servo mass belongs to the link it is BOLTED TO, not the one it drives |
| 6 | Software architecture | Package split + dependency graph. `hexapod_description` depends on nothing — that is what makes hardware a config change |
| 7 | Data flow | The one diagram from ARCHITECTURE.md §1. cmd_vel is the only input; everything downstream is derived |
| 8 | Inverse kinematics | 3-D collapses to 2-D because femur and tibia share an axis. Closed form. Elbow branch choice and its torque consequence |
| 9 | Tripod gait | Static stability as a geometric property. Stride is derived, not tuned. Swing profile and the frame subtlety |
| 10 | **Verification method** | The differentiator. Offline verifiers, and the tooling table |
| 11 | **The stance sign error** | Your best slide. See §4 |
| 12 | Simulation study | Three configurations, one table. See §5 |
| 13 | **Demo video** | The scripted 78 s run |
| 14 | Status, limitations, next steps | Be direct about what is unresolved |

Slides 10, 11 and 12 are what make this a good review rather than a
demonstration. Do not rush them to get to the video.

---

## 3. Numbers to have on the slides

Never present an unsourced number. Every one below is reproducible.

**Model**

- 6 legs x 3 DOF = 18 actuated joints, 25 links
- Link lengths: coxa 150.0, femur 116.6, tibia 150.0 mm
- Masses: body 1.310, coxa 0.201, femur 0.101, tibia 0.035 kg; total 3.39 kg
- Inertias: closed-form box and cylinder tensors, exact rather than estimated

**Kinematics** (`tools/verify_ik.py`, 32 assertions)

- FK -> IK -> FK round-trip error: **3.5e-16 m**, i.e. floating-point noise
- Compared as foot POSITIONS, not joint angles, because the two-solution
  elbow ambiguity means correct code can legitimately return different angles
- Stance solution: coxa 0.00, femur +15.44, tibia +67.82 degrees
- Elbow branch: knee torque **0.19 N·m** for the chosen branch versus
  **1.48 N·m** for the other — nearly 8x, for exactly the same foot position

**Gait** (`tools/verify_gait.py`, 498 assertions)

- Tripod, duty factor 0.5, exactly 3 feet down at every sampled phase
- Stride derived from commanded velocity and stance duration, never tuned
- Stance foot world-frame speed: **0.000000 m/s** at every sampled phase
- Swing clearance 45.0 mm, touchdown velocity 0.000288 m/s against a body
  speed of 0.08 m/s

**Simulation study** — see §5.

---

## 4. Slide 11: the stance sign error

This is the strongest slide in the deck, because it is a real engineering
story with a measurement, a root cause, a fix, and a process improvement. Tell
it in four beats.

**1. The symptom.** The legs executed a visually perfect tripod gait and the
body did not travel. Every visual inspection passed. The offline gait
verifier passed, all assertions green.

**2. The measurement.** Commanded 0.08 m/s forward, we measured the stance
foot's velocity through the body frame: **+0.08 m/s**. It should have been
-0.08. The body was advancing at 0.08 and the planted foot was being driven
forward at 0.08, so every loaded foot was being dragged across the ground at
**0.16 m/s in the wrong direction**, three legs at a time.

**3. The root cause.** `stride_vector` already returns a backward
displacement. `foot_target` then applied it as `+stride/2 -> -stride/2`,
negating it a second time. A double negative, four characters wide.

**4. Why the test missed it, which is the real lesson.** The non-slip check
compared `math.dist(foot_start, foot_end)` against the expected distance. Two
magnitudes. **A magnitude cannot see a direction.** The test was checking the
right property with the wrong operator, and would have passed forever. It is
now a signed vector comparison plus an explicit assertion that stance pushes
the robot the way it was asked.

Closing line worth saying aloud: *a passing test is evidence about the test as
much as about the code.*

If asked how it was found: by reading an independent implementation
(KevinOchs/hexapod_ros) and comparing conventions. Independent implementations
disagreeing is a cheap and underrated way to find your own sign errors.

---

## 5. Slide 12: the simulation study

One table. This is a genuine comparative result, not an apology.

| Configuration | Body travel, 20 s | Speed tracking | Outcome |
|---|---|---|---|
| Position interface, measured model | 0.4 mm | 0 % | Legs swing correctly (coxa 10.0°, femur 14.2°), body does not travel |
| Effort interface, non-physical model | 0.59 m | 99 % | Walks, but climbs 5 cm and tilts 12° |
| Effort interface, measured model | — | — | Body pinned at origin, unresolved |

**Why position mode cannot walk, and this is the interesting one.**
`gazebo_ros2_control` realises a position command with `SetPosition()`, which
relocates the joint without giving it a matching velocity. ODE computes
contact friction from sliding **velocity**, so the solver sees a stationary
foot and generates no propulsive force. The controller tracked commands to
four decimal places — it did exactly what it was told, and the physics still
produced no motion. No gain tuning changes this; it is a property of the
interface.

**Why the non-physical model walks.** Every link masses 1e-5 kg with an
identity inertia tensor. Gravity torque is then negligible, so a crude PID has
thousands of times the authority it needs, and a 1.0 kg·m² tensor on a 1e-5 kg
link is far too reluctant to rotate for anything to destabilise. It is stable
*because* it is unphysical. Say this explicitly — it shows you understand what
you are looking at, and it pre-empts the obvious question.

**The honest framing:** the interface and the model are not independent
choices. Effort control is physically correct and needs gains matched to real
masses; position control is easy and cannot propel a floating base. That
sentence is a legitimate research finding.

---

## 6. Slide 13: the demo

**Run it scripted, not by hand.**

```bash
cd ~/hexapod_ws && hexbuild
ros2 launch hexapod_bringup demo.launch.py
# wait ~20 s until the robot is standing, then in a second terminal:
ros2 run hexapod_gait demo_sequence
```

78 seconds: stand, forward, stop, turn left, forward, turn right, strafe, arc,
reverse, stop. Every segment pre-verified — no stride clamping, all phases
reachable, all joints in limits.

**Record it. Do not run it live.** Record two takes and keep the better one.
A recording cannot be broken by a stale gzserver, a WSLg display glitch, or a
real-time factor drop while the projector is mirroring. Have the live version
ready as a follow-up if they ask.

**What to say while it plays**, matched to the segments:

- *stand* — "the stance pose holds, no drift"
- *forward* — "tripod gait, three feet down at all times"
- *stop* — "the legs finish the cycle and land; stopping is a sequence, not an
  event, or you leave three feet in the air"
- *turn* — "each leg gets a different stride because it is a different
  distance from the turn centre. Same code path as walking forward"
- *strafe* — "sideways with no rotation. A wheeled robot cannot do this"
- *arc* — "linear and angular together, the general case"

**Declare what it is, in one sentence, before anyone asks.** See §7 Q1.

---

## 7. Questions you will be asked

**Q1. Is the body motion real, or are you just moving the robot?**

The one that matters. Answer directly, do not get defensive:

> "This is a kinematic simulation, and I should be clear about that. The leg
> trajectories come from the gait generator and the analytic IK — nothing is
> animated. The body displacement is the displacement those trajectories
> geometrically require, because the gait is provably non-slip: stance feet
> have zero velocity relative to the ground at every phase, which we assert in
> the offline verifier. What we skip is Gazebo rediscovering that displacement
> by integrating friction cones. The dynamic simulation is a separate result
> and I can tell you exactly what we measured there."

Then go to the table in §5. Volunteering the limitation is what makes it
credible.

**Q2. Why doesn't it walk under full physics?**

Position mode cannot, for the `SetPosition` reason in §5 — that one is
understood and explained. Effort mode with the measured model leaves the body
pinned at the origin and we have not root-caused it. Both `/odom` and Gazebo's
own state plugin agree, so it is not a sensor artefact. Say "unresolved", not
"nearly working".

**Q3. How do you know the IK is correct?**

Round-trip FK -> IK -> FK over a joint-space grid, comparing foot positions
rather than angles because of the elbow ambiguity. Worst error 3.5e-16 m. It
runs in a second with no ROS and no simulator, which is why it was trusted
before anything else was.

**Q4. What torque do the servos need?**

From the measured model: standing on three legs at 10.9 N per foot, the chosen
elbow branch gives 0.19 N·m at the knee. The other branch gives 1.48 N·m for
the identical foot position. This drove a real design decision.

**Q5. Anything the simulation told you about the hardware?**

Yes, and this is a strong answer. The femur must swing past vertical to place a
foot under the robot, because the 0.15 m coxa puts the femur joint well
outboard. So its travel is -60° to +120°, not -90° to +90°, which means **the
femur servo horn has to be mounted with an offset at assembly**. A simulation
result driving a mechanical decision.

**Q6. Why Gazebo Classic when it is end-of-life?**

Honest answer: ROS 2 Humble's mature `gazebo_ros2_control` integration, and
the migration was not on the critical path. Note it as future work — you will
get credit for knowing, and none for pretending otherwise.

**Q7. What would you do differently?**

Have one ready. Suggested: build the measurement tooling before the
simulation, not after. Most of the time lost went to symptoms that several
unrelated faults produce identically, and the tools that separate them —
`verify_gait`, `check_drift`, `autotest` — were written in response to
confusion rather than in anticipation of it.

---

## 8. Slide 14: status and limitations

Do not bury this. A clear-eyed limitations slide reads as competence.

**Done**

- CAD to URDF, 18 joints, full TF tree
- Measured masses, closed-form inertias, collision primitives, friction
- ros2_control integration, all controllers active
- Analytic IK and tripod gait, both verified offline
- Kinematic walking demonstration with teleoperation
- Comparative study of three simulation configurations

**Not done, and stated plainly**

- Dynamic walking with the measured model. Root cause not established.
- Effort-mode PID gains untuned. `ff_velocity_scale` is silently rejected by
  the controller because of a parameter-format change, so velocity
  feedforward is currently inactive.
- Hardware interface is architectural only; no servo driver written.

**Next**

1. Root-cause the pinned base under effort control
2. Fix the feedforward parameter format, then tune gains against the measured model
3. Per-foot contact sensing (implemented, off by default) to measure slip
   directly instead of inferring it
4. Servo driver, replacing one `<plugin>` line in `hexapod.ros2_control.xacro`

---

## 9. Two-day plan

**Day 1**

- Record the scripted demo, two takes (30 min)
- Build slides 1-9 from `ARCHITECTURE.md`, which is already written in this
  order (3 h)
- Screenshot the verifier output for slide 10 (10 min)

**Day 2**

- Slides 10-14, the argument slides. Give these the most time (2 h)
- Rehearse Q1 and Q2 out loud until they are fluent (20 min)
- Full timed run-through (30 min)
- Leave the afternoon free. Do not start new code.

**Do not** attempt to fix the pinned base before the review. It is unbounded,
and "characterised, root cause outstanding" is a perfectly respectable answer
that costs you far less than an unfinished new attempt.

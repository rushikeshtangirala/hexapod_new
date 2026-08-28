# Delivering the presentation

Two things: how to run the talk, and how to position it against the benchmark
paper.

---

# PART 1 — IS THE DECK ENOUGH?

Yes. Content is not your risk any more. **Length is.**

30 slides is too many for a 15 minute slot. At 15 minutes you have about 30
seconds a slide, which is not enough to say anything on the technical ones.
Decide your route by the time you are actually given.

## The three routes

**If you get 10 minutes — 12 slides**

```
1 Title · 2 Why a hexapod · 4 The leg · 8 Inverse kinematics
13 Tripod gait · 15 No-slip verification · 17 MATLAB velocity+acceleration
24 Gazebo simulation · 25 Verification · 26 What verification found
28 How our approach compares · 29 Where we are
```

**If you get 15 minutes — 18 slides**

Add: 3 Objectives, 6 Mechanism, 7 Forward kinematics, 12 Structural analysis,
19 MATLAB joint angles, 27 Related work.

**If you get 20 to 25 minutes — all 30.**

Set the unused ones to *Hidden* in PowerPoint rather than deleting them. Right
click the slide, Hide Slide. They stay available if a question goes there,
which looks extremely good: *"we do have that, let me show you."*

## Timing plan for the 15 minute version

| Section | Slides | Time |
|---|---|---|
| Framing | 1–3 | 1.5 min |
| Mechanical design | 4–6, 12 | 3 min |
| Kinematics | 7–9 | 3 min |
| Gait | 13, 15 | 2 min |
| MATLAB | 17, 19 | 2 min |
| Simulation and verification | 24–26 | 2.5 min |
| Paper comparison and status | 27–29 | 1 min |

## How to speak it

**Every slide has speaker notes.** In PowerPoint use Slide Show → Presenter
View. The notes are written as things to say, not as bullet points to read.

**Never read the slide.** The slide is the evidence, you are the argument. Say
the thing the slide cannot say. On the inverse kinematics slide the equations
are already there; what you add is *"the reason this is solvable at all is
that the femur and tibia axes are parallel."*

**Rehearse three slides out loud.** Inverse kinematics, no-slip verification,
and what the verification found. If those three are fluent the rest will
carry. Everything else you can talk around; those three you cannot.

**Own the numbers.** Say "three point three nine kilograms", "ten point nine
newtons per foot", "zero point one nine newton metres at the knee". Specific
numbers said confidently are the single clearest signal that the work is
yours.

**Handing over between speakers:** four people, so split by section, not by
slide. Whoever presents a section should be the one who can answer questions
about it.

## The two questions you will get

**"Is the walking demonstration real?"**

> "It is a kinematic simulation, and I want to be clear about that. The leg
> trajectories come from the gait generator and the analytic inverse
> kinematics, nothing is animated. The body displacement is what those
> trajectories geometrically require, because the gait is provably non-slip.
> The dynamic simulation is a separate result and I can tell you what we
> measured."

**"Why doesn't it walk under full physics?"**

> "Position control cannot, and we can explain why. Gazebo carries out a
> position command by relocating the joint without giving it a velocity, and
> contact friction is computed from sliding velocity, so the solver sees a
> motionless foot and produces no propulsion. Effort control with our measured
> masses is the configuration we want and it is not finished. It is the first
> item on the next phase list."

Volunteering the limitation is what makes the rest credible.

---

# PART 2 — POSITIONING AGAINST THE PAPER

**Xiang, L. et al. (2024). "Development of a bionic hexapod robot with
adaptive gait and clearance for enhanced agricultural field scouting."
*Frontiers in Robotics and AI*, 11:1426269.**

## Why this paper is a good benchmark for you

It is not a superficial match. It is nearly the same machine:

- six legs, three degrees of freedom each
- the same joint naming and function (hip/knee/ankle vs coxa/femur/tibia)
- the **tripod gait** as the starting point
- forward and inverse kinematics presented for the leg
- simulation first, then hardware

So every difference you point out is a genuine engineering choice rather than
an accident of scale.

## What they did

A hexapod for **precision agriculture field scouting**. The contribution is a
**Terrain-Adaptive (TA) gait**: the robot has two modes, a low clearance
*marching* mode at 6 cm and a high clearance *step-over* mode at 18 cm, and it
switches between them based on the obstacle. Instead of walking round an
obstacle it climbs over it.

Hardware: Dynamixel MX-106T servos rated 8.4 N·m, carbon fibre base plates
2 mm thick, CNC aluminium for stressed parts and 3D printing for unstressed
ones, 27 cm curved leg. Design targets under 10 kg with 8 kg payload.

Sensing: a nine-axis IMU plus **one force sensor per foot** for ground contact
detection, with an advanced version adding LiDAR, stereo cameras and distance
sensors.

Simulation: SolidWorks model imported into **Simulink via Simscape
Multibody**.

Results: slopes up to **17°**, pitch held between **−11.5° and 8.6°**,
**14.4%** less energy per obstacle crossed, cost of transport **25.3 versus
30.2** for conventional obstacle avoidance.

## The intellectual contrast — this is the part that will impress

**They deliberately avoid inverse kinematics at run time.** Their words: the
conventional approach "requires computing the landing point of the robot's
foot end at each step and using inverse kinematics", which "generates many
complex computations, requires long computation time and energy consumption".
So they drive the joints with **sinusoidal modulation** instead.

**We do the exact opposite, and can defend it.**

Their objection applies to *numerical* inverse kinematics, which iterates.
Ours is **closed form**: a fixed handful of trigonometric evaluations, no
iteration, no convergence risk, identical cost every call. It is not
expensive. And it buys something their approach cannot have:

- With sinusoidal joint modulation, the foot path is **whatever the sinusoids
  produce**. You select a *gait mode*.
- With analytic IK, you command a **body velocity** and the stride for every
  leg is derived from geometry. Walking, strafing, turning on the spot and
  driving an arc are the **same code path**.

Say it in one line:

> **They trade commandability for computation. We trade computation for
> commandability, and a closed-form solution is what makes that trade
> affordable.**

Both are correct engineering. Theirs is right for a battery-limited field
robot doing long endurance scouting, where cost of transport is the headline
metric. Ours is right for a general platform where an operator or a navigation
stack needs to command velocity directly.

## The second contrast — how each is validated

| | Them | Us |
|---|---|---|
| Method | Empirical, field trials on four terrains | Formal, offline assertions |
| Evidence | Pitch, slope, cost of transport | 498 gait checks, IK to 3.5×10⁻¹⁶ m |
| Needs | Built hardware | Nothing but a laptop |
| Catches | Real-world effects you cannot model | Logic and sign errors |

These are **complementary, not competing**. Field testing is the gold standard
and we will need it. But it would very likely have masked our stance sign
error as "needs tuning", because a robot that walks badly and a robot with a
reversed stance look similar on grass. Formal offline checking found it and
localised it exactly.

That is worth saying: *"our verification approach caught a class of error that
field testing tends to hide."*

## The third point — their paper is our roadmap

Note it explicitly, because it shows you read the paper rather than skimmed
it:

| Their feature | Our status |
|---|---|
| Force sensor on every foot | Implemented in our model, currently disabled |
| IMU for attitude feedback | Architecture has the slot, not yet added |
| Adjustable clearance | Our step height is already a gait parameter, `H` in the swing equation |
| Cost of transport metric | We should adopt it; we have no energy metric yet |

The adjustable clearance point is the strongest. Their contribution is
*varying* a quantity. In our formulation that quantity is **already a
parameter of the swing equation**, so implementing their idea is a matter of
scheduling `H` against terrain, not restructuring the gait.

## The honest difference in stage

They have a built robot with field trials. You have a complete simulation and
verified mathematics, with hardware as the next phase. **Say that plainly.**
Trying to imply parity is the only way this comparison can go wrong.

The right framing:

> "This paper is where we intend to be at the end of the project. What we have
> now is the modelling, the kinematics and the gait proven, and the simulation
> infrastructure complete. Their sensing package is essentially our next phase
> list."

## If asked "what would you do differently after reading it"

Three good answers, all true:

1. **Adopt cost of transport** as a metric. We have no energy measure at all,
   and it is the standard number for comparing legged robots.
2. **Add the IMU earlier.** They use it for attitude feedback and body
   levelling. Our architecture has the place for it and we have not filled it.
3. **Reconsider the leg structure.** Ours is entirely 3D printed PLA. They use
   CNC aluminium for stressed parts and printing only for unstressed ones,
   which is why they achieve 8 kg payload. Our ANSYS results should decide
   whether that matters at our scale.

## A one-sentence citation for the slide

> Xiang et al. (2024) demonstrate a six-legged, three-degree-of-freedom
> agricultural robot using a terrain-adaptive extension of the tripod gait,
> validated in field trials; our work shares the topology and base gait but
> retains closed-form inverse kinematics at run time in order to command body
> velocity directly.

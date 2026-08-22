import math, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
sys.path.insert(0, ".")
from hexapod_gait.gait import GaitParams, foot_target, nominal_foot, stride_vector
from hexapod_gait.kinematics import (LEGS, LEG_INDEX, inverse_kinematics_body,
                                     body_to_leg, forward_kinematics, L1, L2, L3)

plt.rcParams.update({
    "font.family": "DejaVu Sans", "font.size": 11,
    "axes.grid": True, "grid.alpha": 0.3, "grid.linestyle": "--",
    "axes.spines.top": False, "axes.spines.right": False,
    "figure.dpi": 200,
})
ACC = "#C45A11"; BLU = "#1F5C8B"; GRN = "#2E7D32"; GRY = "#666666"

P = GaitParams(cycle_time=2.0, max_stride=0.20, step_height=0.045)
VX, WZ = 0.07, 0.0
N = 600
ph = np.linspace(0, 1, N, endpoint=False)
leg = LEGS[LEG_INDEX["lf"]]

# ---------------------------------------------------------------- 1 foot path
fig, ax = plt.subplots(figsize=(6.6, 4.0))
nom = nominal_foot(leg, P)
xs, zs, st = [], [], []
for p in ph:
    f, s = foot_target(leg, P, p, VX, 0, WZ)
    lf = body_to_leg(leg, f)
    xs.append(lf[0]); zs.append(lf[2]); st.append(s)
xs, zs, st = np.array(xs), np.array(zs), np.array(st)
ax.plot(xs[st], zs[st], color=ACC, lw=3.5, label="Stance  (foot on ground, pushing)")
ax.plot(xs[~st], zs[~st], color=BLU, lw=2.5, label="Swing  (foot in air, returning)")
ax.axhline(zs[st][0], color=GRY, lw=1, ls=":")
ax.annotate("", xy=(xs[st][-1], zs[st][-1]), xytext=(xs[st][0], zs[st][0]),
            arrowprops=dict(arrowstyle="-|>", color=ACC, lw=2.2, mutation_scale=18))
ax.annotate(f"{P.step_height*1000:.0f} mm clearance", xy=(xs.mean(), zs.max()),
            xytext=(xs.mean()-0.030, zs.max()-0.006), fontsize=10.5, color=BLU)
ax.set_xlabel("Foot position along the leg, radial (m)")
ax.set_ylabel("Foot height (m)")
ax.set_title("Foot path through one gait cycle, in the leg plane", fontsize=12.5, pad=10)
ax.legend(loc="upper left", frameon=False, fontsize=9.5,
          bbox_to_anchor=(-0.02, 0.62))
ax.set_aspect("equal", adjustable="datalim")
fig.tight_layout(); fig.savefig("fig1_foot_path.png"); plt.close(fig)

# ---------------------------------------------------------------- 2 joints
fig, ax = plt.subplots(figsize=(6.6, 4.0))
t = ph * P.cycle_time
q = np.array([[math.degrees(a) for a in inverse_kinematics_body(
                leg, foot_target(leg, P, p, VX, 0, WZ)[0])] for p in ph])
for i, (lbl, c) in enumerate([("Coxa  θ₁", ACC), ("Femur  θ₂", BLU), ("Tibia  θ₃", GRN)]):
    ax.plot(t, q[:, i], color=c, lw=2.2, label=lbl)
sw = P.duty_factor * P.cycle_time
ax.axvspan(sw, P.cycle_time, color="#000000", alpha=0.05)
ax.text(sw + (P.cycle_time - sw)/2, ax.get_ylim()[1]*0.93, "swing",
        ha="center", fontsize=10, color=GRY)
ax.text(sw/2, ax.get_ylim()[1]*0.93, "stance", ha="center", fontsize=10, color=GRY)
ax.set_xlabel("Time (s)"); ax.set_ylabel("Joint angle (degrees)")
ax.set_title("Joint angles over one cycle, from inverse kinematics", fontsize=12.5, pad=10)
ax.legend(frameon=False, fontsize=10, ncol=3, loc="lower center")
fig.tight_layout(); fig.savefig("fig2_joint_angles.png"); plt.close(fig)

# ---------------------------------------------------------------- 3 gait diagram
fig, ax = plt.subplots(figsize=(6.6, 4.0))
order = ["lf", "lm", "lr", "rf", "rm", "rr"]
for row, name in enumerate(order):
    lg = LEGS[LEG_INDEX[name]]
    inst = np.array([foot_target(lg, P, p, VX, 0, WZ)[1] for p in ph])
    for k in range(N - 1):
        if inst[k]:
            ax.add_patch(plt.Rectangle((ph[k]*P.cycle_time, row-0.32),
                                       (ph[1]-ph[0])*P.cycle_time*1.05, 0.64,
                                       color=ACC, lw=0))
    ax.text(-0.13, row, name.upper(), va="center", ha="right", fontsize=11)
ax.set_xlim(0, P.cycle_time); ax.set_ylim(-0.7, 5.7)
ax.set_yticks([]); ax.set_xlabel("Time (s)")
ax.set_title("Gait diagram: filled = foot on the ground", fontsize=12.5, pad=10)
ax.grid(axis="y", visible=False)
fig.tight_layout(); fig.savefig("fig3_gait_diagram.png"); plt.close(fig)

# ---------------------------------------------------------------- 4 workspace
fig, ax = plt.subplots(figsize=(6.0, 4.4))
th2 = np.linspace(math.radians(-60), math.radians(90), 130)
th3 = np.linspace(math.radians(-60), math.radians(130), 130)
R, Z = [], []
for a in th2:
    for b in th3:
        r = L1 + L2*math.cos(a) + L3*math.cos(a+b)
        z = -L2*math.sin(a) - L3*math.sin(a+b)
        R.append(r); Z.append(z)
ax.scatter(R, Z, s=1.2, color="#BFD4E4", alpha=0.5, label="Reachable, within joint limits")
sr = P.stance_radius; sz = P.stance_height
ax.plot([sr], [sz], "o", color=ACC, ms=11, zorder=5, label="Stance point")
sx, sy = stride_vector(leg, P, VX, 0, WZ)
half = abs(sx)/2
ax.plot([sr-half, sr+half], [sz, sz], color=ACC, lw=3, zorder=4)
ax.plot([0], [0], "s", color="#333333", ms=8, label="Coxa joint")
ax.set_xlabel("Radial distance from coxa axis (m)")
ax.set_ylabel("Height relative to coxa (m)")
ax.set_title("Leg workspace and where we operate in it", fontsize=12.5, pad=10)
ax.legend(frameon=False, fontsize=9.5, loc="upper left")
ax.set_aspect("equal", adjustable="datalim")
fig.tight_layout(); fig.savefig("fig4_workspace.png"); plt.close(fig)

# ---------------------------------------------------------------- 5 non-slip
fig, ax = plt.subplots(figsize=(6.6, 4.0))
d = 1e-6
vw = []
for p in ph:
    f0, s0 = foot_target(leg, P, p, VX, 0, WZ)
    f1, _ = foot_target(leg, P, p+d, VX, 0, WZ)
    vb = (f1[0]-f0[0]) / (d*P.cycle_time)
    vw.append(vb + VX)
vw = np.array(vw)
ax.plot(t, vw*1000, color=BLU, lw=2.4)
ax.axhline(0, color=GRY, lw=1)
ax.axvspan(0, sw, color=ACC, alpha=0.10)
ax.text(sw/2, ax.get_ylim()[1]*0.85, "STANCE\nfoot speed over ground = 0",
        ha="center", fontsize=10.5, color=ACC, weight="bold")
ax.text(sw + (P.cycle_time-sw)/2, ax.get_ylim()[1]*0.42, "SWING\nfoot in the air",
        ha="center", fontsize=10.5, color=BLU)
ax.set_xlabel("Time (s)"); ax.set_ylabel("Foot speed over the ground (mm/s)")
ax.set_title("Proof of no slipping: stance foot speed is exactly zero", fontsize=12.5, pad=10)
fig.tight_layout(); fig.savefig("fig5_nonslip.png"); plt.close(fig)

# ---------------------------------------------------------------- 6 stride vs speed
fig, ax = plt.subplots(figsize=(6.6, 3.9))
speeds = np.linspace(0, 0.10, 60)
for ct, c, ls in [(1.4, ACC, "-"), (2.0, BLU, "--"), (3.0, GRN, ":")]:
    Q = GaitParams(cycle_time=ct, max_stride=0.20)
    strides = [abs(stride_vector(leg, Q, v, 0, 0)[0])*1000 for v in speeds]
    ax.plot(speeds*100, strides, color=c, lw=2.3, ls=ls, label=f"cycle time {ct} s")
ax.set_xlabel("Commanded body speed (cm/s)")
ax.set_ylabel("Required stride length (mm)")
ax.set_title("Stride is not chosen. It follows from speed and stance time",
             fontsize=12.5, pad=10)
ax.legend(frameon=False, fontsize=10)
fig.tight_layout(); fig.savefig("fig6_stride.png"); plt.close(fig)

print("figures written")

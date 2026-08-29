import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
plt.rcParams.update({"mathtext.fontset": "cm", "font.family": "DejaVu Sans"})
INK="#2B2B2B"; ACC="#C45A11"; GRY="#5F5F5F"

def panel(fname, lines, w=6.6, fs=17, lh=0.125):
    n = len(lines)
    fig = plt.figure(figsize=(w, 0.42*n + 0.3)); fig.patch.set_alpha(0)
    y = 0.94
    for txt, kind in lines:
        if kind == "n":
            fig.text(0.02, y, txt, fontsize=fs-5, color=GRY); y -= lh*0.80
        else:
            fig.text(0.05, y, txt, fontsize=fs, color=INK); y -= lh
    fig.savefig(fname, transparent=True, bbox_inches="tight", dpi=200)
    plt.close(fig)

panel("eq_fk.png", [
 ("In the leg plane, measured out from the coxa axis", "n"),
 (r"$r = L_1 + L_2\cos\theta_2 + L_3\cos(\theta_2+\theta_3)$", "e"),
 (r"$z = -L_2\sin\theta_2 - L_3\sin(\theta_2+\theta_3)$", "e"),
 ("Lifted back into three dimensions by the coxa angle", "n"),
 (r"$x = r\cos\theta_1 \qquad y = r\sin\theta_1$", "e"),
], w=6.8, lh=0.16)

panel("eq_ik.png", [
 ("Step 1    Coxa. The only joint that moves the foot sideways", "n"),
 (r"$\theta_1 = \arctan\left(\frac{y}{x}\right)$", "e"),
 ("Step 2    Reduce to a two link problem in the leg plane", "n"),
 (r"$r' = \sqrt{x^2+y^2} - L_1 \qquad D = \sqrt{r'^2 + z^2}$", "e"),
 ("Step 3    Tibia, by the law of cosines", "n"),
 (r"$\cos\theta_3 = \frac{D^2 - L_2^2 - L_3^2}{2 L_2 L_3}$", "e"),
 ("Step 4    Femur, as two angles added", "n"),
 (r"$\beta = \mathrm{atan2}(-z,\ r') \qquad \cos\psi = \frac{L_2^2 + D^2 - L_3^2}{2 L_2 D}$", "e"),
 (r"$\theta_2 = \beta - \psi$", "e"),
], w=7.4, lh=0.098)

panel("eq_reach.png", [
 ("A solution exists only inside the ring swept by the two links", "n"),
 (r"$|L_2 - L_3| \leq D \leq L_2 + L_3$", "e"),
 (r"$L_1 = 150\ \mathrm{mm},\quad L_2 = 116.6\ \mathrm{mm},\quad L_3 = 150\ \mathrm{mm}$", "e"),
 (r"$33.4\ \mathrm{mm} \leq D \leq 266.6\ \mathrm{mm}$", "e"),
], w=7.0, lh=0.17)

panel("eq_jac.png", [
 ("Velocity kinematics. Differentiating the forward equations", "n"),
 (r"$\dot r = -(L_2\sin\theta_2 + L_3\sin\theta_{23})\,\dot\theta_2 - L_3\sin\theta_{23}\,\dot\theta_3$", "e"),
 (r"$\dot z = -(L_2\cos\theta_2 + L_3\cos\theta_{23})\,\dot\theta_2 - L_3\cos\theta_{23}\,\dot\theta_3$", "e"),
 (r"where $\ \theta_{23} = \theta_2 + \theta_3$", "n"),
 ("Singular when the leg is straight or fully folded. Our stance sits", "n"),
 ("well away from both, so the leg is always well conditioned", "n"),
], w=7.6, lh=0.115)

panel("eq_stride.png", [
 ("A planted foot must be still relative to the ground, so it travels", "n"),
 ("backwards through the body frame at the body's own speed", "n"),
 (r"$s_{ix} = -(v_x - \omega_z\, p_{iy})\ T_{st}$", "e"),
 (r"$s_{iy} = -(v_y + \omega_z\, p_{ix})\ T_{st}$", "e"),
 (r"$T_{st} = \beta\ T_{cycle}, \qquad \beta = 0.5$", "e"),
 ("Each leg gets its own stride when turning, because each sits at a", "n"),
 ("different distance from the centre of rotation", "n"),
], w=7.2, lh=0.108)

panel("eq_swing.png", [
 ("Swing, horizontal profile", "n"),
 (r"$\sigma(\tau) = (2m-2)\tau^3 + (3-3m)\tau^2 + m\tau, \qquad m = -\frac{1-\beta}{\beta}$", "e"),
 ("Swing, vertical profile", "n"),
 (r"$h(\tau) = H\ \frac{1 - \cos 2\pi\tau}{2}, \qquad H = 45\ \mathrm{mm}$", "e"),
 ("End slopes match the stance, so the foot lands with zero speed over", "n"),
 ("the GROUND, not merely zero speed relative to the moving body", "n"),
], w=7.6, lh=0.115)

print("panels written")

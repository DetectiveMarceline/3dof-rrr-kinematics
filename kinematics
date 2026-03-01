"""
3-DOF RRR Planar Robot Arm — Kinematic Analysis
=================================================
Forward Kinematics, Inverse Kinematics, and Workspace Analysis
using Denavit-Hartenberg convention.

"""

import numpy as np
import matplotlib.pyplot as plt

# ============================================================
# 1. Robot Parameters
# ============================================================
L1, L2, L3 = 0.5, 0.4, 0.3  # Link lengths (m)

print("=" * 60)
print("3-DOF RRR Planar Robot Arm — Kinematic Analysis")
print("=" * 60)
print(f"Link lengths: L1={L1} m, L2={L2} m, L3={L3} m")
print(f"Joint limits: theta_i in [-pi, pi]")
print(f"Max reach: {L1 + L2 + L3} m")
print(f"Min reach: {max(0, L1 - L2 - L3)} m")


# ============================================================
# 2. DH Transformation Matrices
# ============================================================
def dh_matrix(theta, d, a, alpha):
    """
    Compute the standard DH homogeneous transformation matrix.

    Parameters:
        theta: joint angle (rad) - rotation about z_{i-1}
        d:     link offset (m)   - translation along z_{i-1}
        a:     link length (m)   - translation along x_i
        alpha: link twist (rad)  - rotation about x_i

    Returns:
        4x4 homogeneous transformation matrix
    """
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d     ],
        [0,   0,        0,       1     ]
    ])


def forward_kinematics(theta1, theta2, theta3):
    """
    Compute the total transformation matrix T_0,3 = A1 * A2 * A3
    and extract end-effector position and orientation.

    DH Parameters for RRR planar arm:
        Joint i | theta_i | d_i | a_i | alpha_i
        --------|---------|-----|-----|--------
          1     | theta1  |  0  | L1  |   0
          2     | theta2  |  0  | L2  |   0
          3     | theta3  |  0  | L3  |   0

    Returns:
        T03: 4x4 total transformation matrix
        px, py, pz: end-effector position
        phi: end-effector orientation angle (rad)
    """
    A1 = dh_matrix(theta1, 0, L1, 0)
    A2 = dh_matrix(theta2, 0, L2, 0)
    A3 = dh_matrix(theta3, 0, L3, 0)

    T03 = A1 @ A2 @ A3

    px = T03[0, 3]
    py = T03[1, 3]
    pz = T03[2, 3]
    phi = np.arctan2(T03[1, 0], T03[0, 0])

    return T03, px, py, pz, phi


# ============================================================
# 3. Forward Kinematics — Test Case
# ============================================================
print("\n" + "=" * 60)
print("FORWARD KINEMATICS")
print("=" * 60)

q_test = [0, np.pi / 4, np.pi / 2]
print(f"\nTest joint vector: q = [0, pi/4, pi/2]")
print(f"                     = [0, {np.degrees(q_test[1]):.1f} deg, {np.degrees(q_test[2]):.1f} deg]")

T03, px, py, pz, phi = forward_kinematics(*q_test)

print(f"\nIndividual DH matrices:")
for i, (theta, L) in enumerate(zip(q_test, [L1, L2, L3]), 1):
    A = dh_matrix(theta, 0, L, 0)
    print(f"\n  A{i} (theta={np.degrees(theta):.1f} deg, a={L} m):")
    print(f"  {np.array2string(A, precision=4, suppress_small=True)}")

print(f"\nTotal transformation T_0,3 = A1 * A2 * A3:")
print(f"  {np.array2string(T03, precision=4, suppress_small=True)}")

print(f"\nEnd-effector position:")
print(f"  px = {px:.4f} m")
print(f"  py = {py:.4f} m")
print(f"  pz = {pz:.4f} m")
print(f"  phi = {np.degrees(phi):.1f} deg")

# Analytical verification
px_a = L1*np.cos(q_test[0]) + L2*np.cos(q_test[0]+q_test[1]) + L3*np.cos(sum(q_test))
py_a = L1*np.sin(q_test[0]) + L2*np.sin(q_test[0]+q_test[1]) + L3*np.sin(sum(q_test))
print(f"\nAnalytical verification:")
print(f"  px = {px_a:.4f} m")
print(f"  py = {py_a:.4f} m")
print(f"  Match: {np.allclose([px, py], [px_a, py_a])}")


# ============================================================
# 4. Inverse Kinematics — Analytical Solution
# ============================================================
def inverse_kinematics(px, py, phi):
    """
    Analytical inverse kinematics for 3-DOF RRR planar robot.

    Given desired end-effector position (px, py) and orientation phi,
    computes joint angles theta1, theta2, theta3.

    Returns list of (theta1, theta2, theta3, name) tuples, or None.
    """
    wx = px - L3 * np.cos(phi)
    wy = py - L3 * np.sin(phi)

    cos_theta2 = (wx**2 + wy**2 - L1**2 - L2**2) / (2 * L1 * L2)

    if abs(cos_theta2) > 1.0:
        print(f"  No solution: |cos(theta2)| = {abs(cos_theta2):.4f} > 1")
        return None

    solutions = []
    for sign, name in [(1, "Elbow-Up"), (-1, "Elbow-Down")]:
        theta2 = sign * np.arccos(cos_theta2)
        theta1 = np.arctan2(wy, wx) - np.arctan2(
            L2 * np.sin(theta2), L1 + L2 * np.cos(theta2)
        )
        theta3 = phi - theta1 - theta2
        solutions.append((theta1, theta2, theta3, name))

    return solutions


print("\n" + "=" * 60)
print("INVERSE KINEMATICS")
print("=" * 60)

target_px, target_py = 0.8, 0.2
target_phi = 0

print(f"\nTarget: (px, py) = ({target_px}, {target_py}) m, phi = {np.degrees(target_phi):.0f} deg")
print(f"\nNote: Assignment target (0.8, 0.2, 0.1) has pz=0.1 m,")
print(f"      but planar robot always has pz=0. Solving for 2D projection.\n")

wx = target_px - L3 * np.cos(target_phi)
wy = target_py - L3 * np.sin(target_phi)
cos_t2 = (wx**2 + wy**2 - L1**2 - L2**2) / (2 * L1 * L2)
print(f"Wrist point: ({wx:.4f}, {wy:.4f}) m")
print(f"cos(theta2) = {cos_t2:.4f}, |cos(theta2)| <= 1: {abs(cos_t2) <= 1}")

solutions = inverse_kinematics(target_px, target_py, target_phi)
if solutions:
    print(f"\n{'Config':<15} {'theta1 (deg)':>13} {'theta2 (deg)':>13} {'theta3 (deg)':>13}")
    print("-" * 58)
    for t1, t2, t3, name in solutions:
        print(f"{name:<15} {np.degrees(t1):>13.2f} {np.degrees(t2):>13.2f} {np.degrees(t3):>13.2f}")

    print(f"\nFK Verification:")
    for t1, t2, t3, name in solutions:
        _, vx, vy, vz, vphi = forward_kinematics(t1, t2, t3)
        err = np.sqrt((vx - target_px)**2 + (vy - target_py)**2)
        print(f"  {name}: ({vx:.4f}, {vy:.4f}) m, error = {err:.2e} m")


# ============================================================
# 5. Jacobian and Singularity Analysis
# ============================================================
def compute_jacobian(theta1, theta2, theta3):
    """
    Compute the 2x3 Jacobian matrix for the planar 3-DOF robot.

    J = [dpx/dtheta1  dpx/dtheta2  dpx/dtheta3]
        [dpy/dtheta1  dpy/dtheta2  dpy/dtheta3]
    """
    s1 = np.sin(theta1)
    s12 = np.sin(theta1 + theta2)
    s123 = np.sin(theta1 + theta2 + theta3)
    c1 = np.cos(theta1)
    c12 = np.cos(theta1 + theta2)
    c123 = np.cos(theta1 + theta2 + theta3)

    J = np.array([
        [-L1*s1 - L2*s12 - L3*s123, -L2*s12 - L3*s123, -L3*s123],
        [ L1*c1 + L2*c12 + L3*c123,  L2*c12 + L3*c123,  L3*c123]
    ])
    return J


print("\n" + "=" * 60)
print("JACOBIAN AND SINGULARITY ANALYSIS")
print("=" * 60)

J_test = compute_jacobian(*q_test)
print(f"\nJacobian at q = [0, pi/4, pi/2]:")
print(f"  {np.array2string(J_test, precision=4, suppress_small=True)}")

print(f"\nSingularity condition (2-link sub-problem):")
print(f"  det(J_sub) = L1 * L2 * sin(theta2) = 0")
print(f"  Singular when theta2 = 0 (fully extended) or theta2 = pi (fully folded)")

for t2_val, desc in [(0, "fully extended"), (np.pi, "fully folded"),
                      (q_test[1], "test config (pi/4)")]:
    det_val = L1 * L2 * np.sin(t2_val)
    print(f"  theta2 = {np.degrees(t2_val):6.1f} deg ({desc:25s}): det = {det_val:.6f}")


# ============================================================
# 6. FIGURE GENERATION
# ============================================================
print("\n" + "=" * 60)
print("GENERATING FIGURES")
print("=" * 60)


def plot_robot(ax, t1, t2, t3):
    """Helper to plot robot arm configuration."""
    xs = [0]
    ys = [0]
    xs.append(L1 * np.cos(t1))
    ys.append(L1 * np.sin(t1))
    xs.append(xs[1] + L2 * np.cos(t1 + t2))
    ys.append(ys[1] + L2 * np.sin(t1 + t2))
    xs.append(xs[2] + L3 * np.cos(t1 + t2 + t3))
    ys.append(ys[2] + L3 * np.sin(t1 + t2 + t3))

    ax.plot([xs[0], xs[1]], [ys[0], ys[1]], 'b-o', lw=3, ms=8, label='Link 1')
    ax.plot([xs[1], xs[2]], [ys[1], ys[2]], 'g-o', lw=3, ms=8, label='Link 2')
    ax.plot([xs[2], xs[3]], [ys[2], ys[3]], 'r-o', lw=3, ms=8, label='Link 3')
    ax.plot(xs[0], ys[0], 'ks', ms=12)
    return xs, ys


# Figure 1: Robot schema
print("\nFigure 1: Robot arm schema...")
fig, ax = plt.subplots(figsize=(8, 8))
xs, ys = plot_robot(ax, np.radians(30), np.radians(45), np.radians(-20))
ax.plot(xs[3], ys[3], 'r^', ms=15, label='End-Effector')
ax.annotate('Base ($\\theta_1$)', xy=(xs[0], ys[0]),
            xytext=(xs[0]-0.15, ys[0]-0.08), fontsize=11)
ax.annotate('Joint 2 ($\\theta_2$)', xy=(xs[1], ys[1]),
            xytext=(xs[1]+0.03, ys[1]+0.05), fontsize=11)
ax.annotate('Joint 3 ($\\theta_3$)', xy=(xs[2], ys[2]),
            xytext=(xs[2]+0.03, ys[2]+0.05), fontsize=11)
ax.annotate(f'End-Effector\n({xs[3]:.2f}, {ys[3]:.2f})',
            xy=(xs[3], ys[3]), xytext=(xs[3]+0.05, ys[3]-0.1),
            fontsize=10, arrowprops=dict(arrowstyle='->', color='red'))
ax.set_xlim(-1.4, 1.4); ax.set_ylim(-1.4, 1.4)
ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
ax.set_xlabel('X (m)', fontsize=12); ax.set_ylabel('Y (m)', fontsize=12)
ax.set_title('3-DOF RRR Planar Robot Arm', fontsize=14)
ax.legend(loc='upper left', fontsize=10)
plt.tight_layout()
plt.savefig('robot_arm.png', dpi=200, bbox_inches='tight')
plt.close()
print("  -> robot_arm.png saved")

# Figure 2: FK test
print("Figure 2: FK test case...")
fig, ax = plt.subplots(figsize=(8, 6))
xs, ys = plot_robot(ax, *q_test)
ax.plot(xs[3], ys[3], 'r^', ms=14)
angle2 = np.linspace(q_test[0], q_test[0]+q_test[1], 30)
angle3 = np.linspace(q_test[0]+q_test[1], sum(q_test), 30)
ax.plot(xs[1]+0.1*np.cos(angle2), ys[1]+0.1*np.sin(angle2), 'g-', lw=1.5)
ax.plot(xs[2]+0.08*np.cos(angle3), ys[2]+0.08*np.sin(angle3), 'r-', lw=1.5)
ax.annotate('$\\theta_1 = 0°$', xy=(xs[0], ys[0]),
            xytext=(xs[0]-0.05, ys[0]-0.08), fontsize=11, color='blue')
ax.annotate('$\\theta_2 = 45°$', xy=(xs[1], ys[1]),
            xytext=(xs[1]+0.05, ys[1]+0.04), fontsize=11, color='green')
ax.annotate('$\\theta_3 = 90°$', xy=(xs[2], ys[2]),
            xytext=(xs[2]+0.05, ys[2]+0.04), fontsize=11, color='red')
ax.annotate(f'End-Effector\n$p = ({xs[3]:.4f},\\; {ys[3]:.4f})$ m',
            xy=(xs[3], ys[3]), xytext=(xs[3]+0.08, ys[3]-0.12), fontsize=10,
            arrowprops=dict(arrowstyle='->', color='red', lw=1.5),
            bbox=dict(boxstyle='round,pad=0.3', facecolor='lightyellow', edgecolor='red'))
ax.set_xlim(-0.3, 1.0); ax.set_ylim(-0.3, 0.9)
ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
ax.set_xlabel('$x$ (m)', fontsize=12); ax.set_ylabel('$y$ (m)', fontsize=12)
ax.set_title('FK Test: $\\mathbf{q} = [0,\\; \\pi/4,\\; \\pi/2]^T$', fontsize=13)
ax.legend(loc='lower right', fontsize=10)
plt.tight_layout()
plt.savefig('fk_test.png', dpi=200, bbox_inches='tight')
plt.close()
print("  -> fk_test.png saved")

# Figures 3-4: IK solutions
print("Figures 3-4: IK solutions...")

def draw_ik(t1, t2, t3, title, filename, target_x, target_y):
    fig, ax = plt.subplots(figsize=(8, 6))
    xs, ys = plot_robot(ax, t1, t2, t3)
    ax.plot(target_x, target_y, 'mx', ms=15, mew=3,
            label=f'Hedef ({target_x}, {target_y})')
    ax.annotate(f'$p = ({xs[3]:.3f},\\; {ys[3]:.3f})$',
                xy=(xs[3], ys[3]), xytext=(xs[3]+0.05, ys[3]-0.12),
                fontsize=10, arrowprops=dict(arrowstyle='->', color='red'),
                bbox=dict(boxstyle='round,pad=0.2', facecolor='lightyellow',
                          edgecolor='red'))
    ax.annotate(f'$\\theta_1={np.degrees(t1):.1f}°$', xy=(xs[0], ys[0]),
                xytext=(xs[0]-0.15, ys[0]-0.08), fontsize=10, color='blue')
    ax.annotate(f'$\\theta_2={np.degrees(t2):.1f}°$', xy=(xs[1], ys[1]),
                xytext=(xs[1]+0.03, ys[1]+0.05), fontsize=10, color='green')
    ax.annotate(f'$\\theta_3={np.degrees(t3):.1f}°$', xy=(xs[2], ys[2]),
                xytext=(xs[2]+0.03, ys[2]+0.05), fontsize=10, color='red')
    ax.set_xlim(-0.5, 1.2); ax.set_ylim(-0.6, 0.8)
    ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
    ax.set_xlabel('$x$ (m)', fontsize=12); ax.set_ylabel('$y$ (m)', fontsize=12)
    ax.set_title(title, fontsize=13)
    ax.legend(loc='lower left', fontsize=10)
    plt.tight_layout()
    plt.savefig(filename, dpi=200, bbox_inches='tight')
    plt.close()

for t1, t2, t3, name in solutions:
    fname = 'ik_elbow_up.png' if 'Up' in name else 'ik_elbow_down.png'
    title = 'Dirsek-Yukarı (Elbow-Up)' if 'Up' in name else 'Dirsek-Aşağı (Elbow-Down)'
    draw_ik(t1, t2, t3, title, fname, target_px, target_py)
    print(f"  -> {fname} saved")

# Figures 5-6: Workspace
print("Figures 5-6: Workspace analysis...")

def compute_workspace(t1_range, t2_range, t3_range, n=80):
    """Compute workspace by sampling joint space with vectorized meshgrid."""
    t1 = np.linspace(*t1_range, n)
    t2 = np.linspace(*t2_range, n)
    t3 = np.linspace(*t3_range, n)
    T1, T2, T3 = np.meshgrid(t1, t2, t3, indexing='ij')
    X = L1*np.cos(T1) + L2*np.cos(T1+T2) + L3*np.cos(T1+T2+T3)
    Y = L1*np.sin(T1) + L2*np.sin(T1+T2) + L3*np.sin(T1+T2+T3)
    return X.ravel(), Y.ravel()

print("  Computing full workspace...")
x_full, y_full = compute_workspace((-np.pi, np.pi), (-np.pi, np.pi), (-np.pi, np.pi))

print("  Computing restricted workspace...")
x_restr, y_restr = compute_workspace(
    (-np.pi/2, np.pi/2), (-2*np.pi/3, 2*np.pi/3), (-np.pi/2, np.pi/2))

for xs, ys, title, color, fname in [
    (x_full, y_full,
     'Tam Dönüş Limitleri: $\\theta_i \\in [-\\pi, \\pi]$',
     'blue', 'workspace_full.png'),
    (x_restr, y_restr,
     'Kısıtlı Limitler: $\\theta_1 \\in [-\\pi/2, \\pi/2]$, '
     '$\\theta_2 \\in [-2\\pi/3, 2\\pi/3]$, $\\theta_3 \\in [-\\pi/2, \\pi/2]$',
     'green', 'workspace_restricted.png'),
]:
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.scatter(xs, ys, s=0.1, c=color, alpha=0.3)
    circle = plt.Circle((0, 0), L1+L2+L3, fill=False, color='red',
                         lw=2, ls='--', label=f'$r_{{max}}$ = {L1+L2+L3} m')
    ax.add_patch(circle)
    ax.plot(0, 0, 'ks', ms=10, label='Taban')
    ax.set_xlim(-1.5, 1.5); ax.set_ylim(-1.5, 1.5)
    ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
    ax.set_xlabel('$x$ (m)', fontsize=12); ax.set_ylabel('$y$ (m)', fontsize=12)
    ax.set_title(title, fontsize=11)
    ax.legend(loc='upper right', fontsize=11)
    plt.tight_layout()
    plt.savefig(fname, dpi=200, bbox_inches='tight')
    plt.close()
    print(f"  -> {fname} saved")

print("\n" + "=" * 60)
print("ALL COMPUTATIONS AND FIGURES COMPLETE")
print("=" * 60)

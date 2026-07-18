#!/usr/bin/env python3
"""
pose_graph.py - the SLAM BACK-END for nav.py.

Pure math, NO hardware imports, so it unit-tests on its own. It takes a 2D pose
graph - SE(2) nodes (robot poses) joined by relative-transform edges - and runs
Gauss-Newton least squares to find the node poses that best satisfy every edge
at once. This is the step that defines graph SLAM: dead-reckoned odometry drifts
without bound, but when a LOOP-CLOSURE edge says "you are back where you started"
the optimizer spreads that correction back over the whole trajectory and the map
becomes globally consistent.

Math follows Grisetti, Kummerle, Stachniss & Burgard, "A Tutorial on Graph-Based
SLAM" (IEEE ITS Magazine, 2010): the SE(2) error function and its analytic
Jacobians A = de/dxi, B = de/dxj.

nav.py owns the front-end (build nodes from keyframe poses, odometry edges from
dead reckoning, loop edges from ORB matching) and persists everything in
world.json under "pose_graph". This module just optimizes.

Run `python3 pose_graph.py` for a synthetic self-test (a noisy square loop that
gets pulled shut by one loop closure) - no robot needed.
"""
import math

import numpy as np


# --- SE(2) helpers ----------------------------------------------------------
def normalize_angle(a):
    """Wrap a radian angle to (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def v2t(v):
    """Pose vector (x, y, theta) -> 3x3 SE(2) homogeneous transform."""
    c, s = math.cos(v[2]), math.sin(v[2])
    return np.array([[c, -s, v[0]],
                     [s,  c, v[1]],
                     [0.0, 0.0, 1.0]])


def t2v(T):
    """3x3 SE(2) transform -> pose vector (x, y, theta)."""
    return np.array([T[0, 2], T[1, 2], math.atan2(T[1, 0], T[0, 0])])


def relative(xi, xj):
    """Relative transform of pose xj expressed in pose xi's frame, as a vector.
    This is the measurement an ideal odometry/loop edge from i to j would carry."""
    return t2v(np.linalg.inv(v2t(xi)) @ v2t(xj))


# --- Edge -------------------------------------------------------------------
class Edge:
    """A constraint: from node i, node j was observed at relative pose z
    (dx, dy, dtheta in i's frame), with information matrix omega (3x3, the
    inverse covariance - bigger = more trusted). kind is just a label."""

    __slots__ = ("i", "j", "z", "omega", "kind")

    def __init__(self, i, j, z, omega, kind="odometry"):
        self.i = int(i)
        self.j = int(j)
        self.z = np.asarray(z, dtype=float).reshape(3)
        self.omega = np.asarray(omega, dtype=float).reshape(3, 3)
        self.kind = kind


def information(sigma_xy_mm, sigma_theta_deg):
    """Diagonal information matrix from 1-sigma translation (mm) and rotation
    (deg) uncertainties. Convenience for building edges."""
    st = math.radians(sigma_theta_deg)
    return np.diag([1.0 / sigma_xy_mm**2, 1.0 / sigma_xy_mm**2, 1.0 / st**2])


# --- Linearization (Grisetti SE(2) analytic Jacobians) ----------------------
def _linearize_edge(xi, xj, e):
    """Return (err, A, B): the 3-vector error of edge e at the current node
    estimates xi, xj, and the Jacobians A=de/dxi, B=de/dxj (each 3x3)."""
    ti = xi[2]
    ci, si = math.cos(ti), math.sin(ti)
    RiT = np.array([[ci, si], [-si, ci]])            # R_i^T
    dRiT = np.array([[-si, ci], [-ci, -si]])         # d(R_i^T)/d(theta_i)
    zt = e.z[2]
    cz, sz = math.cos(zt), math.sin(zt)
    RzT = np.array([[cz, sz], [-sz, cz]])            # R_z^T

    tij = np.array([xj[0] - xi[0], xj[1] - xi[1]])   # t_j - t_i
    e_trans = RzT @ (RiT @ tij - e.z[:2])
    e_rot = normalize_angle(xj[2] - xi[2] - zt)
    err = np.array([e_trans[0], e_trans[1], e_rot])

    A = np.zeros((3, 3))
    A[:2, :2] = -RzT @ RiT
    A[:2, 2] = RzT @ dRiT @ tij
    A[2, 2] = -1.0
    B = np.zeros((3, 3))
    B[:2, :2] = RzT @ RiT
    B[2, 2] = 1.0
    return err, A, B


def chi2(nodes, edges):
    """Total weighted squared error sum(e^T omega e) - the cost being minimized."""
    total = 0.0
    for e in edges:
        err, _, _ = _linearize_edge(nodes[e.i], nodes[e.j], e)
        total += float(err @ e.omega @ err)
    return total


# --- Gauss-Newton optimizer -------------------------------------------------
def optimize(nodes, edges, iterations=100, tol=1e-5, fix_node=0, verbose=False):
    """Least-squares pose-graph optimization.

    nodes    : (N,3) array of initial pose estimates (mutated copy returned).
    edges    : list[Edge].
    fix_node : index of the gauge-fixed node (the world anchor); its pose is
               held so the problem is well-posed.
    Returns (optimized_nodes (N,3), info dict with chi2 history & iterations).
    """
    x = np.array(nodes, dtype=float).copy()
    n = len(x)
    hist = [chi2(x, edges)]
    used_iters = 0
    for it in range(iterations):
        H = np.zeros((3 * n, 3 * n))
        b = np.zeros(3 * n)
        for e in edges:
            err, A, B = _linearize_edge(x[e.i], x[e.j], e)
            oi, oj = 3 * e.i, 3 * e.j
            AtO = A.T @ e.omega
            BtO = B.T @ e.omega
            H[oi:oi + 3, oi:oi + 3] += AtO @ A
            H[oi:oi + 3, oj:oj + 3] += AtO @ B
            H[oj:oj + 3, oi:oi + 3] += BtO @ A
            H[oj:oj + 3, oj:oj + 3] += BtO @ B
            b[oi:oi + 3] += AtO @ err
            b[oj:oj + 3] += BtO @ err
        # Gauge fix: pin the anchor node by making its block dominant.
        a = 3 * fix_node
        H[a:a + 3, a:a + 3] += np.eye(3) * 1e12
        try:
            dx = np.linalg.solve(H, -b)
        except np.linalg.LinAlgError:
            dx = np.linalg.lstsq(H, -b, rcond=None)[0]
        x += dx.reshape(n, 3)
        for k in range(n):
            x[k, 2] = normalize_angle(x[k, 2])
        c = chi2(x, edges)
        hist.append(c)
        used_iters = it + 1
        if verbose:
            print(f"  iter {it:2d}: chi2={c:.4g}  |dx|={np.linalg.norm(dx):.4g}")
        if np.linalg.norm(dx) < tol:
            break
    return x, {"chi2_start": hist[0], "chi2_end": hist[-1],
               "iterations": used_iters, "history": hist}


# --- Self-test: a noisy square loop pulled shut by one loop closure ---------
def _selftest():
    rng = np.random.default_rng(1)
    side = 1000.0            # mm
    steps_per_side = 3
    turn = math.pi / 2.0

    # Ground-truth closed square trajectory (nodes at each step).
    truth = [np.array([0.0, 0.0, 0.0])]
    heading = 0.0
    for s in range(4):
        for _ in range(steps_per_side):
            p = truth[-1].copy()
            step = side / steps_per_side
            p[0] += step * math.cos(heading)
            p[1] += step * math.sin(heading)
            truth.append(p)
        heading = normalize_angle(heading + turn)
        truth[-1][2] = heading
    truth = np.array(truth)
    n = len(truth)

    # Noisy odometry edges. A wheeled car with no encoders drifts SYSTEMATICALLY
    # in heading (a small consistent turn bias), not just randomly - so each edge
    # over-reports its rotation by a fixed bias plus a little random noise. This
    # makes the integrated trajectory spiral open: exactly the drift a loop
    # closure is meant to undo.
    trans_sigma, rot_sigma = 8.0, math.radians(1.0)
    heading_bias = math.radians(2.5)                 # systematic per-edge drift
    odom_omega = information(trans_sigma, 2.5)
    edges, guess = [], [truth[0].copy()]
    for i in range(n - 1):
        z = relative(truth[i], truth[i + 1])
        z_noisy = z + rng.normal(0, [trans_sigma, trans_sigma, rot_sigma])
        z_noisy[2] += heading_bias
        edges.append(Edge(i, i + 1, z_noisy, odom_omega, "odometry"))
        guess.append(t2v(v2t(guess[-1]) @ v2t(z_noisy)))   # integrate -> drifts
    guess = np.array(guess)

    # One loop closure: last node coincides with the first (same place).
    loop_omega = information(15.0, 1.5)
    z_loop = relative(truth[-1], truth[0]) + rng.normal(0, [15, 15, math.radians(1.5)])
    edges.append(Edge(n - 1, 0, z_loop, loop_omega, "loop"))

    opt, info = optimize(guess, edges, verbose=True)

    def rms(a):
        return math.sqrt(np.mean(np.sum((a[:, :2] - truth[:, :2]) ** 2, axis=1)))

    print("\nsynthetic square loop, %d nodes, 1 loop closure" % n)
    print(f"  chi2:            {info['chi2_start']:.1f} -> {info['chi2_end']:.3f} "
          f"in {info['iterations']} iters")
    print(f"  RMS pos error:   {rms(guess):.1f} mm (odometry) -> "
          f"{rms(opt):.1f} mm (optimized)")
    print(f"  loop gap:        {np.linalg.norm(guess[-1,:2]-guess[0,:2]):.1f} mm "
          f"-> {np.linalg.norm(opt[-1,:2]-opt[0,:2]):.1f} mm")
    ok = rms(opt) < rms(guess) * 0.5 and info["chi2_end"] < info["chi2_start"]
    print("  RESULT:", "PASS - loop closure corrected the drift" if ok else "FAIL")
    return ok


if __name__ == "__main__":
    import sys
    sys.exit(0 if _selftest() else 1)

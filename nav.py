#!/usr/bin/env python3
"""
nav.py - a minimal, SLAM-EXTENSIBLE navigation scaffold for the 3-wheeled car.

This is deliberately NOT a full SLAM. The hardware has no wheel encoders and no
lidar, and the only camera is monocular and mounted on the arm's wrist. So real
localization-and-mapping isn't achievable. What this DOES give you is the exact
data substrate a SLAM system is built on, so it can be grown into one later:

  * a robot POSE estimate (x, y, theta) carried in a persistent world file -
    this is the "localization" state a SLAM back-end would correct;
  * a MOTION MODEL: every drive updates the pose by dead reckoning, using the
    car's measured speed->distance calibration (car.py). This is the SLAM
    "predict" step;
  * OBSERVATIONS: at a pose we sweep the neck (arm servo 6) across bearings and
    save one camera frame per bearing, each tagged with (pose, world_bearing,
    time). This is the SLAM "sense" step and the raw material for landmarks;
  * a structured MAP FILE (world.json): a pose graph of keyframes + observations
    + a reserved landmarks list.

HOW TO GROW THIS INTO SLAM later (the whole point of the structure):
  1. Feature front-end: run ORB/AKAZE on each saved frame, store descriptors in
     the observation. -> visual landmarks.
  2. Data association: match features across keyframes -> landmark tracks.
  3. Back-end: feed the pose graph + landmark constraints to an optimizer
     (g2o / GTSAM / a hand-rolled EKF or particle filter) and REPLACE the dead-
     reckoned pose in each keyframe with the optimized one.
  4. Loop closure: when a new keyframe matches an old one, add a constraint.
The commands here (init / forward / scan / map) already produce exactly the
keyframe+observation records those steps consume.

Coordinate frame: world-fixed. Robot starts at the origin facing +x.
  x = forward-at-start (mm), y = left (mm), theta = heading in degrees CCW.
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import car  # noqa: E402  (car.py lives beside this file)

# NAV_STATE_DIR lets you keep several maps / run offline tests without touching
# the live map (defaults to nav_state/ beside this file).
STATE_DIR = os.environ.get("NAV_STATE_DIR", os.path.join(HERE, "nav_state"))
FRAMES_DIR = os.path.join(STATE_DIR, "frames")
WORLD = os.path.join(STATE_DIR, "world.json")

VENV_PY = "/home/astra/tools/venv/bin/python3"
ARM_PY = os.path.join(HERE, "arm.py")

# --- Neck (arm servo 6) geometry -------------------------------------------
# arm.py: servo units 0..1000 map linearly to -125..+125 deg, 500 = 0 deg.
NECK_SERVO = 6
NECK_UNITS_PER_DEG = 1000.0 / 250.0          # = 4.0 units per degree
NECK_SIGN = +1        # flip to -1 if a +neck command turns the view the other way
NECK_MAX_DEG = 105    # servo6 base rotation; ±100 tested safe 2026-07-18, 105 = margin


def _neck_pos(neck_deg):
    pos = int(round(500 + NECK_SIGN * neck_deg * NECK_UNITS_PER_DEG))
    return max(0, min(1000, pos))


def _arm(*args, timeout=30):
    """Call arm.py (needs root for the HID device)."""
    cmd = ["sudo", VENV_PY, ARM_PY, *[str(a) for a in args]]
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)


# --- Lookout ("periscope") pose --------------------------------------------
# Raises the wrist camera off the floor so a scan sees the ROOM (walls, doors,
# furniture) instead of just floor tiles. Found empirically 2026-07-17 by
# snapshot feedback; servo ids are arm.py's numbering (3=shoulder, 4=elbow,
# 5=wrist_pitch). Tune here if the mount or arm base moves.
LOOKOUT_POSE = {"3": 500, "4": 848, "5": 514}


def raise_to_lookout():
    moves = ",".join(f"{s}:{p}" for s, p in LOOKOUT_POSE.items())
    tmp = os.path.join(STATE_DIR, "_lookout.jpg")
    os.makedirs(STATE_DIR, exist_ok=True)
    r = _arm("step", moves, tmp, 1400)
    if r.returncode != 0:
        print(f"lookout raise FAILED: {r.stderr.strip()[:160]}")
    return r.returncode == 0


# --- ORB visual-feature front-end (the bridge toward real SLAM) -------------
def extract_orb(image_path, nfeatures=500):
    """Return (n_keypoints, keypoints, descriptors) for one frame, or (0,None,
    None). ORB descriptors are the raw material for visual landmarks: match them
    across frames to get correspondences, then a SLAM back-end turns those into
    optimized landmark positions."""
    import cv2
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        return 0, None, None
    orb = cv2.ORB_create(nfeatures=nfeatures)
    kp, des = orb.detectAndCompute(img, None)
    return len(kp), kp, des


# --- World model persistence -----------------------------------------------
def _now():
    return time.strftime("%Y-%m-%d %H:%M:%S")


def load_world():
    if not os.path.exists(WORLD):
        raise SystemExit("no world yet - run `nav.py init` first")
    with open(WORLD) as f:
        return json.load(f)


def save_world(w):
    os.makedirs(STATE_DIR, exist_ok=True)
    tmp = WORLD + ".tmp"
    with open(tmp, "w") as f:
        json.dump(w, f, indent=2)
    os.replace(tmp, WORLD)


def new_world():
    return {
        "created": _now(),
        "frame": "x=forward-at-start(mm), y=left(mm), theta=deg CCW",
        "pose": {"x_mm": 0.0, "y_mm": 0.0, "theta_deg": 0.0},
        "trajectory": [{"x_mm": 0.0, "y_mm": 0.0, "theta_deg": 0.0,
                        "t": _now(), "note": "init"}],
        "keyframes": [],
        "landmarks": [],          # reserved for the SLAM front-end
        # SLAM back-end state. "edges" are relative-pose constraints between
        # keyframes: odometry edges (raw dead-reckoned relative transform,
        # captured once at scan time and never overwritten) plus loop-closure
        # edges added by `loop-detect`. `optimize` feeds them to pose_graph.py.
        # z is [dx_mm, dy_mm, dtheta_deg] in the FROM keyframe's frame.
        "pose_graph": {"edges": [], "last_optimize": None},
        "map": {"landmarks": []},  # fused/consolidated landmark cloud
        "calibration": {
            "cal_speed": car.CAL_SPEED, "cal_seconds": car.CAL_SECONDS,
            "cal_mm": car.CAL_MM, "note": "open-loop dead reckoning, no odometry",
        },
    }


# --- Pose-graph plumbing (front-end side; math lives in pose_graph.py) ------
def _pose_graph(w):
    """Return w['pose_graph'], creating it for worlds made before the back-end."""
    pg = w.setdefault("pose_graph", {"edges": [], "last_optimize": None})
    pg.setdefault("edges", [])
    pg.setdefault("last_optimize", None)
    w.setdefault("map", {"landmarks": []})
    return pg


def _rel_transform(pa, pb):
    """Relative transform of pose pb in pose pa's frame -> [dx_mm, dy_mm,
    dtheta_deg]. Pure trig so this file needs no numpy for edge bookkeeping."""
    dth = math.radians(pa["theta_deg"])
    c, s = math.cos(dth), math.sin(dth)
    dx, dy = pb["x_mm"] - pa["x_mm"], pb["y_mm"] - pa["y_mm"]
    return [c * dx + s * dy, -s * dx + c * dy,
            (pb["theta_deg"] - pa["theta_deg"] + 180) % 360 - 180]


def _compose(pa, r):
    """World pose reached by applying relative transform r=[dx,dy,dtheta_deg]
    (in pose pa's frame) to pose pa. Inverse of _rel_transform."""
    th = math.radians(pa["theta_deg"])
    c, s = math.cos(th), math.sin(th)
    return {"x_mm": pa["x_mm"] + c * r[0] - s * r[1],
            "y_mm": pa["y_mm"] + s * r[0] + c * r[1],
            "theta_deg": (pa["theta_deg"] + r[2] + 180) % 360 - 180}


def _add_odometry_edge(w, i, j):
    """Record the dead-reckoned relative transform between keyframes i and j as
    an odometry constraint. Captured NOW, while both poses are still raw dead
    reckoning, so re-optimizing never feeds corrected poses back in as data."""
    pg = _pose_graph(w)
    z = _rel_transform(w["keyframes"][i]["pose"], w["keyframes"][j]["pose"])
    pg["edges"].append({"i": i, "j": j, "z": [round(v, 2) for v in z],
                        "sigma_xy_mm": 40.0, "sigma_theta_deg": 4.0,
                        "kind": "odometry", "t": _now()})


def _kf_frame(kf):
    """Pick the representative frame of a keyframe for place recognition: the
    forward (neck ~0) view, else the one closest to straight ahead."""
    obs = kf.get("observations", [])
    if not obs:
        return None
    o = min(obs, key=lambda o: abs(o.get("neck_deg", 0)))
    return o.get("image")


# --- Commands ---------------------------------------------------------------
def cmd_init(_args):
    os.makedirs(FRAMES_DIR, exist_ok=True)
    save_world(new_world())
    print(f"world reset at origin -> {WORLD}")


def cmd_pose(_args):
    p = load_world()["pose"]
    print(f"x={p['x_mm']:.0f} mm  y={p['y_mm']:.0f} mm  "
          f"theta={p['theta_deg']:.1f} deg")


def _apply_motion(w, mm, sign):
    """Update the dead-reckoned pose after driving |mm| along heading*sign."""
    th = math.radians(w["pose"]["theta_deg"])
    w["pose"]["x_mm"] += sign * mm * math.cos(th)
    w["pose"]["y_mm"] += sign * mm * math.sin(th)
    w["trajectory"].append({
        "x_mm": round(w["pose"]["x_mm"], 1), "y_mm": round(w["pose"]["y_mm"], 1),
        "theta_deg": w["pose"]["theta_deg"], "t": _now(),
        "note": f"{'forward' if sign > 0 else 'back'} {mm:.0f}mm",
    })


def cmd_forward(args):
    w = load_world()
    car.steer("center", 90)
    time.sleep(0.2)
    car.drive_mm("forward", args.mm)
    _apply_motion(w, float(args.mm), +1)
    save_world(w)
    cmd_pose(None)


def cmd_back(args):
    w = load_world()
    car.steer("center", 90)
    time.sleep(0.2)
    car.drive_mm("backward", args.mm)
    _apply_motion(w, float(args.mm), -1)
    save_world(w)
    cmd_pose(None)


def cmd_set_heading(args):
    """Manual pose correction hook - the stand-in for a SLAM back-end until one
    exists. Tell the scaffold the robot's true heading after a turn."""
    w = load_world()
    w["pose"]["theta_deg"] = float(args.deg)
    w["trajectory"].append({**w["pose"], "t": _now(),
                            "note": f"set-heading {args.deg}"})
    save_world(w)
    cmd_pose(None)


def cmd_scan(args):
    w = load_world()
    os.makedirs(FRAMES_DIR, exist_ok=True)
    if args.lookout:
        print("raising camera to lookout pose...")
        raise_to_lookout()
        time.sleep(0.4)
    angles = [float(a) for a in args.neck.split(",")]
    kf_id = len(w["keyframes"])
    pose = dict(w["pose"])
    obs = []
    print(f"keyframe {kf_id} at x={pose['x_mm']:.0f} y={pose['y_mm']:.0f} "
          f"theta={pose['theta_deg']:.0f}; neck sweep {angles}")
    for a in angles:
        a = max(-NECK_MAX_DEG, min(NECK_MAX_DEG, a))
        r = _arm("move", NECK_SERVO, _neck_pos(a), 700)
        if r.returncode != 0:
            print(f"  neck {a:+.0f}: arm move FAILED: {r.stderr.strip()[:120]}")
            continue
        time.sleep(args.settle)
        fname = f"kf{kf_id}_n{int(a):+03d}.jpg"
        fpath = os.path.join(FRAMES_DIR, fname)
        try:
            size = car.snapshot(fpath)
        except Exception as e:  # noqa: BLE001
            print(f"  neck {a:+.0f}: snapshot FAILED: {e}")
            continue
        bearing = pose["theta_deg"] + NECK_SIGN * a
        ob = {"neck_deg": a, "world_bearing_deg": round(bearing, 1),
              "image": fname, "range_mm": None, "t": _now()}
        # visual-feature front-end: store ORB descriptors for future matching
        try:
            import numpy as np
            n, _kp, des = extract_orb(fpath)
            ob["orb_keypoints"] = n
            if des is not None:
                np.save(os.path.join(FRAMES_DIR, fname + ".orb.npy"), des)
                ob["descriptors_file"] = fname + ".orb.npy"
        except Exception as e:  # noqa: BLE001
            ob["orb_keypoints"] = None
            print(f"    (ORB skipped: {e})")
        obs.append(ob)
        print(f"  neck {a:+.0f} -> bearing {bearing:+.0f} : {fname} "
              f"({size} B, {ob.get('orb_keypoints')} ORB kp)")
    # recentre the neck
    _arm("move", NECK_SERVO, _neck_pos(0), 700)
    w["keyframes"].append({"id": kf_id, "pose": pose, "observations": obs,
                           "t": _now()})
    if kf_id > 0:                       # link to the previous keyframe by odometry
        _add_odometry_edge(w, kf_id - 1, kf_id)
    save_world(w)
    print(f"keyframe {kf_id} saved: {len(obs)} observations"
          + (f", odometry edge {kf_id-1}->{kf_id} added" if kf_id > 0 else ""))


# --- Metric triangulation (feature matches -> world landmarks) --------------
CAMERA_FOV_DEG = 60.0   # assumed horizontal FOV - NO calibration; tune if known


def _intrinsics(w_px, h_px, fov_deg=CAMERA_FOV_DEG):
    fx = (w_px / 2.0) / math.tan(math.radians(fov_deg) / 2.0)
    return fx, fx, w_px / 2.0, h_px / 2.0    # fx, fy(=fx), cx, cy


def cmd_motionmap(args):
    """Forward-motion monocular triangulation: look forward, drive a known
    baseline, and turn the radial 'looming' of matched ORB features into metric
    depths (Z = k*B/(k-1), k = radius_B/radius_A). Metric scale comes from the
    dead-reckoned baseline. This is the step that converts feature MATCHES into
    world-coordinate LANDMARKS - the actual map. Rough: no camera calibration
    (assumed FOV) and a dead-reckoned baseline, so treat distances as ballpark."""
    import cv2
    import numpy as np
    w = load_world()
    if args.lookout:
        raise_to_lookout()
        time.sleep(0.4)
    _arm("move", NECK_SERVO, _neck_pos(0), 800)   # aim forward
    time.sleep(0.6)
    poseA = dict(w["pose"])
    fa = os.path.join(FRAMES_DIR, "mm_A.jpg")
    car.snapshot(fa)
    car.steer("center", 90)
    time.sleep(0.2)
    car.drive_mm("forward", args.baseline)
    _apply_motion(w, float(args.baseline), +1)
    fb = os.path.join(FRAMES_DIR, "mm_B.jpg")
    car.snapshot(fb)

    ga = cv2.cvtColor(cv2.imread(fa), cv2.COLOR_BGR2GRAY)
    gb = cv2.cvtColor(cv2.imread(fb), cv2.COLOR_BGR2GRAY)
    orb = cv2.ORB_create(1200)
    ka, da = orb.detectAndCompute(ga, None)
    kb, db = orb.detectAndCompute(gb, None)
    if da is None or db is None:
        raise SystemExit("no features to triangulate")
    matches = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True).match(da, db)
    ptsA = np.float32([ka[m.queryIdx].pt for m in matches])
    ptsB = np.float32([kb[m.trainIdx].pt for m in matches])
    if len(ptsA) < 8:
        raise SystemExit(f"only {len(ptsA)} matches - not enough to triangulate")
    _F, mask = cv2.findFundamentalMat(ptsA, ptsB, cv2.FM_RANSAC, 2.0, 0.99)
    if mask is not None:
        keep = mask.ravel().astype(bool)
        ptsA, ptsB = ptsA[keep], ptsB[keep]

    H, Wd = ga.shape
    fx, _fy, cx, cy = _intrinsics(Wd, H, args.fov)
    B = float(args.baseline)
    phi = math.radians(poseA["theta_deg"])       # forward bearing (neck 0)
    fwd = (math.cos(phi), math.sin(phi))
    rgt = (math.sin(phi), -math.cos(phi))         # image +x (right) -> world
    landmarks, depths = [], []
    for (uA, vA), (uB, vB) in zip(ptsA, ptsB):
        rA = math.hypot(uA - cx, vA - cy)
        rB = math.hypot(uB - cx, vB - cy)
        if rA < 20:                    # too near focus of expansion -> unstable
            continue
        k = rB / rA
        if k <= 1.02:                  # must expand (a point really ahead)
            continue
        Z = k * B / (k - 1.0)          # depth along optical axis (mm)
        if not (150 < Z < 5000):
            continue
        Xc = (uA - cx) * Z / fx
        wx = poseA["x_mm"] + Z * fwd[0] + Xc * rgt[0]
        wy = poseA["y_mm"] + Z * fwd[1] + Xc * rgt[1]
        landmarks.append({"x_mm": round(float(wx), 1), "y_mm": round(float(wy), 1),
                          "z_mm": round(float(Z), 1), "src": "motionmap",
                          "from_x": round(poseA["x_mm"], 1),
                          "from_y": round(poseA["y_mm"], 1)})
        depths.append(float(Z))
    w["landmarks"].extend(landmarks)
    save_world(w)
    print(f"triangulated {len(landmarks)} landmarks from {len(ptsA)} inlier "
          f"matches (baseline {B:.0f}mm, fov {args.fov:.0f}deg)")
    if depths:
        print(f"  depth range {min(depths):.0f}-{max(depths):.0f} mm, "
              f"median {sorted(depths)[len(depths)//2]:.0f} mm")
    if args.png:
        render_map_png(w, args.png)
        print(f"landmark map -> {args.png}")


def cmd_lookout(args):
    ok = raise_to_lookout()
    if ok and args.snapshot:
        car.snapshot(args.snapshot)
        print(f"lookout snapshot -> {args.snapshot}")
    print("camera raised to lookout pose" if ok else "lookout raise failed")


def cmd_landmarks(args):
    """Match ORB features between ADJACENT overlapping frames of a keyframe and
    draw the correspondences. This is the visual front-end of SLAM: these
    matched points are candidate landmarks; a back-end would triangulate them
    across poses and optimize. Demonstrates the pipeline works on real frames."""
    import cv2
    w = load_world()
    if not w["keyframes"]:
        raise SystemExit("no keyframes - run a scan first")
    kf = w["keyframes"][args.kf]
    obs = sorted(kf["observations"], key=lambda o: o["neck_deg"])
    orb = cv2.ORB_create(nfeatures=800)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    panels, total = [], 0
    for a, b in zip(obs, obs[1:]):
        ia = cv2.imread(os.path.join(FRAMES_DIR, a["image"]))
        ib = cv2.imread(os.path.join(FRAMES_DIR, b["image"]))
        if ia is None or ib is None:
            continue
        ka, da = orb.detectAndCompute(cv2.cvtColor(ia, cv2.COLOR_BGR2GRAY), None)
        kb, db = orb.detectAndCompute(cv2.cvtColor(ib, cv2.COLOR_BGR2GRAY), None)
        if da is None or db is None:
            continue
        m = sorted(bf.match(da, db), key=lambda x: x.distance)[:args.top]
        total += len(m)
        print(f"  bearing {a['world_bearing_deg']:+.0f} <-> "
              f"{b['world_bearing_deg']:+.0f} : {len(m)} matched features")
        if args.png:
            panels.append(cv2.drawMatches(ia, ka, ib, kb, m, None,
                          flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS))
    print(f"total adjacent-view feature matches: {total}")
    if args.png and panels:
        h = max(p.shape[0] for p in panels)
        widths = [p.shape[1] for p in panels]
        canvas = cv2.copyMakeBorder(panels[0], 0, h - panels[0].shape[0], 0,
                                    max(widths) - widths[0], cv2.BORDER_CONSTANT)
        for p in panels[1:]:
            p2 = cv2.copyMakeBorder(p, 0, h - p.shape[0], 0,
                                    max(widths) - p.shape[1], cv2.BORDER_CONSTANT)
            canvas = cv2.vconcat([canvas, p2])
        cv2.imwrite(args.png, canvas)
        print(f"match visualization -> {args.png}")


def cmd_map(args):
    w = load_world()
    p = w["pose"]
    print(f"=== world model: {WORLD} ===")
    print(f"created {w['created']}")
    print(f"pose   x={p['x_mm']:.0f} y={p['y_mm']:.0f} theta={p['theta_deg']:.0f}")
    print(f"trajectory points: {len(w['trajectory'])}")
    print(f"keyframes: {len(w['keyframes'])}  landmarks: {len(w['landmarks'])}"
          f"  map(fused): {len(w.get('map', {}).get('landmarks', []))}")
    pg = w.get("pose_graph", {})
    edges = pg.get("edges", [])
    n_loop = sum(1 for e in edges if e.get("kind") == "loop")
    if edges:
        print(f"pose graph: {len(edges)} edges ({n_loop} loop closure"
              f"{'s' if n_loop != 1 else ''})")
    if pg.get("last_optimize"):
        lo = pg["last_optimize"]
        print(f"last optimize: chi2 {lo['chi2_start']}->{lo['chi2_end']}, "
              f"max shift {lo['max_correction_mm']}mm @ {lo['t']}")
    for kf in w["keyframes"]:
        kp = kf["pose"]
        print(f"  kf{kf['id']} @({kp['x_mm']:.0f},{kp['y_mm']:.0f}) "
              f"{len(kf['observations'])} obs: "
              f"{[o['world_bearing_deg'] for o in kf['observations']]}")
    if args.png:
        render_map_png(w, args.png)
        print(f"map image -> {args.png}")
    if args.panorama:
        kf = w["keyframes"][args.kf] if w["keyframes"] else None
        if kf:
            render_panorama(kf, args.panorama)
            print(f"panorama (kf{kf['id']}) -> {args.panorama}")


# --- Loop closure (place recognition -> graph edges) ------------------------
def cmd_loop_detect(args):
    """Place recognition + loop closure. Compare every NON-consecutive keyframe
    pair by ORB matching (Lowe ratio) with essential-matrix geometric
    verification. A strong match means the robot is seeing a place it saw
    before - a LOOP CLOSURE - the constraint that lets `optimize` undo drift.

    Metric scale is unobservable from one uncalibrated camera, so a closure is
    modeled as 'recognized as the same place, with a measured heading change':
    translation ~0 within a place-recognition radius (loose sigma), rotation
    from the essential matrix (tight sigma). Use --rotation-only for the
    conservative case where you don't want to assert co-location, just heading.
    """
    import cv2
    import numpy as np
    w = load_world()
    kfs = w["keyframes"]
    if len(kfs) < 3:
        raise SystemExit(f"need >=3 keyframes for loop closure, have {len(kfs)}")
    pg = _pose_graph(w)
    orb = cv2.ORB_create(1000)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING)

    def feats(img_name):
        g = cv2.imread(os.path.join(FRAMES_DIR, img_name), cv2.IMREAD_GRAYSCALE)
        if g is None:
            return None, None, None
        k, d = orb.detectAndCompute(g, None)
        return g, k, d

    found, candidates = 0, 0
    for i in range(len(kfs)):
        for j in range(i + 2, len(kfs)):     # consecutive pairs -> odometry
            fi, fj = _kf_frame(kfs[i]), _kf_frame(kfs[j])
            if not fi or not fj:
                continue
            gi, ki, di = feats(fi)
            gj, kj, dj = feats(fj)
            if di is None or dj is None or len(ki) < 8 or len(kj) < 8:
                continue
            pairs = [p for p in bf.knnMatch(di, dj, k=2) if len(p) == 2]
            good = [m for m, n in pairs if m.distance < 0.75 * n.distance]
            if len(good) < args.min_matches:
                continue
            ptsI = np.float32([ki[m.queryIdx].pt for m in good])
            ptsJ = np.float32([kj[m.trainIdx].pt for m in good])
            fx, _fy, cx, cy = _intrinsics(gi.shape[1], gi.shape[0], args.fov)
            K = np.array([[fx, 0, cx], [0, fx, cy], [0, 0, 1.0]])
            E, mask = cv2.findEssentialMat(ptsI, ptsJ, K, cv2.RANSAC, 0.999, 1.0)
            if E is None or E.shape != (3, 3):
                continue
            inliers = int(mask.sum()) if mask is not None else 0
            if inliers < args.min_matches:
                continue
            _, R, _t, _ = cv2.recoverPose(E, ptsI, ptsJ, K, mask=mask)
            # yaw about the camera's vertical (down) axis -> robot heading change
            dtheta = -math.degrees(math.atan2(R[0, 2], R[2, 2]))
            candidates += 1
            sig_xy = 1e5 if args.rotation_only else args.place_radius
            edge = {"i": i, "j": j, "z": [0.0, 0.0, round(dtheta, 2)],
                    "sigma_xy_mm": sig_xy, "sigma_theta_deg": 5.0,
                    "kind": "loop", "matches": inliers, "t": _now()}
            print(f"  loop candidate kf{i} <-> kf{j}: {inliers} verified inliers, "
                  f"dtheta ~ {dtheta:+.1f} deg")
            if args.add:
                pg["edges"] = [e for e in pg["edges"]
                               if not (e["kind"] == "loop"
                                       and {e["i"], e["j"]} == {i, j})]
                pg["edges"].append(edge)
                found += 1
    print(f"{candidates} loop candidate(s) over {len(kfs)} keyframes")
    if args.add:
        save_world(w)
        print(f"committed {found} loop-closure edge(s) - run `optimize` next")
    else:
        print("(dry run; pass --add to commit these as graph edges)")


# --- Back-end (pose-graph optimization) -------------------------------------
def cmd_optimize(args):
    """Run the SLAM back-end: feed all graph edges (odometry + loop closures)
    to pose_graph.py's Gauss-Newton optimizer and replace every keyframe's
    dead-reckoned pose with the globally consistent estimate. The original dead
    reckoning is kept in each keyframe as 'pose_odom'."""
    import numpy as np
    import pose_graph as pgm
    w = load_world()
    kfs = w["keyframes"]
    if len(kfs) < 2:
        raise SystemExit(f"need >=2 keyframes to optimize, have {len(kfs)}")
    pg = _pose_graph(w)
    spec = pg["edges"]
    if not spec:
        raise SystemExit("no edges yet - scan more keyframes and/or loop-detect --add")

    odom_poses = [dict(k["pose"]) for k in kfs]      # snapshot before overwrite
    nodes = np.array([[p["x_mm"], p["y_mm"], math.radians(p["theta_deg"])]
                      for p in odom_poses])
    edges = [pgm.Edge(e["i"], e["j"],
                      [e["z"][0], e["z"][1], math.radians(e["z"][2])],
                      pgm.information(e["sigma_xy_mm"], e["sigma_theta_deg"]),
                      e["kind"]) for e in spec]
    n_loops = sum(1 for e in spec if e["kind"] == "loop")
    opt, info = pgm.optimize(nodes, edges, verbose=args.verbose)

    print(f"optimized {len(kfs)} keyframes over {len(edges)} edges "
          f"({n_loops} loop closure{'s' if n_loops != 1 else ''})")
    print(f"  chi2 {info['chi2_start']:.4g} -> {info['chi2_end']:.4g} "
          f"in {info['iterations']} iterations")
    moved = 0.0
    for k, p_odom, row in zip(kfs, odom_poses, opt):
        newp = {"x_mm": round(float(row[0]), 1), "y_mm": round(float(row[1]), 1),
                "theta_deg": round(math.degrees(row[2]), 1)}
        moved = max(moved, math.hypot(newp["x_mm"] - p_odom["x_mm"],
                                      newp["y_mm"] - p_odom["y_mm"]))
        k.setdefault("pose_odom", p_odom)             # preserve dead reckoning
        k["pose"] = newp
    # carry the correction forward from the last keyframe to the live pose
    opt_last = {"x_mm": float(opt[-1][0]), "y_mm": float(opt[-1][1]),
                "theta_deg": math.degrees(opt[-1][2])}
    rel_live = _rel_transform(odom_poses[-1], w["pose"])
    live = _compose(opt_last, rel_live)
    w["pose"] = {k2: round(v, 1) for k2, v in live.items()}
    w["trajectory"].append({**w["pose"], "t": _now(), "note": "pose-graph optimize"})
    pg["last_optimize"] = {"t": _now(),
                           "chi2_start": round(info["chi2_start"], 3),
                           "chi2_end": round(info["chi2_end"], 3),
                           "iterations": info["iterations"],
                           "max_correction_mm": round(moved, 1),
                           "loop_closures": n_loops}
    save_world(w)
    print(f"  corrected keyframe poses (max shift {moved:.0f} mm); live pose now "
          f"x={w['pose']['x_mm']:.0f} y={w['pose']['y_mm']:.0f} "
          f"theta={w['pose']['theta_deg']:.0f}")
    if args.png:
        render_map_png(w, args.png, show_map=True)
        print(f"map -> {args.png}")


# --- Map consolidation (fuse redundant landmarks) ---------------------------
def cmd_fuse_landmarks(args):
    """Consolidate the raw landmark cloud (many noisy triangulations, maybe from
    several motionmap runs) into one map by greedily merging points within a
    radius into a single weighted landmark. This is the persistent map a SLAM
    system maintains: repeated observations reinforce a point instead of the
    cloud growing without bound."""
    w = load_world()
    lms = w.get("landmarks", [])
    if not lms:
        raise SystemExit("no landmarks - run motionmap first")
    r2 = float(args.radius) ** 2
    used = [False] * len(lms)
    fused = []
    for a in range(len(lms)):
        if used[a]:
            continue
        cluster = [lms[a]]
        used[a] = True
        for b in range(a + 1, len(lms)):
            if used[b]:
                continue
            if ((lms[a]["x_mm"] - lms[b]["x_mm"]) ** 2 +
                    (lms[a]["y_mm"] - lms[b]["y_mm"]) ** 2) <= r2:
                cluster.append(lms[b])
                used[b] = True
        n = len(cluster)
        fused.append({"x_mm": round(sum(c["x_mm"] for c in cluster) / n, 1),
                      "y_mm": round(sum(c["y_mm"] for c in cluster) / n, 1),
                      "z_mm": round(sum(c.get("z_mm", 0) for c in cluster) / n, 1),
                      "weight": n})
    fused.sort(key=lambda c: -c["weight"])
    w.setdefault("map", {})["landmarks"] = fused
    save_world(w)
    print(f"fused {len(lms)} raw landmarks -> {len(fused)} map points "
          f"(merge radius {args.radius:.0f} mm)")
    for c in fused[:6]:
        print(f"  ({c['x_mm']:+.0f},{c['y_mm']:+.0f}) mm  weight {c['weight']}")
    if args.png:
        render_map_png(w, args.png, show_map=True)
        print(f"map -> {args.png}")


# --- Rendering (PIL only) ---------------------------------------------------
def render_map_png(w, path, size=700, ray_mm=900, show_map=False):
    from PIL import Image, ImageDraw
    xs = [t["x_mm"] for t in w["trajectory"]] + [0]
    ys = [t["y_mm"] for t in w["trajectory"]] + [0]
    for kf in w["keyframes"]:
        xs.append(kf["pose"]["x_mm"]); ys.append(kf["pose"]["y_mm"])
    for lm in w.get("landmarks", []):
        xs.append(lm["x_mm"]); ys.append(lm["y_mm"])
    span = max(600.0, max(abs(v) for v in xs + ys) + ray_mm + 200)
    scale = (size / 2 - 30) / span
    cx = cy = size / 2

    def to_px(x, y):
        # world +x -> screen up, world +y (left) -> screen left
        return (cx - y * scale, cy - x * scale)

    img = Image.new("RGB", (size, size), (18, 20, 24))
    d = ImageDraw.Draw(img)
    grid = (40, 44, 52)
    for m in range(-4000, 4001, 500):
        px = cx - m * scale
        py = cy - m * scale
        d.line([(px, 0), (px, size)], fill=grid)
        d.line([(0, py), (size, py)], fill=grid)
    d.line([(cx, 0), (cx, size)], fill=(70, 74, 82))
    d.line([(0, cy), (size, cy)], fill=(70, 74, 82))

    # triangulated landmarks (behind trajectory/rays)
    fused = w.get("map", {}).get("landmarks", [])
    if show_map and fused:
        # consolidated map: dot radius grows with observation weight
        for lm in fused:
            lx, ly = to_px(lm["x_mm"], lm["y_mm"])
            rad = 2 + min(6, lm.get("weight", 1))
            d.ellipse([lx - rad, ly - rad, lx + rad, ly + rad], fill=(120, 235, 150))
    else:
        for lm in w.get("landmarks", []):
            lx, ly = to_px(lm["x_mm"], lm["y_mm"])
            d.ellipse([lx - 3, ly - 3, lx + 3, ly + 3], fill=(90, 210, 210))
    # trajectory
    pts = [to_px(t["x_mm"], t["y_mm"]) for t in w["trajectory"]]
    if len(pts) > 1:
        d.line(pts, fill=(90, 160, 255), width=3)
    # keyframes + observation rays
    for kf in w["keyframes"]:
        ox, oy = to_px(kf["pose"]["x_mm"], kf["pose"]["y_mm"])
        for o in kf["observations"]:
            b = math.radians(o["world_bearing_deg"])
            r = o["range_mm"] or ray_mm
            ex, ey = to_px(kf["pose"]["x_mm"] + r * math.cos(b),
                           kf["pose"]["y_mm"] + r * math.sin(b))
            d.line([(ox, oy), (ex, ey)], fill=(240, 190, 90), width=1)
        d.ellipse([ox - 5, oy - 5, ox + 5, oy + 5], fill=(240, 120, 90))
    # loop-closure edges (the constraints that pull the graph consistent)
    kf_by_id = {kf["id"]: kf for kf in w["keyframes"]}
    for e in w.get("pose_graph", {}).get("edges", []):
        if e.get("kind") != "loop":
            continue
        a, b = kf_by_id.get(e["i"]), kf_by_id.get(e["j"])
        if not a or not b:
            continue
        ax, ay = to_px(a["pose"]["x_mm"], a["pose"]["y_mm"])
        bx, by = to_px(b["pose"]["x_mm"], b["pose"]["y_mm"])
        d.line([(ax, ay), (bx, by)], fill=(220, 90, 220), width=2)
    # robot pose arrow
    rx, ry = to_px(w["pose"]["x_mm"], w["pose"]["y_mm"])
    th = math.radians(w["pose"]["theta_deg"])
    hx, hy = to_px(w["pose"]["x_mm"] + 250 * math.cos(th),
                   w["pose"]["y_mm"] + 250 * math.sin(th))
    d.line([(rx, ry), (hx, hy)], fill=(120, 240, 140), width=4)
    d.ellipse([rx - 7, ry - 7, rx + 7, ry + 7], fill=(120, 240, 140))
    d.text((8, 8), f"grid=500mm  pose=({w['pose']['x_mm']:.0f},"
           f"{w['pose']['y_mm']:.0f},{w['pose']['theta_deg']:.0f}deg)",
           fill=(200, 205, 215))
    img.save(path)


def render_panorama(kf, path, tile_w=300):
    from PIL import Image, ImageDraw
    obs = sorted(kf["observations"], key=lambda o: o["world_bearing_deg"])
    tiles = []
    for o in obs:
        fp = os.path.join(FRAMES_DIR, o["image"])
        if not os.path.exists(fp):
            continue
        im = Image.open(fp).convert("RGB")
        h = int(im.height * tile_w / im.width)
        im = im.resize((tile_w, h))
        strip = Image.new("RGB", (tile_w, h + 22), (0, 0, 0))
        strip.paste(im, (0, 22))
        ImageDraw.Draw(strip).text(
            (6, 5), f"bearing {o['world_bearing_deg']:+.0f}deg",
            fill=(255, 235, 120))
        tiles.append(strip)
    if not tiles:
        raise SystemExit("no frames to stitch")
    H = max(t.height for t in tiles)
    pano = Image.new("RGB", (tile_w * len(tiles), H), (0, 0, 0))
    for i, t in enumerate(tiles):
        pano.paste(t, (i * tile_w, 0))
    pano.save(path)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("init", help="reset world model at the origin")
    sub.add_parser("pose", help="print current dead-reckoned pose")
    f = sub.add_parser("forward", help="drive forward N mm, update pose")
    f.add_argument("mm", type=float)
    b = sub.add_parser("back", help="drive backward N mm, update pose")
    b.add_argument("mm", type=float)
    sh = sub.add_parser("set-heading", help="manual pose correction (deg)")
    sh.add_argument("deg", type=float)
    sc = sub.add_parser("scan", help="neck sweep, save pose-tagged frames")
    sc.add_argument("--neck", default="-80,-40,0,40,80",
                    help="comma-separated neck angles in degrees")
    sc.add_argument("--settle", type=float, default=0.5)
    sc.add_argument("--lookout", action="store_true",
                    help="raise camera to lookout pose before sweeping")
    lo = sub.add_parser("lookout", help="raise camera to the lookout pose")
    lo.add_argument("--snapshot", help="save a snapshot from the lookout pose")
    la = sub.add_parser("landmarks", help="match ORB features across a scan's views")
    la.add_argument("--kf", type=int, default=-1, help="keyframe index")
    la.add_argument("--png", help="write the match visualization here")
    la.add_argument("--top", type=int, default=40, help="best matches per pair")
    mm = sub.add_parser("motionmap",
                        help="drive a baseline + triangulate ORB -> world landmarks")
    mm.add_argument("--baseline", type=float, default=200, help="forward drive mm")
    mm.add_argument("--fov", type=float, default=CAMERA_FOV_DEG,
                    help="assumed horizontal FOV in degrees")
    mm.add_argument("--lookout", action="store_true",
                    help="raise camera to lookout pose first")
    mm.add_argument("--png", help="render the landmark map here")
    m = sub.add_parser("map", help="print world model, optional PNG renders")
    m.add_argument("--png", help="write top-down map PNG here")
    m.add_argument("--panorama", help="write neck-sweep panorama PNG here")
    m.add_argument("--kf", type=int, default=-1, help="keyframe index for panorama")
    ld = sub.add_parser("loop-detect",
                        help="find loop closures between keyframes (place recog.)")
    ld.add_argument("--min-matches", type=int, default=25,
                    help="min verified ORB inliers to call it a loop")
    ld.add_argument("--fov", type=float, default=CAMERA_FOV_DEG)
    ld.add_argument("--place-radius", type=float, default=250.0,
                    help="loose translation sigma (mm) for a 'same place' closure")
    ld.add_argument("--rotation-only", action="store_true",
                    help="constrain only relative heading, not co-location")
    ld.add_argument("--add", action="store_true",
                    help="commit detected closures as graph edges")
    op = sub.add_parser("optimize",
                        help="run the pose-graph back-end, correct all poses")
    op.add_argument("--png", help="render the corrected map here")
    op.add_argument("--verbose", action="store_true", help="print each GN iter")
    fl = sub.add_parser("fuse-landmarks",
                        help="merge redundant landmarks into one weighted map")
    fl.add_argument("--radius", type=float, default=120.0,
                    help="merge landmarks within this radius (mm)")
    fl.add_argument("--png", help="render the fused map here")

    args = p.parse_args()
    {"init": cmd_init, "pose": cmd_pose, "forward": cmd_forward, "back": cmd_back,
     "set-heading": cmd_set_heading, "scan": cmd_scan, "lookout": cmd_lookout,
     "landmarks": cmd_landmarks, "motionmap": cmd_motionmap, "map": cmd_map,
     "loop-detect": cmd_loop_detect, "optimize": cmd_optimize,
     "fuse-landmarks": cmd_fuse_landmarks}[args.cmd](args)


if __name__ == "__main__":
    main()

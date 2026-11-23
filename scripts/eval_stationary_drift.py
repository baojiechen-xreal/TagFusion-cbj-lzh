#!/usr/bin/env python3
"""Stationary-window relative drift (indoor large scene, section 3.1.2).

Loop closure and the trajectory are already produced by TagFusion. This script
only detects the stationary revisits and computes the pose error.

For each stationary window i, poses inside the window are averaged in SE(3):

    T_i = Mean_SE3(T_i,1 ... T_i,Ni)

Every trajectory uses its own first window as the reference:

    T_rel,i = T_0^{-1} * T_i

When a ground-truth trajectory is given (same window order):

    T_err,i = T_gt_rel,i^{-1} * T_rel,i

Translation error is ||t_err||. Rotation error is the angle of R_err.
No per-frame ATE, no interpolation, no Sim(3) alignment.

Windows are detected per trajectory, because Quest and the glasses do not
share a clock. Windows are then matched in time order. Tag detections, when
provided, keep only stops where a tag board is visible (this scene has tags
only at the stationary boards).

Trajectory files are TUM: timestamp tx ty tz qx qy qz qw (Hamilton).
Quest files are the recorded quest_pro_state.csv.
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _norm(a):
    return math.sqrt(_dot(a, a))


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _mat_vec(R, v):
    return (
        R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
        R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
        R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
    )


def _mat_mul(A, B):
    return tuple(
        tuple(sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )


def _mat_T(R):
    return tuple(tuple(R[j][i] for j in range(3)) for i in range(3))


def quat_to_R(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        raise ValueError("zero quaternion")
    x, y, z, w = x / n, y / n, z / n, w / n
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def R_to_quat(R):
    t = R[0][0] + R[1][1] + R[2][2]
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2][1] - R[1][2]) / s
        y = (R[0][2] - R[2][0]) / s
        z = (R[1][0] - R[0][1]) / s
    elif R[0][0] > R[1][1] and R[0][0] > R[2][2]:
        s = math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2.0
        w = (R[2][1] - R[1][2]) / s
        x = 0.25 * s
        y = (R[0][1] + R[1][0]) / s
        z = (R[0][2] + R[2][0]) / s
    elif R[1][1] > R[2][2]:
        s = math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2.0
        w = (R[0][2] - R[2][0]) / s
        x = (R[0][1] + R[1][0]) / s
        y = 0.25 * s
        z = (R[1][2] + R[2][1]) / s
    else:
        s = math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2.0
        w = (R[1][0] - R[0][1]) / s
        x = (R[0][2] + R[2][0]) / s
        y = (R[1][2] + R[2][1]) / s
        z = 0.25 * s
    n = math.sqrt(x * x + y * y + z * z + w * w)
    return (x / n, y / n, z / n, w / n)


def so3_hat(w):
    x, y, z = w
    return (
        (0.0, -z, y),
        (z, 0.0, -x),
        (-y, x, 0.0),
    )


def so3_exp(w):
    theta = _norm(w)
    I = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    W = so3_hat(w)
    W2 = _mat_mul(W, W)
    if theta < 1e-8:
        a = 0.5
        b = 1.0 / 6.0
    else:
        a = (1.0 - math.cos(theta)) / (theta * theta)
        b = (theta - math.sin(theta)) / (theta * theta * theta)
    # R = I + sinc * W + a * W^2, with sinc = sin(theta)/theta
    sinc = 1.0 if theta < 1e-8 else math.sin(theta) / theta
    return tuple(
        tuple(I[i][j] + sinc * W[i][j] + a * W2[i][j] for j in range(3))
        for i in range(3)
    )


def so3_log(R):
    cos_theta = max(-1.0, min(1.0, 0.5 * (R[0][0] + R[1][1] + R[2][2] - 1.0)))
    theta = math.acos(cos_theta)
    if theta < 1e-8:
        return (
            0.5 * (R[2][1] - R[1][2]),
            0.5 * (R[0][2] - R[2][0]),
            0.5 * (R[1][0] - R[0][1]),
        )
    scale = theta / (2.0 * math.sin(theta))
    return (
        scale * (R[2][1] - R[1][2]),
        scale * (R[0][2] - R[2][0]),
        scale * (R[1][0] - R[0][1]),
    )


def se3_exp(xi):
    omega = xi[:3]
    rho = xi[3:]
    theta = _norm(omega)
    R = so3_exp(omega)
    W = so3_hat(omega)
    W2 = _mat_mul(W, W)
    I = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    if theta < 1e-8:
        a = 0.5
        b = 1.0 / 6.0
    else:
        a = (1.0 - math.cos(theta)) / (theta * theta)
        b = (theta - math.sin(theta)) / (theta ** 3)
    V = tuple(
        tuple(I[i][j] + a * W[i][j] + b * W2[i][j] for j in range(3))
        for i in range(3)
    )
    return R, _mat_vec(V, rho)


def se3_log(R, t):
    omega = so3_log(R)
    theta = _norm(omega)
    W = so3_hat(omega)
    W2 = _mat_mul(W, W)
    I = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    if theta < 1e-8:
        a = 0.5
        b = 1.0 / 12.0
    else:
        a = 0.5
        half = 0.5 * theta
        b = (1.0 - theta * math.cos(half) / (2.0 * math.sin(half))) / (theta * theta)
    Vinv = tuple(
        tuple(I[i][j] - a * W[i][j] + b * W2[i][j] for j in range(3))
        for i in range(3)
    )
    rho = _mat_vec(Vinv, t)
    return omega + rho


def invert(R, t):
    Rt = _mat_T(R)
    return Rt, _scale(_mat_vec(Rt, t), -1.0)


def compose(Ra, ta, Rb, tb):
    return _mat_mul(Ra, Rb), _add(_mat_vec(Ra, tb), ta)


def se3_mean(poses):
    """Iterative SE(3) mean. poses are (R, t)."""
    R, t = poses[0]
    n = len(poses)
    for _ in range(30):
        acc = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        Rinv, tinv = invert(R, t)
        for Ri, ti in poses:
            Re, te = compose(Rinv, tinv, Ri, ti)
            xi = se3_log(Re, te)
            acc = tuple(acc[k] + xi[k] for k in range(6))
        acc = tuple(v / n for v in acc)
        if _norm(acc[:3]) + _norm(acc[3:]) < 1e-10:
            break
        dR, dt = se3_exp(acc)
        R, t = compose(R, t, dR, dt)
    return R, t


def rotation_angle_deg(R):
    cos_theta = max(-1.0, min(1.0, 0.5 * (R[0][0] + R[1][1] + R[2][2] - 1.0)))
    return math.degrees(math.acos(cos_theta))


@dataclass
class Pose:
    t: float
    R: tuple
    p: tuple


def load_tum(path: Path) -> list[Pose]:
    poses = []
    with path.open() as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            v = line.split()
            if len(v) < 8:
                continue
            ts = float(v[0])
            p = (float(v[1]), float(v[2]), float(v[3]))
            q = (float(v[4]), float(v[5]), float(v[6]), float(v[7]))
            poses.append(Pose(ts, quat_to_R(q), p))
    if len(poses) < 2:
        raise SystemExit(f"not enough poses in {path}")
    poses.sort(key=lambda x: x.t)
    return poses


def load_quest_csv(path: Path) -> list[Pose]:
    with path.open(newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        # Duplicate column names: first xyz/quat block is the app pose.
        def col(name, occurrence=0):
            seen = 0
            for i, h in enumerate(header):
                if h == name:
                    if seen == occurrence:
                        return i
                    seen += 1
            raise SystemExit(f"{path} missing column {name}")

        i_t = col("app_time_s")
        i_px, i_py, i_pz = (col("head_position_3", k) for k in range(3))
        i_qx, i_qy, i_qz, i_qw = (col("head_quaternion_4", k) for k in range(4))
        poses = []
        for row in reader:
            if len(row) <= i_qw:
                continue
            try:
                ts = float(row[i_t])
                p = (float(row[i_px]), float(row[i_py]), float(row[i_pz]))
                q = (float(row[i_qx]), float(row[i_qy]), float(row[i_qz]), float(row[i_qw]))
            except ValueError:
                continue
            poses.append(Pose(ts, quat_to_R(q), p))
    if len(poses) < 2:
        raise SystemExit(f"not enough poses in {path}")
    poses.sort(key=lambda x: x.t)
    return poses


def load_tag_times(path: Path) -> list[float]:
    times = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None or "timestamp" not in reader.fieldnames:
            raise SystemExit(f"{path} has no timestamp column")
        for row in reader:
            try:
                times.append(float(row["timestamp"]))
            except (TypeError, ValueError):
                continue
    times.sort()
    return times


def speeds(poses: list[Pose]) -> list[float]:
    out = [0.0] * len(poses)
    for i in range(len(poses) - 1):
        dt = poses[i + 1].t - poses[i].t
        if dt <= 1e-6:
            out[i] = 0.0
        else:
            out[i] = _norm(_sub(poses[i + 1].p, poses[i].p)) / dt
    out[-1] = out[-2]
    return out


def angular_speeds_deg_s(poses: list[Pose]) -> list[float]:
    out = [0.0] * len(poses)
    for i in range(len(poses) - 1):
        dt = poses[i + 1].t - poses[i].t
        if dt <= 1e-6:
            out[i] = 0.0
        else:
            rel_r = _mat_mul(_mat_T(poses[i].R), poses[i + 1].R)
            out[i] = rotation_angle_deg(rel_r) / dt
    out[-1] = out[-2]
    return out


def refine_segment(
    poses: list[Pose],
    segment: tuple[int, int],
    v: list[float] | None = None,
    w: list[float] | None = None,
    v_max: float = 0.05,
    w_max: float = 3.0,
    trim_s: float = 1.0,
    min_core_s: float = 1.5,
) -> tuple[int, int]:
    """Lock onto the low-angular-rate core inside a coarse stop and trim both ends."""
    a, b = segment
    if v is None:
        v = speeds(poses)
    if w is None:
        w = angular_speeds_deg_s(poses)
    sub_poses = poses[a:b]
    mask = [v[i] <= v_max and w[i] <= w_max for i in range(a, b)]
    runs = mask_to_segments(sub_poses, mask, min_duration=0.5, max_gap=0.5)
    if runs:
        ra, rb = max(runs, key=lambda r: sub_poses[r[1] - 1].t - sub_poses[r[0]].t)
        ca, cb = a + ra, a + rb
    else:
        ca, cb = a, b
    if trim_s <= 0.0:
        return ca, cb
    t0, t1 = poses[ca].t, poses[cb - 1].t
    dur = t1 - t0
    trim = min(trim_s, max(0.0, 0.5 * (dur - min_core_s)))
    ta, tb = t0 + trim, t1 - trim
    idx = [i for i in range(ca, cb) if ta - 1e-6 <= poses[i].t <= tb + 1e-6]
    return (idx[0], idx[-1] + 1) if len(idx) >= 2 else (ca, cb)


def mask_to_segments(poses, mask, min_duration, max_gap):
    """Boolean mask -> [start_idx, end_idx) segments. Gaps shorter than max_gap are joined."""
    raw = []
    i = 0
    n = len(poses)
    while i < n:
        if not mask[i]:
            i += 1
            continue
        j = i + 1
        while j < n and mask[j]:
            j += 1
        raw.append((i, j))
        i = j
    if not raw:
        return []
    merged = [list(raw[0])]
    for a, b in raw[1:]:
        if poses[a].t - poses[merged[-1][1] - 1].t <= max_gap:
            merged[-1][1] = b
        else:
            merged.append([a, b])
    kept = []
    for a, b in merged:
        if poses[b - 1].t - poses[a].t >= min_duration and b - a >= 2:
            kept.append((a, b))
    return kept


def detect_segments(poses, tag_times, v_max, min_duration, max_gap):
    v = speeds(poses)
    slow = [s <= v_max for s in v]
    if not tag_times:
        return mask_to_segments(poses, slow, min_duration, max_gap)
    # Tags in this scene are only on the stationary boards, so a stop counts
    # only when a detection falls within max_gap of the pose and the camera is slow.
    tag_mask = [False] * len(poses)
    j = 0
    for i, pose in enumerate(poses):
        while j < len(tag_times) and tag_times[j] < pose.t - max_gap:
            j += 1
        if j < len(tag_times) and tag_times[j] <= pose.t + max_gap:
            tag_mask[i] = True
    mask = [slow[i] and tag_mask[i] for i in range(len(poses))]
    return mask_to_segments(poses, mask, min_duration, max_gap)


@dataclass
class Window:
    index: int
    t0: float
    t1: float
    n: int
    R: tuple
    p: tuple
    median_speed: float
    raw_t0: float = 0.0
    raw_t1: float = 0.0
    span_disp_m: float = 0.0
    span_rot_deg: float = 0.0


def window_pose(poses, segment, speed, raw_segment=None) -> Window:
    a, b = segment
    samples = poses[a:b]
    R, p = se3_mean([(s.R, s.p) for s in samples])
    sv = sorted(speed[a:b])
    ra, rb = raw_segment if raw_segment is not None else segment
    span_disp = _norm(_sub(samples[-1].p, samples[0].p))
    span_rot = rotation_angle_deg(_mat_mul(_mat_T(samples[0].R), samples[-1].R))
    return Window(
        index=0,
        t0=samples[0].t,
        t1=samples[-1].t,
        n=len(samples),
        R=R,
        p=p,
        median_speed=sv[len(sv) // 2],
        raw_t0=poses[ra].t,
        raw_t1=poses[rb - 1].t,
        span_disp_m=span_disp,
        span_rot_deg=span_rot,
    )


@dataclass
class SegmentError:
    index: int  # 1-based, reference window excluded
    trans_m: float
    rot_deg: float
    trans_vs_gt_m: float | None
    rot_vs_gt_deg: float | None


def relative_errors(windows: list[Window], gt_windows: list[Window] | None):
    if len(windows) < 2:
        return []
    R0, p0 = windows[0].R, windows[0].p
    R0i, p0i = invert(R0, p0)
    errors = []
    n = len(windows) - 1
    if gt_windows is not None:
        n = min(n, len(gt_windows) - 1)
    for i in range(1, n + 1):
        Rrel, prel = compose(R0i, p0i, windows[i].R, windows[i].p)
        trans = _norm(prel)
        rot = rotation_angle_deg(Rrel)
        trans_gt = rot_gt = None
        if gt_windows is not None:
            G0i_R, G0i_p = invert(gt_windows[0].R, gt_windows[0].p)
            Grel_R, Grel_p = compose(G0i_R, G0i_p, gt_windows[i].R, gt_windows[i].p)
            Ginv_R, Ginv_p = invert(Grel_R, Grel_p)
            E_R, E_p = compose(Ginv_R, Ginv_p, Rrel, prel)
            trans_gt = _norm(E_p)
            rot_gt = rotation_angle_deg(E_R)
        errors.append(SegmentError(i, trans, rot, trans_gt, rot_gt))
    return errors


def _mean(xs):
    return sum(xs) / len(xs) if xs else float("nan")


def _rmse(xs):
    return math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else float("nan")


def evaluate(name, poses, tag_times, args, gt_windows):
    segs = detect_segments(poses, tag_times, args.v_max, args.min_duration, args.max_gap)
    spd = speeds(poses)
    windows = []
    for k, seg in enumerate(segs):
        w = window_pose(poses, seg, spd)
        w.index = k
        windows.append(w)
    errors = relative_errors(windows, gt_windows)
    return windows, errors


def write_report(path: Path, results, have_gt: bool):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        header = [
            "method", "seg", "t_start", "t_end", "n_poses", "median_speed_m_s",
            "drift_trans_m", "drift_rot_deg",
        ]
        if have_gt:
            header += ["err_trans_m", "err_rot_deg"]
        w.writerow(header)
        for name, windows, errors in results:
            by_idx = {e.index: e for e in errors}
            for win in windows:
                e = by_idx.get(win.index)
                row = [
                    name, win.index, f"{win.t0:.6f}", f"{win.t1:.6f}", win.n,
                    f"{win.median_speed:.4f}",
                    "" if e is None else f"{e.trans_m:.6f}",
                    "" if e is None else f"{e.rot_deg:.4f}",
                ]
                if have_gt:
                    row += [
                        "" if e is None or e.trans_vs_gt_m is None else f"{e.trans_vs_gt_m:.6f}",
                        "" if e is None or e.rot_vs_gt_deg is None else f"{e.rot_vs_gt_deg:.4f}",
                    ]
                w.writerow(row)


def print_table(results, have_gt: bool):
    print()
    if have_gt:
        print("Translation error against GT (m), relative to the first stationary window")
        value = lambda e: e.trans_vs_gt_m
    else:
        print("Translation of T_rel (m): drift from the first stationary window (no GT)")
        value = lambda e: e.trans_m
    names = [r[0] for r in results]
    nseg = max((len(r[2]) for r in results), default=0)
    header = ["Method"] + [f"Seg.{i}" for i in range(1, nseg + 1)] + ["Mean", "RMSE"]
    rows = []
    for name, _windows, errors in results:
        vals = [value(e) for e in errors if value(e) is not None]
        cells = [name]
        for i in range(1, nseg + 1):
            match = next((e for e in errors if e.index == i and value(e) is not None), None)
            cells.append("" if match is None else f"{value(match):.5f}")
        cells.append("" if not vals else f"{_mean(vals):.5f}")
        cells.append("" if not vals else f"{_rmse(vals):.5f}")
        rows.append(cells)
    widths = [max(len(header[c]), *(len(r[c]) for r in rows)) for c in range(len(header))]
    def fmt(cells):
        return " | ".join(cells[c].ljust(widths[c]) for c in range(len(cells)))
    print(fmt(header))
    print("-+-".join("-" * n for n in widths))
    for row in rows:
        print(fmt(row))
    print()
    print("Rotation (deg)")
    value_r = (lambda e: e.rot_vs_gt_deg) if have_gt else (lambda e: e.rot_deg)
    for name, _windows, errors in results:
        vals = [value_r(e) for e in errors if value_r(e) is not None]
        parts = [f"Seg.{e.index}={value_r(e):.3f}" for e in errors if value_r(e) is not None]
        if vals:
            parts.append(f"Mean={_mean(vals):.3f}")
            parts.append(f"RMSE={_rmse(vals):.3f}")
        print(f"  {name}: {', '.join(parts) if parts else '(no stationary return)'}")
        print(f"    windows={len(_windows)}")


def parse_named(spec: str, default_name: str) -> tuple[str, Path]:
    if "=" in spec and not spec.startswith("/"):
        name, path = spec.split("=", 1)
        return name, Path(path)
    return default_name, Path(spec)


def main() -> int:
    parser = argparse.ArgumentParser(description="Stationary-window relative drift")
    parser.add_argument(
        "--traj", action="append", default=[],
        help="TUM trajectory, as name=path or a bare path (repeatable)",
    )
    parser.add_argument(
        "--quest", action="append", default=[],
        help="Quest quest_pro_state.csv, as name=path or a bare path (repeatable)",
    )
    parser.add_argument("--tags", type=Path, default=None,
                        help="tag_detections.csv. Applied to every --traj, not to Quest")
    parser.add_argument("--gt", type=Path, default=None,
                        help="ground-truth TUM trajectory, e.g. cam0_pose.txt")
    parser.add_argument("--gt-tags", type=Path, default=None,
                        help="optional tag_detections.csv aligned to the GT clock")
    parser.add_argument("--v-max", type=float, default=0.05,
                        help="speed below this (m/s) counts as stationary (default 0.05)")
    parser.add_argument("--min-duration", type=float, default=3.0,
                        help="minimum stationary duration in seconds (default 3)")
    parser.add_argument("--max-gap", type=float, default=1.0,
                        help="merge stationary gaps shorter than this (default 1s)")
    parser.add_argument("--out", type=Path, default=Path("stationary_drift.csv"))
    args = parser.parse_args()

    if not args.traj and not args.quest:
        parser.error("provide at least one --traj or --quest")

    tag_times = load_tag_times(args.tags) if args.tags else None
    gt_windows = None
    if args.gt:
        gt_poses = load_tum(args.gt)
        gt_tag_times = load_tag_times(args.gt_tags) if args.gt_tags else None
        gt_windows, _ = evaluate("GT", gt_poses, gt_tag_times, args, None)
        print(f"GT windows: {len(gt_windows)}")
        if len(gt_windows) < 2:
            raise SystemExit("GT has fewer than 2 stationary windows; cannot form T_gt_rel")

    results = []
    for spec in args.traj:
        name, path = parse_named(spec, path_stem(spec))
        windows, errors = evaluate(name, load_tum(path), tag_times, args, gt_windows)
        results.append((name, windows, errors))
        print(f"{name}: {len(windows)} stationary windows from {path}")
    for spec in args.quest:
        name, path = parse_named(spec, "Quest")
        # Quest has no tag stream; detect stops from speed only.
        windows, errors = evaluate(name, load_quest_csv(path), None, args, gt_windows)
        results.append((name, windows, errors))
        print(f"{name}: {len(windows)} stationary windows from {path}")

    if gt_windows is not None:
        counts = {name: len(w) for name, w, _ in results}
        gt_n = len(gt_windows)
        for name, n in counts.items():
            if n != gt_n:
                print(f"WARNING: {name} has {n} windows, GT has {gt_n}. "
                      "Later segments are matched by order and the extra ones are dropped.")

    write_report(args.out, results, gt_windows is not None)
    print_table(results, gt_windows is not None)
    print(f"wrote {args.out}")
    return 0


def path_stem(spec: str) -> str:
    return Path(spec.split("=", 1)[-1]).stem


if __name__ == "__main__":
    raise SystemExit(main())

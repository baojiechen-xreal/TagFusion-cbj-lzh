#!/usr/bin/env python3
"""Large-scene stationary evaluation (section 3.1.2).

Collection: a cross mark at the start, return about every 5 minutes and stand
still for about 5 seconds. AprilTags are only on that stationary board.

For each stationary window the poses are averaged in SE(3). Every trajectory
uses its own first window as the reference

    T_rel,i = T_0^{-1} * T_i

and the error against ground truth is

    T_err,i = T_gt_rel,i^{-1} * T_rel,i

Translation error is the length of the translation of T_err. Rotation error
is the angle of its rotation. No per-frame ATE and no Sim(3) alignment.

Ground truth and any TagFusion / ORB-SLAM3 trajectory share the glass clock,
so they use the stationary windows detected on cam0_pose.txt. Quest has its
own clock; its stops are detected the same way and paired in time order.

    python3 scripts/eval_large_scene_stationary.py \
        --seq /path/to/5fr_pat_led_m/01-1 \
        --traj "TagFusion=f_with_tag.txt" \
        --traj "TagFusion (w/o Tag)=f_wo_tag.txt" \
        --traj "ORB-SLAM3=orb.txt"
"""

from __future__ import annotations

import argparse
import base64
import csv
from dataclasses import dataclass
import html
import math
import os
import re
import struct
import subprocess
import sys
import zlib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import eval_stationary_drift as core


COLORS = {
    "Quest": "#4C78A8",
    "ORB-SLAM3": "#9D9D9D",
    "TagFusion (w/o Tag)": "#F58518",
    "TagFusion": "#54A24B",
}
METHOD_ORDER = ["Quest", "ORB-SLAM3", "TagFusion (w/o Tag)", "TagFusion"]


@dataclass
class StaticJitter:
    index: int
    t0: float
    t1: float
    duration_s: float
    n: int
    p2p_trans_mm: float
    p2p_rot_deg: float
    max_jump_trans_mm: float
    max_jump_rot_deg: float
    rms_trans_mm: float
    rms_rot_deg: float
    p95_trans_mm: float
    p95_rot_deg: float
    drift_rate_mm_s: float
    drift_rate_deg_s: float
    net_drift_mm: float
    net_drift_deg: float


def adaptive_core_bounds(
    raw_t0: float,
    raw_t1: float,
    core_duration: float = 3.0,
    min_trim: float = 0.75,
    min_core: float = 0.5,
) -> tuple[float, float] | None:
    """Keep central `core_duration` (3.0s) for normal/long stops, and adaptively
    shrink shorter stops while trimming both ends ( down to 1.0s raw initial stops)."""
    raw_dur = raw_t1 - raw_t0
    if raw_dur + 1e-6 < 2.0 * min_core:
        return None
    eff_core = min(core_duration, max(0.5 * raw_dur, raw_dur - 2.0 * min_trim))
    mid_t = 0.5 * (raw_t0 + raw_t1)
    half = 0.5 * eff_core
    return (mid_t - half, mid_t + half)


def _percentile(sorted_vals: list[float], q: float) -> float:
    if not sorted_vals:
        return 0.0
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    pos = q * (len(sorted_vals) - 1)
    lo = int(math.floor(pos))
    hi = min(lo + 1, len(sorted_vals) - 1)
    frac = pos - lo
    return sorted_vals[lo] * (1.0 - frac) + sorted_vals[hi] * frac


def compute_static_jitter(
    poses,
    core_t0: float,
    core_t1: float,
    seg_index: int,
    min_duration: float = 1.0,
) -> StaticJitter | None:
    """Evaluate single-segment static stability (jitter) on the unified core window
    [core_t0, core_t1] (same window as Task 1, e.g. 3.00s for normal stops).

    Skips seg_index == 0 (first window) and rejects core windows shorter than min_duration.
    """
    if seg_index <= 0:
        return None
    if core_t1 - core_t0 + 1e-9 < min_duration:
        return None
    samples = [p for p in poses if core_t0 - 1e-6 <= p.t <= core_t1 + 1e-6]
    if len(samples) < 5:
        return None
    dur = samples[-1].t - samples[0].t
    if dur + 0.05 < min_duration:
        return None

    mean_R, mean_p = core.se3_mean([(s.R, s.p) for s in samples])
    mean_Rt = core._mat_T(mean_R)
    dev_p_mm = [core._norm(core._sub(s.p, mean_p)) * 1000.0 for s in samples]
    dev_r_deg = [core.rotation_angle_deg(core._mat_mul(mean_Rt, s.R)) for s in samples]

    rms_trans_mm = core._rmse(dev_p_mm)
    rms_rot_deg = core._rmse(dev_r_deg)
    p95_trans_mm = _percentile(sorted(dev_p_mm), 0.95)
    p95_rot_deg = _percentile(sorted(dev_r_deg), 0.95)

    max_jump_trans_mm = max(
        core._norm(core._sub(samples[k + 1].p, samples[k].p)) * 1000.0
        for k in range(len(samples) - 1)
    )
    max_jump_rot_deg = max(
        core.rotation_angle_deg(core._mat_mul(core._mat_T(samples[k].R), samples[k + 1].R))
        for k in range(len(samples) - 1)
    )

    n = len(samples)
    if n <= 350:
        cand_idx = list(range(n))
    else:
        dirs = (
            (1, 0, 0), (0, 1, 0), (0, 0, 1),
            (1, 1, 0), (1, -1, 0), (1, 0, 1), (1, 0, -1), (0, 1, 1), (0, 1, -1),
            (1, 1, 1), (1, 1, -1), (1, -1, 1), (-1, 1, 1),
        )
        chosen = set(range(0, n, max(1, n // 150)))
        for dx, dy, dz in dirs:
            ranked = sorted(
                range(n),
                key=lambda k: dx * samples[k].p[0] + dy * samples[k].p[1] + dz * samples[k].p[2],
            )
            chosen.update(ranked[:12])
            chosen.update(ranked[-12:])
        top_rot = sorted(range(n), key=lambda k: dev_r_deg[k], reverse=True)[:30]
        chosen.update(top_rot)
        cand_idx = sorted(chosen)

    p2p_trans_mm = 0.0
    p2p_rot_deg = 0.0
    for ii in range(len(cand_idx)):
        si = samples[cand_idx[ii]]
        Ri_t = core._mat_T(si.R)
        pi = si.p
        for jj in range(ii + 1, len(cand_idx)):
            sj = samples[cand_idx[jj]]
            d_mm = core._norm(core._sub(sj.p, pi)) * 1000.0
            if d_mm > p2p_trans_mm:
                p2p_trans_mm = d_mm
            d_deg = core.rotation_angle_deg(core._mat_mul(Ri_t, sj.R))
            if d_deg > p2p_rot_deg:
                p2p_rot_deg = d_deg

    net_drift_mm = core._norm(core._sub(samples[-1].p, samples[0].p)) * 1000.0
    net_drift_deg = core.rotation_angle_deg(
        core._mat_mul(core._mat_T(samples[0].R), samples[-1].R)
    )
    drift_rate_mm_s = net_drift_mm / dur if dur > 1e-6 else 0.0
    drift_rate_deg_s = net_drift_deg / dur if dur > 1e-6 else 0.0

    return StaticJitter(
        index=seg_index,
        t0=samples[0].t,
        t1=samples[-1].t,
        duration_s=dur,
        n=n,
        p2p_trans_mm=p2p_trans_mm,
        p2p_rot_deg=p2p_rot_deg,
        max_jump_trans_mm=max_jump_trans_mm,
        max_jump_rot_deg=max_jump_rot_deg,
        rms_trans_mm=rms_trans_mm,
        rms_rot_deg=rms_rot_deg,
        p95_trans_mm=p95_trans_mm,
        p95_rot_deg=p95_rot_deg,
        drift_rate_mm_s=drift_rate_mm_s,
        drift_rate_deg_s=drift_rate_deg_s,
        net_drift_mm=net_drift_mm,
        net_drift_deg=net_drift_deg,
    )


def detect_windows(
    poses,
    v_max,
    min_duration,
    max_gap,
    w_max=3.0,
    core_duration=3.0,
    min_trim=0.75,
    min_core=0.5,
):
    init_min = min(min_duration, max(1.2, 2.0 * min_core))
    return_min = min(min_duration, max(2.0, min_core + 2.0 * min_trim))
    segments = core.detect_segments(poses, None, v_max, init_min, max_gap)
    speed = core.speeds(poses)
    ang_speed = core.angular_speeds_deg_s(poses)
    windows = []
    for segment in segments:
        ca, cb = core.refine_segment(
            poses, segment, speed, ang_speed, v_max=v_max, w_max=w_max, trim_s=0.0)
        raw_t0, raw_t1 = poses[ca].t, poses[cb - 1].t
        req_min = init_min if not windows else return_min
        if raw_t1 - raw_t0 + 1e-6 < req_min:
            continue
        bounds = adaptive_core_bounds(
            raw_t0, raw_t1, core_duration=core_duration, min_trim=min_trim, min_core=min_core
        )
        if bounds is None:
            continue
        ct0, ct1 = bounds
        idx = [i for i in range(ca, cb) if ct0 - 1e-6 <= poses[i].t <= ct1 + 1e-6]
        if len(idx) < 5:
            continue
        window = core.window_pose(
            poses, (idx[0], idx[-1] + 1), speed, raw_segment=(ca, cb))
        window.index = len(windows)
        windows.append(window)
    return windows


def _patrol_time_bounds(sequence_dir: Path, fallback_poses=None) -> tuple[float, float]:
    """Return [t_min, t_max] skipping pre-experiment 8-motion while keeping all patrol stops."""
    infos_path = sequence_dir / "ground_truth" / "static_result" / "infos.yaml"
    if infos_path.is_file():
        text = infos_path.read_text(errors="ignore")
        m_start = re.search(r"imu_first_stationary:\s*\[\s*([0-9.+-eE]+)", text)
        m_end = re.search(r"eight_start_time:\s*([0-9.+-eE]+)", text)
        if m_start:
            t_min = float(m_start.group(1)) - 5.0
            t_max = float(m_end.group(1)) + 1.0 if m_end else float("inf")
            return (t_min, t_max)
    gt_path = sequence_dir / "ground_truth" / "data" / "cam0_pose.txt"
    if gt_path.is_file():
        with gt_path.open() as handle:
            for line in handle:
                if line and line[0] != "#":
                    return (float(line.split()[0]) - 10.0, float("inf"))
    if fallback_poses:
        return (fallback_poses[0].t, fallback_poses[-1].t)
    return (float("-inf"), float("inf"))


def _camera_clock_poses(sequence_dir: Path):
    """Build identity placeholder poses at 30 Hz camera timestamps when no full pose file is given."""
    ts_path = sequence_dir / "raw_data" / "glass" / "cam0" / "images" / "timestamps.txt"
    if not ts_path.is_file():
        return []
    eye = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    zero = (0.0, 0.0, 0.0)
    poses = []
    with ts_path.open() as handle:
        for line in handle:
            if not line or line[0] == "#":
                continue
            parts = line.split()
            if len(parts) >= 2:
                poses.append(core.Pose(float(parts[1]), eye, zero))
    return poses


def detect_windows_from_imu(
    imu_path: Path,
    ref_poses,
    acc_std_max: float = 0.15,
    gyro_deg_max: float = 3.5,
    bin_dt: float = 0.1,
    min_duration: float = 3.0,
    max_gap: float = 1.0,
    core_duration: float = 3.0,
    min_gt_poses: int = 5,
    t_min: float | None = None,
    t_max: float | None = None,
    min_trim: float = 0.75,
    min_core: float = 0.5,
):
    """Detect stationary stops from 1000 Hz IMU inside [t_min, t_max], keeping
    `core_duration` for long stops and adaptively shrinking shorter stops so at
    least `min_trim` seconds are trimmed from both front and back."""
    bound_t0 = ref_poses[0].t if t_min is None else t_min
    bound_t1 = ref_poses[-1].t if t_max is None else t_max
    gyro_rad_max = math.radians(gyro_deg_max)
    bins = {}
    with imu_path.open() as handle:
        for line in handle:
            if not line or line[0] == "#":
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            stamp = float(parts[0])
            if stamp < bound_t0 or stamp > bound_t1:
                continue
            bucket = int(stamp / bin_dt)
            bins.setdefault(bucket, []).append((
                stamp,
                float(parts[2]), float(parts[3]), float(parts[4]),
                float(parts[5]), float(parts[6]), float(parts[7]),
            ))
    times = []
    mask = []
    for bucket in sorted(bins):
        rows = bins[bucket]
        count = len(rows)
        t_mid = 0.5 * (rows[0][0] + rows[-1][0])
        ma0 = sum(row[4] for row in rows) / count
        ma1 = sum(row[5] for row in rows) / count
        ma2 = sum(row[6] for row in rows) / count
        acc_std = math.sqrt(
            sum((row[4] - ma0) ** 2 + (row[5] - ma1) ** 2 + (row[6] - ma2) ** 2 for row in rows)
            / count
        )
        acc_norm = math.sqrt(ma0 ** 2 + ma1 ** 2 + ma2 ** 2)
        gyro_rms = math.sqrt(
            sum(row[1] ** 2 + row[2] ** 2 + row[3] ** 2 for row in rows) / count
        )
        times.append(t_mid)
        mask.append(
            acc_std <= acc_std_max
            and abs(acc_norm - 9.8) <= 0.5
            and gyro_rms <= gyro_rad_max
        )

    raw = []
    idx = 0
    while idx < len(mask):
        if not mask[idx]:
            idx += 1
            continue
        end = idx + 1
        while end < len(mask) and mask[end]:
            end += 1
        raw.append([idx, end])
        idx = end
    if not raw:
        return []
    merged = [raw[0]]
    for start, end in raw[1:]:
        if times[start] - times[merged[-1][1] - 1] <= max_gap:
            merged[-1][1] = end
        else:
            merged.append([start, end])

    spd = core.speeds(ref_poses)
    init_min = min(min_duration, max(1.2, 2.0 * min_core))
    return_min = min(min_duration, max(2.0, min_core + 2.0 * min_trim))
    windows = []
    for start, end in merged:
        imu_t0, imu_t1 = times[start], times[end - 1]
        raw_indices = [
            k for k, pose in enumerate(ref_poses) if imu_t0 - 0.05 <= pose.t <= imu_t1 + 0.05
        ]
        if len(raw_indices) < min_gt_poses:
            continue
        seg_t0 = ref_poses[raw_indices[0]].t
        seg_t1 = ref_poses[raw_indices[-1]].t
        req_min = init_min if not windows else return_min
        if seg_t1 - seg_t0 + 0.05 < req_min:
            continue
        bounds = adaptive_core_bounds(
            seg_t0, seg_t1, core_duration=core_duration, min_trim=min_trim, min_core=min_core
        )
        if bounds is None:
            continue
        ct0, ct1 = bounds
        pose_indices = [
            k for k in raw_indices if ct0 - 1e-6 <= ref_poses[k].t <= ct1 + 1e-6
        ]
        if len(pose_indices) < min_gt_poses:
            continue
        raw_seg = (raw_indices[0], raw_indices[-1] + 1)
        win = core.window_pose(
            ref_poses, (pose_indices[0], pose_indices[-1] + 1), spd, raw_segment=raw_seg)
        win.index = len(windows)
        windows.append(win)
    return windows


def _read_pgm(path: Path) -> tuple[int, int, bytes]:
    data = path.read_bytes()
    idx = 0
    tokens = []
    while len(tokens) < 4:
        while idx < len(data) and data[idx:idx + 1].isspace():
            idx += 1
        if data[idx:idx + 1] == b"#":
            while idx < len(data) and data[idx:idx + 1] != b"\n":
                idx += 1
            continue
        end = idx
        while end < len(data) and not data[end:end + 1].isspace():
            end += 1
        tokens.append(data[idx:end].decode("ascii"))
        idx = end
    width, height = int(tokens[1]), int(tokens[2])
    idx += 1
    return width, height, data[idx:idx + width * height]


def _write_gray_png(path: Path, width: int, height: int, pixels: bytes) -> bytes:
    def chunk(tag: bytes, body: bytes) -> bytes:
        crc = zlib.crc32(tag + body) & 0xFFFFFFFF
        return struct.pack(">I", len(body)) + tag + body + struct.pack(">I", crc)

    raw_rows = bytearray()
    stride = width
    for row in range(height):
        raw_rows.append(0)
        raw_rows.extend(pixels[row * stride:(row + 1) * stride])
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)
    idat = zlib.compress(bytes(raw_rows), level=6)
    png = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr) + chunk(b"IDAT", idat) + chunk(b"IEND", b"")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(png)
    return png


def _inspect_window_frames(sequence_dir: Path, gt_windows, out_dir: Path):
    """Find first/last cam0 frames per stationary window and write 3-panel PNGs."""
    img_dir = sequence_dir / "raw_data" / "glass" / "cam0" / "images"
    ts_path = img_dir / "timestamps.txt"
    if not ts_path.is_file():
        return []
    stamps = []
    for line in ts_path.read_text().splitlines():
        parts = line.split()
        if len(parts) >= 2:
            stamps.append((parts[0], float(parts[1])))
    checks = []
    frames_dir = out_dir / "window_frames"
    for win in gt_windows:
        in_win = [(name, ts) for name, ts in stamps if win.t0 - 1e-6 <= ts <= win.t1 + 1e-6]
        if len(in_win) < 2:
            continue
        first_name, first_t = in_win[0]
        last_name, last_t = in_win[-1]
        first_path = img_dir / first_name
        last_path = img_dir / last_name
        if not (first_path.is_file() and last_path.is_file()):
            continue
        w1, h1, b1 = _read_pgm(first_path)
        w2, h2, b2 = _read_pgm(last_path)
        if (w1, h1) != (w2, h2) or len(b1) != len(b2):
            continue
        mad_val = sum(abs(x - y) for x, y in zip(b1, b2)) / len(b1)
        # Downsample 2x horizontally and vertically so the 3-panel PNG stays compact
        dw, dh = w1 // 2, h1 // 2
        gap = 6
        out_w = dw * 3 + gap * 2
        canvas = bytearray(out_w * dh)
        for ry in range(dh):
            src_row = (ry * 2) * w1
            dst_row = ry * out_w
            for rx in range(dw):
                p1 = b1[src_row + rx * 2]
                p2 = b2[src_row + rx * 2]
                diff = min(255, abs(p1 - p2) * 4)
                canvas[dst_row + rx] = p1
                canvas[dst_row + dw + gap + rx] = p2
                canvas[dst_row + (dw + gap) * 2 + rx] = diff
            for gx in range(gap):
                canvas[dst_row + dw + gx] = 240
                canvas[dst_row + dw * 2 + gap + gx] = 240
        rel_png = f"window_frames/window_{win.index}.png"
        png_path = frames_dir / f"window_{win.index}.png"
        png_bytes = _write_gray_png(png_path, out_w, dh, bytes(canvas))
        data_uri = "data:image/png;base64," + base64.b64encode(png_bytes).decode("ascii")
        trim_head = max(0.0, win.t0 - win.raw_t0)
        trim_tail = max(0.0, win.raw_t1 - win.t1)
        checks.append({
            "index": win.index,
            "raw_t0": win.raw_t0,
            "raw_t1": win.raw_t1,
            "raw_dur": win.raw_t1 - win.raw_t0,
            "core_t0": win.t0,
            "core_t1": win.t1,
            "core_dur": win.t1 - win.t0,
            "trim_head": trim_head,
            "trim_tail": trim_tail,
            "trim_total": trim_head + trim_tail,
            "n_poses": win.n,
            "first_name": first_name,
            "last_name": last_name,
            "first_t": first_t,
            "last_t": last_t,
            "mad": mad_val,
            "disp_cm": win.span_disp_m * 100.0,
            "rot_deg": win.span_rot_deg,
            "png": str(png_path),
            "rel_png": rel_png,
            "data_uri": data_uri,
        })
    return checks


def slice_windows(poses, gt_windows, min_poses=5):
    """Poses on the ground-truth clock, one SE(3) mean per GT window."""
    windows = []
    for gt in gt_windows:
        chosen = [pose for pose in poses if gt.t0 <= pose.t <= gt.t1]
        if len(chosen) < min_poses:
            windows.append(None)
            continue
        rotation, position = core.se3_mean([(pose.R, pose.p) for pose in chosen])
        windows.append(core.Window(
            index=gt.index, t0=gt.t0, t1=gt.t1, n=len(chosen),
            R=rotation, p=position, median_speed=0.0,
        ))
    return windows


def errors_against_gt(windows, gt_windows):
    """Pair by index and evaluate revisit drift against identity GT (T_gt,i = I).

    Each revisit returns to the same stationary mark as window 0, so the
    defined ground-truth relative pose is T_gt,i = I_4 and
    T_err,i = T_gt,i^{-1} * T_rel,i = T_0^{-1} * T_i.
    """
    if not windows or windows[0] is None or windows[0].index != 0:
        return []
    R0, p0 = windows[0].R, windows[0].p
    R0i, p0i = core.invert(R0, p0)
    limit = len(gt_windows) if gt_windows is not None else len(windows)
    errors = []
    for i in range(1, min(len(windows), limit)):
        win = windows[i]
        if win is None:
            continue
        Rrel, prel = core.compose(R0i, p0i, win.R, win.p)
        trans = core._norm(prel)
        rot = core.rotation_angle_deg(Rrel)
        errors.append(core.SegmentError(i, trans, rot, trans, rot))
    return errors


def _metric(errors, field):
    return [getattr(error, field) for error in errors if getattr(error, field) is not None]


def evaluate_large_scene_stationary(sequence_dir, trajectories=None, out_dir=None,
                                    v_max=0.05, min_duration=3.0, max_gap=1.0,
                                    w_max=3.0, trim_s=0.75):
    """Evaluate one sequence. trajectories maps a method name to a TUM path.

    Returns a dict with windows, per-method errors, static jitter metrics, and written paths.
    """
    sequence_dir = Path(sequence_dir)
    gt_path = sequence_dir / "ground_truth" / "data" / "cam0_pose.txt"
    quest_dir = sequence_dir / "raw_data" / "quest"
    quest_files = sorted(quest_dir.glob("*quest_pro_state.csv"))
    if not quest_files:
        raise SystemExit(f"missing Quest csv under {quest_dir}")

    loaded_trajs: list[tuple[str, list]] = []
    if trajectories:
        for name in METHOD_ORDER:
            if name in trajectories and name != "Quest":
                loaded_trajs.append((name, core.load_tum(Path(trajectories[name]))))
        for name, path in trajectories.items():
            if name not in METHOD_ORDER:
                loaded_trajs.append((name, core.load_tum(Path(path))))

    gt_poses = core.load_tum(gt_path) if gt_path.is_file() else []
    # Prefer a full-sequence camera trajectory (e.g. TagFusion) or camera timestamps
    # so trailing stationary segments beyond a prematurely-ended cam0_pose.txt are kept.
    ref_poses = None
    if loaded_trajs:
        best_traj = max(loaded_trajs, key=lambda kv: kv[1][-1].t - kv[1][0].t)[1]
        if not gt_poses or best_traj[-1].t > gt_poses[-1].t + 1.0:
            ref_poses = best_traj
    if ref_poses is None:
        if gt_poses:
            cam_clock = _camera_clock_poses(sequence_dir)
            ref_poses = cam_clock if (cam_clock and cam_clock[-1].t > gt_poses[-1].t + 1.0) else gt_poses
        else:
            ref_poses = _camera_clock_poses(sequence_dir)
    if not ref_poses:
        raise SystemExit(f"missing reference poses / timestamps under {sequence_dir}")

    imu_path = sequence_dir / "raw_data" / "glass" / "imu0.txt"
    core_duration = min_duration
    min_trim = 0.75
    patrol_t0, patrol_t1 = _patrol_time_bounds(sequence_dir, ref_poses)
    if imu_path.is_file():
        gt_windows = detect_windows_from_imu(
            imu_path, ref_poses, gyro_deg_max=max(w_max, 3.5),
            min_duration=min_duration, max_gap=max_gap, core_duration=core_duration,
            t_min=patrol_t0, t_max=patrol_t1, min_trim=min_trim)
    else:
        gt_windows = detect_windows(
            ref_poses, v_max, min_duration, max_gap,
            w_max=w_max, core_duration=core_duration, min_trim=min_trim)
    if len(gt_windows) < 2:
        raise SystemExit(f"{sequence_dir}: found {len(gt_windows)} stationary windows")

    methods = []
    jitter_methods = []
    quest_poses = core.load_quest_csv(quest_files[0])
    quest_windows = detect_windows(
        quest_poses, v_max, min_duration, max_gap,
        w_max=w_max, core_duration=core_duration, min_trim=min_trim)
    if len(quest_windows) != len(gt_windows):
        print(f"WARNING: Quest has {len(quest_windows)} stationary windows, "
              f"GT has {len(gt_windows)}. Extra windows at the end are dropped.")
    methods.append(("Quest", quest_windows, errors_against_gt(quest_windows, gt_windows)))
    quest_jitters = []
    for seg_i in range(1, min(len(quest_windows), len(gt_windows))):
        qw = quest_windows[seg_i]
        if qw is not None:
            jit = compute_static_jitter(
                quest_poses, qw.t0, qw.t1, seg_index=seg_i, min_duration=1.0,
            )
            if jit is not None:
                quest_jitters.append(jit)
    jitter_methods.append(("Quest", quest_jitters))

    for name, poses in loaded_trajs:
        windows = slice_windows(poses, gt_windows)
        methods.append((name, windows, errors_against_gt(windows, gt_windows)))
        jitters = []
        for gw in gt_windows[1:]:
            jit = compute_static_jitter(
                poses, gw.t0, gw.t1, seg_index=gw.index, min_duration=1.0,
            )
            if jit is not None:
                jitters.append(jit)
        jitter_methods.append((name, jitters))

    if out_dir is None:
        out_dir = sequence_dir / "stationary_eval"
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    frame_checks = _inspect_window_frames(sequence_dir, gt_windows, out_dir)
    csv_path = out_dir / "stationary_errors.csv"
    jitter_csv_path = out_dir / "stationary_jitter.csv"
    html_path = out_dir / "stationary_report.html"
    _write_csv(csv_path, gt_windows, methods)
    _write_jitter_csv(jitter_csv_path, jitter_methods)
    html_path.write_text(
        _render_html(
            sequence_dir, gt_windows, methods, frame_checks,
            trim_s=trim_s, jitter_methods=jitter_methods,
        ),
        encoding="utf-8",
    )
    return {
        "sequence": str(sequence_dir),
        "gt_windows": gt_windows,
        "frame_checks": frame_checks,
        "methods": methods,
        "jitter_methods": jitter_methods,
        "csv": csv_path,
        "jitter_csv": jitter_csv_path,
        "html": html_path,
    }


def _write_jitter_csv(path: Path, jitter_methods) -> None:
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow([
            "method", "seg", "t_start", "t_end", "duration_s", "n_poses",
            "p2p_trans_mm", "p2p_rot_deg",
            "max_jump_trans_mm", "max_jump_rot_deg",
            "rms_trans_mm", "rms_rot_deg",
            "p95_trans_mm", "p95_rot_deg",
            "drift_rate_mm_s", "drift_rate_deg_s",
            "net_drift_mm", "net_drift_deg",
        ])
        for name, jitters in jitter_methods:
            for j in jitters:
                writer.writerow([
                    name,
                    j.index,
                    f"{j.t0:.3f}",
                    f"{j.t1:.3f}",
                    f"{j.duration_s:.3f}",
                    j.n,
                    f"{j.p2p_trans_mm:.4f}",
                    f"{j.p2p_rot_deg:.5f}",
                    f"{j.max_jump_trans_mm:.4f}",
                    f"{j.max_jump_rot_deg:.5f}",
                    f"{j.rms_trans_mm:.4f}",
                    f"{j.rms_rot_deg:.5f}",
                    f"{j.p95_trans_mm:.4f}",
                    f"{j.p95_rot_deg:.5f}",
                    f"{j.drift_rate_mm_s:.4f}",
                    f"{j.drift_rate_deg_s:.5f}",
                    f"{j.net_drift_mm:.4f}",
                    f"{j.net_drift_deg:.5f}",
                ])


def _write_csv(path, gt_windows, methods):
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow([
            "method", "seg", "gt_t_start", "gt_t_end", "n_poses",
            "err_trans_m", "err_rot_deg",
        ])
        for name, windows, errors in methods:
            by_index = {error.index: error for error in errors}
            for window in windows:
                if window is None or window.index == 0:
                    continue
                error = by_index.get(window.index)
                gt = gt_windows[window.index] if window.index < len(gt_windows) else None
                writer.writerow([
                    name,
                    window.index,
                    "" if gt is None else f"{gt.t0:.3f}",
                    "" if gt is None else f"{gt.t1:.3f}",
                    window.n,
                    "" if error is None else f"{error.trans_vs_gt_m:.6f}",
                    "" if error is None else f"{error.rot_vs_gt_deg:.4f}",
                ])


def _series(methods, field):
    series = []
    nseg = 0
    for name, _windows, errors in methods:
        values = {error.index: getattr(error, field) for error in errors}
        nseg = max(nseg, max(values, default=0))
        series.append((name, COLORS.get(name, "#72B7B2"), values))
    return series, nseg


def _bars(title, categories, series):
    """Grouped bar chart as SVG. series is (name, color, {category: value})."""
    width, height = 760, 360
    left, right, top, bottom = 56, 16, 36, 78
    plot_w = width - left - right
    plot_h = height - top - bottom
    ymax = 0.0
    for _name, _color, values in series:
        for value in values.values():
            if value is not None:
                ymax = max(ymax, value)
    ymax = ymax * 1.15 if ymax > 0 else 1.0
    ncat = max(len(categories), 1)
    ngroup = max(len(series), 1)
    group_w = plot_w / ncat
    bar_w = min(28, group_w * 0.8 / ngroup)
    parts = [
        f'<svg viewBox="0 0 {width} {height}" xmlns="http://www.w3.org/2000/svg" '
        f'role="img" aria-label="{html.escape(title)}">',
        f'<text x="{width / 2}" y="22" text-anchor="middle" font-size="15" '
        f'font-family="sans-serif">{html.escape(title)}</text>',
        f'<line x1="{left}" y1="{top + plot_h}" x2="{width - right}" y2="{top + plot_h}" stroke="#444"/>',
        f'<line x1="{left}" y1="{top}" x2="{left}" y2="{top + plot_h}" stroke="#444"/>',
    ]
    for tick in range(6):
        value = ymax * tick / 5
        y = top + plot_h - plot_h * tick / 5
        parts.append(
            f'<text x="{left - 8}" y="{y + 4}" text-anchor="end" font-size="11" '
            f'font-family="sans-serif" fill="#333">{value:.3g}</text>'
        )
        parts.append(
            f'<line x1="{left}" y1="{y}" x2="{width - right}" y2="{y}" stroke="#eee"/>'
        )
    legend = []
    for cat_i, category in enumerate(categories):
        cx = left + group_w * cat_i + group_w / 2
        parts.append(
            f'<text x="{cx}" y="{top + plot_h + 18}" text-anchor="middle" font-size="12" '
            f'font-family="sans-serif">{html.escape(str(category))}</text>'
        )
        for series_i, (name, color, values) in enumerate(series):
            value = values.get(category if isinstance(category, int) else category)
            # categories may be "Seg.1" while keys are 1, or "Mean"/"RMSE"
            if value is None:
                continue
            x = cx - (ngroup * bar_w) / 2 + series_i * bar_w
            bar_h = plot_h * value / ymax
            y = top + plot_h - bar_h
            parts.append(
                f'<rect x="{x:.1f}" y="{y:.1f}" width="{bar_w - 2:.1f}" height="{bar_h:.1f}" '
                f'fill="{color}"><title>{html.escape(name)} {category}: {value:.5f}</title></rect>'
            )
            parts.append(
                f'<text x="{x + bar_w / 2:.1f}" y="{y - 3:.1f}" text-anchor="middle" '
                f'font-size="9" font-family="sans-serif">{value:.3f}</text>'
            )
        legend.append(None)
    for series_i, (name, color, _values) in enumerate(series):
        x = left + series_i * 170
        y = height - 22
        parts.append(f'<rect x="{x}" y="{y}" width="12" height="12" fill="{color}"/>')
        parts.append(
            f'<text x="{x + 16}" y="{y + 11}" font-size="12" font-family="sans-serif">'
            f'{html.escape(name)}</text>'
        )
    parts.append("</svg>")
    return "\n".join(parts)


def _chart_values(methods, field):
    """Return (segment_categories, segment_series, summary_categories, summary_series)."""
    raw, nseg = _series(methods, field)
    seg_cats = list(range(1, nseg + 1))
    seg_series = []
    sum_series = []
    for name, color, values in raw:
        seg_series.append((name, color, {i: values[i] for i in seg_cats if i in values}))
        nums = [values[i] for i in seg_cats if i in values]
        summary = {}
        if nums:
            summary["Mean"] = core._mean(nums)
            summary["RMSE"] = core._rmse(nums)
        sum_series.append((name, color, summary))
    return seg_cats, seg_series, ["Mean", "RMSE"], sum_series


def _table(methods, field, unit):
    raw, nseg = _series(methods, field)
    header = ["Method"] + [f"Seg.{i} ({unit})" for i in range(1, nseg + 1)] + [
        f"Mean ({unit})", f"RMSE ({unit})",
    ]
    rows = []
    for name, _color, values in raw:
        nums = [values[i] for i in range(1, nseg + 1) if i in values]
        cells = [name]
        for i in range(1, nseg + 1):
            cells.append("" if i not in values else f"{values[i]:.5f}")
        cells.append("" if not nums else f"{core._mean(nums):.5f}")
        cells.append("" if not nums else f"{core._rmse(nums):.5f}")
        rows.append(cells)
    return header, rows


def _jitter_summary_table(jitter_methods):
    """Summary table across Seg.1..K (excluding Window 0) on the unified core windows."""
    header = [
        "Method",
        "有效段数",
        "窗口时长 (s)",
        "P2P 位置 (mm)",
        "P2P 旋转 (°)",
        "MaxJump 位置 (mm)",
        "MaxJump 旋转 (°)",
        "Std 位置 (mm)",
        "Std 旋转 (°)",
        "P95 位置 (mm)",
        "P95 旋转 (°)",
        "漂移速率 (mm/s)",
        "漂移速率 (°/s)",
    ]
    rows = []
    for name, jitters in jitter_methods:
        if not jitters:
            continue
        rows.append([
            name,
            str(len(jitters)),
            f"{core._mean([j.duration_s for j in jitters]):.2f}",
            f"{core._mean([j.p2p_trans_mm for j in jitters]):.3f}",
            f"{core._mean([j.p2p_rot_deg for j in jitters]):.4f}",
            f"{core._mean([j.max_jump_trans_mm for j in jitters]):.3f}",
            f"{core._mean([j.max_jump_rot_deg for j in jitters]):.4f}",
            f"{core._mean([j.rms_trans_mm for j in jitters]):.3f}",
            f"{core._mean([j.rms_rot_deg for j in jitters]):.4f}",
            f"{core._mean([j.p95_trans_mm for j in jitters]):.3f}",
            f"{core._mean([j.p95_rot_deg for j in jitters]):.4f}",
            f"{core._mean([j.drift_rate_mm_s for j in jitters]):.3f}",
            f"{core._mean([j.drift_rate_deg_s for j in jitters]):.4f}",
        ])
    return header, rows


def _jitter_seg_table(jitter_methods, field: str, unit: str, digits: int = 3):
    nseg = 0
    series = []
    for name, jitters in jitter_methods:
        vals = {j.index: getattr(j, field) for j in jitters}
        nseg = max(nseg, max(vals, default=0))
        series.append((name, COLORS.get(name, "#72B7B2"), vals))
    header = (
        ["Method"]
        + [f"Seg.{i} ({unit})" for i in range(1, nseg + 1)]
        + [f"Mean ({unit})", f"Max ({unit})"]
    )
    rows = []
    fmt_str = f"{{:.{digits}f}}"
    for name, _color, vals in series:
        nums = [vals[i] for i in range(1, nseg + 1) if i in vals]
        cells = [name]
        for i in range(1, nseg + 1):
            cells.append("" if i not in vals else fmt_str.format(vals[i]))
        cells.append("" if not nums else fmt_str.format(core._mean(nums)))
        cells.append("" if not nums else fmt_str.format(max(nums)))
        rows.append(cells)
    return header, rows, nseg, series


def _html_table(header, rows, left_cols=(0,)):
    head = "".join(
        f'<th style="text-align:{"left" if idx in left_cols else "right"}">{html.escape(cell)}</th>'
        for idx, cell in enumerate(header)
    )
    body = []
    for row in rows:
        cells_html = "".join(
            f'<td style="text-align:{"left" if idx in left_cols else "right"}">{html.escape(cell)}</td>'
            for idx, cell in enumerate(row)
        )
        body.append(f"<tr>{cells_html}</tr>")
    return f"<table><thead><tr>{head}</tr></thead><tbody>{''.join(body)}</tbody></table>"


def _render_html(sequence_dir, gt_windows, methods, frame_checks=None,
                 trim_s=0.75, jitter_methods=None):
    trans_header, trans_rows = _table(methods, "trans_vs_gt_m", "m")
    rot_header, rot_rows = _table(methods, "rot_vs_gt_deg", "deg")
    seg_cats, seg_series, sum_cats, sum_series = _chart_values(methods, "trans_vs_gt_m")
    rseg_cats, rseg_series, rsum_cats, rsum_series = _chart_values(methods, "rot_vs_gt_deg")
    checks_by_idx = {c["index"]: c for c in (frame_checks or [])}
    win_header = [
        "窗口", "对应评测段", "IMU 检出原始窗口 (s)", "原始时长 (s)",
        "前端剔除 (s)", "后端剔除 (s)", "合计剔除 (s)",
        "保留核心窗口 (s)", "保留时长 (s)", "参与平均帧数",
        "核心段首尾位移 (cm)", "核心段首尾转角 (°)", "首末帧图像 (像素差 MAD)",
    ]
    win_rows = []
    for w in gt_windows:
        trim_head = max(0.0, w.t0 - w.raw_t0)
        trim_tail = max(0.0, w.raw_t1 - w.t1)
        chk = checks_by_idx.get(w.index)
        img_summary = (
            f"{chk['first_name']} .. {chk['last_name']} (MAD={chk['mad']:.2f})"
            if chk else "-"
        )
        win_rows.append([
            f"窗口 {w.index}",
            "参考起点 (T_0)" if w.index == 0 else f"Seg.{w.index}",
            f"{w.raw_t0:.2f} – {w.raw_t1:.2f}",
            f"{w.raw_t1 - w.raw_t0:.2f}",
            f"{trim_head:.2f}",
            f"{trim_tail:.2f}",
            f"{trim_head + trim_tail:.2f}",
            f"{w.t0:.2f} – {w.t1:.2f}",
            f"{w.t1 - w.t0:.2f}",
            str(w.n),
            f"{w.span_disp_m * 100:.2f}",
            f"{w.span_rot_deg:.2f}",
            img_summary,
        ])

    jitter_html = ""
    if jitter_methods:
        j_sum_hdr, j_sum_rows = _jitter_summary_table(jitter_methods)
        p2p_hdr, p2p_rows, p2p_n, p2p_series = _jitter_seg_table(
            jitter_methods, "p2p_trans_mm", "mm", digits=3
        )
        p2p_r_hdr, p2p_r_rows, p2p_rn, p2p_r_series = _jitter_seg_table(
            jitter_methods, "p2p_rot_deg", "deg", digits=4
        )
        jmp_hdr, jmp_rows, jmp_n, jmp_series = _jitter_seg_table(
            jitter_methods, "max_jump_trans_mm", "mm", digits=3
        )
        rms_hdr, rms_rows, _rms_n, _rms_series = _jitter_seg_table(
            jitter_methods, "rms_trans_mm", "mm", digits=3
        )
        jitter_html = f"""
<h2>静止基测（单段静态抖动评估，跳过第 0 段，与回环评测统一核心窗口）</h2>
<div class="method-box">
  <ul>
    <li><b>评估范围与统一窗口长度：</b>不使用开头第 0 段（<code>窗口 0</code>），仅对后续巡逻返回静止段 <code>Seg.1 .. Seg.{p2p_n}</code> 逐段独立统计静态抖动；<b>直接复用与回环误差完全一致的核心窗口 <code>[core_t0, core_t1]</code></b>（常规长段统一固定为正中间 <b><code>3.00 s</code></b>，确保极值指标 <code>P2P</code> 与 <code>MaxJump</code> 在所有段落和所有方法上具有严格相同的统计时长；若遇较短静止段则同样按自适应缩窄窗口，有效时长不足 <code>1.0 s</code> 的自动剔除）。</li>
    <li><b>五项静态指标：</b>① <b>P2P（峰峰值）</b>：核心窗内任意两帧间最大三维距离 / 最大姿态夹角；② <b>MaxJump（最大单帧跳变）</b>：相邻两帧最大位置/角度突变；③ <b>Std（抖动标准差 / RMS）</b>：相对该段 <code>SE(3)</code> 均值位姿的偏差标准差；④ <b>P95（95% 分位数）</b>：相对均值偏差的 95% 分位数；⑤ <b>漂移速率</b>：核心窗首末帧净位移/净转角除以窗口时长。</li>
  </ul>
</div>
<h3>各方法静止基测五项指标汇总（各有效静止段均值）</h3>
{_html_table(j_sum_hdr, j_sum_rows, left_cols=(0,))}
<div class="charts">
{_bars("各静止段位置峰峰值 P2P (mm)", [f"Seg.{i}" for i in range(1, p2p_n + 1)], [(n, c, {f"Seg.{k}": v for k, v in vals.items()}) for n, c, vals in p2p_series])}
{_bars("各静止段最大单帧位置跳变 MaxJump (mm)", [f"Seg.{i}" for i in range(1, jmp_n + 1)], [(n, c, {f"Seg.{k}": v for k, v in vals.items()}) for n, c, vals in jmp_series])}
</div>
<h3>分段明细：位置峰峰值 P2P (mm)</h3>
{_html_table(p2p_hdr, p2p_rows, left_cols=(0,))}
<h3>分段明细：旋转峰峰值 P2P (deg)</h3>
{_html_table(p2p_r_hdr, p2p_r_rows, left_cols=(0,))}
<h3>分段明细：相邻帧最大位置跳变 MaxJump (mm)</h3>
{_html_table(jmp_hdr, jmp_rows, left_cols=(0,))}
<h3>分段明细：位置抖动标准差 Std (mm)</h3>
{_html_table(rms_hdr, rms_rows, left_cols=(0,))}
"""

    frame_cards = ""
    if frame_checks:
        items = []
        for chk in frame_checks:
            label = "参考窗口 0 (T_0)" if chk["index"] == 0 else f"窗口 {chk['index']} (Seg.{chk['index']})"
            img_src = chk.get("data_uri") or chk["rel_png"]
            items.append(
                f'<div style="border:1px solid #ddd;padding:12px;margin:12px 0;background:#fafafa">'
                f'<div style="margin-bottom:4px"><b>{label}</b> ｜ '
                f'IMU 检出窗口：<code>{chk["raw_t0"]:.2f}s – {chk["raw_t1"]:.2f}s</code>（{chk["raw_dur"]:.2f}s） ｜ '
                f'剔除秒数：前端 <b>{chk["trim_head"]:.2f}s</b>、后端 <b>{chk["trim_tail"]:.2f}s</b>（合计剔除 <b>{chk["trim_total"]:.2f}s</b>） ｜ '
                f'保留核心窗口：<b><code>{chk["core_t0"]:.2f}s – {chk["core_t1"]:.2f}s</code></b>（{chk["core_dur"]:.2f}s，{chk["n_poses"]} 帧）</div>'
                f'<div style="margin-bottom:4px">首帧 <code>{html.escape(chk["first_name"])}</code> ({chk["first_t"]:.2f}s) '
                f'→ 末帧 <code>{html.escape(chk["last_name"])}</code> ({chk["last_t"]:.2f}s) ｜ '
                f'核心段首尾位移 <b>{chk["disp_cm"]:.2f} cm</b>，首尾转角 <b>{chk["rot_deg"]:.2f}°</b>，'
                f'首末帧像素平均绝对差 MAD = <b>{chk["mad"]:.2f}</b> / 255</div>'
                f'<div style="font-size:12px;color:#555;margin:4px 0">左：核心窗口首帧图像 ｜ 中：核心窗口末帧图像 ｜ 右：|首帧 - 末帧|×4 像素差分图（越黑代表越静止）</div>'
                f'<img src="{img_src}" style="max-width:100%;border:1px solid #ccc"/>'
                f'</div>'
            )
        frame_cards = "<h2>静止段首末帧图像一致性检查</h2>" + "".join(items)
    return f"""<!DOCTYPE html>
<html lang="zh"><head><meta charset="utf-8">
<title>静止段位姿漂移与静态稳定性 {html.escape(str(sequence_dir))}</title>
<style>
body {{ font-family: sans-serif; margin: 24px; color: #222; line-height: 1.5; }}
table {{ border-collapse: collapse; margin: 12px 0 28px; width: 100%; }}
th, td {{ border: 1px solid #ccc; padding: 6px 10px; font-size: 13px; }}
th {{ background: #f4f4f4; }}
.charts {{ display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }}
.method-box {{ background: #f8f9fb; border: 1px solid #d8dee9; padding: 12px 16px; margin: 14px 0 22px; }}
.method-box ul {{ margin: 6px 0; padding-left: 20px; }}
h2 {{ margin-top: 28px; }}
h3 {{ margin-top: 18px; margin-bottom: 6px; font-size: 15px; }}
</style></head><body>
<h1>室内大场景静止评测报告（回环重复性 &amp; 静止基测）</h1>
<p><b>数据序列：</b><code>{html.escape(str(sequence_dir))}</code></p>

<h2>静止检测的判断方法与自适应去头去尾规则</h2>
<div class="method-box">
  <ul>
    <li><b>1. 1000 Hz 原始 IMU 静止检测（无滤波滞后）：</b>直接读取眼镜原始 IMU 数据 <code>raw_data/glass/imu0.txt</code>，按 <code>0.1 s</code>（约 100 个采样点）分桶统计，同时满足以下三条物理判据才标为静止：
      <ul>
        <li><b>加速度计三轴波动标准差：</b>&sigma;<sub>a</sub> = &radic;(&frac1N;&sum;||a<sub>i</sub> &minus; a&#772;||<sup>2</sup>) &le; <b>0.15 m/s<sup>2</sup></b>（滤除迈步触地与身体晃动；完全站稳时实测约为 0.03～0.06 m/s<sup>2</sup>）；</li>
        <li><b>加速度计均值模长接近重力：</b>| ||a&#772;|| &minus; 9.8 | &le; <b>0.5 m/s<sup>2</sup></b>（排除加减速运动）；</li>
        <li><b>陀螺仪角速度有效值：</b>||&omega;||<sub>rms</sub> &le; <b>3.5&deg;/s</b>（排除原地转头与身体旋转）。</li>
      </ul>
    </li>
    <li><b>2. 全程巡逻覆盖与短初始窗兼容：</b>覆盖从起点首次静止到巡逻结束前（<code>eight_start_time</code> 之前）的全部静止段，不再受离线真值提前截断影响；合并相邻 &le; <code>1.0 s</code> 间隙后，最小检出时长放宽至 <code>2.00 s</code>（兼容约 <code>2 s</code> 的短起点静止段）。</li>
    <li><b>3. 自适应核心窗口截取（长窗保持 3.00s，短窗缩窄保最少 0.75s 去头尾）：</b>对每个静止段 <code>[raw_t0, raw_t1]</code>（时长 <code>D</code>）：当 <code>D &ge; 4.50 s</code> 时保持选取正中间 <b><code>3.00 s</code></b> 核心窗口；当窗口较短（<code>2.00 s &le; D &lt; 4.50 s</code>）时，优先保证前后端<b>各截去至少 <code>0.75 s</code></b>，自适应将核心窗口缩窄为 <code>D &minus; 1.50 s</code>（最小 <code>0.50 s</code>），彻底消除短窗口仅截去几毫秒导致的边缘晃动污染。</li>
    <li><b>4. 重新定义闭环真值与重复性位姿误差（无外部 GT）：</b>每次回到同一回环标定板处静止一次，以首个静止窗对齐后的理想真值相对位姿定义为<b>恒等变换 <code>T_gt,i = I_4</code></b>。将每个核心窗口内的位姿在 <code>SE(3)</code> 空间迭代平均得到 <code>T_i</code>，每条轨迹以自身首个静止窗口 <code>T_0</code> 为参考计算相对位姿误差 <code>T_err,i = T_0^{{-1}} T_i</code>。Quest 采用平移速度 &le; 0.05 m/s 且角速度 &le; 3.0&deg;/s 配合相同的自适应去头去尾规则提取窗口，按先后顺序一一配对。</li>
  </ul>
</div>

<h2>检测出的静止窗口与剔除秒数明细</h2>
{_html_table(win_header, win_rows, left_cols=(0, 1, 12))}

<h2>Translation Error（回环相对位置重复性误差）</h2>
{_html_table(trans_header, trans_rows, left_cols=(0,))}
<div class="charts">
{_bars("每段静止段平移误差 (m)", [f"Seg.{i}" for i in seg_cats], [(n, c, {f"Seg.{k}": v for k, v in vals.items()}) for n, c, vals in seg_series])}
{_bars("静止段平移平均 (m)", sum_cats, sum_series)}
</div>
<h2>Rotation Error（回环相对姿态重复性误差）</h2>
{_html_table(rot_header, rot_rows, left_cols=(0,))}
<div class="charts">
{_bars("每段静止段旋转误差 (deg)", [f"Seg.{i}" for i in rseg_cats], [(n, c, {f"Seg.{k}": v for k, v in vals.items()}) for n, c, vals in rseg_series])}
{_bars("静止段旋转平均 (deg)", rsum_cats, rsum_series)}
</div>
{jitter_html}
{frame_cards}
</body></html>
"""


def _print_table(title, header, rows):
    print(f"\n{title}")
    widths = [max(len(header[c]), *(len(row[c]) for row in rows)) for c in range(len(header))]
    def fmt(cells):
        return " | ".join(cells[c].ljust(widths[c]) for c in range(len(cells)))
    print(fmt(header))
    print("-+-".join("-" * n for n in widths))
    for row in rows:
        print(fmt(row))


# Three tracking methods. Quest is loaded inside evaluate_large_scene_stationary.
# ORB-SLAM3 is the upstream stereo-inertial example. Its EuRoC writer stores the
# IMU body pose with nanosecond timestamps; the runner converts that to the
# cam0 pose in seconds before evaluation. TagFusion (w/o Tag) is this binary
# with Tag.enable=0. TagFusion is this binary with Tag.enable=1.
METHOD_RUNS = (
    {"name": "ORB-SLAM3", "kind": "orbslam3", "dirname": "orbslam3"},
    {"name": "TagFusion (w/o Tag)", "kind": "tagfusion", "dirname": "wo_tag",
     "tag_enable": "0", "traj_name": "wo_tag"},
    {"name": "TagFusion", "kind": "tagfusion", "dirname": "with_tag",
     "tag_enable": "1", "traj_name": "with_tag"},
)


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _read_Tbc(settings_yaml: Path):
    text = Path(settings_yaml).read_text(errors="ignore")
    match = re.search(r"IMU\.T_b_c1:.*?data:\s*\[([^\]]+)\]", text, re.S)
    if not match:
        raise SystemExit(f"IMU.T_b_c1 not found in {settings_yaml}")
    vals = [float(item) for item in match.group(1).replace("\n", " ").split(",") if item.strip()]
    if len(vals) != 16:
        raise SystemExit(f"IMU.T_b_c1 has {len(vals)} values in {settings_yaml}")
    rotation = tuple(tuple(vals[row * 4 + col] for col in range(3)) for row in range(3))
    translation = (vals[3], vals[7], vals[11])
    return rotation, translation


def _ns(timestamp: float) -> int:
    return int(round(timestamp * 1e9))


def _prepare_euroc(glass_dir: Path, dst: Path) -> Path:
    """EuRoC layout the upstream example reads. Images stay symlinks.

    Frames earlier than the first IMU sample are dropped. The example indexes
    IMU with an unsigned-style decrement and aborts if IMU starts later.
    """
    dst = Path(dst)
    cam0 = dst / "mav0" / "cam0" / "data"
    cam1 = dst / "mav0" / "cam1" / "data"
    cam0.mkdir(parents=True, exist_ok=True)
    cam1.mkdir(parents=True, exist_ok=True)
    times_path = dst / "times.txt"
    imu_csv = dst / "mav0" / "imu0" / "data.csv"
    imu_csv.parent.mkdir(parents=True, exist_ok=True)
    stamp_path = glass_dir / "cam0" / "images" / "timestamps.txt"
    imu_path = glass_dir / "imu0.txt"
    frames = []
    for line in stamp_path.read_text().splitlines():
        parts = line.split()
        if len(parts) < 2:
            continue
        frames.append((parts[0], float(parts[1])))
    imu_rows = []
    for line in imu_path.read_text().splitlines():
        parts = line.split()
        if len(parts) < 8 or parts[0].startswith("#"):
            continue
        imu_rows.append((float(parts[0]), float(parts[2]), float(parts[3]), float(parts[4]),
                         float(parts[5]), float(parts[6]), float(parts[7])))
    if not frames or not imu_rows:
        raise SystemExit(f"empty images or IMU under {glass_dir}")
    first_imu = imu_rows[0][0]
    kept = [(name, stamp) for name, stamp in frames if stamp + 1e-6 >= first_imu]
    if len(kept) < 2:
        raise SystemExit(f"no image at or after the first IMU sample in {glass_dir}")
    left_src = glass_dir / "cam0" / "images"
    right_src = glass_dir / "cam1" / "images"
    with times_path.open("w") as handle:
        for name, stamp in kept:
            token = str(_ns(stamp))
            for folder, src_root in ((cam0, left_src), (cam1, right_src)):
                link = folder / f"{token}.png"
                target = src_root / name
                if link.is_symlink() or link.exists():
                    if link.is_symlink() and os.readlink(link) == str(target):
                        continue
                    link.unlink()
                link.symlink_to(target)
            handle.write(token + "\n")
    with imu_csv.open("w") as handle:
        for stamp, gx, gy, gz, ax, ay, az in imu_rows:
            handle.write(f"{_ns(stamp)},{gx:.9f},{gy:.9f},{gz:.9f},{ax:.9f},{ay:.9f},{az:.9f}\n")
    print(f"euroc {dst}: images {len(kept)}/{len(frames)} imu {len(imu_rows)}")
    return times_path


def _body_euroc_to_camera_tum(src: Path, dst: Path, Tbc) -> None:
    """Upstream IMU_STEREO EuRoC file is Twb, timestamp in ns, qx qy qz qw."""
    rotation_bc, translation_bc = Tbc
    lines = []
    for line in src.read_text(errors="ignore").splitlines():
        parts = line.split()
        if len(parts) < 8:
            continue
        stamp = float(parts[0]) / 1e9
        translation = tuple(float(value) for value in parts[1:4])
        rotation = core.quat_to_R(tuple(float(value) for value in parts[4:8]))
        rotation_wc, translation_wc = core.compose(
            rotation, translation, rotation_bc, translation_bc)
        qx, qy, qz, qw = core.R_to_quat(rotation_wc)
        tx, ty, tz = translation_wc
        lines.append(
            f"{stamp:.6f} {tx:.9f} {ty:.9f} {tz:.9f} {qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}")
    dst.write_text("\n".join(lines) + ("\n" if lines else ""))


def _setting_value(text: str, key: str):
    for line in text.splitlines():
        head = line.split("#", 1)[0].strip()
        if head.startswith(key + ":"):
            return head.split(":", 1)[1].strip()
    return None


def _write_tag_settings(src: Path, dst: Path, tag_enable: str) -> None:
    """Headless settings. TagFusion (enable=1) uses gamma 0.5; tag-off does not detect."""
    gamma_enable = "1" if tag_enable == "1" else "0"
    replacements = {
        "Tag.enable": tag_enable,
        "Tag.gamma_enable": gamma_enable,
        "Tag.gamma": "0.5",
        "Tag.show_in_orb_viewer": "0",
        "Tag.verbose": "0",
        "Tag.local_ba.verbose": "0",
        "Tag.factor_weight.verbose": "0",
        "Tag.loop.verbose": "0",
    }
    out = []
    seen = set()
    for line in src.read_text().splitlines():
        head = line.split("#", 1)[0].strip()
        replaced = False
        for key, value in replacements.items():
            if head.startswith(key + ":"):
                out.append(f"{key}: {value}")
                seen.add(key)
                replaced = True
                break
        if not replaced:
            out.append(line)
    for key, value in replacements.items():
        if key not in seen:
            out.append(f"{key}: {value}")
    dst.write_text("\n".join(out) + "\n")


def _saved_settings_match(settings: Path, tag_enable: str) -> bool:
    if not settings.is_file():
        return False
    text = settings.read_text()
    gamma_enable = "1" if tag_enable == "1" else "0"
    return (_setting_value(text, "Tag.enable") == tag_enable
            and _setting_value(text, "Tag.gamma_enable") == gamma_enable
            and _setting_value(text, "Tag.show_in_orb_viewer") == "0")


def _command_running(fragment: str) -> bool:
    proc = Path("/proc")
    for cmdline in proc.glob("[0-9]*/cmdline"):
        try:
            raw = cmdline.read_bytes().replace(b"\x00", b" ").decode(errors="ignore")
        except OSError:
            continue
        if fragment in raw and "stereo_inertial" in raw:
            return True
    return False


def _runtime_env(prefer_orbslam: bool = False) -> dict:
    """Library paths for a home-directory build (no system install)."""
    env = os.environ.copy()
    home = Path.home()
    repo = _repo_root()
    tag_libs = [
        repo / "lib",
        repo / "Thirdparty/DBoW2/lib",
        repo / "Thirdparty/g2o/lib",
    ]
    orb_libs = [
        home / "code/ORB_SLAM3/lib",
        home / "code/ORB_SLAM3/Thirdparty/DBoW2/lib",
        home / "code/ORB_SLAM3/Thirdparty/g2o/lib",
    ]
    extra = [
        home / "opt/sysroot/usr/lib/x86_64-linux-gnu",
        home / "opt/pangolin/lib",
        *(orb_libs + tag_libs if prefer_orbslam else tag_libs + orb_libs),
    ]
    prefix = [str(path) for path in extra if path.is_dir()]
    if prefix:
        old = env.get("LD_LIBRARY_PATH", "")
        env["LD_LIBRARY_PATH"] = ":".join(prefix + ([old] if old else []))
    return env


def _run_logged(cmd, cwd: Path, log_path: Path, prefer_orbslam: bool = False) -> int:
    print("RUN", " ".join(cmd))
    with log_path.open("w") as handle:
        status = subprocess.call(
            cmd, cwd=cwd, stdout=handle, stderr=subprocess.STDOUT,
            env=_runtime_env(prefer_orbslam=prefer_orbslam))
    print(f"EXIT {status} log={log_path}")
    return status


def run_large_scene_stationary(sequence_dir, settings_yaml, out_dir=None,
                               tagfusion_bin=None, vocabulary=None, orbslam_bin=None,
                               v_max=0.05, min_duration=3.0, max_gap=1.0,
                               w_max=3.0, trim_s=1.0):
    """Run ORB-SLAM3, TagFusion (w/o Tag) and TagFusion, then evaluate.

    A method is skipped when its camera trajectory already exists, or when a
    stereo-inertial process is already tracking that sequence. Returns the
    evaluate_large_scene_stationary result.
    """
    sequence_dir = Path(sequence_dir)
    settings_yaml = Path(settings_yaml)
    glass_dir = sequence_dir / "raw_data" / "glass"
    if not glass_dir.is_dir():
        raise SystemExit(f"missing {glass_dir}")
    repo = _repo_root()
    if out_dir is None:
        out_dir = repo / "results" / sequence_dir.parent.parent.name / sequence_dir.parent.name / sequence_dir.name
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    if tagfusion_bin is None:
        tagfusion_bin = repo / "Examples/TagFusion/XREAL_Glass/stereo_inertial_xreal_glass_tag"
    if vocabulary is None:
        vocabulary = repo / "Vocabulary/ORBvoc.txt"
    if orbslam_bin is None:
        orbslam_bin = Path(os.environ.get(
            "ORBSLAM_BIN",
            str(Path.home() / "code/ORB_SLAM3/Examples/Stereo-Inertial/stereo_inertial_euroc")))
    tagfusion_bin = Path(tagfusion_bin)
    vocabulary = Path(vocabulary)
    orbslam_bin = Path(orbslam_bin)
    trajectories = {}

    for spec in METHOD_RUNS:
        method_dir = out_dir / spec["dirname"]
        method_dir.mkdir(parents=True, exist_ok=True)
        if spec["kind"] == "tagfusion":
            traj = method_dir / f"f_{spec['traj_name']}.txt"
            settings = method_dir / "settings.yaml"
            if (traj.is_file() and traj.stat().st_size > 0
                    and _saved_settings_match(settings, spec["tag_enable"])):
                trajectories[spec["name"]] = str(traj)
                print(f"SKIP {spec['name']} {traj}")
                continue
            if traj.is_file() and traj.stat().st_size > 0:
                print(f"RERUN {spec['name']}: existing trajectory was not gamma={'1' if spec['tag_enable']=='1' else '0'} / viewer off")
            if _command_running(str(glass_dir)):
                print(f"BUSY {spec['name']} {glass_dir}")
                continue
            _write_tag_settings(settings_yaml, settings, spec["tag_enable"])
            status = _run_logged(
                [str(tagfusion_bin), str(vocabulary), str(settings), str(glass_dir), spec["traj_name"]],
                method_dir, method_dir / "run.log")
            if status == 0 and traj.is_file() and traj.stat().st_size > 0:
                trajectories[spec["name"]] = str(traj)
        else:
            camera_tum = method_dir / "camera_tum.txt"
            if camera_tum.is_file() and camera_tum.stat().st_size > 0:
                trajectories[spec["name"]] = str(camera_tum)
                print(f"SKIP {spec['name']} {camera_tum}")
                continue
            if not orbslam_bin.is_file():
                print(f"MISSING binary {orbslam_bin}")
                continue
            if _command_running(str(method_dir)):
                print(f"BUSY {spec['name']} {method_dir}")
                continue
            times_path = _prepare_euroc(glass_dir, method_dir / "euroc")
            status = _run_logged(
                [str(orbslam_bin), str(vocabulary), str(settings_yaml),
                 str(method_dir / "euroc"), str(times_path), "orbslam3"],
                method_dir, method_dir / "run.log", prefer_orbslam=True)
            raw = method_dir / "f_orbslam3.txt"
            if status == 0 and raw.is_file() and raw.stat().st_size > 0:
                _body_euroc_to_camera_tum(raw, camera_tum, _read_Tbc(settings_yaml))
                trajectories[spec["name"]] = str(camera_tum)
                print(f"CAMERA {camera_tum}")
    return evaluate_large_scene_stationary(
        sequence_dir, trajectories, out_dir / "stationary",
        v_max, min_duration, max_gap, w_max=w_max, trim_s=trim_s)


def _report(result) -> None:
    print(f"\n==== {result['sequence']}  windows={len(result['gt_windows'])} ====")
    checks_by_idx = {c["index"]: c for c in result.get("frame_checks", [])}
    for window in result["gt_windows"]:
        chk = checks_by_idx.get(window.index)
        trim_head = max(0.0, window.t0 - window.raw_t0)
        trim_tail = max(0.0, window.raw_t1 - window.t1)
        img_info = ""
        if chk:
            img_info = (f" | {chk['first_name']}..{chk['last_name']} "
                        f"MAD={chk['mad']:.2f} ({chk['png']})")
        print(f"  [{window.index}] IMU {window.raw_t0:.2f}-{window.raw_t1:.2f}s "
              f"(trim -{trim_head:.2f}s / -{trim_tail:.2f}s, total -{trim_head + trim_tail:.2f}s) -> "
              f"core {window.t0:.2f}-{window.t1:.2f}s  n={window.n}  "
              f"disp={window.span_disp_m * 100:.2f}cm  rot={window.span_rot_deg:.2f}deg{img_info}")
    trans_header, trans_rows = _table(result["methods"], "trans_vs_gt_m", "m")
    rot_header, rot_rows = _table(result["methods"], "rot_vs_gt_deg", "deg")
    _print_table("Translation Error", trans_header, trans_rows)
    _print_table("Rotation Error", rot_header, rot_rows)
    if result.get("jitter_methods"):
        j_header, j_rows = _jitter_summary_table(result["jitter_methods"])
        _print_table("Static Jitter Summary (Seg.1..K, unified core window, min 1.0s)", j_header, j_rows)
    print(f"wrote {result['html']}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Large-scene stationary pose drift")
    parser.add_argument("--seq", action="append", default=[], help="sequence directory, repeatable")
    parser.add_argument("--traj", action="append", default=[],
                        help='same-clock TUM trajectory, "Name=path"')
    parser.add_argument("--out-dir", type=Path, default=None,
                        help="output directory when a single --seq is given")
    parser.add_argument("--run", action="store_true",
                        help="run ORB-SLAM3, TagFusion (w/o Tag) and TagFusion, then evaluate")
    parser.add_argument("--settings", type=Path, default=None,
                        help="stereo-inertial yaml used by --run")
    parser.add_argument("--orbslam-bin", type=Path, default=None,
                        help="upstream stereo_inertial_euroc; default $ORBSLAM_BIN or ~/code/ORB_SLAM3/...")
    parser.add_argument("--v-max", type=float, default=0.05)
    parser.add_argument("--min-duration", type=float, default=3.0)
    parser.add_argument("--max-gap", type=float, default=1.0)
    parser.add_argument("--w-max", type=float, default=3.0,
                        help="max angular speed (deg/s) inside stationary core (default 3.0)")
    parser.add_argument("--trim-s", type=float, default=1.0,
                        help="seconds trimmed from both ends of each stationary core (default 1.0)")
    args = parser.parse_args()
    sequences = args.seq or [
        "/home/zhhliu/20261013/5fr_pat_led_m/01-1",
        "/home/zhhliu/20261013/5fr_pat_led_m/01-2",
        "/home/zhhliu/20261013/5fr_pat_led_f/01-2",
    ]
    if args.run:
        settings = args.settings or (
            _repo_root() / "Examples/TagFusion/XREAL_Glass/20261013_stereo+imu.yaml")
        for sequence in sequences:
            sequence = Path(sequence)
            out_dir = args.out_dir if len(sequences) == 1 and args.out_dir else None
            orbslam_bin = args.orbslam_bin or os.environ.get("ORBSLAM_BIN") or (
                Path.home() / "code/ORB_SLAM3/Examples/Stereo-Inertial/stereo_inertial_euroc")
            result = run_large_scene_stationary(
                sequence, settings, out_dir, orbslam_bin=orbslam_bin,
                v_max=args.v_max, min_duration=args.min_duration, max_gap=args.max_gap,
                w_max=args.w_max, trim_s=args.trim_s)
            _report(result)
        return 0

    trajectories = {}
    for spec in args.traj:
        if "=" not in spec:
            parser.error(f'--traj expects Name=path, got {spec}')
        name, path = spec.split("=", 1)
        trajectories[name] = path

    root = Path(__file__).resolve().parent / "large_scene_stationary_out"
    for sequence in sequences:
        sequence = Path(sequence)
        out_dir = args.out_dir if len(sequences) == 1 and args.out_dir else (
            root / f"{sequence.parent.name}_{sequence.name}"
        )
        result = evaluate_large_scene_stationary(
            sequence, trajectories, out_dir, args.v_max, args.min_duration, args.max_gap,
            w_max=args.w_max, trim_s=args.trim_s)
        _report(result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

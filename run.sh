#!/usr/bin/env bash
# Batch-run q3hy OptiTrack sequences (stereo + IMU).
# Estimated poses are camera-frame Twc. Before evo, convert with
# Twb = Twc * inv(IMU.T_b_c1), then score APE against imu_pose.txt.
#
# Usage:
#   ./run.sh
#       Evaluate every sequence under 20260807 and 20260819 that has raw_data/glass (including bad_*).
#   ./run.sh /path/to/raw_data/glass [traj_name]
#       Run one sequence. The date in the path (/20260807/ or /20260819/) selects the yaml.
#
# Environment variables (all optional):
#   SETTINGS     Override the date-selected calibration yaml
#   GT           Ground truth for single-sequence mode; default <seq>/ground_truth/data/imu_pose.txt
#   SKIP_RUN=1   Skip tracking and evaluate an existing trajectory
#   SKIP_EVAL=1  Track only, do not evaluate
#   FORCE=1      Re-run even if APE results already exist
#   T_MAX_DIFF   Association threshold in seconds, default 0.02
#   TAG_ENABLE   Tag.enable written into the temporary yaml, default 1
#                (the 20260807 yaml ships with Tag.enable 0; batch eval turns it on)
#   GAMMA_ENABLE Tag.gamma_enable written into the temporary yaml, default 1; 0 skips gamma before AprilTag
#   RESULTS      Output directory, default <repo>/results
#
# Calibration (same as README, by capture date rather than by scene):
#   20260807 -> Examples/TagFusion/XREAL_Glass/20260807_stereo+imu.yaml
#   20260819 -> Examples/TagFusion/XREAL_Glass/20260819_stereo+imu.yaml
#
# This machine has no display. The script copies the yaml above, turns imshow off,
# and writes trajectories, logs, and APE under results/<date>/<scene>/<seq>/.
# summary.tsv records translation RMSE (meters) and rotation RMSE (degrees)
# for both the final trajectory and the online trajectory.
# Rotation uses evo -r angle_deg. Alignment matches translation (-a, SE3 Umeyama).
# Tracking writes f_*.txt / kf_*.txt / FrameTimeStats.csv into the per-sequence
# output directory (WORK_DIR), not into Examples/TagFusion/XREAL_Glass/.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GLASS="${ROOT}/Examples/TagFusion/XREAL_Glass"
RESULTS="${RESULTS:-${ROOT}/results}"
Q3HY_ROOT="${Q3HY_ROOT:-/data/NEW_AI_DATA/data_sip/dataset/q3hy}"
T_MAX_DIFF="${T_MAX_DIFF:-0.02}"
TAG_ENABLE="${TAG_ENABLE:-1}"
GAMMA_ENABLE="${GAMMA_ENABLE:-1}"
export PATH="${HOME}/.local/bin:${PATH}"

settings_for_date() {
  local date="$1"
  case "${date}" in
    20260807) echo "${GLASS}/20260807_stereo+imu.yaml" ;;
    20260819) echo "${GLASS}/20260819_stereo+imu.yaml" ;;
    *)
      echo "ERROR: no settings mapped for date ${date}" >&2
      return 1
      ;;
  esac
}

date_from_path() {
  local path="$1"
  if [[ "${path}" =~ /(20260807|20260819)/ ]]; then
    echo "${BASH_REMATCH[1]}"
    return 0
  fi
  echo "ERROR: cannot infer capture date from ${path}" >&2
  return 1
}

if command -v evo_ape >/dev/null 2>&1; then
  EVO_APE=(evo_ape)
elif python3 -c 'import evo' >/dev/null 2>&1; then
  EVO_APE=(python3 -m evo.main_ape)
else
  echo "ERROR: evo is not installed. pip3 install --user 'numpy==1.23.5' 'evo==1.15.0'"
  exit 1
fi

if [[ ! -x "${GLASS}/stereo_inertial_xreal_glass_tag" ]]; then
  echo "ERROR: binary not found: ${GLASS}/stereo_inertial_xreal_glass_tag"
  echo "Build first: ./build.sh"
  exit 1
fi

# Build a headless yaml. OpenCV FileStorage accepts "key: value".
prepare_settings() {
  local src="$1"
  local dst="$2"
  local export_dir="$3"
  local online_file="$4"
  python3 - "${src}" "${dst}" "${export_dir}" "${online_file}" "${TAG_ENABLE}" "${GAMMA_ENABLE}" <<'PY'
import sys
from pathlib import Path
src, dst, export_dir, online_file, tag_enable, gamma_enable = sys.argv[1:]
text = Path(src).read_text()
keys = {
    "Tag.show_in_orb_viewer": "0",
    "Tag.enable": tag_enable,
    "Tag.gamma_enable": gamma_enable,
    "Tag.verbose": "0",
    "Tag.local_ba.verbose": "0",
    "Tag.export": "1",
    "Tag.export_dir": '"' + export_dir + '"',
    "System.saveOnlinePoseFile": '"' + online_file + '"',
}
seen = set()
out = []
for line in text.splitlines():
    head = line.split("#", 1)[0].strip()
    replaced = False
    for key, value in keys.items():
        if head.startswith(key + ":"):
            out.append(f"{key}: {value}")
            seen.add(key)
            replaced = True
            break
    if not replaced:
        out.append(line)
for key, value in keys.items():
    if key not in seen:
        out.append(f"{key}: {value}")
Path(dst).write_text("\n".join(out) + "\n")
PY
}

strip_tum() {
  local src="$1"
  local dst="$2"
  python3 - "${src}" "${dst}" <<'PY'
import sys
from pathlib import Path
src, dst = Path(sys.argv[1]), Path(sys.argv[2])
lines = []
for line in src.read_text(errors="ignore").splitlines():
    stripped = line.strip()
    if not stripped or stripped.startswith("#"):
        continue
    lines.append(stripped)
dst.write_text("\n".join(lines) + ("\n" if lines else ""))
PY
}

ape_rmse() {
  local report="$1"
  awk '$1 == "rmse" { print $2; found=1 } END { if (!found) print "nan" }' "${report}"
}

# f_*.txt / OnlineCameraTrajectory.txt store the camera pose Twc.
# The IMU pose matches KeyFrame::GetImuPose: Twb = Twc * inv(IMU.T_b_c1).
cam_traj_to_imu() {
  local src="$1"
  local dst="$2"
  local settings="$3"
  python3 - "${src}" "${dst}" "${settings}" <<'PY'
import sys
from pathlib import Path
import numpy as np
import re

src, dst, settings = sys.argv[1:]
text = Path(settings).read_text(errors="ignore")
m = re.search(r"IMU\.T_b_c1:.*?data:\s*\[([^\]]+)\]", text, re.S)
if not m:
    raise SystemExit(f"IMU.T_b_c1 not found in {settings}")
vals = [float(x) for x in m.group(1).replace("\n", " ").split(",") if x.strip()]
if len(vals) != 16:
    raise SystemExit(f"IMU.T_b_c1 has {len(vals)} values in {settings}")
Tbc = np.array(vals, dtype=float).reshape(4, 4)
Tcb = np.linalg.inv(Tbc)

def q_to_R(qx, qy, qz, qw):
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])

def R_to_q(R):
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    return qx, qy, qz, qw

out = []
for line in Path(src).read_text(errors="ignore").splitlines():
    stripped = line.strip()
    if not stripped or stripped.startswith("#"):
        continue
    p = stripped.split()
    if len(p) < 8:
        continue
    ts = float(p[0])
    tx, ty, tz, qx, qy, qz, qw = map(float, p[1:8])
    Twc = np.eye(4)
    Twc[:3, :3] = q_to_R(qx, qy, qz, qw)
    Twc[:3, 3] = (tx, ty, tz)
    Twb = Twc @ Tcb
    qx, qy, qz, qw = R_to_q(Twb[:3, :3])
    t = Twb[:3, 3]
    out.append(f"{ts:.6f} {t[0]:.9f} {t[1]:.9f} {t[2]:.9f} {qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}")
Path(dst).write_text("\n".join(out) + ("\n" if out else ""))
PY
}

# relation: trans_part (meters) or angle_deg (degrees)
eval_one() {
  local est="$1"
  local tag="$2"
  local out_dir="$3"
  local gt_tum="$4"
  local relation="$5"
  local kind="$6"
  local est_tum="${out_dir}/${tag}.tum"
  strip_tum "${est}" "${est_tum}"
  echo "-------"
  echo "APE ${tag} ${kind} (${relation})"
  echo "EST = ${est}"
  "${EVO_APE[@]}" tum "${gt_tum}" "${est_tum}" \
    -a -r "${relation}" --t_max_diff "${T_MAX_DIFF}" --no_warnings \
    --save_results "${out_dir}/${tag}_${kind}_ape.zip" \
    | tee "${out_dir}/${tag}_${kind}_ape.txt"
}

SUMMARY_HEADER=$'date\tscene\tseq\tstatus\tf_trans_rmse_m\tf_rot_rmse_deg\tonline_trans_rmse_m\tonline_rot_rmse_deg'

ensure_summary() {
  mkdir -p "${RESULTS}"
  if [[ "${FORCE:-0}" == "1" || ! -f "${RESULTS}/summary.tsv" ]]; then
    printf '%s\n' "${SUMMARY_HEADER}" >"${RESULTS}/summary.tsv"
    return
  fi
  local first
  first="$(head -n 1 "${RESULTS}/summary.tsv")"
  if [[ "${first}" != "${SUMMARY_HEADER}" ]]; then
    printf '%s\n' "${SUMMARY_HEADER}" >"${RESULTS}/summary.tsv"
  fi
}

# Run one sequence. Returns 0 on success.
# Arguments: date scene seq_name glass_dir gt_file out_dir
run_sequence() {
  local date="$1"
  local scene="$2"
  local seq_name="$3"
  local glass_dir="$4"
  local gt_file="$5"
  local out_dir="$6"
  local traj_name="${date}_${scene}_${seq_name}"
  local settings_src="${SETTINGS:-}"
  if [[ -z "${settings_src}" ]]; then
    settings_src="$(settings_for_date "${date}")"
  fi
  local settings_dst="${out_dir}/settings.yaml"
  local online_file="${out_dir}/OnlineCameraTrajectory.txt"
  local export_dir="${out_dir}/tag_export"
  local summary="${RESULTS}/summary.tsv"

  mkdir -p "${out_dir}" "${export_dir}"
  if [[ "${FORCE:-0}" != "1" \
      && -f "${out_dir}/f_${traj_name}_trans_ape.txt" \
      && -f "${out_dir}/f_${traj_name}_rot_ape.txt" \
      && -f "${out_dir}/online_trans_ape.txt" \
      && -f "${out_dir}/online_rot_ape.txt" ]]; then
    echo "SKIP ${date}/${scene}/${seq_name} (already evaluated)"
    return 0
  fi

  prepare_settings "${settings_src}" "${settings_dst}" "${export_dir}" "${online_file}"

  if [[ -f "${glass_dir}/imu.txt" && ! -e "${glass_dir}/imu0.txt" ]]; then
    ln -sfn imu.txt "${glass_dir}/imu0.txt"
  fi

  echo "======== ${date}/${scene}/${seq_name} ========"
  echo "SETTINGS = ${settings_dst}  (from ${settings_src})"
  echo "SEQ      = ${glass_dir}"
  echo "GT       = ${gt_file}"
  echo "OUT      = ${out_dir}"

  if [[ "${SKIP_RUN:-0}" != "1" ]]; then
    # Write tracking output in this sequence directory so another eval does not share Examples/TagFusion/XREAL_Glass/.
    set +e
    (
      cd "${out_dir}"
      WORK_DIR="${out_dir}" SETTINGS="${settings_dst}" \
        "${GLASS}/run_xreal_glass_stereo_inertial.sh" "${glass_dir}" "${traj_name}"
    ) >"${out_dir}/run.log" 2>&1
    local run_status=$?
    set -e
    if [[ -f "${out_dir}/f_${traj_name}.txt" ]]; then
      :
    elif [[ -f "${GLASS}/f_${traj_name}.txt" ]]; then
      mv -f "${GLASS}/f_${traj_name}.txt" "${out_dir}/"
    fi
    if [[ -f "${out_dir}/kf_${traj_name}.txt" ]]; then
      :
    elif [[ -f "${GLASS}/kf_${traj_name}.txt" ]]; then
      mv -f "${GLASS}/kf_${traj_name}.txt" "${out_dir}/"
    fi
    if [[ -f "${out_dir}/FrameTimeStats.csv" ]]; then
      :
    elif [[ -f "${GLASS}/FrameTimeStats.csv" ]]; then
      mv -f "${GLASS}/FrameTimeStats.csv" "${out_dir}/"
    fi
    if [[ "${run_status}" -ne 0 ]]; then
      echo "FAIL track ${date}/${scene}/${seq_name} (exit ${run_status}), see ${out_dir}/run.log"
      printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' "${date}" "${scene}" "${seq_name}" "track_fail:${run_status}" "-" "-" "-" "-" >>"${summary}"
      return 1
    fi
  fi

  if [[ "${SKIP_EVAL:-0}" == "1" ]]; then
    return 0
  fi

  if [[ ! -f "${gt_file}" ]]; then
    echo "FAIL gt missing: ${gt_file}"
    printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' "${date}" "${scene}" "${seq_name}" "no_gt" "-" "-" "-" "-" >>"${summary}"
    return 1
  fi

  local gt_tum="${out_dir}/gt_imu_pose.tum"
  strip_tum "${gt_file}" "${gt_tum}"

  local found=0
  local f_trans="nan" f_rot="nan"
  local online_trans="nan" online_rot="nan"
  local eval_status=0
  local f_cam="${out_dir}/f_${traj_name}.txt"
  local f_imu="${out_dir}/f_${traj_name}_imu.txt"
  local online_imu="${out_dir}/online_imu.txt"
  if [[ -f "${f_cam}" ]]; then
    cam_traj_to_imu "${f_cam}" "${f_imu}" "${settings_dst}"
    if eval_one "${f_imu}" "f_${traj_name}" "${out_dir}" "${gt_tum}" trans_part trans \
      && eval_one "${f_imu}" "f_${traj_name}" "${out_dir}" "${gt_tum}" angle_deg rot; then
      f_trans="$(ape_rmse "${out_dir}/f_${traj_name}_trans_ape.txt")"
      f_rot="$(ape_rmse "${out_dir}/f_${traj_name}_rot_ape.txt")"
      found=1
    else
      eval_status=1
    fi
  fi
  if [[ -f "${online_file}" ]]; then
    cam_traj_to_imu "${online_file}" "${online_imu}" "${settings_dst}"
    if eval_one "${online_imu}" "online" "${out_dir}" "${gt_tum}" trans_part trans \
      && eval_one "${online_imu}" "online" "${out_dir}" "${gt_tum}" angle_deg rot; then
      online_trans="$(ape_rmse "${out_dir}/online_trans_ape.txt")"
      online_rot="$(ape_rmse "${out_dir}/online_rot_ape.txt")"
      found=1
    else
      eval_status=1
    fi
  fi
  if [[ "${eval_status}" -ne 0 ]]; then
    echo "FAIL evo ${date}/${scene}/${seq_name}"
    printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' "${date}" "${scene}" "${seq_name}" "evo_fail" "${f_trans}" "${f_rot}" "${online_trans}" "${online_rot}" >>"${summary}"
    return 1
  fi
  if [[ "${found}" -eq 0 ]]; then
    echo "FAIL no trajectory for ${date}/${scene}/${seq_name}"
    printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' "${date}" "${scene}" "${seq_name}" "no_traj" "-" "-" "-" "-" >>"${summary}"
    return 1
  fi
  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' "${date}" "${scene}" "${seq_name}" "ok" "${f_trans}" "${f_rot}" "${online_trans}" "${online_rot}" >>"${summary}"
  echo "APE final  trans=${f_trans} m  rot=${f_rot} deg"
  echo "APE online trans=${online_trans} m  rot=${online_rot} deg"
  return 0
}

run_single() {
  local seq_arg="$1"
  local traj_name="${2:-single}"
  local glass_dir="${seq_arg}"
  if [[ -d "${seq_arg}/raw_data/glass" ]]; then
    glass_dir="${seq_arg}/raw_data/glass"
  fi
  local seq_root
  seq_root="$(cd "${glass_dir}/.." && pwd)"
  if [[ "$(basename "${seq_root}")" == "raw_data" ]]; then
    seq_root="$(cd "${seq_root}/.." && pwd)"
  fi
  local date scene seq_name gt_file
  date="$(date_from_path "${seq_root}")"
  seq_name="$(basename "${seq_root}")"
  scene="$(basename "$(dirname "${seq_root}")")"
  gt_file="${GT:-${seq_root}/ground_truth/data/imu_pose.txt}"
  traj_name="${2:-${date}_${scene}_${seq_name}}"
  ensure_summary
  run_sequence "${date}" "${scene}" "${seq_name}" "${glass_dir}" "${gt_file}" \
    "${RESULTS}/${date}/${scene}/${seq_name}"
}

run_batch() {
  local dates=(20260807 20260819)
  ensure_summary
  local date scene_dir seq_dir scene seq_name glass_dir gt_file
  local n_ok=0 n_fail=0 n_skip=0
  for date in "${dates[@]}"; do
    if [[ ! -d "${Q3HY_ROOT}/${date}" ]]; then
      echo "ERROR: missing ${Q3HY_ROOT}/${date}"
      exit 1
    fi
    for scene_dir in "${Q3HY_ROOT}/${date}"/*; do
      [[ -d "${scene_dir}" ]] || continue
      scene="$(basename "${scene_dir}")"
      for seq_dir in "${scene_dir}"/*; do
        [[ -d "${seq_dir}/raw_data/glass" ]] || continue
        seq_name="$(basename "${seq_dir}")"
        glass_dir="${seq_dir}/raw_data/glass"
        gt_file="${seq_dir}/ground_truth/data/imu_pose.txt"
        local out_dir="${RESULTS}/${date}/${scene}/${seq_name}"
        if [[ "${FORCE:-0}" != "1" \
            && -f "${out_dir}/f_${date}_${scene}_${seq_name}_trans_ape.txt" \
            && -f "${out_dir}/f_${date}_${scene}_${seq_name}_rot_ape.txt" \
            && -f "${out_dir}/online_trans_ape.txt" \
            && -f "${out_dir}/online_rot_ape.txt" ]]; then
          echo "SKIP ${date}/${scene}/${seq_name}"
          n_skip=$((n_skip + 1))
          continue
        fi
        if run_sequence "${date}" "${scene}" "${seq_name}" "${glass_dir}" "${gt_file}" "${out_dir}"; then
          n_ok=$((n_ok + 1))
        else
          n_fail=$((n_fail + 1))
        fi
      done
    done
  done
  echo "-------"
  echo "done ok=${n_ok} fail=${n_fail} skip=${n_skip}"
  echo "summary: ${RESULTS}/summary.tsv"
}

if [[ $# -ge 1 ]]; then
  run_single "$@"
else
  run_batch
fi

#!/usr/bin/env bash
# 批量跑 q3hy OptiTrack 序列（双目+IMU）。
# 估计轨迹是相机系 Twc。评测前固定做 Twb = Twc * inv(IMU.T_b_c1)，再对 imu_pose.txt 做 evo APE。
#
# 用法:
#   ./run.sh
#       评测 20260807 与 20260819 下所有含 raw_data/glass 的序列（含 bad_*）。
#   ./run.sh /path/to/raw_data/glass [traj_name]
#       只跑一条。日期从路径里的 /20260807/ 或 /20260819/ 判断，用来选标定 yaml。
#
# 环境变量（均可选）:
#   SETTINGS     覆盖自动选择的标定 yaml
#   GT           单序列模式下的真值，默认 <seq>/ground_truth/data/imu_pose.txt
#   SKIP_RUN=1   跳过跟踪，只评测已有轨迹
#   SKIP_EVAL=1  只跟踪，不评测
#   FORCE=1      已有 APE 结果也重跑
#   T_MAX_DIFF   时间关联阈值（秒），默认 0.02
#   TAG_ENABLE   写入临时 yaml 的 Tag.enable，默认 1
#                （20260807 的 01-1_stereo+imu.yaml 原文件是 0，批量评测会打开）
#   GAMMA_ENABLE 写入临时 yaml 的 Tag.gamma_enable，默认 1；0=AprilTag 前不做 gamma
#   RESULTS      结果目录，默认 <repo>/results
#
# 标定（与 README 一致，按采集日期而不是按场景）:
#   20260807 -> Examples/TagFusion/XREAL_Glass/01-1_stereo+imu.yaml
#   20260819 -> Examples/TagFusion/XREAL_Glass/0819_ror_pat_led_f_stereo+imu.yaml
#
# 本机没有显示器。脚本会基于上述 yaml 生成临时配置，关掉 imshow，
# 并把轨迹、日志、APE 写到 results/<date>/<scene>/<seq>/。
# summary.tsv 同时记录最终轨迹和在线轨迹的平移 RMSE（米）与旋转 RMSE（度）。
# 旋转用 evo -r angle_deg，对齐与平移相同（-a，SE3 Umeyama）。
# 跟踪过程中 f_*.txt / kf_*.txt / FrameTimeStats.csv 先落在
# Examples/TagFusion/XREAL_Glass/，结束后搬进 results。

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
    20260807) echo "${GLASS}/01-1_stereo+imu.yaml" ;;
    20260819) echo "${GLASS}/0819_ror_pat_led_f_stereo+imu.yaml" ;;
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

# 生成无界面 yaml。OpenCV FileStorage 认 "key: value"。
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

# f_*.txt / OnlineCameraTrajectory.txt 是相机位姿 Twc。
# IMU 位姿与 KeyFrame::GetImuPose 一致：Twb = Twc * inv(IMU.T_b_c1)。
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

# relation: trans_part（米）或 angle_deg（度）
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

# 跑一条序列。成功返回 0。
# 参数: date scene seq_name glass_dir gt_file out_dir
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
    # 跟踪输出写在本序列结果目录，避免和另一路评测抢 Examples/TagFusion/XREAL_Glass/。
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

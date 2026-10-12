#!/usr/bin/env bash
# 跑 0309 01-1 双目+IMU，并用 evo 对 cam0 真值做 APE。
#
# 用法:
#   ./run.sh
#   ./run.sh /path/to/raw_data/glass [traj_name]
#
# 环境变量（均可选）:
#   SETTINGS   默认 Examples/TagFusion/XREAL_Glass/0309_01-1_stereo+imu.yaml
#   GT         默认 data/01-1/ground_truth/data/cam0_pose.txt
#   SKIP_RUN=1 跳过跟踪，只评测已有轨迹
#   SKIP_EVAL=1 只跟踪，不评测
#   T_MAX_DIFF 时间关联阈值（秒），默认 0.02
#
# 跟踪输出写在 Examples/TagFusion/XREAL_Glass/（f_<name>.txt、OnlineCameraTrajectory.txt）。
# 评测结果写在 evaluation/0309_01-1/。
# 当前 yaml 里 Tag.show_in_orb_viewer=1，需要显示器。

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GLASS="${ROOT}/Examples/TagFusion/XREAL_Glass"
SETTINGS="${SETTINGS:-${GLASS}/0309_01-1_stereo+imu.yaml}"
SEQ="${1:-${SEQ:-${ROOT}/data/01-1/raw_data/glass}}"
TRAJ_NAME="${2:-0309_01-1}"
GT="${GT:-${ROOT}/data/01-1/ground_truth/data/cam0_pose.txt}"
T_MAX_DIFF="${T_MAX_DIFF:-0.02}"
OUT_DIR="${ROOT}/evaluation/0309_01-1"

if [[ ! -x "${GLASS}/stereo_inertial_xreal_glass_tag" ]]; then
  echo "ERROR: binary not found: ${GLASS}/stereo_inertial_xreal_glass_tag"
  echo "Build first: ./build.sh"
  exit 1
fi

if [[ ! -f "${SETTINGS}" ]]; then
  echo "ERROR: settings not found: ${SETTINGS}"
  exit 1
fi

if [[ "${SKIP_RUN:-0}" != "1" ]]; then
  if [[ -f "${SEQ}/imu.txt" && ! -e "${SEQ}/imu0.txt" ]]; then
    ln -sfn imu.txt "${SEQ}/imu0.txt"
  fi
  echo "SETTINGS = ${SETTINGS}"
  echo "SEQ      = ${SEQ}"
  echo "TRAJ     = ${TRAJ_NAME}"
  echo "-------"
  (
    cd "${GLASS}"
    SETTINGS="${SETTINGS}" ./run_xreal_glass_stereo_inertial.sh "${SEQ}" "${TRAJ_NAME}"
  )
fi

if [[ "${SKIP_EVAL:-0}" == "1" ]]; then
  exit 0
fi

if [[ ! -f "${GT}" ]]; then
  echo "ERROR: ground truth not found: ${GT}"
  exit 1
fi

if command -v evo_ape >/dev/null 2>&1; then
  EVO_APE=(evo_ape)
elif python3 -c 'import evo' >/dev/null 2>&1; then
  EVO_APE=(python3 -m evo.main_ape)
else
  echo "ERROR: evo is not installed. pip3 install --user evo"
  exit 1
fi

mkdir -p "${OUT_DIR}"
GT_TUM="${OUT_DIR}/gt_cam0_pose.tum"
python3 - "${GT}" "${GT_TUM}" <<'PY'
import sys
from pathlib import Path
src, dst = Path(sys.argv[1]), Path(sys.argv[2])
lines = []
for line in src.read_text().splitlines():
    stripped = line.strip()
    if not stripped or stripped.startswith("#"):
        continue
    lines.append(stripped)
dst.write_text("\n".join(lines) + ("\n" if lines else ""))
PY

eval_one() {
  local est="$1"
  local tag="$2"
  local est_tum="${OUT_DIR}/${tag}.tum"
  python3 - "${est}" "${est_tum}" <<'PY'
import sys
from pathlib import Path
src, dst = Path(sys.argv[1]), Path(sys.argv[2])
lines = []
for line in src.read_text().splitlines():
    stripped = line.strip()
    if not stripped or stripped.startswith("#"):
        continue
    lines.append(stripped)
dst.write_text("\n".join(lines) + ("\n" if lines else ""))
PY
  echo "-------"
  echo "APE ${tag}"
  echo "EST = ${est}"
  echo "GT  = ${GT}"
  "${EVO_APE[@]}" tum "${GT_TUM}" "${est_tum}" \
    -a --t_max_diff "${T_MAX_DIFF}" \
    --save_results "${OUT_DIR}/${tag}_ape.zip" \
    | tee "${OUT_DIR}/${tag}_ape.txt"
}

found=0
if [[ -f "${GLASS}/f_${TRAJ_NAME}.txt" ]]; then
  eval_one "${GLASS}/f_${TRAJ_NAME}.txt" "f_${TRAJ_NAME}"
  found=1
fi
if [[ -f "${GLASS}/OnlineCameraTrajectory.txt" ]]; then
  eval_one "${GLASS}/OnlineCameraTrajectory.txt" "online"
  found=1
fi

if [[ "${found}" -eq 0 ]]; then
  echo "ERROR: no trajectory under ${GLASS} (f_${TRAJ_NAME}.txt or OnlineCameraTrajectory.txt)"
  exit 1
fi

echo "-------"
echo "Results: ${OUT_DIR}"

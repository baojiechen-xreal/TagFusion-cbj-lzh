#!/usr/bin/env bash
# Launch XREAL Glass cam0+cam1 TagFusion (stereo)
#
# Dependencies (local prefix on this machine):
#   source /home/jxpang/project/deps/env.sh
#
# Usage:
#   ./run_xreal_glass_stereo.sh
#   ./run_xreal_glass_stereo.sh /path/to/glass [traj_name]
#   ./run_xreal_glass_stereo.sh /path/to/seq_root [traj_name]   # sequence root containing raw_data/glass also works
#   ./run_xreal_glass_stereo.sh /path/to/glass/cam0/images [traj_name]
#   SEQ=/path/to/.../raw_data/glass ./run_xreal_glass_stereo.sh
#
# Switch config (default XREAL_Glass_stereo.yaml):
#   SETTINGS=./q3hy_20260807_der_pat_led_f_stereo.yaml \
#     ./run_xreal_glass_stereo.sh \
#     /data/NEW_AI_DATA/data_sip/dataset/q3hy/20260807/der_pat_led_f/01-1 \
#     der_pat_01-1
#
# Environment variables (all optional):
#   VOC / SETTINGS / BIN / SEQ
#
# Before running, confirm Camera1/2, Stereo.T_c1_c2, and resolution in the yaml match the sequence
# (q3hy is 640x512@60; the old room-patrol sample is 480x640@30; do not mix them)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ORB_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

VOC="${VOC:-${ORB_ROOT}/Vocabulary/ORBvoc.txt}"
SETTINGS="${SETTINGS:-${SCRIPT_DIR}/XREAL_Glass_stereo.yaml}"
BIN="${BIN:-${SCRIPT_DIR}/stereo_xreal_glass_tag}"

DEFAULT_SEQ="/home/geneta/dataset/20260227-room-patrol-11-1/raw_data/glass"
SEQ_PATH="${1:-${SEQ:-${DEFAULT_SEQ}}}"
TRAJ_NAME="${2:-glass_stereo}"

if [[ ! -x "${BIN}" ]]; then
  echo "ERROR: binary not found: ${BIN}"
  echo "Build first, e.g.:"
  echo "  cmake --build ${ORB_ROOT}/build --target stereo_xreal_glass_tag -j"
  exit 1
fi

if [[ ! -f "${VOC}" ]]; then
  echo "ERROR: vocabulary not found: ${VOC}"
  exit 1
fi

if [[ ! -f "${SETTINGS}" ]]; then
  echo "ERROR: settings not found: ${SETTINGS}"
  exit 1
fi

if [[ ! -d "${SEQ_PATH}" ]]; then
  echo "ERROR: sequence path not found: ${SEQ_PATH}"
  exit 1
fi

if grep -qE 'Camera1\.fx:\s*0(\.0)?\s*$' "${SETTINGS}"; then
  echo "WARNING: Camera1.fx in ${SETTINGS} is still 0. Fill fisheye intrinsics and distortion before a real run."
fi

echo "VOC      = ${VOC}"
echo "SETTINGS = ${SETTINGS}"
echo "SEQ      = ${SEQ_PATH}"
echo "TRAJ     = ${TRAJ_NAME}"
echo "BIN      = ${BIN}"
echo "-------"

cd "${SCRIPT_DIR}"
exec "${BIN}" "${VOC}" "${SETTINGS}" "${SEQ_PATH}" "${TRAJ_NAME}"

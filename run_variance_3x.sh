#!/usr/bin/env bash
# Three independent TagFusion runs per sequence, for run-to-run variance.
# 20260807: the 13 sequences in results/summary.tsv.
# 20260819: every sequence whose glass images, timestamps, and imu_pose are complete.
# Each round writes to results/results_var/runN/ and is skipped if that round already has APE.
# Reported ATE stays SE3 (run.sh does not pass evo -s).

set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${ROOT}"

export PATH="${HOME}/.local/bin:${PATH}"
export LD_LIBRARY_PATH="/home/bjchen/Pangolin/build:/home/bjchen/local/deps/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export Q3HY_ROOT=/data/q3hy
export TAG_ENABLE=1
export GAMMA_ENABLE=0

SEQS=(
  /data/q3hy/20260807/der_ofa_led_f/01-2
  /data/q3hy/20260807/der_ofa_led_f/01-3
  /data/q3hy/20260807/ror_pat_led_m/01-1
  /data/q3hy/20260807/ror_pat_led_m/01-2
  /data/q3hy/20260807/ror_pat_led_m/01-3
  /data/q3hy/20260807/ror_pat_led_m/bad_01-3
  /data/q3hy/20260807/ror_tbk_led_f/01-1
  /data/q3hy/20260807/ror_tbk_led_f/01-2
  /data/q3hy/20260807/ror_tbk_led_f/01-3
  /data/q3hy/20260807/ror_tbk_led_m/01-1
  /data/q3hy/20260807/ror_tbk_led_m/01-2
  /data/q3hy/20260807/ror_tbk_led_m/01-3
  /data/q3hy/20260807/ror_tbk_led_m/bad_01-3
  /data/q3hy/20260819/der_ofa_led_f/01-1
  /data/q3hy/20260819/der_ofa_led_f/01-2
  /data/q3hy/20260819/der_ofa_led_f/01-3
  /data/q3hy/20260819/der_ofa_led_m/01-1
  /data/q3hy/20260819/der_ofa_led_m/01-2
  /data/q3hy/20260819/der_ofa_led_m/01-3
  /data/q3hy/20260819/ror_pat_led_f/01-1
  /data/q3hy/20260819/ror_pat_led_f/01-2
  /data/q3hy/20260819/ror_pat_led_f/01-3
  /data/q3hy/20260819/ror_pat_led_m/01-1
  /data/q3hy/20260819/ror_pat_led_m/01-2
  /data/q3hy/20260819/ror_pat_led_m/01-3
)

mkdir -p "${ROOT}/results/results_var"
echo "variance batch start $(date -Is) nseq=${#SEQS[@]} rounds=3"

for seq in "${SEQS[@]}"; do
  for round in 1 2 3; do
    echo "======== round ${round} ${seq} $(date -Is) ========"
    RESULTS="${ROOT}/results/results_var/run${round}" "${ROOT}/run.sh" "${seq}"
    status=$?
    if [[ "${status}" -ne 0 ]]; then
      echo "FAIL round ${round} ${seq} exit ${status}"
    fi
  done
done

python3 - << 'PY'
import csv
from pathlib import Path
from statistics import mean, pstdev
root = Path("/home/bjchen/TagFusion/results/results_var")
rows = {}
for round in (1, 2, 3):
    path = root / f"run{round}" / "summary.tsv"
    if not path.is_file():
        continue
    for rec in csv.DictReader(path.open(), delimiter="\t"):
        key = (rec["date"], rec["scene"], rec["seq"])
        rows.setdefault(key, {})[round] = rec
out = root / "variance.tsv"
with out.open("w") as f:
    f.write("date\tscene\tseq\tn\ttrans_mean\ttrans_std\ttrans_min\ttrans_max\trot_mean\trot_std\ttrans_1\ttrans_2\ttrans_3\n")
    for key in sorted(rows):
        recs = rows[key]
        trans, rot = [], []
        cells = []
        for round in (1, 2, 3):
            rec = recs.get(round)
            if rec and rec.get("status") == "ok":
                trans.append(float(rec["f_trans_rmse_m"]))
                rot.append(float(rec["f_rot_rmse_deg"]))
                cells.append(f"{trans[-1]:.6f}")
            else:
                cells.append("-")
        if not trans:
            continue
        tstd = pstdev(trans) if len(trans) > 1 else 0.0
        rstd = pstdev(rot) if len(rot) > 1 else 0.0
        f.write("{}\t{}\t{}\t{}\t{:.6f}\t{:.6f}\t{:.6f}\t{:.6f}\t{:.6f}\t{:.6f}\t{}\t{}\t{}\n".format(
            *key, len(trans), mean(trans), tstd, min(trans), max(trans), mean(rot), rstd, *cells))
print(f"wrote {out}")
PY

echo "variance batch done $(date -Is)"

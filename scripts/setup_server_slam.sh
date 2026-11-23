#!/bin/bash
# Build Pangolin, upstream ORB-SLAM3 and TagFusion into $HOME without sudo.
set -euo pipefail

OPT="${HOME}/opt"
SYS="${OPT}/sysroot"
JOBS="${JOBS:-16}"
LOG="${OPT}/setup_slam.log"
exec > >(tee -a "${LOG}") 2>&1

echo "===== $(date) setup start ====="
mkdir -p "${OPT}/debs" "${SYS}" "${OPT}/src"

export CC="${CC:-gcc}"
export CXX="${CXX:-g++}"
export CPLUS_INCLUDE_PATH="${SYS}/usr/include:${SYS}/usr/include/x86_64-linux-gnu${CPLUS_INCLUDE_PATH:+:${CPLUS_INCLUDE_PATH}}"
export CPATH="${SYS}/usr/include:${SYS}/usr/include/x86_64-linux-gnu${CPATH:+:${CPATH}}"
export LIBRARY_PATH="${SYS}/usr/lib/x86_64-linux-gnu${LIBRARY_PATH:+:${LIBRARY_PATH}}"
export LD_LIBRARY_PATH="${SYS}/usr/lib/x86_64-linux-gnu:${OPT}/pangolin/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export PKG_CONFIG_PATH="${SYS}/usr/lib/x86_64-linux-gnu/pkgconfig:${SYS}/usr/lib/pkgconfig${PKG_CONFIG_PATH:+:${PKG_CONFIG_PATH}}"

if [[ ! -x "${OPT}/cmake/bin/cmake" ]]; then
  echo "===== cmake ====="
  curl -L --retry 3 -o "${OPT}/cmake.tgz" \
    https://github.com/Kitware/CMake/releases/download/v3.22.6/cmake-3.22.6-linux-x86_64.tar.gz
  tar -C "${OPT}" -xzf "${OPT}/cmake.tgz"
  rm -rf "${OPT}/cmake"
  mv "${OPT}/cmake-3.22.6-linux-x86_64" "${OPT}/cmake"
fi
export PATH="${OPT}/cmake/bin:${PATH}"
cmake --version | head -1

if [[ ! -e "${SYS}/usr/include/openssl/md5.h" || ! -e "${SYS}/usr/include/boost/serialization/serialization.hpp" || ! -e "${SYS}/usr/include/GL/glew.h" ]]; then
  echo "===== apt packages into ${SYS} ====="
  python3 - <<'PY'
import os, subprocess
from pathlib import Path
debs = Path.home() / "opt" / "debs"
debs.mkdir(parents=True, exist_ok=True)
seeds = [
    "libssl-dev",
    "libboost-dev",
    "libboost-serialization-dev",
    "libglew-dev",
    "libgl1-mesa-dev",
    "libglu1-mesa-dev",
    "libegl1-mesa-dev",
    "libglvnd-dev",
    "libx11-dev",
    "libxext-dev",
    "libxi-dev",
    "libxrandr-dev",
    "libxxf86vm-dev",
]
seen = set()
queue = list(seeds)
closure = []
while queue:
    name = queue.pop()
    if name in seen or name.startswith("<"):
        continue
    seen.add(name)
    show = subprocess.run(["apt-cache", "show", name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if show.returncode != 0:
        continue
    closure.append(name)
    dep = subprocess.run(
        ["apt-cache", "depends", "--no-recommends", "--no-suggests",
         "--no-conflicts", "--no-breaks", "--no-replaces", "--no-enhances", name],
        check=True, stdout=subprocess.PIPE, text=True)
    for line in dep.stdout.splitlines():
        line = line.strip()
        if not line.startswith("Depends:"):
            continue
        item = line.split(":", 1)[1].strip().split("|")[0].strip().split()[0]
        if item and not item.startswith("<"):
            queue.append(item)

def installed(pkg):
    query = subprocess.run(
        ["dpkg-query", "-W", "-f", "${Status}", pkg],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    return query.returncode == 0 and query.stdout.strip() == "install ok installed"

missing = [pkg for pkg in closure if not installed(pkg)]
print(f"download {len(missing)} packages")
os.chdir(debs)
for pkg in missing:
    if list(debs.glob(pkg + "_*.deb")):
        continue
    print("get", pkg)
    subprocess.run(["apt-get", "download", pkg], check=False)
PY
  shopt -s nullglob
  for deb in "${OPT}/debs/"*.deb; do
    dpkg-deb -x "${deb}" "${SYS}"
  done
  python3 - <<'PY'
import os
from pathlib import Path
sysroot = Path.home() / "opt" / "sysroot"
libdirs = [Path("/usr/lib/x86_64-linux-gnu"), Path("/lib/x86_64-linux-gnu")]
for link in sysroot.rglob("*"):
    if not link.is_symlink():
        continue
    if link.exists():
        continue
    target = os.readlink(link)
    base = os.path.basename(target)
    for libdir in libdirs:
        candidate = libdir / base
        if candidate.exists():
            link.unlink()
            link.symlink_to(candidate)
            break
PY
fi

test -f "${SYS}/usr/include/openssl/md5.h"
test -f "${SYS}/usr/include/boost/serialization/serialization.hpp"
test -f "${SYS}/usr/include/GL/glew.h"
ls "${SYS}/usr/lib/x86_64-linux-gnu"/libboost_serialization.so* >/dev/null
ls "${SYS}/usr/lib/x86_64-linux-gnu"/libGLEW.so* >/dev/null
ls "${SYS}/usr/lib/x86_64-linux-gnu"/libcrypto.so >/dev/null

if [[ ! -f "${OPT}/pangolin/lib/cmake/Pangolin/PangolinConfig.cmake" ]]; then
  echo "===== pangolin ====="
  if [[ ! -d "${OPT}/src/Pangolin/.git" ]]; then
    git clone --depth 1 --branch v0.8 https://github.com/stevenlovegrove/Pangolin.git "${OPT}/src/Pangolin"
  fi
  rm -rf "${OPT}/src/Pangolin/build"
  cmake -S "${OPT}/src/Pangolin" -B "${OPT}/src/Pangolin/build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${OPT}/pangolin" \
    -DCMAKE_PREFIX_PATH="${SYS}/usr" \
    -DEigen3_DIR=/usr/lib/cmake/eigen3 \
    -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DBUILD_TESTS=OFF -DBUILD_PANGOLIN_PYTHON=OFF
  cmake --build "${OPT}/src/Pangolin/build" -j"${JOBS}"
  cmake --install "${OPT}/src/Pangolin/build"
fi
test -f "${OPT}/pangolin/lib/cmake/Pangolin/PangolinConfig.cmake"

cat > "${HOME}/slam-env.sh" <<EOF
export PATH="\${HOME}/opt/cmake/bin:\${PATH}"
export LD_LIBRARY_PATH="\${HOME}/opt/sysroot/usr/lib/x86_64-linux-gnu:\${HOME}/opt/pangolin/lib:\${HOME}/code/TagFusion/lib:\${HOME}/code/TagFusion/Thirdparty/DBoW2/lib:\${HOME}/code/TagFusion/Thirdparty/g2o/lib:\${HOME}/code/ORB_SLAM3/lib:\${HOME}/code/ORB_SLAM3/Thirdparty/DBoW2/lib:\${HOME}/code/ORB_SLAM3/Thirdparty/g2o/lib\${LD_LIBRARY_PATH:+:\${LD_LIBRARY_PATH}}"
export Pangolin_DIR="\${HOME}/opt/pangolin/lib/cmake/Pangolin"
export CPLUS_INCLUDE_PATH="\${HOME}/opt/sysroot/usr/include:\${HOME}/opt/sysroot/usr/include/x86_64-linux-gnu\${CPLUS_INCLUDE_PATH:+:\${CPLUS_INCLUDE_PATH}}"
export LIBRARY_PATH="\${HOME}/opt/sysroot/usr/lib/x86_64-linux-gnu\${LIBRARY_PATH:+:\${LIBRARY_PATH}}"
EOF

patch_orb() {
  python3 - <<'PY'
from pathlib import Path
settings = Path("src/Settings.cc")
text = settings.read_text()
marker = 'fSettings,"Camera2.fx"'
pos = text.find(marker)
if pos < 0:
    raise SystemExit("Camera2.fx not found")
head, tail = text[:pos], text[pos:]
old = '''            float k0 = readParameter<float>(fSettings,"Camera1.k1",found);
            float k1 = readParameter<float>(fSettings,"Camera1.k2",found);
            float k2 = readParameter<float>(fSettings,"Camera1.k3",found);
            float k3 = readParameter<float>(fSettings,"Camera1.k4",found);'''
new = old.replace("Camera1.", "Camera2.")
if old in tail:
    tail = tail.replace(old, new, 1)
    settings.write_text(head + tail)
    print("patched Camera2 distortion")
elif new in tail:
    print("Camera2 distortion already patched")
else:
    raise SystemExit("Camera2 distortion block not found")

opt = Path("src/Optimizer.cc")
ot = opt.read_text()
src = "Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(), VP->estimate().tcw[0].cast<float>());"
dst = "Sophus::SE3f Tcw(NormalizeRotation(VP->estimate().Rcw[0]).cast<float>(), VP->estimate().tcw[0].cast<float>());"
count = ot.count(src)
if count:
    opt.write_text(ot.replace(src, dst))
    print(f"patched Optimizer.cc x{count}")
elif dst in ot:
    print("Optimizer.cc already patched")
else:
    raise SystemExit("Optimizer write-back not found")

cmake = Path("CMakeLists.txt")
ct = cmake.read_text()
ct2 = ct.replace("find_package(OpenCV 4.4)", "find_package(OpenCV 4.2)")
ct2 = ct2.replace("OpenCV > 4.4 not found.", "OpenCV >= 4.2 not found.")
ct2 = ct2.replace("-std=c++11", "-std=c++14")
if ct2 != ct:
    cmake.write_text(ct2)
    print("patched CMakeLists OpenCV/C++14")
else:
    print("CMakeLists already patched")
PY
}

build_thirdparty() {
  local std="$1"
  local root="$2"
  cmake -S "${root}/Thirdparty/DBoW2" -B "${root}/Thirdparty/DBoW2/build" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-std=${std} -O3 -march=native"
  cmake --build "${root}/Thirdparty/DBoW2/build" -j"${JOBS}"
  cmake -S "${root}/Thirdparty/g2o" -B "${root}/Thirdparty/g2o/build" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-std=${std} -O3 -march=native"
  cmake --build "${root}/Thirdparty/g2o/build" -j"${JOBS}"
  cmake -S "${root}/Thirdparty/Sophus" -B "${root}/Thirdparty/Sophus/build" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-std=${std} -O3 -march=native" \
    -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF -DEigen3_DIR=/usr/lib/cmake/eigen3
  cmake --build "${root}/Thirdparty/Sophus/build" -j"${JOBS}"
}

if [[ ! -d "${HOME}/code/ORB_SLAM3/.git" ]]; then
  echo "===== clone ORB_SLAM3 ====="
  git clone --depth 1 https://github.com/UZ-SLAMLab/ORB_SLAM3.git "${HOME}/code/ORB_SLAM3"
fi
echo "===== patch and build ORB_SLAM3 ====="
cd "${HOME}/code/ORB_SLAM3"
patch_orb
if [[ ! -f Vocabulary/ORBvoc.txt ]]; then
  tar -C Vocabulary -xf Vocabulary/ORBvoc.txt.tar.gz
fi
build_thirdparty c++14 "${HOME}/code/ORB_SLAM3"
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DPangolin_DIR="${OPT}/pangolin/lib/cmake/Pangolin" \
  -DEigen3_DIR=/usr/lib/cmake/eigen3 \
  -DCMAKE_PREFIX_PATH="${SYS}/usr"
cmake --build build -j"${JOBS}" --target stereo_inertial_euroc
test -x Examples/Stereo-Inertial/stereo_inertial_euroc

echo "===== build TagFusion ====="
cd "${HOME}/code/TagFusion"
if ! grep -q "NormalizeRotation(VP->estimate().Rcw[0])" src/Optimizer.cc; then
  echo "Optimizer.cc on the server is missing the Sophus fix" >&2
  exit 1
fi
if [[ ! -f Vocabulary/ORBvoc.txt ]]; then
  tar -C Vocabulary -xf Vocabulary/ORBvoc.txt.tar.gz
fi
build_thirdparty c++17 "${HOME}/code/TagFusion"
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DPangolin_DIR="${OPT}/pangolin/lib/cmake/Pangolin" \
  -DEigen3_DIR=/usr/lib/cmake/eigen3 \
  -DCMAKE_PREFIX_PATH="${SYS}/usr"
cmake --build build -j"${JOBS}" --target stereo_inertial_xreal_glass_tag
test -x Examples/TagFusion/XREAL_Glass/stereo_inertial_xreal_glass_tag

echo "===== link check ====="
source "${HOME}/slam-env.sh"
ldd "${HOME}/code/ORB_SLAM3/Examples/Stereo-Inertial/stereo_inertial_euroc" | grep -E "not found|libORB|libpangolin|libboost|libcrypto|libg2o" || true
if ldd "${HOME}/code/ORB_SLAM3/Examples/Stereo-Inertial/stereo_inertial_euroc" | grep -q "not found"; then
  echo "ORB binary has missing libraries" >&2
  ldd "${HOME}/code/ORB_SLAM3/Examples/Stereo-Inertial/stereo_inertial_euroc" | grep "not found"
  exit 1
fi
if ldd "${HOME}/code/TagFusion/Examples/TagFusion/XREAL_Glass/stereo_inertial_xreal_glass_tag" | grep -q "not found"; then
  echo "TagFusion binary has missing libraries" >&2
  ldd "${HOME}/code/TagFusion/Examples/TagFusion/XREAL_Glass/stereo_inertial_xreal_glass_tag" | grep "not found"
  exit 1
fi

echo "===== smoke ====="
SMOKE="${OPT}/smoke"
rm -rf "${SMOKE}"
mkdir -p "${SMOKE}"
python3 - <<'PY'
import sys
from pathlib import Path
sys.path.insert(0, str(Path.home() / "code/TagFusion/scripts"))
from eval_large_scene_stationary import _prepare_euroc
glass = Path("/data/q3hy/20261013/5fr_pat_led_m/01-1/raw_data/glass")
dst = Path.home() / "opt/smoke/euroc"
times = _prepare_euroc(glass, dst)
lines = times.read_text().splitlines()
times.write_text("\n".join(lines[:400]) + "\n")
print(f"smoke times {min(400, len(lines))}")
PY
cd "${SMOKE}"
set +e
timeout 240 "${HOME}/code/ORB_SLAM3/Examples/Stereo-Inertial/stereo_inertial_euroc" \
  "${HOME}/code/ORB_SLAM3/Vocabulary/ORBvoc.txt" \
  "${HOME}/code/TagFusion/Examples/TagFusion/XREAL_Glass/20261013_stereo+imu.yaml" \
  "${SMOKE}/euroc" "${SMOKE}/euroc/times.txt" smoke_orb \
  > "${SMOKE}/orb.log" 2>&1
orb_status=$?
timeout 240 "${HOME}/code/TagFusion/Examples/TagFusion/XREAL_Glass/stereo_inertial_xreal_glass_tag" \
  "${HOME}/code/TagFusion/Vocabulary/ORBvoc.txt" \
  "${HOME}/code/TagFusion/Examples/TagFusion/XREAL_Glass/20261013_stereo+imu.yaml" \
  /data/q3hy/20261013/5fr_pat_led_m/01-1 smoke_tag \
  > "${SMOKE}/tag.log" 2>&1
tag_status=$?
set -e
echo "ORB smoke exit ${orb_status}"
echo "TagFusion smoke exit ${tag_status}"
if grep -E "double free|error while loading shared libraries|Segmentation fault|Sophus ensure failed" "${SMOKE}/orb.log" "${SMOKE}/tag.log"; then
  echo "smoke hit a fatal error" >&2
  exit 1
fi
grep -E "Camera 2 parameters|New Map created|Fail to track|Vocabulary loaded|Tag.enable" "${SMOKE}/orb.log" "${SMOKE}/tag.log" | head -40
echo "===== $(date) setup done ====="

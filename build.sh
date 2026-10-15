# /usr/bin/c++ on this machine is Clang, which cannot see GCC's stdint-gcc.h
# (included by Thirdparty/DBoW2). Force the system GCC toolchain.
export CC="${CC:-gcc}"
export CXX="${CXX:-g++}"

echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../Sophus

echo "Configuring and building Thirdparty/Sophus ..."

mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM3 ..."

mkdir -p build
cd build
PANGOLIN_CMAKE_ARGS=()
if [[ -f "${HOME}/Pangolin/build/PangolinConfig.cmake" ]]; then
  PANGOLIN_CMAKE_ARGS+=(-DPangolin_DIR="${HOME}/Pangolin/build")
elif [[ -f "${HOME}/Pangolin/build/src/PangolinConfig.cmake" ]]; then
  PANGOLIN_CMAKE_ARGS+=(-DPangolin_DIR="${HOME}/Pangolin/build/src")
fi
cmake .. -DCMAKE_BUILD_TYPE=Release "${PANGOLIN_CMAKE_ARGS[@]}"
JOBS="${JOBS:-$(nproc)}"
make -j"${JOBS}"

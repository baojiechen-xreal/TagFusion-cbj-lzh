cmake_minimum_required(VERSION 3.16)
project(ORB_SLAM3)

# Generate compile_commands.json for IDE (clangd / C++ IntelliSense)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# C++17: ua_tag uses std::optional; the other modules stay compatible
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
if(COMPILER_SUPPORTS_CXX17)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
   add_definitions(-DCOMPILEDWITHC17)
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++17.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support (required for std::optional in TagObservation).")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 4.2)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV >= 4.2 not found.")
   endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(Eigen3 3.1.0 REQUIRED)

# Prefer a locally-built Pangolin with matching C++11 ABI.
# /usr/local/lib/libpangolin.so may be an older ABI build (std::string without __cxx11),
# which causes undefined references when linking examples against libORB_SLAM3.so.
if(NOT Pangolin_DIR)
    set(_PANGOLIN_CANDIDATE "${PROJECT_SOURCE_DIR}/../../Pangolin/build/src")
    if(EXISTS "${_PANGOLIN_CANDIDATE}/PangolinConfig.cmake")
        set(Pangolin_DIR "${_PANGOLIN_CANDIDATE}" CACHE PATH "Pangolin CMake config directory" FORCE)
        message(STATUS "Using local Pangolin: ${Pangolin_DIR}")
    endif()
    unset(_PANGOLIN_CANDIDATE)
endif()
find_package(Pangolin REQUIRED)
find_package(realsense2)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/src
${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
# core: 基础配置、传感器类型、词袋、几何工具与序列化
src/core/Config.cc
src/core/Config.h
src/core/Settings.cc
src/core/Settings.h
src/core/ImuTypes.cc
src/core/ImuTypes.h
src/core/Converter.cc
src/core/Converter.h
src/core/GeometricTools.cc
src/core/GeometricTools.h
src/core/ORBVocabulary.h
src/core/SerializationUtils.h
src/core/TagObservation.h

# camera: 相机模型与双视图几何重建
src/camera/GeometricCamera.h
src/camera/Pinhole.cpp
src/camera/Pinhole.h
src/camera/KannalaBrandt8.cpp
src/camera/KannalaBrandt8.h
src/camera/TwoViewReconstruction.cc
src/camera/TwoViewReconstruction.h

# database: 帧、关键帧、地图点、地图、Atlas、词袋数据库与 Tag 地图数据
src/database/Frame.cc
src/database/Frame.h
src/database/KeyFrame.cc
src/database/KeyFrame.h
src/database/MapPoint.cc
src/database/MapPoint.h
src/database/Map.cc
src/database/Map.h
src/database/Atlas.cc
src/database/Atlas.h
src/database/KeyFrameDatabase.cc
src/database/KeyFrameDatabase.h
src/database/TagFrameData.cc
src/database/TagFrameData.h
src/database/MapTagData.cc
src/database/MapTagData.h

# features: ORB 特征提取与匹配
src/features/ORBextractor.cc
src/features/ORBextractor.h
src/features/ORBmatcher.cc
src/features/ORBmatcher.h
src/features/AprilTagDetector.h

# system: 系统总控、前端跟踪、局部建图与 Tag 初始化/跟踪
src/system/System.cc
src/system/System.h
src/system/Tracking.cc
src/system/Tracking.h
src/system/LocalMapping.cc
src/system/LocalMapping.h
src/system/TagInitializer.cc
src/system/TagInitializer.h
src/system/TagTracker.cc
src/system/TagTracker.h

# optimization: 非线性优化器、g2o 图元素、MLPnP 与 Tag 约束
src/optimization/Optimizer.cc
src/optimization/Optimizer.h
src/optimization/G2oTypes.cc
src/optimization/G2oTypes.h
src/optimization/OptimizableTypes.cpp
src/optimization/OptimizableTypes.h
src/optimization/MLPnPsolver.cpp
src/optimization/MLPnPsolver.h
src/optimization/TagOptimizer.cc
src/optimization/TagOptimizer.h
src/optimization/TagPoseConstraints.cc
src/optimization/TagPoseConstraints.h

# loop: 回环检测与闭环 Sim3 求解
src/loop/LoopClosing.cc
src/loop/LoopClosing.h
src/loop/Sim3Solver.cc
src/loop/Sim3Solver.h

# viz: 可视化线程与绘制器
src/viz/Viewer.cc
src/viz/Viewer.h
src/viz/FrameDrawer.cc
src/viz/FrameDrawer.h
src/viz/MapDrawer.cc
src/viz/MapDrawer.h

# io: Tag 地图导出
src/io/TagMapExporter.cc
src/io/TagMapExporter.h
)

add_subdirectory(Thirdparty/g2o)

# Thirdparty: AprilTag
option(ORB_SLAM3_WITH_APRILTAG "Build and link AprilTag detection library" ON)
if(ORB_SLAM3_WITH_APRILTAG)
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(BUILD_PYTHON_WRAPPER OFF CACHE BOOL "" FORCE)
    add_subdirectory(Thirdparty/apriltag)
endif()

set(ORB_SLAM3_THIRDPARTY_LIBS
    ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
)
if(ORB_SLAM3_WITH_APRILTAG)
    list(APPEND ORB_SLAM3_THIRDPARTY_LIBS apriltag)
    target_compile_definitions(${PROJECT_NAME} PRIVATE HAS_APRILTAG)
    target_sources(${PROJECT_NAME} PRIVATE
        src/features/AprilTagDetector.cc
        src/viz/AprilTagVisualizer.cc
        src/viz/AprilTagVisualizer.h
        src/io/TagPoseLogger.cc
        src/io/TagPoseLogger.h
        src/viz/TagViewer.cc
        src/viz/TagViewer.h)
    target_include_directories(${PROJECT_NAME} PRIVATE
        ${PROJECT_SOURCE_DIR}/Thirdparty/apriltag/include
        ${PROJECT_SOURCE_DIR}/Thirdparty/apriltag/ethz_apriltag2/include)
endif()

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${ORB_SLAM3_THIRDPARTY_LIBS}
-lboost_serialization
-lcrypto
)

# If RealSense SDK is found the library is added and its examples compiled
if(realsense2_FOUND)
    include_directories(${PROJECT_NAME}
    ${realsense_INCLUDE_DIR}
    )
    target_link_libraries(${PROJECT_NAME}
    ${realsense2_LIBRARY}
    )
endif()


# Build examples
# Original ORB-SLAM3 dataset examples (EuRoC, KITTI, TUM, RealSense) have been removed.

# TagFusion examples (per-dataset folders under Examples/TagFusion/)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/TagFusion/test)

add_executable(stereo_test_tag
        Examples/TagFusion/test/stereo_test_tag.cc)
target_link_libraries(stereo_test_tag ${PROJECT_NAME})

add_executable(stereo_inertial_test_tag
        Examples/TagFusion/test/stereo_inertial_test_tag.cc)
target_link_libraries(stereo_inertial_test_tag ${PROJECT_NAME})

add_executable(test_tag_corner_jac
        Examples/TagFusion/test/test_tag_corner_jac.cc)
target_link_libraries(test_tag_corner_jac ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/TagFusion/PennCOSYVIO)
add_executable(mono_penncosyvio_tag
        Examples/TagFusion/PennCOSYVIO/mono_penncosyvio_tag.cc)
target_link_libraries(mono_penncosyvio_tag ${PROJECT_NAME})

add_executable(stereo_penncosyvio_tag
        Examples/TagFusion/PennCOSYVIO/stereo_penncosyvio_tag.cc)
target_link_libraries(stereo_penncosyvio_tag ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/TagFusion/XREAL_Glass)
add_executable(mono_xreal_glass_tag
        Examples/TagFusion/XREAL_Glass/mono_xreal_glass_tag.cc)
target_link_libraries(mono_xreal_glass_tag ${PROJECT_NAME})

add_executable(stereo_xreal_glass_tag
        Examples/TagFusion/XREAL_Glass/stereo_xreal_glass_tag.cc)
target_link_libraries(stereo_xreal_glass_tag ${PROJECT_NAME})

add_executable(stereo_inertial_xreal_glass_tag
        Examples/TagFusion/XREAL_Glass/stereo_inertial_xreal_glass_tag.cc)
target_link_libraries(stereo_inertial_xreal_glass_tag ${PROJECT_NAME})

# ethz_apriltag2 CLI for Python scripts (same backend as ua_tag::AprilTagDetector)
if(ORB_SLAM3_WITH_APRILTAG)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/scripts)
    add_executable(ethz_tag_detect scripts/ethz_tag_detect_cli.cc)
    target_link_libraries(ethz_tag_detect apriltag ${OpenCV_LIBS})
    target_include_directories(ethz_tag_detect PRIVATE
        ${PROJECT_SOURCE_DIR}/Thirdparty/apriltag/include
        ${PROJECT_SOURCE_DIR}/Thirdparty/apriltag/ethz_apriltag2/include)

    find_package(Threads REQUIRED)
    add_executable(ethz_tag_enhance_eval scripts/ethz_tag_enhance_eval.cc)
    target_link_libraries(ethz_tag_enhance_eval apriltag ${OpenCV_LIBS} Threads::Threads)
    target_include_directories(ethz_tag_enhance_eval PRIVATE
        ${PROJECT_SOURCE_DIR}/Thirdparty/apriltag/include
        ${PROJECT_SOURCE_DIR}/Thirdparty/apriltag/ethz_apriltag2/include)
endif()

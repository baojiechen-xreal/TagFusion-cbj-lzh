# TagFusion

### Based on ORB-SLAM3 V1.0
**ORB-SLAM3 authors:** Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, [José M. M. Montiel](http://webdiis.unizar.es/~josemari/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/).

TagFusion is a **Visual-Inertial SLAM system that fuses AprilTag fiducial markers with ORB features and IMU measurements**. It is built on top of [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) and targets offline processing of **stereo-inertial** sequences recorded with XREAL glasses, using **pin-hole and fisheye (Kannala-Brandt)** lens models.

Compared to ORB-SLAM3, TagFusion adds:

* **AprilTag frontend.** Tag corners are detected in every frame with the ethz AprilTag library (*Thirdparty/apriltag*, default family `tag36h11`). The tag pose is solved with IPPE_SQUARE, and the two-fold IPPE ambiguity is resolved by single-frame error ratio, stereo cross-reprojection or multi-keyframe observations.
* **Tag map.** Tags are first-class map elements (`MapTagData`) with a CANDIDATE → ACTIVE lifecycle. The tag map is initialized in stereo and stereo-inertial modes only, once ORB tracking is OK, the first visual-inertial BA (VIBA 1) has finished (stereo-inertial), and at least 3 disambiguated tags are visible in the same frame.
* **Tag constraints in optimization.** Tag corner reprojection edges are added to pose optimization, visual Local BA, inertial Local BA and full inertial BA.
* **Tag loop closing.** Re-observed tags are checked for loops before the BoW place recognition, and the original `CorrectLoop` is reused for correction.
* **Synchronous offline backend.** LocalMapping and LoopClosing no longer run in their own threads; they are executed after each tracked frame (`System::RunOfflineBackend`), which makes runs reproducible.
* **No Pangolin viewer.** Visualization is done with OpenCV `imshow`, controlled by `Tag.show_in_orb_viewer`.

The original ORB-SLAM3 examples (EuRoC, TUM-VI, KITTI, RealSense and ROS) have been removed.

### Related Publications:

[ORB-SLAM3] Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M. M. Montiel and Juan D. Tardós, **ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM**, *IEEE Transactions on Robotics 37(6):1874-1890, Dec. 2021*. **[PDF](https://arxiv.org/abs/2007.11898)**.

[IMU-Initialization] Carlos Campos, J. M. M. Montiel and Juan D. Tardós, **Inertial-Only Optimization for Visual-Inertial Initialization**, *ICRA 2020*. **[PDF](https://arxiv.org/pdf/2003.05766.pdf)**

[AprilTag] Edwin Olson, **AprilTag: A robust and flexible visual fiducial system**, *ICRA 2011*.

[IPPE] Toby Collins and Adrien Bartoli, **Infinitesimal Plane-Based Pose Estimation**, *International Journal of Computer Vision 109(3):252-286, 2014*.

[DBoW2 Place Recognition] Dorian Gálvez-López and Juan D. Tardós. **Bags of Binary Words for Fast Place Recognition in Image Sequences**. *IEEE Transactions on Robotics,* vol. 28, no. 5, pp. 1188-1197, 2012. **[PDF](http://doriangalvez.com/php/dl.php?dlp=GalvezTRO12.pdf)**

# 1. License

TagFusion is a modified version of ORB-SLAM3 and is therefore released under the [GPLv3 license](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/LICENSE). For a list of all code/library dependencies (and associated licenses), please see [Dependencies.md](Dependencies.md).

If you use this work in an academic context, please cite ORB-SLAM3:

    @article{ORBSLAM3_TRO,
      title={{ORB-SLAM3}: An Accurate Open-Source Library for Visual, Visual-Inertial 
               and Multi-Map {SLAM}},
      author={Campos, Carlos AND Elvira, Richard AND G\´omez, Juan J. AND Montiel, 
              Jos\'e M. M. AND Tard\'os, Juan D.},
      journal={IEEE Transactions on Robotics}, 
      volume={37},
      number={6},
      pages={1874-1890},
      year={2021}
     }

# 2. Prerequisites
We have tested the library in **Ubuntu 22.04** with **g++ 11.4** and **CMake 3.22**. On Ubuntu 22.04 most dependencies can be installed with:

```
sudo apt install cmake build-essential git pkg-config \
  libopencv-dev libeigen3-dev libboost-all-dev libssl-dev \
  libglew-dev libgl1-mesa-dev libepoxy-dev python3-dev
```

## C++17 Compiler
The project is compiled with C++17.

## Pangolin
The Pangolin viewer is disabled at runtime, but several sources still include Pangolin headers, so [Pangolin](https://github.com/stevenlovegrove/Pangolin) is **required at build time. Tested with v0.8** (v0.6 is also supported). CMake first looks for it in `../../Pangolin/build/src` relative to the repository root; otherwise pass `-DPangolin_DIR=...`:

```
cd ~
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin && git checkout v0.8
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j8
```

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features and to solve IPPE. **Required at least 4.2. Tested with OpenCV 4.5.4**.

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## Boost and OpenSSL
Boost serialization and OpenSSL (`libcrypto`) are used by the Atlas serialization code.

## DBoW2, g2o, Sophus and AprilTag (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition, [g2o](https://github.com/RainerKuemmerle/g2o) to perform non-linear optimizations and [Sophus](https://github.com/strasdat/Sophus) for Lie groups. Tags are detected with the ethz AprilTag library (`ethz_apriltag2`). All of them are included in the *Thirdparty* folder and built automatically.

## GTSAM (optional)
GTSAM 4.2.2 is included in *Thirdparty* but only used by the monocular tag initializer, which is not reached in stereo or stereo-inertial runs. Its bundled `timing/` folder is incomplete, so **`build.sh` disables it** (`-DORB_SLAM3_WITH_GTSAM=OFF`).

## Python (optional)
Required by the evaluation and visualization scripts in *scripts*. **Required Numpy module**; [evo](https://github.com/MichaelGrupp/evo) is needed for trajectory evaluation (`pip install evo`).

# 3. Building TagFusion library and examples

Clone the repository:
```
git clone https://github.com/xreal-image-process/TagFusion.git TagFusion
```

We provide a script `build.sh` to build the *Thirdparty* libraries, uncompress the ORB vocabulary and build *TagFusion*. Please make sure you have installed all required dependencies (see section 2). Execute:
```
cd TagFusion
chmod +x build.sh
./build.sh
```

This will create **libORB_SLAM3.so** at *lib* folder and the following executables:

* `Examples/TagFusion/XREAL_Glass/`: `stereo_inertial_xreal_glass_tag` (**main entry**), `stereo_xreal_glass_tag`, `mono_xreal_glass_tag`.
* `Examples/TagFusion/PennCOSYVIO/`: `stereo_penncosyvio_tag`, `mono_penncosyvio_tag`.
* `Examples/TagFusion/test/`: `stereo_test_tag`, `stereo_inertial_test_tag`, `test_tag_corner_jac` (numerical check of the tag corner Jacobians).
* `scripts/`: `ethz_tag_detect`, `ethz_tag_enhance_eval` (command line tag detection tools used by the Python scripts).

All SLAM executables take the same arguments: `path_to_vocabulary path_to_settings path_to_sequence [trajectory_file_name]`. To rebuild a single target after changing the code:
```
cd build
make -j6 stereo_inertial_xreal_glass_tag
```

# 4. Running TagFusion with XREAL Glass sequences

## Sequence format
Each sequence is expected to have the following layout:

```
<sequence>/raw_data/glass/
├── cam0/images/m0000000.pgm ...
├── cam0/images/timestamps.txt    # one line per frame: "m0000000.pgm 37.763832704" (seconds)
├── cam1/images/m0000000.pgm ...  # same file names as cam0
└── imu0.txt                      # one line per sample: "t temp gx gy gz ax ay az" (rad/s, m/s^2)
```

The sequence argument can point to `<sequence>`, `.../glass`, `.../glass/cam0` or `.../glass/cam0/images`; the right camera and IMU paths are derived automatically. Only the first 8 columns of `imu0.txt` are read (extra columns are ignored), and the 7-column format `t gx gy gz ax ay az` is also accepted. If the IMU file on disk is named `imu.txt`, create a link before running:
```
ln -sfn imu.txt <sequence>/raw_data/glass/imu0.txt
```

## Settings files
Settings files are in *Examples/TagFusion/XREAL_Glass*. **The settings file must match the capture date of the sequence**, since resolution, frame rate, calibration and tag size differ between batches:

* **20260819**: `20260819_stereo+imu.yaml` (640x512 @ 60 fps, tag size 0.144 m)
* **20260812**: `20260812_stereo+imu.yaml` (480x640 @ 30 fps, tag size 0.144 m with per-id overrides)
* **20260807**: `20260807_stereo+imu.yaml` (640x512 @ 60 fps, tag size 0.144 m)
* **20260309**: `20260309_stereo+imu.yaml` (480x640 @ 30 fps, tag size 0.06 m)

The most relevant parameters are:

* `System.maxFrames`: process only the first N frames (0 or absent = all). Recommended for debugging.
* `Tag.enable`: 0 disables the whole tag module (plain ORB-SLAM3 behaviour).
* `Tag.black_borders`: tag black border width(s), e.g. `[1, 2]` (one detector per value). The legacy key `Tag.black_border: 1` is still accepted. **A wrong value means no tag will be detected.**
* `Tag.size`, `Tag.size_by_id`: default tag side length (meters) and per-id overrides. `Tag.ignore_ids` drops detections of the listed ids.
* `Tag.pose_optimization`, `Tag.local_ba.enable`, `Tag.loop.enable`: enable tag constraints in pose optimization, Local BA and tag loop closing. `Tag.factor_weight.*` adapts the tag weight to tag area, viewing angle and ambiguity.
* `Tag.show_in_orb_viewer`: 1 shows the current frame and tag detections with `imshow` every frame. **It requires a display; set it to 0 on headless machines or over SSH.**
* `Tag.export`, `Tag.export_dir`: export of the tag map and detections. `Tag.verbose` (and the `verbose` flag of each sub-module) produces very detailed logs.

## Running
Execute the following, choosing the settings file according to the capture date:
```
cd Examples/TagFusion/XREAL_Glass
SETTINGS=./20260309_stereo+imu.yaml ./run_xreal_glass_stereo_inertial.sh <sequence>/raw_data/glass test_0309
```

The script changes to its own folder, so all outputs are written to *Examples/TagFusion/XREAL_Glass*. The defaults can be overridden with the environment variables `VOC`, `SETTINGS`, `BIN` and `SEQ`. For a stereo-only run use `run_xreal_glass_stereo.sh`. Long sequences can be run in background:
```
SETTINGS=./20260309_stereo+imu.yaml nohup ./run_xreal_glass_stereo_inertial.sh <sequence>/raw_data/glass test_0309 > run_0309.log 2>&1 &
grep -c "Tracking begin" run_0309.log    # processed frames
pkill -f stereo_inertial_xreal_glass_tag  # stop the run
```

**Remark:** Signals are not handled and the final trajectories are only written after the last frame. If the run is stopped with Ctrl+C or `kill`, `f_*.txt` and `kf_*.txt` are not saved; use `System.maxFrames` for shorter runs instead. Closing the `imshow` windows does not stop the program.

## Output
* `f_<name>.txt` and `kf_<name>.txt`: frame and keyframe trajectories in TUM format (`CameraTrajectory.txt` and `KeyFrameTrajectory.txt` if no name is given).
* `OnlineCameraTrajectory.txt`: online trajectory, i.e. the pose of each frame right after tracking, not affected by later BA.
* `FrameTimeStats.csv`: per-frame timings (frontend, tag detection, IPPE, LocalMapping, LoopClosing).
* `tag_export/`: tag map corners (selected and mirror IPPE solutions), detections, first registrations, initialization keyframes and the final map as `final_map.ply`.

# 5. Other Examples

## PennCOSYVIO
[PennCOSYVIO](https://daniilidis-group.github.io/penncosyvio/) was recorded with a VI-Sensor stereo camera. Edit the default sequence path in `Examples/TagFusion/PennCOSYVIO/run_stereo_af.sh` or pass it as first argument:
```
./Examples/TagFusion/PennCOSYVIO/run_stereo_af.sh <path>/PennCOSYVIO/visensor/af
```

## Tank test sequences
`Examples/TagFusion/test/run_stereo_test.sh` and `run_stereo_inertial_test.sh` run the stereo and stereo-inertial test programs. The stereo-inertial case needs an `imu0.txt`, which can be extracted from a rosbag with `extract_imu_from_bag.py`.

# 6. Evaluation and tools

The *scripts* folder contains:

* `eval_evo_ape.sh`: APE of an estimated trajectory against the `cam0_pose.txt` ground truth, using evo.
* `visualize_init_maps.py`: interactive 3D view of the tag map and ORB map exported to `tag_export/` (`--export-dir`, `--final`, `--html`, `--png`).
* `evaluate_tag_map_calibr_plane.py`, `evaluate_tag_map_calibr_planes.py`: coplanarity of the tag map against the calibration board planes.
* `tag_detect.py`, `tag_detect1.py`: offline tag detection on images through `ethz_tag_detect`.
* `ethz_tag_enhance_eval`: tag detection rate under different image enhancements.

# 7. Running time analysis
Each run writes `FrameTimeStats.csv` with per-frame timings. A summary (and an optional Plotly report) can be obtained with:
```
python3 scripts/summarize_frame_times.py --csv Examples/TagFusion/XREAL_Glass/FrameTimeStats.csv [--html report.html] [--ok-only]
```

# 8. Troubleshooting
* **The tag map is never initialized.** At least 3 disambiguated tags must be visible in the same frame, in stereo or stereo-inertial mode. Check `Tag.black_borders` and `Tag.size`, then enable `Tag.verbose` and look at the `[TagStereo]` and `[TagInitializer]` messages.
* **Many `ambiguity_unresolved` messages.** Some tags do not pass the ambiguity ratio threshold yet; they are retried as more keyframes are created. This is expected.
* **IMU initialization fails or the map is reset repeatedly.** Check `IMU.Frequency` and the `IMU.T_b_c1` extrinsics against the data, and make sure there is enough motion at the beginning of the sequence.
* **The program exits with `requires Camera2.*`.** Stereo settings need `Camera1.*`, `Camera2.*` and `Stereo.T_c1_c2`.

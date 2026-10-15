#ifndef UA_TAG_TAG_MAP_EXPORTER_H
#define UA_TAG_TAG_MAP_EXPORTER_H

#include "ua_tag/TagPoseConstraints.h"

#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <sophus/se3.hpp>

namespace ORB_SLAM3 {

class KeyFrame;
class Map;

namespace xreal_tag {

class MapTagData;
class TagFrameData;

// Export, from the Map, each MapTag world-frame corner and the associated keyframe camera Pose
class TagMapExporter
{
public:
    // Under output_dir, write tag_map_corners.csv / tag_map_mirror_corners.csv
    //                    / tag_camera_trajectory.txt(TUM)
    //                    / tag_detections.csv (detections of every frame)
    //                    / tag_first_registration.csv (geometry when each MapTag is first registered)
    explicit TagMapExporter(const std::string &output_dir,
                            TagFactorWeightParams factor_weight = {});
    ~TagMapExporter();

    TagMapExporter(const TagMapExporter &) = delete;
    TagMapExporter &operator=(const TagMapExporter &) = delete;

    bool IsEnabled() const noexcept { return enabled_; }
    const std::string &GetOutputDir() const noexcept { return output_dir_; }

    // Append one camera-trajectory sample (input Tcw; stored as TUM: timestamp tx ty tz qx qy qz qw, pose is Twc)
    void AppendCameraPose(unsigned long frame_id,
                          double timestamp,
                          const Sophus::SE3f &Tcw);

    // Append this frame AprilTag detection log (one row per eye): area / perimeter / hamming / center
    void AppendTagDetections(unsigned long frame_id,
                             double timestamp,
                             const TagFrameData &frame_data);

    // When a MapTag is first inserted into the map: record on that KF the observed area, distance, and the angles between the optical axis and the plane / normal
    // left_idx / right_idx index KF.tag_frame_data_; -1 when that eye has no observation
    void AppendMapTagFirstRegistration(KeyFrame *pKF,
                                       int tag_id,
                                       int left_idx,
                                       int right_idx,
                                       MapTagData *pTag);

    // After a successful init Commit: one first-registration row for every MapTag already linked to an init KF
    void LogKeyFrameMapTagFirstRegistrations(KeyFrame *pKF);

    // Export every MapTag in the current Map: tag_id + four world-frame corners + factor-weight summary (repeat calls overwrite)
    // Also write tag_map_mirror_corners.csv (the IPPE mirror solution not selected when the pose was first assigned)
    // verbose=false is for online updates, so the log is not spammed
    bool SaveTagMapCorners(Map &map, bool verbose = true) const;

    // Export poses of keyframes that are associated with a MapTag (Twc, TUM)
    bool SaveTagInitKeyFrames(Map &map) const;

    // Export the ORB init map points after scale alignment, plus the two keyframe poses (same directory as the Tag export, for scale comparison)
    // orb_kf_tcw: (timestamp, Tcw)
    bool SaveOrbInitMap(
        const std::vector<Eigen::Vector3f> &points,
        const std::vector<std::pair<double, Sophus::SE3f>> &orb_kf_tcw) const;

    // Shutdown: final Tag corners/planes + every ORB point + the KF trajectory + an ASCII PLY (includes Tag quad faces)
    bool SaveFinalMap(Map &map, bool verbose = true) const;

    // Close the trajectory, detection, and first-registration log files
    void CloseTrajectory();

    // After a map Reset, truncate and reopen the export files so trajectories from the old and new world frames are not concatenated
    void ResetSession();

private:
    bool OpenTrajectoryFile();
    bool OpenDetectionLogFile();
    bool OpenFirstRegistrationLogFile();
    bool WriteTagCornersCsv(Map &map, const std::string &filename, bool mirror,
                            bool verbose) const;
    bool WriteKeyFramesTum(Map &map, const std::string &filename,
                           bool tag_associated_only, bool verbose) const;
    bool WriteOrbPointsCsv(const std::vector<Eigen::Vector3f> &points,
                           const std::string &filename, bool verbose) const;
    bool WriteFinalMapPly(Map &map,
                          const std::vector<Eigen::Vector3f> &orb_points,
                          bool verbose) const;

    std::string output_dir_;
    TagFactorWeightParams factor_weight_;
    bool enabled_{false};
    mutable std::ofstream trajectory_file_;
    mutable std::ofstream detection_file_;
    mutable std::ofstream first_registration_file_;
};

}  // namespace xreal_tag
}  // namespace ORB_SLAM3

#endif

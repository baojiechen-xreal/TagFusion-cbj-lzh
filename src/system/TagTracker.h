#ifndef UA_TAG_TAG_TRACKER_H
#define UA_TAG_TAG_TRACKER_H

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <sophus/se3.hpp>

namespace ORB_SLAM3 {

class Frame;
class KeyFrame;
class Map;

namespace xreal_tag {

class MapTagData;
class TagMapExporter;

class TagTracker
{
public:
    TagTracker();

    // Read Tag.verbose, Tag.export_dir, and related keys from the settings yaml
    explicit TagTracker(const std::string &settings_file);
    ~TagTracker();

    TagTracker(const TagTracker &) = delete;
    TagTracker &operator=(const TagTracker &) = delete;

    // Dormant tracking API (not yet called from the Tracking main loop; the signature already takes a Map)
    bool Track(Frame &frame, Map &map);
    bool Relocalize(Frame &frame, Map &map);

    // After init succeeds, cache the first-frame pose (velocity is not available yet)
    void SeedLastPose(const Sophus::SE3f &Tcw);

    // Clear the temporary pose, velocity, and reference-keyframe caches (on tracking failure or reset)
    void ClearMotionCache();

    // After a map Reset, truncate the Tag export files (trajectory / detections / first registration)
    void ResetExports();

    // When export is enabled: record the current-frame camera Pose trajectory
    void LogCameraPose(const Frame &frame);

    // When export is enabled: record this frame AprilTag detections (including observed area) to tag_detections.csv
    void LogTagDetections(const Frame &frame);

    // When export is enabled: append tag_first_registration.csv when a MapTag is first registered
    void LogMapTagFirstRegistration(KeyFrame *pKF, int tag_id, int left_idx,
                                    int right_idx, MapTagData *pTag);

    // When export is enabled: after init Commit, record first-registration geometry for every MapTag on each init KF
    void LogKeyFrameMapTagFirstRegistrations(KeyFrame *pKF);

    // When export is enabled: write the final Tag+ORB map (CSV/PLY) and close the trajectory file
    void SaveExports(Map &map);

    // When export is enabled: overwrite tag_map_corners.csv and tag_map_mirror_corners.csv online (silent)
    void SaveTagMapOnline(Map &map);

    // When export is enabled: right after scale alignment, write MapTag + ORB init point cloud / keyframes (for scale comparison)
    bool SaveInitializationMaps(Map &map,
                      const std::vector<Eigen::Vector3f> &orb_points,
                      const std::vector<std::pair<double, Sophus::SE3f>> &orb_kf_tcw);

    long unsigned int GetReferenceKeyFrameId() const noexcept { return reference_key_frame_id_; }
    bool HasReferenceKeyFrame() const noexcept { return has_reference_key_frame_; }

    // Whether condition 3 is enabled: optional reference-KF baseline test (dormant path)
    void SetCheckReferenceKeyFrameBaseline(bool enable) noexcept { check_reference_key_frame_baseline_ = enable; }
    bool GetCheckReferenceKeyFrameBaseline() const noexcept { return check_reference_key_frame_baseline_; }

    void SetMaxVisibleFramesPerMarker(int n) noexcept { max_visible_frames_per_marker_ = n; }
    void SetMinBaseline(float meters) noexcept { min_baseline_m_ = meters; }

private:
    std::optional<Sophus::SE3f> PredictPoseWithMotionModel() const;
    void SelectReferenceKeyFrame(const Frame &frame, Map &map);
    bool ShouldCreateKeyFrame(const Frame &frame, const Map &map) const;
    void SetMotionCache(const Sophus::SE3f &tcw_current);

    Sophus::SE3f last_tcw_;
    bool has_last_tcw_{false};

    Sophus::SE3f velocity_;
    bool has_velocity_{false};

    long unsigned int reference_key_frame_id_{0};
    bool has_reference_key_frame_{false};

    int max_visible_frames_per_marker_{10};
    float min_baseline_m_{0.07f};
    bool check_reference_key_frame_baseline_{false};

    bool verbose_{false};

    std::unique_ptr<TagMapExporter> exporter_;
};

}  // namespace xreal_tag
}  // namespace ORB_SLAM3

#endif

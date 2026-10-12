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

namespace tag {

class MapTagData;
class TagMapExporter;

class TagTracker
{
public:
    TagTracker();

    // 从 settings yaml 读取 Tag.verbose、Tag.export_dir 等
    explicit TagTracker(const std::string &settings_file);
    ~TagTracker();

    TagTracker(const TagTracker &) = delete;
    TagTracker &operator=(const TagTracker &) = delete;

    // 休眠跟踪接口（尚未接入 Tracking 主循环；签名已切到 Map）
    bool Track(Frame &frame, Map &map);
    bool Relocalize(Frame &frame, Map &map);

    // 初始化成功后写入首帧位姿缓存（尚无速度）
    void SeedLastPose(const Sophus::SE3f &Tcw);

    // 清空位姿/速度/参考关键帧临时缓存（跟踪失败或重置时调用）
    void ClearMotionCache();

    // 地图 Reset 后截断 Tag 导出文件（轨迹/检测/首次注册）
    void ResetExports();

    // 若开启导出：记录当前帧相机 Pose 轨迹
    void LogCameraPose(const Frame &frame);

    // 若开启导出：记录本帧 AprilTag 检测（含观测面积）到 tag_detections.csv
    void LogTagDetections(const Frame &frame);

    // 若开启导出：MapTag 首次注册时追加 tag_first_registration.csv
    void LogMapTagFirstRegistration(KeyFrame *pKF, int tag_id, int left_idx,
                                    int right_idx, MapTagData *pTag);

    // 若开启导出：初始化 Commit 后，对初始化 KF 上全部 MapTag 记首次注册几何
    void LogKeyFrameMapTagFirstRegistrations(KeyFrame *pKF);

    // 若开启导出：写出终局 Tag+ORB 地图（CSV/PLY）并关闭轨迹文件
    void SaveExports(Map &map);

    // 若开启导出：在线覆盖写入 tag_map_corners.csv 与 tag_map_mirror_corners.csv（静默）
    void SaveTagMapOnline(Map &map);

    // 若开启导出：定标后立刻写出 MapTag + ORB 初始化点云/关键帧（尺度对比用）
    bool SaveInitializationMaps(Map &map,
                      const std::vector<Eigen::Vector3f> &orb_points,
                      const std::vector<std::pair<double, Sophus::SE3f>> &orb_kf_tcw);

    long unsigned int GetReferenceKeyFrameId() const noexcept { return reference_key_frame_id_; }
    bool HasReferenceKeyFrame() const noexcept { return has_reference_key_frame_; }

    // 是否启用条件 3：可选参考 KF baseline 判定（休眠路径）
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

}  // namespace tag
}  // namespace ORB_SLAM3

#endif

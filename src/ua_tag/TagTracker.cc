#include "ua_tag/TagTracker.h"

#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"
#include "ua_tag/MapTagData.h"
#include "ua_tag/TagMapExporter.h"
#include "ua_tag/TagPoseConstraints.h"

#include <iostream>

#include <opencv2/core.hpp>

namespace ORB_SLAM3 {
namespace tag {

TagTracker::TagTracker() = default;

TagTracker::~TagTracker() = default;

TagTracker::TagTracker(const std::string &settings_file)
{
    cv::FileStorage fs(settings_file, cv::FileStorage::READ);
    if(!fs.isOpened())
        return;

    cv::FileNode node = fs["Tag.verbose"];
    if(!node.empty())
        verbose_ = static_cast<int>(node) != 0;

    // Tag.export=1 时启用导出；目录默认 tag_export，可用 Tag.export_dir 覆盖
    bool enable_export = false;
    node = fs["Tag.export"];
    if(!node.empty())
        enable_export = static_cast<int>(node) != 0;

    std::string export_dir = "tag_export";
    node = fs["Tag.export_dir"];
    if(!node.empty())
        export_dir = static_cast<std::string>(node);

    if(enable_export)
    {
        exporter_ = std::make_unique<TagMapExporter>(
            export_dir, TagFactorWeightParams::FromSettings(settings_file));
    }
}

void TagTracker::LogCameraPose(const Frame &frame)
{
    if(!exporter_ || !exporter_->IsEnabled() || !frame.HasPose())
        return;
    exporter_->AppendCameraPose(frame.mnId, frame.mTimeStamp, frame.GetPose());
}

void TagTracker::LogTagDetections(const Frame &frame)
{
    if(!exporter_ || !exporter_->IsEnabled())
        return;
    exporter_->AppendTagDetections(frame.mnId, frame.mTimeStamp, frame.tag_frame_data_);
}

void TagTracker::LogMapTagFirstRegistration(KeyFrame *pKF, int tag_id,
                                            int left_idx, int right_idx,
                                            MapTagData *pTag)
{
    if(!exporter_ || !exporter_->IsEnabled() || !pKF)
        return;
    exporter_->AppendMapTagFirstRegistration(pKF, tag_id, left_idx, right_idx,
                                              pTag);
}

void TagTracker::LogKeyFrameMapTagFirstRegistrations(KeyFrame *pKF)
{
    if(!exporter_ || !exporter_->IsEnabled() || !pKF)
        return;
    exporter_->LogKeyFrameMapTagFirstRegistrations(pKF);
}

void TagTracker::SaveExports(Map &map)
{
    if(!exporter_ || !exporter_->IsEnabled())
        return;
    exporter_->SaveFinalMap(map, true);
    exporter_->CloseTrajectory();
}

void TagTracker::SaveTagMapOnline(Map &map)
{
    if(!exporter_ || !exporter_->IsEnabled())
        return;
    exporter_->SaveTagMapCorners(map, false);
}

bool TagTracker::SaveInitializationMaps(
    Map &map,
    const std::vector<Eigen::Vector3f> &orb_points,
    const std::vector<std::pair<double, Sophus::SE3f>> &orb_kf_tcw)
{
    if(!exporter_ || !exporter_->IsEnabled())
        return false;

    bool ok = true;
    ok = exporter_->SaveTagMapCorners(map) && ok;
    ok = exporter_->SaveTagInitKeyFrames(map) && ok;
    ok = exporter_->SaveOrbInitMap(orb_points, orb_kf_tcw) && ok;
    return ok;
}

void TagTracker::SeedLastPose(const Sophus::SE3f &Tcw)
{
    last_tcw_ = Tcw;
    has_last_tcw_ = true;
    has_velocity_ = false;
}

void TagTracker::ClearMotionCache()
{
    has_last_tcw_ = false;
    has_velocity_ = false;
    reference_key_frame_id_ = 0;
    has_reference_key_frame_ = false;
}

void TagTracker::ResetExports()
{
    if(exporter_ && exporter_->IsEnabled())
        exporter_->ResetSession();
}

std::optional<Sophus::SE3f> TagTracker::PredictPoseWithMotionModel() const
{
    if(!has_velocity_ || !has_last_tcw_)
        return std::nullopt;

    return velocity_ * last_tcw_;
}

void TagTracker::SelectReferenceKeyFrame(const Frame &frame, Map &map)
{
    (void)frame;
    (void)map;
    // 休眠：完整共视参考 KF 选择尚未接入真实 Map/KF
    has_reference_key_frame_ = false;
    reference_key_frame_id_ = 0;
}

bool TagTracker::ShouldCreateKeyFrame(const Frame &frame, const Map &map) const
{
    (void)frame;
    (void)map;
    return false;
}

void TagTracker::SetMotionCache(const Sophus::SE3f &tcw_current)
{
    if(has_last_tcw_)
    {
        velocity_ = tcw_current * last_tcw_.inverse();
        has_velocity_ = true;
    }

    last_tcw_ = tcw_current;
    has_last_tcw_ = true;
}

bool TagTracker::Track(Frame &frame, Map &map)
{
    (void)frame;
    (void)map;
    // 休眠：Tag 跟踪尚未接入 Tracking 主循环
    return false;
}

bool TagTracker::Relocalize(Frame &frame, Map &map)
{
    (void)frame;
    (void)map;
    // TODO: Tag 重定位
    return false;
}

}  // namespace tag
}  // namespace ORB_SLAM3

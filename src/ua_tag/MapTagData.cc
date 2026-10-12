#include "ua_tag/MapTagData.h"

#include "KeyFrame.h"

#include <algorithm>
#include <vector>

namespace ORB_SLAM3 {
namespace tag {

namespace {

std::array<Eigen::Vector3f, 4> CornersFromPose(const Sophus::SE3f &T_wt,
                                               float tag_size)
{
    std::array<Eigen::Vector3f, 4> corners{};
    if(tag_size <= 0.0f)
        return corners;

    // 与 OpenCV IPPE_SQUARE / BuildSquareObjectPoints 约定一致
    const float h = tag_size * 0.5f;
    const Eigen::Vector3f pts_t[4] = {
        Eigen::Vector3f(-h,  h, 0.f),
        Eigen::Vector3f( h,  h, 0.f),
        Eigen::Vector3f( h, -h, 0.f),
        Eigen::Vector3f(-h, -h, 0.f)};

    for(int i = 0; i < 4; ++i)
        corners[i] = T_wt * pts_t[i];
    return corners;
}

}  // namespace

int MapTagData::GetId() const
{
    std::unique_lock<std::mutex> lock(mutex_features_);
    return id_;
}

void MapTagData::SetId(int id)
{
    std::unique_lock<std::mutex> lock(mutex_features_);
    id_ = id;
}

float MapTagData::GetTagSize() const
{
    std::unique_lock<std::mutex> lock(mutex_pose_);
    return tag_size_;
}

void MapTagData::SetTagSize(float tag_size)
{
    std::unique_lock<std::mutex> lock(mutex_pose_);
    tag_size_ = tag_size;
}

void MapTagData::SetPose(const Sophus::SE3f &t_wt)
{
    std::unique_lock<std::mutex> lock(mutex_pose_);
    twt_ = t_wt;
    has_pose_ = true;
}

Sophus::SE3f MapTagData::GetPose() const
{
    std::unique_lock<std::mutex> lock(mutex_pose_);
    return twt_;
}

bool MapTagData::HasPose() const
{
    std::unique_lock<std::mutex> lock(mutex_pose_);
    return has_pose_;
}

void MapTagData::SetMirrorPose(const Sophus::SE3f &t_wt)
{
    std::unique_lock<std::mutex> lock(mutex_pose_);
    twt_mirror_ = t_wt;
    has_mirror_pose_ = true;
}

Sophus::SE3f MapTagData::GetMirrorPose() const
{
    std::unique_lock<std::mutex> lock(mutex_pose_);
    return twt_mirror_;
}

bool MapTagData::HasMirrorPose() const
{
    std::unique_lock<std::mutex> lock(mutex_pose_);
    return has_mirror_pose_;
}

std::array<Eigen::Vector3f, 4> MapTagData::GetWorldCorners() const
{
    std::unique_lock<std::mutex> lock(mutex_pose_);
    if(!has_pose_)
        return {};
    return CornersFromPose(twt_, tag_size_);
}

bool MapTagData::HasWorldCorners() const
{
    std::unique_lock<std::mutex> lock(mutex_pose_);
    return has_pose_ && tag_size_ > 0.0f;
}

std::array<Eigen::Vector3f, 4> MapTagData::GetMirrorWorldCorners() const
{
    std::unique_lock<std::mutex> lock(mutex_pose_);
    if(!has_mirror_pose_)
        return {};
    return CornersFromPose(twt_mirror_, tag_size_);
}

bool MapTagData::HasMirrorWorldCorners() const
{
    std::unique_lock<std::mutex> lock(mutex_pose_);
    return has_mirror_pose_ && tag_size_ > 0.0f;
}

void MapTagData::SetState(MapTagState state)
{
    std::unique_lock<std::mutex> lock(mutex_features_);
    state_ = state;
}

MapTagState MapTagData::GetState() const
{
    std::unique_lock<std::mutex> lock(mutex_features_);
    return state_;
}

bool MapTagData::IsFixed() const
{
    std::unique_lock<std::mutex> lock(mutex_features_);
    return state_ == MapTagState::FIXED_ANCHOR;
}

bool MapTagData::IsBad() const
{
    std::unique_lock<std::mutex> lock(mutex_features_);
    return state_ == MapTagState::BAD;
}

Map *MapTagData::GetMap() const
{
    std::unique_lock<std::mutex> lock(mutex_features_);
    return map_;
}

void MapTagData::SetMapInternal(Map *map)
{
    std::unique_lock<std::mutex> lock(mutex_features_);
    map_ = map;
}

void MapTagData::AddObservationInternal(KeyFrame *key_frame, int left_index, int right_index)
{
    if(!key_frame)
        return;
    std::unique_lock<std::mutex> lock(mutex_features_);
    KeyFrameObservation obs;
    obs.left_index_ = left_index;
    obs.right_index_ = right_index;
    observations_[key_frame] = obs;
}

void MapTagData::EraseObservationInternal(KeyFrame *key_frame)
{
    if(!key_frame)
        return;
    std::unique_lock<std::mutex> lock(mutex_features_);
    observations_.erase(key_frame);
}

bool MapTagData::IsInKeyFrame(KeyFrame *key_frame) const
{
    if(!key_frame)
        return false;
    std::unique_lock<std::mutex> lock(mutex_features_);
    return observations_.find(key_frame) != observations_.end();
}

MapTagData::KeyFrameObservations MapTagData::GetObservations() const
{
    std::unique_lock<std::mutex> lock(mutex_features_);
    return observations_;
}

std::size_t MapTagData::GetObservationCount() const
{
    std::unique_lock<std::mutex> lock(mutex_features_);
    return observations_.size();
}

void MapTagData::SetFactorVizWeight(const FactorVizWeight &w)
{
    std::unique_lock<std::mutex> lock(mutex_features_);
    factor_viz_weight_ = w;
}

MapTagData::FactorVizWeight MapTagData::GetFactorVizWeight() const
{
    std::unique_lock<std::mutex> lock(mutex_features_);
    return factor_viz_weight_;
}

KeyFrame *MapTagData::SelectReferenceKeyFrame(int min_inlier_corners,
                                             bool allow_unresolved_fallback) const
{
    KeyFrameObservations observations;
    {
        std::unique_lock<std::mutex> lock(mutex_features_);
        observations = observations_;
    }

    if(min_inlier_corners < 1)
        min_inlier_corners = 1;
    if(min_inlier_corners > 4)
        min_inlier_corners = 4;

    struct Candidate
    {
        KeyFrame *key_frame = nullptr;
        bool resolved = false;
        bool opt_quality = false;
        int n_tracked = 0;
    };
    std::vector<Candidate> candidates;

    auto observationAt = [](KeyFrame *key_frame, int index, bool right) -> const TagObservation * {
        if(!key_frame || index < 0)
            return nullptr;
        const TagFrameData::Observations &vec =
            right ? key_frame->tag_frame_data_.right_ : key_frame->tag_frame_data_.left_;
        if(index >= static_cast<int>(vec.size()))
            return nullptr;
        return &vec[static_cast<size_t>(index)];
    };

    auto countCornerInliers = [](const TagObservation &obs) -> int {
        int nIn = 0;
        for(bool outlier : obs.corner_outliers_)
        {
            if(!outlier)
                ++nIn;
        }
        return nIn;
    };

    for(const auto &kv : observations)
    {
        KeyFrame *key_frame = kv.first;
        if(!key_frame || key_frame->isBad())
            continue;

        bool detect_valid = false;
        bool resolved = false;
        bool opt_quality = false;

        const TagObservation *cams[2] = {
            observationAt(key_frame, kv.second.left_index_, false),
            observationAt(key_frame, kv.second.right_index_, true)};
        for(const TagObservation *obs : cams)
        {
            if(!obs || !obs->IsDetectValid())
                continue;
            detect_valid = true;

            // 第二层：已消歧
            if(obs->IsAmbiguityResolved() && obs->pose_estimate_.has_value() &&
               obs->pose_estimate_->GetSelectedCandidate() != nullptr)
                resolved = true;

            // 第三层：优化内点质量
            if(!obs->is_optimization_outlier_ && countCornerInliers(*obs) >= min_inlier_corners)
                opt_quality = true;
        }

        // 第一层：至少一目检测有效，否则该 KF 不能作为参考
        if(!detect_valid)
            continue;

        Candidate c;
        c.key_frame = key_frame;
        c.resolved = resolved;
        c.opt_quality = opt_quality;
        c.n_tracked = key_frame->TrackedMapPoints(1);
        candidates.push_back(c);
    }

    if(candidates.empty())
        return nullptr;

    // 第二层：存在已消歧参考 KF 时，不再从未消歧 KF 中选择
    bool any_resolved = false;
    for(const Candidate &c : candidates)
    {
        if(c.resolved)
        {
            any_resolved = true;
            break;
        }
    }
    if(any_resolved)
    {
        candidates.erase(std::remove_if(candidates.begin(), candidates.end(),
                                        [](const Candidate &c) { return !c.resolved; }),
                         candidates.end());
    }
    else if(!allow_unresolved_fallback)
    {
        // 回环锚点：全部历史观测未消歧时不回退
        return nullptr;
    }

    // 第三层：若池中存在优化内点合格的 KF，则只在其中选择
    bool any_opt_quality = false;
    for(const Candidate &c : candidates)
    {
        if(c.opt_quality)
        {
            any_opt_quality = true;
            break;
        }
    }
    if(any_opt_quality)
    {
        candidates.erase(std::remove_if(candidates.begin(), candidates.end(),
                                        [](const Candidate &c) { return !c.opt_quality; }),
                         candidates.end());
    }

    // TODO 第四层：几何质量（IPPE 重投影误差、Tag 面积、左右目一致、
    // decision_margin_、hamming_）。当前空出，不参与排序。

    // 第五层：同等 Tag 质量下用 ORB 地图稳定性排序
    KeyFrame *best = nullptr;
    int best_tracked = -1;
    for(const Candidate &c : candidates)
    {
        if(c.n_tracked > best_tracked)
        {
            best_tracked = c.n_tracked;
            best = c.key_frame;
        }
    }
    return best;
}

}  // namespace tag
}  // namespace ORB_SLAM3

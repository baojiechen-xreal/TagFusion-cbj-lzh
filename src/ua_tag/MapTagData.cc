#include "ua_tag/MapTagData.h"

#include "KeyFrame.h"

#include <algorithm>
#include <vector>

namespace ORB_SLAM3 {
namespace xreal_tag {

namespace {

std::array<Eigen::Vector3f, 4> CornersFromPose(const Sophus::SE3f &T_wt,
                                               float tag_size)
{
    std::array<Eigen::Vector3f, 4> corners{};
    if(tag_size <= 0.0f)
        return corners;

    // Matches the OpenCV IPPE_SQUARE / BuildSquareObjectPoints convention
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
    obs.left_index = left_index;
    obs.right_index = right_index;
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
        for(bool outlier : obs.corner_outliers)
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
            observationAt(key_frame, kv.second.left_index, false),
            observationAt(key_frame, kv.second.right_index, true)};
        for(const TagObservation *obs : cams)
        {
            if(!obs || !obs->IsDetectValid())
                continue;
            detect_valid = true;

            // Layer 2: already disambiguated
            if(obs->IsAmbiguityResolved() && obs->pose_estimate.has_value() &&
               obs->pose_estimate->GetSelectedCandidate() != nullptr)
                resolved = true;

            // Layer 3: optimization-inlier quality
            if(!obs->is_optimization_outlier && countCornerInliers(*obs) >= min_inlier_corners)
                opt_quality = true;
        }

        // Layer 1: at least one eye must be detect-valid, otherwise this KF cannot be the reference
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

    // Layer 2: once a disambiguated reference KF exists, do not select from unresolved KFs
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
        // Loop anchor: do not fall back when every historical observation is unresolved
        return nullptr;
    }

    // Layer 3: if the pool contains a KF that passes the optimization-inlier test, select only among those KFs
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

    // TODO layer 4: geometric quality (IPPE reprojection error, Tag area, left/right agreement,
    // decision_margin, hamming). Currently left empty and not used for ranking.

    // Layer 5: when Tag quality is equal, rank by ORB map stability
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

}  // namespace xreal_tag
}  // namespace ORB_SLAM3

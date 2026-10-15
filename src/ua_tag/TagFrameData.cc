#include "ua_tag/TagFrameData.h"

#include <utility>

namespace ORB_SLAM3 {
namespace xreal_tag {

void TagFrameData::Add(TagObservation observation)
{
    if(observation.camera_id == CameraId::RIGHT)
        right_.push_back(std::move(observation));
    else
        left_.push_back(std::move(observation));
}

const TagObservation *TagFrameData::Find(int tag_id, CameraId camera_id) const noexcept
{
    const Observations &obs =
        (camera_id == CameraId::RIGHT) ? right_ : left_;

    for(const TagObservation &o : obs)
    {
        if(o.tag_id == tag_id)
            return &o;
    }
    return nullptr;
}

TagObservation *TagFrameData::Find(int tag_id, CameraId camera_id) noexcept
{
    Observations &obs = (camera_id == CameraId::RIGHT) ? right_ : left_;

    for(TagObservation &o : obs)
    {
        if(o.tag_id == tag_id)
            return &o;
    }
    return nullptr;
}

bool TagFrameData::WriteBackSelectedCandidate(int tag_id, CameraId camera_id,
                                              int selected_candidate,
                                              const Sophus::SE3f *t_ct)
{
    if(selected_candidate < 0 || selected_candidate >= 2)
        return false;

    TagObservation *obs = Find(tag_id, camera_id);
    if(!obs || !obs->pose_estimate.has_value())
        return false;

    TagPoseEstimate &est = *obs->pose_estimate;
    if(!est.candidates[selected_candidate].valid)
        return false;

    est.selected_candidate = selected_candidate;
    if(t_ct)
        est.candidates[selected_candidate].t_ct = *t_ct;
    return true;
}

void TagFrameData::Clear()
{
    left_.clear();
    right_.clear();
}

void TagFrameData::ResetOptimizationOutliers()
{
    for(TagObservation &obs : left_)
        obs.ResetOptimizationOutliers();
    for(TagObservation &obs : right_)
        obs.ResetOptimizationOutliers();
}

bool TagFrameData::IsEmpty() const noexcept
{
    return left_.empty() && right_.empty();
}

std::size_t TagFrameData::GetSize() const noexcept
{
    return left_.size() + right_.size();
}

}  // namespace xreal_tag
}  // namespace ORB_SLAM3

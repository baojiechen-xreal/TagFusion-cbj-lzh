#ifndef UA_TAG_TAG_FRAME_DATA_H
#define UA_TAG_TAG_FRAME_DATA_H

#include "ua_tag/TagObservation.h"

#include <cstddef>
#include <vector>

namespace ORB_SLAM3 {
namespace xreal_tag {

// Left and right Tag observations of one frame (attached by Frame / Tracking)
class TagFrameData
{
public:
    using Observations = std::vector<TagObservation>;

    Observations left_;
    Observations right_;

    void Add(TagObservation observation);

    const TagObservation *Find(int tag_id, CameraId camera_id) const noexcept;
    TagObservation *Find(int tag_id, CameraId camera_id) noexcept;

    // Write back disambiguation: update only selected_candidate and the chosen candidate t_ct (t_ct is optional)
    bool WriteBackSelectedCandidate(int tag_id, CameraId camera_id,
                                    int selected_candidate,
                                    const Sophus::SE3f *t_ct = nullptr);

    void Clear();

    // Reset optimization outliers of every Tag observation in this frame (call before repeated PoseOptimization on the same frame)
    void ResetOptimizationOutliers();

    bool IsEmpty() const noexcept;
    std::size_t GetSize() const noexcept;
};

}  // namespace xreal_tag
}  // namespace ORB_SLAM3

#endif

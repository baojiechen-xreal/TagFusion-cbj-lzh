#ifndef UA_TAG_TAG_OPTIMIZER_H
#define UA_TAG_TAG_OPTIMIZER_H

#include <memory>
#include <optional>
#include <unordered_map>

#include <sophus/se3.hpp>

namespace ORB_SLAM3 {

class Frame;

namespace xreal_tag {

class MapTagData;

// Tag optimization API (no dedicated thread; called synchronously by Tracking / TagTracker)
class TagOptimizer
{
public:
    using MapTagPtr = std::shared_ptr<MapTagData>;
    using TagContainer = std::unordered_map<int, MapTagPtr>;

    TagOptimizer() = delete;

    // Core Motion-only BA (GTSAM) API: uses each FIXED Tag and its world-frame 3D corners directly.
    //   1) World-frame corners of each FIXED Tag are Point3 states, pinned by NonlinearEquality
    //   2) Undistorted corners + GenericProjectionFactor<Pose3,Point3,Cal3_S2> (Huber)
    //   3) After optimization, mark outliers by chi2
    //   4) Optimize once more on inliers only, with Huber removed
    // The seed prefers Tcw_pred, otherwise the Pose already on the frame; on success SetPose and return true.
    // On failure, do not clear a Pose already on the frame (constant-velocity prediction stays as fallback) and return false.
    static bool OptimizePose(
        Frame &frame,
        const TagContainer &fixed_tags,
        const std::optional<Sophus::SE3f> &Tcw_pred = std::nullopt,
        bool verbose = false);

    // After single-frame Tag init, estimate the second-frame pose by reusing OptimizePose(TagContainer).
    // fixed_tags: each Tag whose world pose is held fixed for this optimization (usually from Result::tags; state may be ACTIVE).
    // The covisible Tag threshold is checked beforehand by TagInitializer::TryInitialize and is not checked again here.
    // If Tcw_pred is omitted and the frame has no Pose, seed Tcw from IPPE of each covisible Tag.
    // On success write Tcw_out (identical to frame.GetPose()); on failure leave Tcw_out unchanged.
    static bool OptimizePoseForSecondFrame(
        Frame &second_frame,
        const TagContainer &fixed_tags,
        Sophus::SE3f &Tcw_out,
        const std::optional<Sophus::SE3f> &Tcw_pred = std::nullopt,
        bool verbose = false);
};

}  // namespace xreal_tag
}  // namespace ORB_SLAM3

#endif

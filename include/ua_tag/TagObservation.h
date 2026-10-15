#ifndef UA_TAG_TAG_OBSERVATION_H
#define UA_TAG_TAG_OBSERVATION_H

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>

#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

namespace ORB_SLAM3 {
namespace xreal_tag {

enum class CameraId : std::uint8_t
{
    LEFT_OR_MONO = 0,
    RIGHT = 1
};

// Derived pose estimate: Tag -> Camera (OpenCV / IPPE convention)
struct TagPoseCandidate
{
    Sophus::SE3f t_ct;

    float reprojection_error = -1.0f;
    bool valid = false;
};

struct TagPoseEstimate
{
    // Stores valid IPPE solutions only; IPPE_SQUARE is usually 2
    std::array<TagPoseCandidate, 2> candidates{};

    // -1: unresolved; 0/1: index of the selected candidate
    int selected_candidate = -1;

    float ambiguity_ratio = -1.0f;

    // Whether IPPE ambiguity is resolved (selected_candidate is a valid index and that candidate has valid)
    bool IsAmbiguityResolved() const noexcept
    {
        return GetSelectedCandidate() != nullptr;
    }

    const TagPoseCandidate *GetSelectedCandidate() const noexcept
    {
        if(selected_candidate < 0 || selected_candidate >= 2)
            return nullptr;

        const TagPoseCandidate &candidate = candidates[selected_candidate];
        return candidate.valid ? &candidate : nullptr;
    }

    // Mirror IPPE left unselected after disambiguation (when the other candidate has valid)
    const TagPoseCandidate *GetUnselectedCandidate() const noexcept
    {
        if(selected_candidate < 0 || selected_candidate >= 2)
            return nullptr;

        const TagPoseCandidate &candidate = candidates[1 - selected_candidate];
        return candidate.valid ? &candidate : nullptr;
    }
};

// Raw detection; an empty pose_estimate means IPPE / disambiguation has not run
struct TagObservation
{
    int tag_id = -1;
    CameraId camera_id = CameraId::LEFT_OR_MONO;

    // Raw detected corners (not undistorted; same pixels as the input image)
    std::array<cv::Point2f, 4> corners_raw{};

    // Undistorted corners (written by DetectCorners; equal to corners_raw when distortion is zero)
    std::array<cv::Point2f, 4> corners_undistorted{};

    // Decode quality: mean margin of bit intensity above the decision threshold; larger is more reliable
    // When ethz has no such field, DetectCorners uses observed_perimeter as a coarse proxy
    float decision_margin = 0.0f;
    // Number of bit errors corrected while decoding; 0 is best, larger means a higher false-detection risk
    int hamming = 0;

    // Detection geometry quality (raw-image pixels): quad area and perimeter
    float observed_area = 0.0f;
    float observed_perimeter = 0.0f;

    // nullopt: not yet estimated; set: candidates have been computed (not necessarily disambiguated)
    std::optional<TagPoseEstimate> pose_estimate;

    // Permanently invalid at detection time (bad detection, etc.); snapshots, disambiguation, and mapping all read this flag
    bool is_detect_outlier = false;

    // Group-level outlier from the latest pose optimization (temporary); ResetOptimizationOutliers before the next PoseOptimization on the same frame
    bool is_optimization_outlier = false;

    // Per-corner optimization outliers (written by PoseOptimization / TagOptimizer; default all inliers)
    std::array<bool, 4> corner_outliers{{false, false, false, false}};

    // Detect-valid: usable for disambiguation, mapping, and joint-optimization snapshots (ignore optimization outliers)
    bool IsDetectValid() const noexcept
    {
        return tag_id >= 0 && !is_detect_outlier;
    }

    // Quad area in raw-image pixels (shoelace); returns 0 when corners are invalid
    static float ComputeQuadArea(const std::array<cv::Point2f, 4> &corners) noexcept
    {
        float area2 = 0.0f;
        for(int i = 0; i < 4; ++i)
        {
            const cv::Point2f &p0 = corners[static_cast<size_t>(i)];
            const cv::Point2f &p1 = corners[static_cast<size_t>((i + 1) % 4)];
            area2 += p0.x * p1.y - p1.x * p0.y;
        }
        return std::abs(area2) * 0.5f;
    }

    // Clear temporary outliers produced by optimization (does not touch is_detect_outlier)
    void ResetOptimizationOutliers() noexcept
    {
        is_optimization_outlier = false;
        corner_outliers = {{false, false, false, false}};
    }

    // Whether IPPE ambiguity has been resolved
    bool IsAmbiguityResolved() const noexcept
    {
        return pose_estimate.has_value() && pose_estimate->IsAmbiguityResolved();
    }
};

// Express the Tag pose from the observing camera frame in the left-camera frame:
//   Left / mono: T_cL_t = T_ct
//   Right: T_cL_t = T_lr * T_cR_t (ORB mTlr: P_left = T_lr * P_right)
// The left-camera Twc then yields the world pose: T_wt = Twc * T_cL_t
inline Sophus::SE3f ConvertTagPoseToLeftCamera(CameraId camera_id,
                                               const Sophus::SE3f &t_ct,
                                               const Sophus::SE3f &t_lr)
{
    if(camera_id == CameraId::RIGHT)
        return t_lr * t_ct;
    return t_ct;
}

}  // namespace xreal_tag
}  // namespace ORB_SLAM3

#endif

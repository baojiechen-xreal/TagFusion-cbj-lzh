#ifndef UA_TAG_TAG_INITIALIZER_H
#define UA_TAG_TAG_INITIALIZER_H

#include "Frame.h"
#include "ua_tag/MapTagData.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

namespace ORB_SLAM3 {

class GeometricCamera;
class KeyFrame;
class Map;

namespace xreal_tag {

class TagInitializer
{
public:
    using MapTagPtr = std::shared_ptr<MapTagData>;
    using TagContainer = std::unordered_map<int, MapTagPtr>;

    // Init mode: Tracking passes it explicitly from mSensor
    enum class InitializationMode : std::uint8_t
    {
        Monocular = 0,     // Two-frame path (covisibility check / single frame + second frame / joint two-frame)
        Stereo = 1         // A single frame is enough (STEREO / IMU_STEREO)
    };

    // Tag-map initialization result
    struct Result
    {
        // Tag map entities produced by initialization
        TagContainer tags;

        // Tag pose of each init frame (Tag world -> Camera), one-to-one with keyframes:
        //   Stereo: {I}; Monocular single-frame + second frame: {I, Tcw_2}; joint two-frame: {I, T_21}
        std::vector<Sophus::SE3f> Tcw_current;

        // Each init Frame (already SetPose; historical field; commit uses the real KeyFrame):
        //   Stereo: first frame only; Monocular: first + second, two frames
        std::vector<Frame> keyframes;

        // true: single-frame mapping (Stereo, or Monocular single-frame + second-frame motion-only);
        // false: joint two-frame disambiguation
        bool from_single_frame = false;
    };

    // Joint two-view IPPE disambiguation result
    struct TwoViewAmbiguityResult
    {
        Sophus::SE3f T_21;  // Relative pose from reference frame (1) to current frame (2)
        float mean_reproj_error = std::numeric_limits<float>::infinity();

        // Selected IPPE candidate index of each covisible Tag after disambiguation
        std::unordered_map<int, int> selected_ref_candidate;
        std::unordered_map<int, int> selected_cur_candidate;
        // Disambiguated Tag->Camera poses
        std::unordered_map<int, Sophus::SE3f> T_ct_ref;
        std::unordered_map<int, Sophus::SE3f> T_ct_cur;

        bool valid = false;
    };

    // Minimum shared Tag count required for two-frame init
    static constexpr std::size_t kMinCommonTagsForTwoFrame = 2;
    // Minimum mean corner pixel shift, relative to image width, required for two-frame init
    static constexpr float kMinMeanPixelDispRatio = 0.02f;
    // Max mean reprojection error (px) of the two-frame optimum
    static constexpr float kMaxMeanReprojErrorPx = 2.5f;
    // Min translation norm (m) of the two-frame relative pose
    static constexpr float kMinTranslationNormM = 0.07f;

public:
    // Read Tag.size and Tag.verbose from the settings yaml
    explicit TagInitializer(const std::string &settings_file);

    // Compute only: write Result and, when needed, write disambiguation/pose back to the Frame; do not write the Map or link a KeyFrame.
    // Must be called before new KeyFrame(Frame) so the KF copies the final TagFrameData.
    //   Stereo: ignore second_frame; single-frame mapping on first_frame only
    //   Monocular:
    //     0) Require the first/second covisible Tag count >= kMinCommonTagsForTwoFrame; otherwise fail
    //     1) Map first_frame as a single frame; on success, estimate second_frame with motion-only BA and register both frames
    //     2) On failure, run joint two-frame init with the covisibility already computed
    bool TryInitialize(Frame &first_frame, Frame &second_frame, Result &result,
                       InitializationMode mode);

    // Commit only: insert each MapTag from Result into the ORB Map and link it both ways with the real KeyFrame.
    // Must be called after the KeyFrame exists and AddKeyFrame, and before InsertKeyFrame.
    // On success SetTagInitialized(true); a mid-way failure rolls back each MapTag already inserted.
    bool CommitTagInitialization(const Result &result, Map *pMap,
                                 const std::vector<KeyFrame *> &vpKFs) const;

    // Align a single-frame tag init from "current left camera = Tag world" into the existing ORB world.
    // Requires result.from_single_frame==true, and each MapTag::GetPose() is still T_ct (Tag -> current left camera).
    // Transform: T_wt_orb = Tcw_orb.inverse() * T_ct; also write result.Tcw_current = {Tcw_orb}.
    // Does not modify the given Frame or the ORB camera pose.
    bool AlignResultToOrbWorld(Result &result, const Sophus::SE3f &Tcw_orb) const;

    // Single frame: success when the frame has an unambiguous/valid IPPE Tag count >= 3.
    // Called internally by the Stereo path and the Monocular single-frame subpath.
    bool TryInitializeSingleFrame(Frame &frame, Result &result);

private:
    // tag_id -> observation that already has two valid IPPE candidates (disambiguation not required)
    using TagObsByIdMap = std::unordered_map<int, const TagObservation *>;
    // tag_id -> unambiguous and valid IPPE observation
    using UnambiguousObsMap = std::unordered_map<int, const TagObservation *>;

    // Observation pair of the same tag_id in the two frames (cur = second, ref = first)
    struct CommonTagPair
    {
        const TagObservation *cur = nullptr;
        const TagObservation *ref = nullptr;
    };
    using CommonTagObsMap = std::unordered_map<int, CommonTagPair>;

    // Inlier, and pose_estimate contains two valid IPPE candidates
    static bool HasTwoValidIppeCandidates(const TagObservation &obs) noexcept;

    // Collect observations by tag_id; for the same id the left camera wins over the right (requires two valid IPPE candidates)
    static void CollectObservationById(TagObsByIdMap &by_id, const TagObservation &obs);
    static TagObsByIdMap CollectObservationsById(const Frame &frame);

    // Insert unambiguous valid observations; for the same tag_id the left camera wins over the right
    static void CollectUnambiguousObservation(UnambiguousObsMap &unambiguous,
                                              const TagObservation &obs);

    static UnambiguousObsMap CollectUnambiguousObservations(const Frame &frame);

    // Find Tag observation pairs that share an id between second and first
    static CommonTagObsMap FindCommonTagObservations(const Frame &frame,
                                                     const Frame &ref_frame);

    // Mean Euclidean pixel shift, between the two frames, of undistorted corners of each shared Tag
    static float ComputeMeanCornerPixelDisplacement(const CommonTagObsMap &common_obs);

    // RMSE of GeometricCamera::project vs corners_raw (raw-image pixels)
    static float ComputeTagCornerReprojRmse(const Sophus::SE3f &T_ct,
                                            const std::array<cv::Point2f, 4> &corners_obs,
                                            GeometricCamera *pCamera,
                                            const std::vector<cv::Point3f> &object_pts);

    // Joint two-view IPPE disambiguation (standalone flow):
    //   1) Enumerate the 2x2 combinations of every covisible Tag; pick the global T_21* by mean bidirectional reprojection
    //   2) Fix T_21*; for each covisible Tag try the 4 IPPE combinations, keep the minimum-error one, and write it back
    bool ResolveTwoViewIppeAmbiguity(const CommonTagObsMap &common_obs,
                                     GeometricCamera *pCamera,
                                     TwoViewAmbiguityResult &out) const;

    // After single-frame mapping succeeds: motion-only BA estimates the second_frame pose, then register both frames as in two-frame init
    // (Tcw_current={I,Tcw_2}, two keyframes, append the second-frame observations onto tags)
    bool CompleteSingleFrameInitializationWithSecondFrame(Frame &first_frame,
                                                Frame &second_frame,
                                                Result &result);

    // Two-frame: joint init of first_frame + second_frame (the caller precomputes common_obs)
    bool TryInitializeTwoFrames(Frame &frame, Frame &ref_frame,
                                const CommonTagObsMap &common_obs, Result &result);

    // Read from Tag.size / Tag.size_by_id in the settings yaml (meters)
    double tag_size_ = 0.16;
    std::map<int, double> tag_size_by_id_;

    double GetTagSize(int tag_id) const;

    // Read Tag.verbose from the settings yaml; when true, print initialization logs
    bool verbose_ = false;
};

}  // namespace xreal_tag
}  // namespace ORB_SLAM3

#endif

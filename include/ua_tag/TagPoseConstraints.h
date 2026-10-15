#ifndef UA_TAG_TAG_POSE_CONSTRAINTS_H
#define UA_TAG_TAG_POSE_CONSTRAINTS_H

#include "ua_tag/TagObservation.h"

#include <array>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace ORB_SLAM3 {

class Frame;
class Map;

namespace xreal_tag {

class MapTagData;

// Tag.enable: master switch. Default true; 0 skips detection, mapping, optimization, and Tag-driven KF insertion
bool ModuleEnabledFromSettings(const std::string &settings_path);

// Read-only snapshot: do not hold the Map/MapTag lock during optimization
struct TagPoseConstraint
{
    int tagId = -1;
    std::array<Eigen::Vector3d, 4> worldCorners{};
};

// Adaptive weights for Tag corner factors (shared yaml for PoseOpt / LocalBA / GBA / loop 4DoF)
// Ω_tag = (α * w_bar / σ²) I₂, w_bar = max(w_S w_θ w_amb, w_min)
// When Tag.factor_weight.enable=0, fall back to Ω = I/σ²
struct TagFactorWeightParams
{
    bool enable = false;
    bool verbose = false;
    float alpha = 3.0f;
    float s_sat = 400.0f;
    float lambda_s = 3.0f;
    float theta0_deg = 60.0f;
    float lambda_theta = 1.0f;
    float phi0_deg = 15.0f;
    float e0 = 1.0f;
    float lambda_amb = 0.5f;
    float w_min = 1e-3f;

    static TagFactorWeightParams FromSettings(const std::string &settings_path);
};

struct TagFactorWeight
{
    float area = 0.0f;
    float theta_rad = 0.0f;
    float dphi_rad = 0.0f;
    float de = 0.0f;
    float w_s = 1.0f;
    float w_theta = 1.0f;
    float w_amb = 1.0f;
    float w_obs = 1.0f;
    float w_bar = 1.0f;
    float alpha_w = 1.0f;
    bool has_theta = false;
    bool has_amb = false;
};

TagFactorWeight EvaluateTagFactorWeight(const TagObservation &obs,
                                        const TagFactorWeightParams &params);

// Summarize every detect-valid KF observation of a MapTag (same EvaluateTagFactorWeight as [TagW])
struct TagFactorWeightSummary
{
    int n_obs = 0;
    float mean_area = 0.0f;
    float mean_w_s = 1.0f;
    float mean_w_theta = 1.0f;
    float mean_w_amb = 1.0f;
    float mean_w_obs = 1.0f;
    float mean_w_bar = 1.0f;
    float min_w_bar = 1.0f;
    float max_w_bar = 0.0f;
    float mean_alpha_w = 1.0f;
    bool valid = false;
};

TagFactorWeightSummary SummarizeMapTagFactorWeight(
    const MapTagData &tag, const TagFactorWeightParams &params);

// enable=0: I/σ²; enable=1: (α w_bar / σ²) I. When verbose, log one line per observation (the caller invokes this once per observation)
Eigen::Matrix2d TagCornerInformation(float corner_sigma,
                                     const TagObservation &obs,
                                     const TagFactorWeightParams &params,
                                     const char *log_src = nullptr,
                                     unsigned long log_id = 0);

struct TagPoseOptParams
{
    bool enable = false;                  // Tag.pose_optimization: shared by PoseOptimization and PoseInertial*
    bool inertial_enable = false;         // Kept in sync with enable; takes effect only when a legacy yaml sets Tag.inertial_pose_optimization alone
    float corner_sigma = 2.0f;            // Shared by visual-only and inertial
    int min_inlier_corners = 3;           // Per camera, per Tag: keep it only if at least N/4 corners are inliers; shared
    bool fixed_only = true;               // Used only by the visual-only snapshot; the inertial path always keeps ACTIVE
    bool verbose = false;                 // Shared
    TagFactorWeightParams factor_weight;

    // Tag.pose_optimization controls both visual PoseOptimization and inertial PoseInertial*
    // (two code paths; only one runs). The legacy key Tag.inertial_pose_optimization is compatibility only.
    static TagPoseOptParams FromSettings(const std::string &settings_path);
};

// Tag rigid pose + four-corner reprojection in Local BA (independent of Tracking TagPoseOptParams)
struct TagLocalBAParams
{
    bool enable = false;             // Tag.local_ba.enable, off by default
    bool optimize_active = true;     // 1 = ACTIVE may be optimized; 0 = only FIXED_ANCHOR enters the graph
    float corner_sigma = 2.0f;
    int min_inlier_corners = 3;
    bool verbose = false;
    TagFactorWeightParams factor_weight;

    static TagLocalBAParams FromSettings(const std::string &settings_path);
};

// Tag loop closure: a detection branch independent of BoW that finally reuses the CorrectLoop pipeline
struct TagLoopParams
{
    bool enable = false;

    int min_history_observations = 2;
    int min_keyframe_gap = 30;
    int min_reobservation_gap_kfs = 30;     // Gap since the last valid observation; a smaller gap counts as continuously visible
    int min_historical_kf_map_points = 20;  // Map-point quality threshold for the historical KF

    float max_ippe_reprojection_error = 5.0f;
    float max_stereo_translation_error = 0.20f;
    float max_stereo_rotation_error = 0.20f;

    float max_roll_pitch_correction = 0.15f;
    float max_yaw_correction = 1.57f;

    int min_consistent_tags = 1;
    int reference_timeout_kfs = 4;  // Drop the frozen reference after this many KF without a sighting
    bool verbose = false;

    static TagLoopParams FromSettings(const std::string &settings_path);
};

struct TagLocalBAStats
{
    int num_tags = 0;
    int num_tag_edges = 0;
    int num_tag_outliers = 0;
};

// Before PoseOptimization / PoseInertial*, query MapTag and build a constraint snapshot aligned with the current-frame observations.
// Use only detect-valid observations (IsDetectValid); ignore is_optimization_outlier.
// The joint-optimization entry calls ResetOptimizationOutliers, so the same frame may add the Tag again.
// inertial=false: consult params.enable; filtering follows params.fixed_only.
// inertial=true: consult params.inertial_enable; ACTIVE only (fixed_only is not read).
std::vector<TagPoseConstraint> BuildTagMapSnapshot(
    Map *pMap, Frame *pFrame, const TagPoseOptParams &params,
    bool inertial = false);

}  // namespace xreal_tag
}  // namespace ORB_SLAM3

#endif

#include "ua_tag/TagInitializer.h"

#include "GeometricCamera.h"
#include "KeyFrame.h"
#include "Map.h"
#include "ua_tag/AprilTagDetector.h"
#include "ua_tag/TagOptimizer.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <utility>

namespace ORB_SLAM3 {
namespace xreal_tag {

namespace {

void LogFrameTagAmbiguityRatios(const Frame &frame, const char *label)
{
    auto log_obs = [&](const TagObservation &obs, const char *cam) {
        if(!obs.IsDetectValid())
            return;

        int selected = -1;
        float ambiguity_ratio = -1.0f;
        int cand0_valid = 0;
        int cand1_valid = 0;
        if(obs.pose_estimate.has_value())
        {
            const TagPoseEstimate &est = *obs.pose_estimate;
            selected = est.selected_candidate;
            ambiguity_ratio = est.ambiguity_ratio;
            cand0_valid = est.candidates[0].valid ? 1 : 0;
            cand1_valid = est.candidates[1].valid ? 1 : 0;
        }

        std::cout << "[TagInitializer] " << label
                  << " frame_id=" << frame.mnId
                  << " tag_id=" << obs.tag_id
                  << " cam=" << cam
                  << " selected=" << selected
                  << " ambiguity_ratio=" << ambiguity_ratio
                  << " cand0_valid=" << cand0_valid
                  << " cand1_valid=" << cand1_valid
                  << std::endl;
    };

    for(const TagObservation &obs : frame.tag_frame_data_.left_)
        log_obs(obs, "left");
    for(const TagObservation &obs : frame.tag_frame_data_.right_)
        log_obs(obs, "right");
}

}  // namespace

TagInitializer::TagInitializer(const std::string &settings_file)
{
    cv::FileStorage fs(settings_file, cv::FileStorage::READ);
    if(!fs.isOpened())
        return;

    cv::FileNode node = fs["Tag.size"];
    if(!node.empty())
        tag_size_ = static_cast<double>(node);

    node = fs["Tag.size_by_id"];
    if(!node.empty() && node.isSeq())
    {
        for(cv::FileNodeIterator it = node.begin(); it != node.end(); ++it)
        {
            const cv::FileNode item = *it;
            const cv::FileNode id_node = item["id"];
            const cv::FileNode size_node = item["size"];
            if(id_node.empty() || size_node.empty())
                continue;
            const int id = static_cast<int>(id_node);
            const double sz = static_cast<double>(size_node);
            if(id < 0 || sz <= 0.0)
                continue;
            tag_size_by_id_[id] = sz;
        }
    }

    node = fs["Tag.verbose"];
    if(!node.empty())
        verbose_ = static_cast<int>(node) != 0;
}

double TagInitializer::GetTagSize(int tag_id) const
{
    const auto it = tag_size_by_id_.find(tag_id);
    if(it != tag_size_by_id_.end() && it->second > 0.0)
        return it->second;
    return tag_size_;
}

bool TagInitializer::HasTwoValidIppeCandidates(const TagObservation &obs) noexcept
{
    if(!obs.IsDetectValid() || !obs.pose_estimate.has_value())
        return false;

    const TagPoseEstimate &est = *obs.pose_estimate;
    return est.candidates[0].valid && est.candidates[1].valid;
}

// Collect observations by tag_id; for the same id the left camera wins over the right (requires two valid IPPE candidates)
void TagInitializer::CollectObservationById(TagObsByIdMap &by_id,
                                            const TagObservation &obs)
{
    if(!HasTwoValidIppeCandidates(obs))
        return;

    auto it = by_id.find(obs.tag_id);
    if(it == by_id.end())
    {
        by_id.emplace(obs.tag_id, &obs);
        return;
    }

    // When an observation already exists: the left camera wins over the right; the same camera keeps the entry written first
    const TagObservation &prev = *it->second;
    if(prev.camera_id == CameraId::LEFT_OR_MONO)
        return;
    if(obs.camera_id == CameraId::LEFT_OR_MONO)
        it->second = &obs;
}

// Collect observations of the reference frame and the current frame by tag_id; for the same id the left camera wins over the right (IPPE need not be unambiguous)
TagInitializer::TagObsByIdMap TagInitializer::CollectObservationsById(const Frame &frame)
{
    TagObsByIdMap by_id;
    for(const TagObservation &obs : frame.tag_frame_data_.left_)
        CollectObservationById(by_id, obs);
    for(const TagObservation &obs : frame.tag_frame_data_.right_)
        CollectObservationById(by_id, obs);
    return by_id;
}

// Collect one unambiguous valid IPPE observation per tag_id; for the same id the left camera wins over the right
void TagInitializer::CollectUnambiguousObservation(UnambiguousObsMap &unambiguous,
                                                   const TagObservation &obs)
{
    // Unambiguous valid IPPE solution: already disambiguated, and the selected candidate is valid
    if(!obs.IsDetectValid() || !obs.pose_estimate.has_value() ||
       obs.pose_estimate->GetSelectedCandidate() == nullptr)
        return;

    auto it = unambiguous.find(obs.tag_id);
    if(it == unambiguous.end())
    {
        unambiguous.emplace(obs.tag_id, &obs);
        return;
    }

    // When an observation already exists: the left camera wins over the right; the same camera keeps the entry written first
    const TagObservation &prev = *it->second;
    if(prev.camera_id == CameraId::LEFT_OR_MONO)
        return;
    if(obs.camera_id == CameraId::LEFT_OR_MONO)
        it->second = &obs;
}

// Collect unambiguous valid IPPE observations by tag_id; for the same id the left camera wins over the right
TagInitializer::UnambiguousObsMap TagInitializer::CollectUnambiguousObservations(
    const Frame &frame)
{
    UnambiguousObsMap unambiguous;
    for(const TagObservation &obs : frame.tag_frame_data_.left_)
        CollectUnambiguousObservation(unambiguous, obs);
    for(const TagObservation &obs : frame.tag_frame_data_.right_)
        CollectUnambiguousObservation(unambiguous, obs);
    return unambiguous;
}

TagInitializer::CommonTagObsMap TagInitializer::FindCommonTagObservations(
    const Frame &frame, const Frame &ref_frame)
{
    // Collect observations of the current frame and the reference frame (indexed by id)
    const TagObsByIdMap cur_by_id = CollectObservationsById(frame);
    const TagObsByIdMap ref_by_id = CollectObservationsById(ref_frame);

    CommonTagObsMap common;
    for(const auto &kv : cur_by_id)
    {
        const auto ref_it = ref_by_id.find(kv.first);
        if(ref_it == ref_by_id.end())
            continue;

        CommonTagPair pair;
        pair.cur = kv.second;
        pair.ref = ref_it->second;
        common.emplace(kv.first, pair);
    }
    return common;
}

float TagInitializer::ComputeMeanCornerPixelDisplacement(const CommonTagObsMap &common_obs)
{
    double sum = 0.0;
    std::size_t count = 0;

    for(const auto &kv : common_obs)
    {
        const CommonTagPair &pair = kv.second;
        if(!pair.cur || !pair.ref)
            continue;

        const auto &corners_cur = pair.cur->corners_undistorted;
        const auto &corners_ref = pair.ref->corners_undistorted;

        for(int i = 0; i < 4; ++i)
        {
            const float dx = corners_cur[i].x - corners_ref[i].x;
            const float dy = corners_cur[i].y - corners_ref[i].y;
            sum += std::sqrt(static_cast<double>(dx * dx + dy * dy));
            ++count;
        }
    }

    if(count == 0)
        return 0.0f;

    return static_cast<float>(sum / static_cast<double>(count));
}

float TagInitializer::ComputeTagCornerReprojRmse(
    const Sophus::SE3f &T_ct,
    const std::array<cv::Point2f, 4> &corners_obs,
    GeometricCamera *pCamera,
    const std::vector<cv::Point3f> &object_pts)
{
    if(!pCamera || object_pts.size() != 4)
        return std::numeric_limits<float>::infinity();

    double sum_sq = 0.0;
    for(int i = 0; i < 4; ++i)
    {
        const Eigen::Vector3f P_t(object_pts[i].x, object_pts[i].y, object_pts[i].z);
        const Eigen::Vector3f P_c = T_ct * P_t;
        if(P_c.z() <= 0.0f)
            return std::numeric_limits<float>::infinity();

        // GeometricCamera::project: for fisheye this is distorted raw-image pixels, consistent with corners_raw / ORB BA
        const cv::Point2f uv = pCamera->project(cv::Point3f(P_c.x(), P_c.y(), P_c.z()));
        const double dx = static_cast<double>(uv.x - corners_obs[i].x);
        const double dy = static_cast<double>(uv.y - corners_obs[i].y);
        sum_sq += dx * dx + dy * dy;
    }

    return static_cast<float>(std::sqrt(sum_sq / 4.0));
}

bool TagInitializer::ResolveTwoViewIppeAmbiguity(const CommonTagObsMap &common_obs,
                                                GeometricCamera *pCamera,
                                                TwoViewAmbiguityResult &out) const
{
    out = TwoViewAmbiguityResult{};
    if(!pCamera || tag_size_ <= 0.0 || common_obs.empty())
        return false;

    // With T_21 fixed, the mean of the forward and backward reprojection errors of one (i_ref, i_cur) pair (side length looked up by tag_id)
    auto ComputePairReprojError =
        [&](const Sophus::SE3f &T_21, const Sophus::SE3f &T_12,
            const TagObservation &obs_ref, const TagObservation &obs_cur,
            int i_ref, int i_cur) -> float {
        const TagPoseEstimate &est_ref = *obs_ref.pose_estimate;
        const TagPoseEstimate &est_cur = *obs_cur.pose_estimate;
        if(!est_ref.candidates[i_ref].valid || !est_cur.candidates[i_cur].valid)
            return std::numeric_limits<float>::infinity();

        std::vector<cv::Point3f> object_pts;
        xreal_tag::BuildSquareObjectPoints(GetTagSize(obs_ref.tag_id), object_pts);

        // Observations use corners_raw, the same domain as GeometricCamera::project (raw-image pixels / KB)
        const Sophus::SE3f T_c2t_pred = T_21 * est_ref.candidates[i_ref].t_ct;
        const float err_12 = ComputeTagCornerReprojRmse(
            T_c2t_pred, obs_cur.corners_raw, pCamera, object_pts);

        const Sophus::SE3f T_c1t_pred = T_12 * est_cur.candidates[i_cur].t_ct;
        const float err_21 = ComputeTagCornerReprojRmse(
            T_c1t_pred, obs_ref.corners_raw, pCamera, object_pts);

        if(!std::isfinite(err_12) || !std::isfinite(err_21))
            return std::numeric_limits<float>::infinity();

        return 0.5f * (err_12 + err_21);
    };

    // With T_21 fixed: each covisible Tag takes the minimum bidirectional error among the 4 combinations, then those minima are averaged
    auto EvaluateMeanReprojError = [&](const Sophus::SE3f &T_21,
                                       float &mean_err) -> bool {
        const Sophus::SE3f T_12 = T_21.inverse();
        double sum_min_err = 0.0;
        std::size_t n_tags = 0;

        for(const auto &kv : common_obs)
        {
            const TagObservation &obs_ref = *kv.second.ref;
            const TagObservation &obs_cur = *kv.second.cur;

            float min_err = std::numeric_limits<float>::infinity();
            for(int i_ref = 0; i_ref < 2; ++i_ref)
            {
                for(int i_cur = 0; i_cur < 2; ++i_cur)
                {
                    const float err = ComputePairReprojError(
                        T_21, T_12, obs_ref, obs_cur, i_ref, i_cur);
                    if(err < min_err)
                        min_err = err;
                }
            }

            if(!std::isfinite(min_err))
                return false;

            sum_min_err += static_cast<double>(min_err);
            ++n_tags;
        }

        if(n_tags == 0)
            return false;

        mean_err = static_cast<float>(sum_min_err / static_cast<double>(n_tags));
        return true;
    };

    // 1) Enumerate T_21 over the 2x2 combinations of every covisible Tag, pick the one with the smallest global mean reprojection, and fix T_21
    float best_mean_err = std::numeric_limits<float>::infinity();
    Sophus::SE3f best_T21;

    for(const auto &kv : common_obs)
    {
        const TagPoseEstimate &est_ref = *kv.second.ref->pose_estimate;
        const TagPoseEstimate &est_cur = *kv.second.cur->pose_estimate;

        for(int i_ref = 0; i_ref < 2; ++i_ref)
        {
            if(!est_ref.candidates[i_ref].valid)
                continue;
            const Sophus::SE3f &T_c1t = est_ref.candidates[i_ref].t_ct;

            for(int i_cur = 0; i_cur < 2; ++i_cur)
            {
                if(!est_cur.candidates[i_cur].valid)
                    continue;
                const Sophus::SE3f &T_c2t = est_cur.candidates[i_cur].t_ct;
                const Sophus::SE3f T_21 = T_c2t * T_c1t.inverse();

                float mean_err = std::numeric_limits<float>::infinity();
                if(!EvaluateMeanReprojError(T_21, mean_err))
                    continue;

                if(mean_err < best_mean_err)
                {
                    best_mean_err = mean_err;
                    best_T21 = T_21;
                }
            }
        }
    }

    if(!std::isfinite(best_mean_err))
        return false;

    out.T_21 = best_T21;
    out.mean_reproj_error = best_mean_err;

    const float translation_norm = out.T_21.translation().norm();
    if(verbose_)
        std::cout << "[TagInitializer] best two-view solution"
                  << " mean_reproj_error=" << out.mean_reproj_error << " px"
                  << " (need < " << kMaxMeanReprojErrorPx << " px)"
                  << ", |t|=" << translation_norm << " m"
                  << " (need > " << kMinTranslationNormM << " m)"
                  << std::endl;

    if(out.mean_reproj_error >= kMaxMeanReprojErrorPx ||
       translation_norm <= kMinTranslationNormM)
    {
        if(verbose_)
            std::cout << "[TagInitializer] TryInitializeTwoFrames failed"
                      << " (best solution rejected"
                      << ", mean_reproj_error=" << out.mean_reproj_error << " px"
                      << ", |t|=" << translation_norm << " m)"
                      << std::endl;
        return false;
    }

    // 2) Fix the global optimum T_21* and, for each covisible Tag, try the 4 IPPE combinations;
    //    write back the combination with the smallest sum of bidirectional reprojection errors as the disambiguation result
    const Sophus::SE3f T_12 = out.T_21.inverse();
    for(const auto &kv : common_obs)
    {
        const int tag_id = kv.first;
        const TagObservation &obs_ref = *kv.second.ref;
        const TagObservation &obs_cur = *kv.second.cur;
        const TagPoseEstimate &est_ref = *obs_ref.pose_estimate;
        const TagPoseEstimate &est_cur = *obs_cur.pose_estimate;

        float best_err = std::numeric_limits<float>::infinity();
        int best_i_ref = -1;
        int best_i_cur = -1;

        for(int i_ref = 0; i_ref < 2; ++i_ref)
        {
            for(int i_cur = 0; i_cur < 2; ++i_cur)
            {
                const float err = ComputePairReprojError(
                    out.T_21, T_12, obs_ref, obs_cur, i_ref, i_cur);
                if(err < best_err)
                {
                    best_err = err;
                    best_i_ref = i_ref;
                    best_i_cur = i_cur;
                }
            }
        }

        if(best_i_ref < 0 || best_i_cur < 0 || !std::isfinite(best_err))
            continue;

        out.selected_ref_candidate[tag_id] = best_i_ref;
        out.selected_cur_candidate[tag_id] = best_i_cur;
        out.T_ct_ref.emplace(tag_id, est_ref.candidates[best_i_ref].t_ct);
        out.T_ct_cur.emplace(tag_id, est_cur.candidates[best_i_cur].t_ct);
    }

    out.valid = !out.T_ct_ref.empty();
    return out.valid;
}

bool TagInitializer::TryInitializeSingleFrame(Frame &frame, Result &result)
{
    constexpr std::size_t kMinUnambiguousTagsForSingleFrame = 3;

    if(verbose_)
        std::cout << "[TagInitializer] TryInitializeSingleFrame running"
                  << " (frame_id=" << frame.mnId << ")" << std::endl;

    LogFrameTagAmbiguityRatios(frame, "init_single");

    // Collect unambiguous valid observations
    const UnambiguousObsMap unambiguous = CollectUnambiguousObservations(frame);

    if(verbose_)
    {
        std::cout << "[TagInitializer] unambiguous valid observations: "
                  << unambiguous.size()
                  << " (need >= " << kMinUnambiguousTagsForSingleFrame << ")"
                  << " ids=[";
        bool first = true;
        for(const auto &kv : unambiguous)
        {
            if(!first)
                std::cout << ", ";
            std::cout << kv.first;
            first = false;
        }
        std::cout << "]" << std::endl;
    }

    // Too few unambiguous observations
    if(unambiguous.size() < kMinUnambiguousTagsForSingleFrame)
    {
        if(verbose_)
            std::cout << "[TagInitializer] TryInitializeSingleFrame failed"
                      << " (insufficient unambiguous observations)" << std::endl;
        return false;
    }

    // Tag world := left-camera frame of first_frame => Tcw = I
    // T_wt = T_cL_t; a right-camera observation is transformed to the left camera by T_lr first
    Result out;
    out.Tcw_current = {Sophus::SE3f()};
    out.from_single_frame = true;

    const Sophus::SE3f T_lr = frame.GetRelativePoseTlr();

    // Walk the unambiguous observations, build the Tag init result, and store the observations (always use Frame::mnId)
    for(const auto &kv : unambiguous)
    {
        const int tag_id = kv.first;
        const TagObservation &obs = *kv.second;
        const Sophus::SE3f &T_ct = obs.pose_estimate->GetSelectedCandidate()->t_ct;
        const Sophus::SE3f T_wt =
            ConvertTagPoseToLeftCamera(obs.camera_id, T_ct, T_lr);

        MapTagPtr map_tag = std::make_shared<MapTagData>();
        map_tag->SetId(tag_id);
        map_tag->SetTagSize(static_cast<float>(GetTagSize(tag_id)));
        map_tag->SetPose(T_wt);
        if(const TagPoseCandidate *mir = obs.pose_estimate->GetUnselectedCandidate())
        {
            map_tag->SetMirrorPose(
                ConvertTagPoseToLeftCamera(obs.camera_id, mir->t_ct, T_lr));
        }
        // World pose estimated by the system: ACTIVE (later LBA may optimize it). FIXED_ANCHOR is reserved for an external trusted Tag map.
        map_tag->SetState(MapTagState::ACTIVE);
        // Observation corners stay in Frame/KeyFrame::tag_frame_data_; they are not copied onto the MapTag

        out.tags.emplace(tag_id, std::move(map_tag));
    }

    // Register first_frame first; second_frame is appended by CompleteSingleFrameInitializationWithSecondFrame
    Frame kf = frame;
    kf.SetPose(out.Tcw_current.front());
    out.keyframes.push_back(std::move(kf));

    result = std::move(out);

    if(verbose_)
        std::cout << "[TagInitializer] TryInitializeSingleFrame succeeded"
                  << " (tags=" << result.tags.size() << ")" << std::endl;

    return true;
}

bool TagInitializer::CompleteSingleFrameInitializationWithSecondFrame(Frame &first_frame,
                                                            Frame &second_frame,
                                                            Result &result)
{
    if(verbose_)
        std::cout << "[TagInitializer] CompleteSingleFrameInitializationWithSecondFrame"
                  << " (second_frame_id=" << second_frame.mnId << ")" << std::endl;

    if(result.tags.empty() || result.Tcw_current.empty() || result.keyframes.empty())
        return false;

    for(auto &kv : result.tags)
    {
        if(kv.second && kv.second->GetTagSize() <= 0.0f)
            kv.second->SetTagSize(static_cast<float>(GetTagSize(kv.first)));
    }

    Sophus::SE3f Tcw_second;
    if(!TagOptimizer::OptimizePoseForSecondFrame(
           second_frame, result.tags, Tcw_second, std::nullopt, verbose_))
    {
        if(verbose_)
            std::cout << "[TagInitializer] CompleteSingleFrameInitializationWithSecondFrame failed"
                      << " (second-frame motion-only BA failed)" << std::endl;
        return false;
    }

    // Observations stay in second_frame.tag_frame_data_; the frozen path does not copy them into MapTagData

    // Same as two-frame init: both Tcw_current and keyframes hold two frames
    result.Tcw_current = {Sophus::SE3f(), Tcw_second};
    result.keyframes.clear();

    Frame kf_ref = first_frame;
    kf_ref.SetPose(result.Tcw_current[0]);
    Frame kf_cur = second_frame;
    kf_cur.SetPose(result.Tcw_current[1]);
    result.keyframes.push_back(std::move(kf_ref));
    result.keyframes.push_back(std::move(kf_cur));

    if(verbose_)
        std::cout << "[TagInitializer] CompleteSingleFrameInitializationWithSecondFrame succeeded"
                  << " (keyframes=" << result.keyframes.size() << ")" << std::endl;

    return true;
}

bool TagInitializer::TryInitializeTwoFrames(Frame &frame, Frame &ref_frame,
                                            const CommonTagObsMap &common_obs,
                                            Result &result)
{
    if(verbose_)
        std::cout << "[TagInitializer] TryInitializeTwoFrames running"
                  << " (frame_id=" << frame.mnId
                  << ", ref_frame_id=" << ref_frame.mnId
                  << ", common_tags=" << common_obs.size() << ")" << std::endl;

    LogFrameTagAmbiguityRatios(ref_frame, "init_two_ref");
    LogFrameTagAmbiguityRatios(frame, "init_two_cur");

    // Mean pixel shift of the undistorted corners of the shared observations
    const float mean_pixel_disp = ComputeMeanCornerPixelDisplacement(common_obs);

    if(verbose_)
        std::cout << "[TagInitializer] mean corner pixel displacement: "
                  << mean_pixel_disp << " px" << std::endl;

    // Two-frame init fails when parallax is too small (relative to image width)
    const float image_width = Frame::mnMaxX - Frame::mnMinX;
    const float min_pixel_disp = kMinMeanPixelDispRatio * image_width;
    if(image_width <= 0.0f || mean_pixel_disp < min_pixel_disp)
    {
        if(verbose_)
            std::cout << "[TagInitializer] TryInitializeTwoFrames failed"
                      << " (insufficient parallax"
                      << ", mean_pixel_disp=" << mean_pixel_disp
                      << " px, need >= " << min_pixel_disp << " px)"
                      << std::endl;
        return false;
    }

    GeometricCamera *pCamera = frame.mpCamera;
    if(!pCamera)
        pCamera = ref_frame.mpCamera;
    if(!pCamera)
        return false;

    TwoViewAmbiguityResult ambiguity;
    if(!ResolveTwoViewIppeAmbiguity(common_obs, pCamera, ambiguity))
        return false;

    // Tag world := camera frame of first_frame (ref); pose sequence {I, T_21}
    Result out;
    out.Tcw_current = {Sophus::SE3f(), ambiguity.T_21};
    out.from_single_frame = false;

    // Walk the disambiguated Tag->Camera poses and write the init result (always use Frame::mnId)
    for(const auto &kv : ambiguity.T_ct_ref)
    {
        const int tag_id = kv.first;
        const auto common_it = common_obs.find(tag_id);
        if(common_it == common_obs.end() || !common_it->second.ref ||
           !common_it->second.cur)
            continue;

        const auto cur_pose_it = ambiguity.T_ct_cur.find(tag_id);
        if(cur_pose_it == ambiguity.T_ct_cur.end())
            continue;

        // IPPE indices of the reference frame and the current frame for the given Tag after disambiguation
        const auto sel_ref_it = ambiguity.selected_ref_candidate.find(tag_id);
        const auto sel_cur_it = ambiguity.selected_cur_candidate.find(tag_id);
        if(sel_ref_it == ambiguity.selected_ref_candidate.end() ||
           sel_cur_it == ambiguity.selected_cur_candidate.end())
            continue;

        const int i_ref = sel_ref_it->second;
        const int i_cur = sel_cur_it->second;
        const Sophus::SE3f &T_c1t = kv.second;
        const Sophus::SE3f &T_c2t = cur_pose_it->second;

        // Write back into Frame.tag_frame_data_: update only selected_candidate and the selected candidate solution
        ref_frame.tag_frame_data_.WriteBackSelectedCandidate(
            tag_id, common_it->second.ref->camera_id, i_ref, &T_c1t);
        frame.tag_frame_data_.WriteBackSelectedCandidate(
            tag_id, common_it->second.cur->camera_id, i_cur, &T_c2t);

        // Take the observation from the Frame after write-back and register it on MapTagData
        const TagObservation *obs_ref_ptr =
            ref_frame.tag_frame_data_.Find(tag_id, common_it->second.ref->camera_id);
        const TagObservation *obs_cur_ptr =
            frame.tag_frame_data_.Find(tag_id, common_it->second.cur->camera_id);
        if(!obs_ref_ptr || !obs_cur_ptr)
            continue;

        // world := left camera of ref; right-camera IPPE is transformed to the left camera before T_wt is written
        const Sophus::SE3f T_wt = ConvertTagPoseToLeftCamera(
            obs_ref_ptr->camera_id, T_c1t, ref_frame.GetRelativePoseTlr());

        MapTagPtr map_tag = std::make_shared<MapTagData>();
        map_tag->SetId(tag_id);
        map_tag->SetTagSize(static_cast<float>(GetTagSize(tag_id)));
        map_tag->SetPose(T_wt);
        if(obs_ref_ptr->pose_estimate.has_value())
        {
            if(const TagPoseCandidate *mir = obs_ref_ptr->pose_estimate->GetUnselectedCandidate())
            {
                map_tag->SetMirrorPose(ConvertTagPoseToLeftCamera(
                    obs_ref_ptr->camera_id, mir->t_ct,
                    ref_frame.GetRelativePoseTlr()));
            }
        }
        // World pose estimated by the system: ACTIVE (later LBA may optimize it). FIXED_ANCHOR is reserved for an external trusted Tag map.
        map_tag->SetState(MapTagState::ACTIVE);
        out.tags.emplace(tag_id, std::move(map_tag));
    }

    if(out.tags.empty())
        return false;

    // Reference frame (Tcw=I) + current frame (Tcw=T_21), for the later commit of the real KF
    // selected_candidate of the matching Tag in Frame.tag_frame_data_ has already been written back
    Frame kf_ref = ref_frame;
    kf_ref.SetPose(out.Tcw_current[0]);
    Frame kf_cur = frame;
    kf_cur.SetPose(out.Tcw_current[1]);
    out.keyframes.push_back(std::move(kf_ref));
    out.keyframes.push_back(std::move(kf_cur));

    result = std::move(out);
    return true;
}

bool TagInitializer::TryInitialize(Frame &first_frame, Frame &second_frame,
                                   Result &result, InitializationMode mode)
{
    // ---------- Stereo ----------
    // Stereo: metric scale is already available, so map from a single frame only (ignore second_frame)
    if(mode == InitializationMode::Stereo)
    {
        if(verbose_)
            std::cout << "[TagInitializer] TryInitialize Stereo"
                      << " (frame_id=" << first_frame.mnId << ")" << std::endl;
        return TryInitializeSingleFrame(first_frame, result);
    }

    // ---------- Monocular ----------
    // 0) First check the covisible Tag set on the two ORB-init frames; if there are too few, the whole initialization fails
    const CommonTagObsMap common_obs =
        FindCommonTagObservations(second_frame, first_frame);

    if(verbose_)
        std::cout << "[TagInitializer] common tag observations: "
                  << common_obs.size()
                  << " (need >= " << kMinCommonTagsForTwoFrame << ")"
                  << std::endl;

    if(common_obs.size() < kMinCommonTagsForTwoFrame)
    {
        if(verbose_)
            std::cout << "[TagInitializer] TryInitialize failed"
                      << " (insufficient common tags between ORB init frames)"
                      << std::endl;
        return false;
    }

    // 1) Single-frame mapping + motion-only estimate of the second frame (Result::tags world corners are temporary anchors inside)
    if(TryInitializeSingleFrame(first_frame, result))
    {
        if(CompleteSingleFrameInitializationWithSecondFrame(first_frame, second_frame, result))
            return true;

        // Second-frame pose failed: clear the partial result and fall back to joint two-frame init
        result = Result{};
    }

    // 2) Reuse the covisibility already computed and run joint two-frame init
    return TryInitializeTwoFrames(second_frame, first_frame, common_obs, result);
}

bool TagInitializer::AlignResultToOrbWorld(Result &result,
                                           const Sophus::SE3f &Tcw_orb) const
{
    if(result.tags.empty() || !result.from_single_frame)
        return false;

    const Sophus::SE3f Twc_orb = Tcw_orb.inverse();
    for(auto &kv : result.tags)
    {
        if(!kv.second || !kv.second->HasPose())
            return false;

        // TryInitializeSingleFrame wrote T_ct (Tag world := current left camera)
        const Sophus::SE3f T_ct = kv.second->GetPose();
        kv.second->SetPose(Twc_orb * T_ct);
        if(kv.second->HasMirrorPose())
        {
            const Sophus::SE3f T_ct_m = kv.second->GetMirrorPose();
            kv.second->SetMirrorPose(Twc_orb * T_ct_m);
        }
    }

    result.Tcw_current = {Tcw_orb};

    if(verbose_)
        std::cout << "[TagInitializer] AlignResultToOrbWorld succeeded"
                  << " tags=" << result.tags.size() << std::endl;

    return true;
}

bool TagInitializer::CommitTagInitialization(const Result &result, Map *pMap,
                                             const std::vector<KeyFrame *> &vpKFs) const
{
    if(!pMap || result.tags.empty() || vpKFs.empty())
        return false;

    std::vector<int> inserted_tag_ids;
    inserted_tag_ids.reserve(result.tags.size());

    auto rollback = [&]() {
        for(int id : inserted_tag_ids)
            pMap->EraseMapTag(id);
        pMap->SetTagInitialized(false);
    };

    // 1) The Map owns the MapTag
    for(const auto &kv : result.tags)
    {
        if(!kv.second)
            continue;
        if(!pMap->AddMapTag(kv.second))
        {
            if(verbose_)
                std::cout << "[TagInitializer] CommitTagInitialization failed"
                          << " AddMapTag tag_id=" << kv.first << std::endl;
            rollback();
            return false;
        }
        inserted_tag_ids.push_back(kv.first);
    }

    // 2) Bidirectional KeyFrame <-> MapTag association (indices come from KF.tag_frame_data_)
    for(KeyFrame *pKF : vpKFs)
    {
        if(!pKF || pKF->GetMap() != pMap)
            continue;

        for(const auto &kv : result.tags)
        {
            const int tag_id = kv.first;
            MapTagData *pTag = kv.second.get();
            if(!pTag)
                continue;

            int left_idx = -1;
            int right_idx = -1;
            for(int i = 0; i < static_cast<int>(pKF->tag_frame_data_.left_.size()); ++i)
            {
                if(pKF->tag_frame_data_.left_[i].tag_id == tag_id)
                {
                    left_idx = i;
                    break;
                }
            }
            for(int i = 0; i < static_cast<int>(pKF->tag_frame_data_.right_.size()); ++i)
            {
                if(pKF->tag_frame_data_.right_[i].tag_id == tag_id)
                {
                    right_idx = i;
                    break;
                }
            }

            if(left_idx < 0 && right_idx < 0)
                continue;

            if(!pKF->AddMapTag(pTag, left_idx, right_idx))
            {
                if(verbose_)
                    std::cout << "[TagInitializer] CommitTagInitialization failed"
                              << " KF AddMapTag tag_id=" << tag_id
                              << " kf_id=" << pKF->mnId << std::endl;
                rollback();
                return false;
            }
        }
    }

    std::string check_err;
    if(!pMap->CheckMapTagAssociations(&check_err))
    {
        if(verbose_)
            std::cout << "[TagInitializer] CommitTagInitialization consistency failed: "
                      << check_err << std::endl;
        rollback();
        return false;
    }

    pMap->SetTagInitialized(true);

    if(verbose_)
        std::cout << "[TagInitializer] CommitTagInitialization succeeded"
                  << " tags=" << result.tags.size()
                  << " kfs=" << vpKFs.size() << std::endl;

    return true;
}

}  // namespace xreal_tag
}  // namespace ORB_SLAM3

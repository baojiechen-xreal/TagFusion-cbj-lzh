#ifndef UA_TAG_TAG_OBSERVATION_H
#define UA_TAG_TAG_OBSERVATION_H

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>

#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

namespace ORB_SLAM3 {
namespace tag {

enum class CameraId : std::uint8_t
{
    LEFT_OR_MONO = 0,
    RIGHT = 1
};

// 派生的位姿估计：Tag -> Camera（OpenCV / IPPE 约定）
struct TagPoseCandidate
{
    Sophus::SE3f t_ct_;

    float reprojection_error_ = -1.0f;
    bool valid_ = false;
};

struct TagPoseEstimate
{
    // 仅存放有效 IPPE 解；IPPE_SQUARE 通常为 2 个
    std::array<TagPoseCandidate, 2> candidates_{};

    // -1: 未消歧；0/1: 已选定的候选下标
    int selected_candidate_ = -1;

    float ambiguity_ratio_ = -1.0f;

    // 是否已消除 IPPE 二义性（selected_candidate_ 为有效下标且对应候选 valid_）
    bool IsAmbiguityResolved() const noexcept
    {
        return GetSelectedCandidate() != nullptr;
    }

    const TagPoseCandidate *GetSelectedCandidate() const noexcept
    {
        if(selected_candidate_ < 0 || selected_candidate_ >= 2)
            return nullptr;

        const TagPoseCandidate &candidate = candidates_[selected_candidate_];
        return candidate.valid_ ? &candidate : nullptr;
    }

    // 消歧后未被选中的镜像 IPPE（另一候选 valid_ 时）
    const TagPoseCandidate *GetUnselectedCandidate() const noexcept
    {
        if(selected_candidate_ < 0 || selected_candidate_ >= 2)
            return nullptr;

        const TagPoseCandidate &candidate = candidates_[1 - selected_candidate_];
        return candidate.valid_ ? &candidate : nullptr;
    }
};

// 原始检测数据；pose_estimate_ 为空表示尚未做 IPPE / 消歧
struct TagObservation
{
    int tag_id_ = -1;
    CameraId camera_id_ = CameraId::LEFT_OR_MONO;

    // 原始检测角点（未去畸变，与输入图像像素一致）
    std::array<cv::Point2f, 4> corners_raw_{};

    // 去畸变后的角点（DetectCorners 写入；无畸变时等于 corners_raw_）
    std::array<cv::Point2f, 4> corners_undistorted_{};

    // 解码质量：bit 强度相对判决阈值的平均裕度，越大越可靠
    // ethz 无此字段时，DetectCorners 用 observed_perimeter_ 作为粗糙代理
    float decision_margin_ = 0.0f;
    // 解码时纠正的错误 bit 数，0 最好，越大误检风险越高
    int hamming_ = 0;

    // 检测几何质量（原图像素）：四边形面积与周长
    float observed_area_ = 0.0f;
    float observed_perimeter_ = 0.0f;

    // nullopt: 尚未估计；有值: 已计算候选（未必已消歧）
    std::optional<TagPoseEstimate> pose_estimate_;

    // 检测阶段永久无效（坏检测等）；快照/消歧/建图均看此标志
    bool is_detect_outlier_ = false;

    // 最近一次位姿优化的组级外点（临时）；同帧下一次 PoseOptimization 前必须 ResetOptimizationOutliers
    bool is_optimization_outlier_ = false;

    // 角点级优化外点（PoseOptimization / TagOptimizer 写入；默认全内点）
    std::array<bool, 4> corner_outliers_{{false, false, false, false}};

    // 检测有效：可用于消歧、建图、构建联合优化快照（忽略优化外点）
    bool IsDetectValid() const noexcept
    {
        return tag_id_ >= 0 && !is_detect_outlier_;
    }

    // 原图像素四边形面积（鞋带公式）；角点无效时返回 0
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

    // 清除优化产生的临时外点（不影响 is_detect_outlier_）
    void ResetOptimizationOutliers() noexcept
    {
        is_optimization_outlier_ = false;
        corner_outliers_ = {{false, false, false, false}};
    }

    // 是否已消除 IPPE 二义性
    bool IsAmbiguityResolved() const noexcept
    {
        return pose_estimate_.has_value() && pose_estimate_->IsAmbiguityResolved();
    }
};

// 将观测相机系下的 Tag 位姿表达到左目系：
//   左目/单目：T_cL_t = T_ct
//   右目：T_cL_t = T_lr * T_cR_t（ORB mTlr：P_left = T_lr * P_right）
// 再经左目 Twc 得到世界系：T_wt = Twc * T_cL_t
inline Sophus::SE3f ConvertTagPoseToLeftCamera(CameraId camera_id,
                                               const Sophus::SE3f &t_ct,
                                               const Sophus::SE3f &t_lr)
{
    if(camera_id == CameraId::RIGHT)
        return t_lr * t_ct;
    return t_ct;
}

}  // namespace tag
}  // namespace ORB_SLAM3

#endif

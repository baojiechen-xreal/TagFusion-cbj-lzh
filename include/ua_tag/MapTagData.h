#ifndef UA_TAG_MAP_TAG_DATA_H
#define UA_TAG_MAP_TAG_DATA_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <mutex>

#include <Eigen/Core>
#include <sophus/se3.hpp>

namespace ORB_SLAM3 {

class KeyFrame;
class Map;

namespace xreal_tag {

// MapTag lifecycle state (transition logic is implemented in a later stage)
enum class MapTagState : std::uint8_t
{
    CANDIDATE = 0,   // Just created; observations are insufficient / not yet disambiguated
    ACTIVE,          // Tag created by the system; world pose is optimizable (including initialization)
    FIXED_ANCHOR,    // External trusted world pose only (surveyed, calibrated, or loaded Tag map)
    BAD              // Invalid; waiting to be removed from the Map
};

/**
 * A single Tag entity in the Map.
 *
 * Ownership (do not violate destruction order: unlink KF<->MapTag first, then release the shared_ptr held by the Map):
 *   Map                -> shared_ptr<MapTagData>   sole persistent owner
 *   KeyFrame           -> MapTagData*              non-owning
 *   MapTagData::map_  -> Map*                     non-owning
 *   MapTagData observations -> KeyFrame*          non-owning
 *
 * Sole source of corner measurements: KeyFrame::tag_frame_data_ (this class stores only the left/right observation indices).
 */
class MapTagData
{
public:
    struct KeyFrameObservation
    {
        int left_index = -1;
        int right_index = -1;
    };

    using KeyFrameObservations = std::map<KeyFrame *, KeyFrameObservation>;

    MapTagData() = default;

    int GetId() const;
    void SetId(int id);

    float GetTagSize() const;
    void SetTagSize(float tag_size);

    void SetPose(const Sophus::SE3f &t_wt);
    Sophus::SE3f GetPose() const;
    bool HasPose() const;

    // Unselected IPPE mirror solution frozen when the pose is first assigned (world-frame T_wt); LBA does not update it
    void SetMirrorPose(const Sophus::SE3f &t_wt);
    Sophus::SE3f GetMirrorPose() const;
    bool HasMirrorPose() const;

    // Computed on demand: P_w = T_wt * P_t (requires an existing pose and tag_size > 0)
    std::array<Eigen::Vector3f, 4> GetWorldCorners() const;
    bool HasWorldCorners() const;
    std::array<Eigen::Vector3f, 4> GetMirrorWorldCorners() const;
    bool HasMirrorWorldCorners() const;

    void SetState(MapTagState state);
    MapTagState GetState() const;
    bool IsFixed() const;  // FIXED_ANCHOR
    bool IsBad() const;

    Map *GetMap() const;

    bool IsInKeyFrame(KeyFrame *key_frame) const;
    KeyFrameObservations GetObservations() const;
    std::size_t GetObservationCount() const;
    bool IsEmpty() const { return GetObservationCount() == 0; }

    // Written on export / online overwrite so Pangolin can show mean w_bar on the Tag
    struct FactorVizWeight
    {
        bool valid = false;
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
    };
    void SetFactorVizWeight(const FactorVizWeight &w);
    FactorVizWeight GetFactorVizWeight() const;

    // Select the reference observation KF by layers; returns nullptr when none qualifies.
    // allow_unresolved_fallback: whether to fall back when every historical observation is unresolved; loop anchors should pass false.
    KeyFrame *SelectReferenceKeyFrame(int min_inlier_corners = 3,
                                      bool allow_unresolved_fallback = true) const;

    // Loop-closing / GBA scratch (aligned with MapPoint; not locked)
    unsigned long corrected_by_key_frame_ = 0;
    unsigned long corrected_reference_ = 0;
    Sophus::SE3f twt_gba_;
    unsigned long bundle_adjustment_global_for_key_frame_ = 0;

private:
    friend class ORB_SLAM3::KeyFrame;
    friend class ORB_SLAM3::Map;

    void AddObservationInternal(KeyFrame *key_frame, int left_index, int right_index);
    void EraseObservationInternal(KeyFrame *key_frame);
    void SetMapInternal(Map *map);

    // mutex_pose_: twt_ / has_pose_ / twt_mirror_ / has_mirror_pose_ / tag_size_
    mutable std::mutex mutex_pose_;
    // mutex_features_: observations_ / state_ / factor_viz_weight_; map_ is written by Map while the Map lock is held
    mutable std::mutex mutex_features_;

    int id_ = -1;
    float tag_size_ = 0.0f;

    Sophus::SE3f twt_;
    bool has_pose_ = false;

    Sophus::SE3f twt_mirror_;
    bool has_mirror_pose_ = false;

    MapTagState state_ = MapTagState::CANDIDATE;
    KeyFrameObservations observations_;

    FactorVizWeight factor_viz_weight_;

    Map *map_ = nullptr;
};

}  // namespace xreal_tag
}  // namespace ORB_SLAM3

#endif

#ifndef UA_TAG_TAG_VIEWER_H
#define UA_TAG_TAG_VIEWER_H

#include "ua_tag/TagFrameData.h"

#include <array>
#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <sophus/se3.hpp>

namespace ORB_SLAM3 {

class Map;

namespace xreal_tag {

// Tag-only visualization: AprilTag detection image on the left, tag map + camera trajectory on the right (separate Pangolin window)
class TagViewer
{
public:
    explicit TagViewer(const std::string &settings_file = std::string());
    ~TagViewer();

    TagViewer(const TagViewer &) = delete;
    TagViewer &operator=(const TagViewer &) = delete;

    void Start();
    void RequestFinish();
    bool IsFinished() const;

    // Once frozen, ignore later Update calls (used to show only the two init-frame pose + MapTag)
    void Freeze();
    bool IsFrozen() const;

    // Copies image / pose / MapTag snapshots internally and does not hold external pointers
    // tracking_state: Tracking::eTrackingState
    void Update(const cv::Mat &image,
                const TagFrameData &frame_data,
                bool has_pose,
                const Sophus::SE3f &Tcw,
                unsigned long frame_id,
                Map *pMap,
                int tracking_state);

private:
    struct TagVis
    {
        int id = -1;
        bool fixed = false;
        bool has_pose = false;
        bool has_corners = false;
        Sophus::SE3f T_wt;
        std::array<Eigen::Vector3f, 4> corners{};
    };

    void Run();
    bool CheckFinish();
    void SetFinish();

    void DrawCameraFrustum(const Sophus::SE3f &Twc, float r, float g, float b,
                           float size) const;
    void DrawTagSquare(const TagVis &tag) const;
    void DrawTagId(const TagVis &tag) const;
    void DrawAxes(const Sophus::SE3f &Tw, float len) const;

    mutable std::mutex mutex_data_;
    cv::Mat image_;
    bool has_pose_ = false;
    Sophus::SE3f tcw_;
    unsigned long frame_id_ = 0;
    int tracking_state_ = 1;  // Tracking::NOT_INITIALIZED
    std::vector<TagVis> tags_;
    std::vector<Sophus::SE3f> key_frame_twc_;

    std::atomic<bool> finish_requested_{false};
    std::atomic<bool> finished_{true};
    std::atomic<bool> frozen_{false};
    std::thread thread_;

    float camera_size_ = 0.08f;
    float viewpoint_x_ = 0.0f;
    float viewpoint_y_ = -0.7f;
    float viewpoint_z_ = -1.8f;
    float viewpoint_f_ = 500.0f;
};

}  // namespace xreal_tag
}  // namespace ORB_SLAM3

#endif

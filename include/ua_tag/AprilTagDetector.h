#ifndef UA_TAG_APRILTAG_DETECTOR_H
#define UA_TAG_APRILTAG_DETECTOR_H

#include "ua_tag/TagObservation.h"

#include <opencv2/core/core.hpp>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace ORB_SLAM3
{

class Settings;
class GeometricCamera;

namespace xreal_tag
{

// Camera model: corner undistortion + IPPE PnP
// Detection runs on the raw image; corners_raw are raw-image pixels;
// corners_undistorted are ideal pinhole pixels matching GeometricCamera::unproject (IPPE seed)
struct CameraModel
{
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    cv::Mat dist_coeffs;  // PinHole: k1,k2,p1,p2[,k3]; Fisheye(KB8): k1..k4
    bool is_fisheye = false;

    cv::Matx33d K() const
    {
        return cv::Matx33d(fx, 0.0, cx,
                           0.0, fy, cy,
                           0.0, 0.0, 1.0);
    }

    // Reuse the ORB-SLAM3 Settings already loaded (may include newWidth/newHeight scaling)
    static CameraModel FromSettings(Settings *settings);
};

// Optional temporal prior: previous-frame T_ct of a tag, for ambiguity resolution
struct PosePrediction
{
    int tag_id = -1;
    cv::Matx33d R = cv::Matx33d::eye();
    cv::Vec3d t = cv::Vec3d(0, 0, 0);
    bool valid = false;
};

struct AprilTagDetectorConfig
{
    // ethz_apriltag2 (Thirdparty/apriltag)
    std::string family = "tag36h11";
    int hamming = 2;              // Max accepted ethz hammingDistance
    // ethz TagDetector blackBorder; several may be set. DetectCorners scans each border, then merges by id
    // yaml: Tag.black_borders: [1, 2] or the legacy key Tag.black_border: 1
    std::vector<int> black_borders = {1};
    double tag_size = 0.16;       // Default Tag side length (m); an id absent from size_by_id uses this value
    // Per-tag_id side-length override, for example yaml:
    //   Tag.size_by_id: [ {id: 0, size: 0.06}, {id: 10, size: 0.144} ]
    std::map<int, double> tag_size_by_id;

    // Drop detections of a listed id in DetectCorners
    // yaml: Tag.ignore_ids: [1, 2, 99]
    std::set<int> ignore_ids;

    // gamma applied before AprilTag: out = 255*(in/255)^gamma; <1 brightens dark regions
    // Tag.gamma_enable: 0 off, 1 on. Default on; gamma==1 leaves the image unchanged even when enabled
    bool gamma_enable = true;
    double gamma = 1.0;

    // Use the override when tag_id is in tag_size_by_id, otherwise tag_size
    double GetTagSize(int tag_id) const;
    bool IsIgnored(int tag_id) const;

    // Load Tag.* from the ORB-SLAM3 settings yaml (keep defaults when a key is absent)
    static AprilTagDetectorConfig FromYaml(const std::string &settings_file);
};

// AprilTag corner order matches OpenCV IPPE_SQUARE object points:
// (-s/2,s/2,0), (s/2,s/2,0), (s/2,-s/2,0), (-s/2,-s/2,0)
inline void BuildSquareObjectPoints(double tag_size, std::vector<cv::Point3f> &object_pts)
{
    const float h = static_cast<float>(tag_size) * 0.5f;
    object_pts.clear();
    object_pts.reserve(4);
    object_pts.emplace_back(-h,  h, 0.f);
    object_pts.emplace_back( h,  h, 0.f);
    object_pts.emplace_back( h, -h, 0.f);
    object_pts.emplace_back(-h, -h, 0.f);
}

// Cross-reprojection disambiguation (convention P_left = T_lr * P_right):
//   1) L->R: if the left camera is unresolved, project both candidates through T_rl onto the right image and write back left
//   2) If the left camera still fails, R->L: project both right candidates through T_lr onto the left image and write back right
// Return true if either side succeeds.
// log_stereo=false: do not print [TagStereo] ok/fail (for example when the Tag is already in the map)
bool DisambiguateWithStereo(xreal_tag::TagObservation &left_obs,
                            xreal_tag::TagObservation &right_obs,
                            const Sophus::SE3f &T_lr,
                            GeometricCamera *cam_left,
                            GeometricCamera *cam_right,
                            double tag_size,
                            double tau_rho = 3.0,
                            bool log_stereo = true);

class AprilTagDetector
{
public:
    explicit AprilTagDetector(const AprilTagDetectorConfig &config = AprilTagDetectorConfig());
    explicit AprilTagDetector(const std::string &settings_file);
    ~AprilTagDetector();

    // Not copyable: Impl owns raw apriltag pointers; a copy would double-free
    AprilTagDetector(const AprilTagDetector &) = delete;
    AprilTagDetector &operator=(const AprilTagDetector &) = delete;

    // Camera used to undistort corners (includes real distortion; set is_fisheye=true for fisheye)
    void SetCameraModel(const CameraModel &camera);

    // Inject ORB GeometricCamera so fisheye undistortion and reprojection error share project/unproject with BA
    void SetGeometricCamera(GeometricCamera *camera);

    // Detect on the raw image (optional gamma); fill corners_raw and corners_undistorted (no IPPE)
    // camera_id: camera that owns the observation; a non-null geometric_camera overrides the default undistort model (right camera)
    bool DetectCorners(const cv::Mat &image,
                       std::vector<xreal_tag::TagObservation> &observations,
                       xreal_tag::CameraId camera_id = xreal_tag::CameraId::LEFT_OR_MONO,
                       GeometricCamera *geometric_camera = nullptr);

    // Image actually passed to the detector on the previous frame (raw grayscale + optional gamma), for visualization
    const cv::Mat &GetLastPreprocessedImage() const;

    // IPPE_SQUARE PnP on pinhole pixels consistent with GeometricCamera; prediction is optional
    bool EstimatePose(xreal_tag::TagObservation &observation,
                      const CameraModel &camera,
                      const PosePrediction *prediction = nullptr);

    const AprilTagDetectorConfig &GetConfig() const;

    // Physical side length (m) for a tag_id; see AprilTagDetectorConfig::GetTagSize
    double GetTagSize(int tag_id) const;

private:
    struct Impl;
    Impl *impl_;
};

}  // namespace xreal_tag
}  // namespace ORB_SLAM3

#endif

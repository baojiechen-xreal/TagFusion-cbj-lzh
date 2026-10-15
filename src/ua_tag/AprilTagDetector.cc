#ifdef HAS_APRILTAG

#include "ua_tag/AprilTagDetector.h"

#include "Settings.h"
#include "CameraModels/GeometricCamera.h"

#include <apriltags/TagDetector.h>
#include <apriltags/TagFamily.h>
#include <apriltags/Tag16h5.h>
#include <apriltags/Tag25h7.h>
#include <apriltags/Tag25h9.h>
#include <apriltags/Tag36h11.h>
#include <apriltags/Tag36h9.h>

#include <Eigen/Core>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <unordered_map>

namespace ORB_SLAM3
{
namespace xreal_tag
{

namespace
{

const AprilTags::TagCodes *GetTagCodes(const std::string &family)
{
    if(family == "tag36h11")
        return &AprilTags::tagCodes36h11;
    if(family == "tag36h9")
        return &AprilTags::tagCodes36h9;
    if(family == "tag25h9")
        return &AprilTags::tagCodes25h9;
    if(family == "tag25h7")
        return &AprilTags::tagCodes25h7;
    if(family == "tag16h5")
        return &AprilTags::tagCodes16h5;
    return nullptr;
}

// Convert the image to grayscale
cv::Mat ToGray(const cv::Mat &image)
{
    if(image.empty())
        throw std::invalid_argument("AprilTagDetector: empty image");

    cv::Mat gray;
    if(image.channels() == 1)
        gray = image;
    else if(image.channels() == 3)
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    else if(image.channels() == 4)
        cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY);
    else
        throw std::invalid_argument("AprilTagDetector: unsupported channel count");

    if(!gray.isContinuous())
        gray = gray.clone();
    return gray;
}

Sophus::SE3f MakeTct(const cv::Matx33d &R, const cv::Vec3d &t)
{
    Eigen::Matrix3f Rf;
    for(int r = 0; r < 3; ++r)
        for(int c = 0; c < 3; ++c)
            Rf(r, c) = static_cast<float>(R(r, c));
    return Sophus::SE3f(Rf, Eigen::Vector3f(static_cast<float>(t[0]),
                                            static_cast<float>(t[1]),
                                            static_cast<float>(t[2])));
}

bool HasSignificantDistortion(const cv::Mat &dist)
{
    if(dist.empty())
        return false;
    cv::Mat d64;
    dist.convertTo(d64, CV_64F);
    for(int i = 0; i < d64.rows * d64.cols; ++i)
    {
        if(std::abs(d64.at<double>(i)) > 1e-12)
            return true;
    }
    return false;
}

// Lift raw-image corners to ideal pinhole pixels consistent with ORB via GeometricCamera::unproject (same camera K)
void UndistortCornersWithGeometricCamera(const xreal_tag::TagObservation &src,
                                         GeometricCamera *geom,
                                         std::array<cv::Point2f, 4> &out)
{
    const float fx = geom->getParameter(0);
    const float fy = geom->getParameter(1);
    const float cx = geom->getParameter(2);
    const float cy = geom->getParameter(3);
    for(int k = 0; k < 4; ++k)
    {
        const cv::Point3f ray = geom->unproject(src.corners_raw[k]);
        const float invz = (std::abs(ray.z) > 1e-12f) ? (1.f / ray.z) : 1.f;
        out[k] = cv::Point2f(fx * ray.x * invz + cx, fy * ray.y * invz + cy);
    }
}

// Undistort corners: prefer GeometricCamera (same source as ORB BA); otherwise pinhole uses cv::undistortPoints
void UndistortCorners(const xreal_tag::TagObservation &src, const CameraModel &camera,
                      GeometricCamera *geom, std::array<cv::Point2f, 4> &out)
{
    if(geom)
    {
        UndistortCornersWithGeometricCamera(src, geom, out);
        return;
    }

    if(!HasSignificantDistortion(camera.dist_coeffs))
    {
        out = src.corners_raw;
        return;
    }

    // Without GeometricCamera: pinhole goes through OpenCV; fisheye no longer uses cv::fisheye (inconsistent with KB)
    if(camera.is_fisheye)
    {
        out = src.corners_raw;
        return;
    }

    std::vector<cv::Point2f> pts(src.corners_raw.begin(), src.corners_raw.end());
    std::vector<cv::Point2f> undist;
    const cv::Mat K = cv::Mat(camera.K());
    cv::undistortPoints(pts, undist, K, camera.dist_coeffs, cv::noArray(), K);
    for(int k = 0; k < 4; ++k)
        out[k] = undist[k];
}

// Pinhole zero-distortion reprojection (fallback only when GeometricCamera is absent)
double ComputeReprojectionErrorPinhole(const std::vector<cv::Point3f> &object_pts,
                                       const std::vector<cv::Point2f> &image_pts,
                                       const cv::Mat &rvec,
                                       const cv::Mat &tvec,
                                       const cv::Mat &K)
{
    if(object_pts.empty() || object_pts.size() != image_pts.size())
        return std::numeric_limits<double>::infinity();

    std::vector<cv::Point2f> projected;
    cv::projectPoints(object_pts, rvec, tvec, K, cv::Mat(), projected);
    if(projected.size() != image_pts.size())
        return std::numeric_limits<double>::infinity();

    double sum_sq = 0.0;
    for(size_t i = 0; i < image_pts.size(); ++i)
    {
        const double dx = projected[i].x - image_pts[i].x;
        const double dy = projected[i].y - image_pts[i].y;
        sum_sq += dx * dx + dy * dy;
    }
    return std::sqrt(sum_sq / static_cast<double>(image_pts.size()));
}

// Consistent with ORB BA: GeometricCamera::project onto raw-image pixels, compared with corners_raw
double ComputeReprojectionErrorGeometric(const std::vector<cv::Point3f> &object_pts,
                                         const std::array<cv::Point2f, 4> &corners_raw,
                                         const Sophus::SE3f &T_ct,
                                         GeometricCamera *geom)
{
    if(!geom || object_pts.size() != 4)
        return std::numeric_limits<double>::infinity();

    double sum_sq = 0.0;
    for(int i = 0; i < 4; ++i)
    {
        const Eigen::Vector3f P_t(object_pts[i].x, object_pts[i].y, object_pts[i].z);
        const Eigen::Vector3f P_c = T_ct * P_t;
        if(P_c.z() <= 0.0f)
            return std::numeric_limits<double>::infinity();

        const cv::Point2f uv = geom->project(cv::Point3f(P_c.x(), P_c.y(), P_c.z()));
        const double dx = static_cast<double>(uv.x - corners_raw[i].x);
        const double dy = static_cast<double>(uv.y - corners_raw[i].y);
        sum_sq += dx * dx + dy * dy;
    }
    return std::sqrt(sum_sq / 4.0);
}

// Rotation-angle error relative to the prediction (radians): R_err = R_pred^T * R_cand
double RotationDeltaToPrediction(const xreal_tag::TagPoseCandidate &cand, const PosePrediction &pred)
{
    const Eigen::Matrix3f Rf = cand.t_ct.rotationMatrix();
    cv::Matx33d R_cand;
    for(int r = 0; r < 3; ++r)
        for(int c = 0; c < 3; ++c)
            R_cand(r, c) = static_cast<double>(Rf(r, c));

    const cv::Matx33d R_err = pred.R.t() * R_cand;
    const double trace = R_err(0, 0) + R_err(1, 1) + R_err(2, 2);
    const double cos_angle = std::max(-1.0, std::min(1.0, 0.5 * (trace - 1.0)));
    return std::acos(cos_angle);
}

// Euclidean translation error relative to the prediction (meters)
double TranslationDeltaToPrediction(const xreal_tag::TagPoseCandidate &cand, const PosePrediction &pred)
{
    const Eigen::Vector3f tf = cand.t_ct.translation();
    const cv::Vec3d dt = cv::Vec3d(tf[0], tf[1], tf[2]) - pred.t;
    return std::sqrt(dt.dot(dt));
}

// Ambiguity resolution, layers 2/3.
// Input candidates must already have passed the positive-depth check (candidate-generation stage).
// This function then checks numerical validity, absolute reprojection RMSE, the error ratio, and temporal gating / cost.
// IPPE_SQUARE has at most 2 candidates; num_candidates > 2 fails immediately.
// Returns selected_candidate; returns -1 when untrusted or unresolved (the official return is not a provisional best).
// If both solutions are valid, write ambiguity_ratio = e_worse / e_better.
int ResolvePoseAmbiguity(const std::array<xreal_tag::TagPoseCandidate, 2> &candidates,
                         int num_candidates,
                         const PosePrediction *prediction,
                         int tag_id,
                         float &ambiguity_ratio)
{
    ambiguity_ratio = -1.0f;

    if(num_candidates <= 0 || num_candidates > 2)
        return -1;

    constexpr double kMaxReprojectionRmse = 3.0;  // needs calibration
    constexpr double kTauRhoStrong = 3.0;
    constexpr double kErrorEps = 1e-9; // avoid division by zero

    int i_better = 0;
    int i_worse = -1;
    if(num_candidates == 2)
    {
        i_worse = 1;
        if(candidates[1].reprojection_error < candidates[0].reprojection_error)
        {
            i_better = 1;
            i_worse = 0;
        }
    }

    const double e1 = static_cast<double>(candidates[i_better].reprojection_error);
    if(!std::isfinite(e1) || e1 > kMaxReprojectionRmse)
        return -1;

    // Only one valid candidate
    if(num_candidates == 1)
        return i_better;

    const double e2 = static_cast<double>(candidates[i_worse].reprojection_error);
    if(!std::isfinite(e2))
        return -1;

    // ρ = e2/e1 (add eps so e1=e2=0 is not treated as strong support)
    const double rho = (e2 + kErrorEps) / (e1 + kErrorEps);
    ambiguity_ratio = static_cast<float>(rho);
    const bool strong_reproj = (rho > kTauRhoStrong);

    const bool has_pred = prediction && prediction->valid &&
                          (prediction->tag_id < 0 || prediction->tag_id == tag_id);

    // -----------------------------------------------------
    // Strong reprojection evidence: prefer the lower-error solution; when a prediction exists, only a loose outlier check
    // -----------------------------------------------------
    if(strong_reproj)
    {
        if(has_pred)
        {
            constexpr double kLooseTauR = 1.0;   // rad, needs calibration
            constexpr double kLooseTauT = 0.30;  // m, needs calibration

            const double dtheta =
                RotationDeltaToPrediction(candidates[i_better], *prediction);
            const double dtrans =
                TranslationDeltaToPrediction(candidates[i_better], *prediction);

            // The image and a reliable prediction disagree severely
            if(dtheta >= kLooseTauR || dtrans >= kLooseTauT)
                return -1;
        }

        return i_better;
    }

    // -----------------------------------------------------
    // Reprojection cannot disambiguate and there is no temporal prediction, so the result is untrusted
    // -----------------------------------------------------
    if(!has_pred)
        return -1;

    // -----------------------------------------------------
    // Weak reprojection evidence: strict temporal gate, then the in-gate combined cost (a sufficient cost gap is required)
    // -----------------------------------------------------
    constexpr double kTauR = 0.5;   // rad
    constexpr double kTauT = 0.15;  // m

    double dtheta[2];
    double dtrans[2];
    bool pass[2];
    int n_pass = 0;
    int sole_pass = -1;

    for(int i = 0; i < 2; ++i)
    {
        dtheta[i] = RotationDeltaToPrediction(candidates[i], *prediction);
        dtrans[i] = TranslationDeltaToPrediction(candidates[i], *prediction);
        pass[i] = (dtheta[i] < kTauR) && (dtrans[i] < kTauT);
        if(pass[i])
        {
            ++n_pass;
            sole_pass = i;
        }
    }

    if(n_pass == 0)
        return -1;

    if(n_pass == 1)
        return sole_pass;

    // Both candidates passed the temporal gate; compute the combined cost
    constexpr double kWe = 1.0;
    constexpr double kWR = 1.0;
    constexpr double kWt = 1.0;
    constexpr double kSigmaE = 1.0;   // px
    constexpr double kSigmaR = 0.2;   // rad
    constexpr double kSigmaT = 0.05;  // m
    constexpr double kMinJGap = 0.2;  // needs calibration

    double J[2];
    for(int i = 0; i < 2; ++i)
    {
        J[i] = kWe * static_cast<double>(candidates[i].reprojection_error) / kSigmaE +
               kWR * dtheta[i] / kSigmaR +
               kWt * dtrans[i] / kSigmaT;
    }

    const int best = (J[0] <= J[1]) ? 0 : 1;
    const int second = 1 - best;

    // The combined costs are too close to claim that ambiguity is resolved
    if(J[second] - J[best] < kMinJGap)
        return -1;

    return best;
}

}  // namespace

namespace {

cv::Mat BuildGammaLut(double gamma)
{
    cv::Mat lut(1, 256, CV_8U);
    uchar *p = lut.ptr<uchar>(0);
    const double inv = 1.0 / 255.0;
    for(int i = 0; i < 256; ++i)
        p[i] = cv::saturate_cast<uchar>(std::pow(static_cast<double>(i) * inv, gamma) * 255.0);
    return lut;
}

cv::Mat ApplyGamma(const cv::Mat &im, const cv::Mat &lut)
{
    if(im.empty() || lut.empty())
        return im;
    cv::Mat out;
    cv::LUT(im, lut, out);
    return out;
}

}  // namespace

struct AprilTagDetector::Impl
{
    // ethz_apriltag2: one detector per black_border; DetectCorners merges the results
    std::vector<std::unique_ptr<AprilTags::TagDetector>> detectors;
    std::string family_name;
    double tag_size;
    AprilTagDetectorConfig config;

    // Camera used to undistort corners (detection still runs on the raw image)
    CameraModel camera_model;
    bool has_camera_model = false;
    // Shared with ORB Tracking/BA; the fisheye path prefers its project/unproject
    GeometricCamera *geometric_camera = nullptr;

    // Image most recently passed to the detector (raw grayscale + optional gamma)
    cv::Mat last_preprocessed;
    cv::Mat gamma_lut;

    Impl(const AprilTagDetectorConfig &cfg)
        : family_name(cfg.family), tag_size(cfg.tag_size), config(cfg)
    {
        const AprilTags::TagCodes *codes = GetTagCodes(cfg.family);
        if(codes == nullptr)
            throw std::invalid_argument("AprilTagDetector: unsupported tag family: " + cfg.family);

        std::vector<int> borders = cfg.black_borders;
        if(borders.empty())
            borders.push_back(1);

        // Deduplicate, force >= 1, and keep the configured order
        std::vector<int> unique_borders;
        unique_borders.reserve(borders.size());
        for(int b : borders)
        {
            const int bb = std::max(1, b);
            if(std::find(unique_borders.begin(), unique_borders.end(), bb) == unique_borders.end())
                unique_borders.push_back(bb);
        }
        config.black_borders = unique_borders;

        detectors.reserve(unique_borders.size());
        // One detector per black_border; DetectCorners merges the results
        for(int bb : unique_borders)
            detectors.emplace_back(new AprilTags::TagDetector(*codes, static_cast<size_t>(bb)));

        if(cfg.gamma_enable && cfg.gamma > 0.0 && std::abs(cfg.gamma - 1.0) > 1e-6)
            gamma_lut = BuildGammaLut(cfg.gamma);
    }

    ~Impl() = default;
};

// Reuse ORB-SLAM3 Settings to obtain the camera model
CameraModel CameraModel::FromSettings(Settings *settings)
{
    if(!settings)
        throw std::invalid_argument("CameraModel::FromSettings: settings is null");

    GeometricCamera *geom = settings->camera1();
    if(!geom)
        throw std::runtime_error("CameraModel::FromSettings: camera1 is null");

    CameraModel cam;
    cam.fx = static_cast<double>(geom->getParameter(0));
    cam.fy = static_cast<double>(geom->getParameter(1));
    cam.cx = static_cast<double>(geom->getParameter(2));
    cam.cy = static_cast<double>(geom->getParameter(3));

    if(settings->cameraType() == Settings::KannalaBrandt)
    {
        cam.is_fisheye = true;
        cam.dist_coeffs = (cv::Mat_<double>(4, 1) <<
            geom->getParameter(4), geom->getParameter(5),
            geom->getParameter(6), geom->getParameter(7));
        return cam;
    }

    cv::Mat dist = settings->camera1DistortionCoef();
    if(!dist.empty())
        dist.convertTo(cam.dist_coeffs, CV_64F);

    return cam;
}

// Load the AprilTagDetector configuration from the yaml file
AprilTagDetectorConfig AprilTagDetectorConfig::FromYaml(const std::string &settings_file)
{
    AprilTagDetectorConfig config;

    cv::FileStorage fs(settings_file, cv::FileStorage::READ);
    if(!fs.isOpened())
        throw std::runtime_error("AprilTagDetectorConfig: failed to open settings file: " + settings_file);

    cv::FileNode node;
    node = fs["Tag.family"];
    if(!node.empty() && node.isString())
        config.family = static_cast<std::string>(node);

    node = fs["Tag.hamming"];
    if(!node.empty())
        config.hamming = static_cast<int>(node);

    // Prefer Tag.black_borders: [1, 2] or the scalar 2; otherwise accept the legacy Tag.black_border: 1
    node = fs["Tag.black_borders"];
    if(!node.empty() && node.isSeq())
    {
        config.black_borders.clear();
        for(cv::FileNodeIterator it = node.begin(); it != node.end(); ++it)
        {
            const int b = static_cast<int>(*it);
            if(b >= 1)
                config.black_borders.push_back(b);
        }
        if(config.black_borders.empty())
            config.black_borders.push_back(1);
    }
    else if(!node.empty())
    {
        config.black_borders = {std::max(1, static_cast<int>(node))};
    }
    else
    {
        node = fs["Tag.black_border"];
        if(!node.empty())
            config.black_borders = {std::max(1, static_cast<int>(node))};
    }

    node = fs["Tag.size"];
    if(!node.empty())
        config.tag_size = static_cast<double>(node);

    // Override side length by id: Tag.size_by_id: [ {id: 0, size: 0.06}, {id: 10, size: 0.144} ]
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
            config.tag_size_by_id[id] = sz;
        }
    }

    // Ignore each listed id: Tag.ignore_ids: [1, 2, 99]
    node = fs["Tag.ignore_ids"];
    if(!node.empty() && node.isSeq())
    {
        for(cv::FileNodeIterator it = node.begin(); it != node.end(); ++it)
        {
            const int id = static_cast<int>(*it);
            if(id >= 0)
                config.ignore_ids.insert(id);
        }
    }

    node = fs["Tag.gamma_enable"];
    if(!node.empty())
        config.gamma_enable = static_cast<int>(node) != 0;

    node = fs["Tag.gamma"];
    if(!node.empty())
        config.gamma = static_cast<double>(node);

    return config;
}

double AprilTagDetectorConfig::GetTagSize(int tag_id) const
{
    const auto it = tag_size_by_id.find(tag_id);
    if(it != tag_size_by_id.end() && it->second > 0.0)
        return it->second;
    return tag_size;
}

bool AprilTagDetectorConfig::IsIgnored(int tag_id) const
{
    return ignore_ids.find(tag_id) != ignore_ids.end();
}

const AprilTagDetectorConfig &AprilTagDetector::GetConfig() const
{
    return impl_->config;
}

double AprilTagDetector::GetTagSize(int tag_id) const
{
    return impl_->config.GetTagSize(tag_id);
}

void AprilTagDetector::SetCameraModel(const CameraModel &camera)
{
    impl_->camera_model = camera;
    impl_->has_camera_model = (camera.fx > 0.0 && camera.fy > 0.0);
}

void AprilTagDetector::SetGeometricCamera(GeometricCamera *camera)
{
    impl_->geometric_camera = camera;
}

// Return the preprocessed image from the latest DetectCorners (references an internal buffer overwritten on the next frame)
const cv::Mat &AprilTagDetector::GetLastPreprocessedImage() const
{
    return impl_->last_preprocessed;
}

// Default-construct AprilTagDetector
AprilTagDetector::AprilTagDetector(const AprilTagDetectorConfig &config)
    : impl_(new Impl(config))
{
}

// Construct AprilTagDetector from a yaml file
AprilTagDetector::AprilTagDetector(const std::string &settings_file)
    : impl_(new Impl(AprilTagDetectorConfig::FromYaml(settings_file)))
{
}

// Destroy AprilTagDetector
AprilTagDetector::~AprilTagDetector()
{
    delete impl_;
}

// Detect AprilTags corners on the raw image (optional gamma), then undistort the corners
// Backend: Thirdparty/apriltag ethz_apriltag2; scan each black_border once and merge by id
bool AprilTagDetector::DetectCorners(const cv::Mat &image,
                                     std::vector<xreal_tag::TagObservation> &observations,
                                     xreal_tag::CameraId camera_id,
                                     GeometricCamera *geometric_camera)
{
    observations.clear();
    impl_->last_preprocessed.release();

    cv::Mat gray = ToGray(image);
    cv::Mat processed = ApplyGamma(gray, impl_->gamma_lut);
    if(!processed.isContinuous())
        processed = processed.clone();

    impl_->last_preprocessed = processed;

    if(impl_->detectors.empty())
        return false;

    // The right camera model must be passed explicitly; do not fall back to the default or the left camera
    GeometricCamera *geom = geometric_camera;
    if(!geom)
    {
        if(camera_id == xreal_tag::CameraId::RIGHT)
        {
            std::cerr << "ERROR: DetectCorners(RIGHT) requires GeometricCamera "
                         "(no left/default fallback)"
                      << std::endl;
            return false;
        }
        geom = impl_->geometric_camera;
    }

    // tag_id -> best observation (lower hamming first, then larger observed_perimeter)
    std::unordered_map<int, xreal_tag::TagObservation> best_by_id;

    for(const auto &det_ptr : impl_->detectors)
    {
        if(!det_ptr)
            continue;

        const std::vector<AprilTags::TagDetection> dets = det_ptr->extractTags(processed);
        for(const AprilTags::TagDetection &det : dets)
        {
            if(!det.good)
                continue;
            if(det.hammingDistance > impl_->config.hamming)
                continue;
            if(impl_->config.IsIgnored(det.id))
                continue;

            xreal_tag::TagObservation obs;
            obs.tag_id = det.id;
            obs.camera_id = camera_id;
            obs.hamming = det.hammingDistance;
            obs.observed_perimeter = det.observedPerimeter;
            obs.decision_margin = obs.observed_perimeter;
            for(int k = 0; k < 4; ++k)
                obs.corners_raw[k] = cv::Point2f(det.p[k].first, det.p[k].second);
            obs.observed_area = xreal_tag::TagObservation::ComputeQuadArea(obs.corners_raw);

            if(impl_->has_camera_model || geom)
                UndistortCorners(obs, impl_->camera_model, geom, obs.corners_undistorted);
            else
                obs.corners_undistorted = obs.corners_raw;

            auto it = best_by_id.find(obs.tag_id);
            if(it == best_by_id.end())
            {
                best_by_id.emplace(obs.tag_id, std::move(obs));
                continue;
            }

            const xreal_tag::TagObservation &prev = it->second;
            const bool better =
                (obs.hamming < prev.hamming) ||
                (obs.hamming == prev.hamming &&
                 obs.observed_perimeter > prev.observed_perimeter);
            if(better)
                it->second = std::move(obs);
        }
    }

    observations.reserve(best_by_id.size());
    for(auto &kv : best_by_id)
        observations.push_back(std::move(kv.second));

    return true;
}

// Estimate AprilTags poses
bool AprilTagDetector::EstimatePose(xreal_tag::TagObservation &observation,
                                    const CameraModel &camera,
                                    const PosePrediction *prediction)
{
    observation.pose_estimate = std::nullopt;

    const double tag_size = impl_->config.GetTagSize(observation.tag_id);
    if(tag_size <= 0.0 || camera.fx <= 0.0 || camera.fy <= 0.0)
        return false;

    // Build AprilTag object points: 3D coordinates of the 4 corners in the Tag frame
    std::vector<cv::Point3f> object_pts;
    BuildSquareObjectPoints(tag_size, object_pts);

    // IPPE seed: pinhole pixels consistent with GeometricCamera, plus a zero-distortion K
    // Candidate error: prefer GeometricCamera::project vs corners_raw (same domain as ORB BA)
    const cv::Mat K = cv::Mat(camera.K());
    const cv::Mat dist_empty;
    std::vector<cv::Point2f> image_pts(observation.corners_undistorted.begin(),
                                       observation.corners_undistorted.end());

    std::vector<cv::Mat> rvecs, tvecs;
    int nsol = cv::solvePnPGeneric(object_pts, image_pts, K, dist_empty,
                                   rvecs, tvecs,
                                   false, cv::SOLVEPNP_IPPE_SQUARE);
    if(nsol <= 0 || rvecs.empty() || tvecs.empty())
        return false;

    GeometricCamera *geom = impl_->geometric_camera;
    xreal_tag::TagPoseEstimate estimate;

    // Only positive-depth solutions enter the candidate list (keep valid IPPE solutions only)
    int num_candidates = 0;
    for(int i = 0; i < nsol && num_candidates < 2; ++i)
    {
        const cv::Vec3d t(tvecs[i].at<double>(0),
                          tvecs[i].at<double>(1),
                          tvecs[i].at<double>(2));
        if(t[2] <= 0.0)
            continue;

        cv::Matx33d R;
        cv::Rodrigues(rvecs[i], R);

        xreal_tag::TagPoseCandidate &cand = estimate.candidates[num_candidates++];
        cand.t_ct = MakeTct(R, t);
        if(geom)
        {
            cand.reprojection_error = static_cast<float>(ComputeReprojectionErrorGeometric(
                object_pts, observation.corners_raw, cand.t_ct, geom));
        }
        else
        {
            cand.reprojection_error = static_cast<float>(ComputeReprojectionErrorPinhole(
                object_pts, image_pts, rvecs[i], tvecs[i], K));
        }
        cand.valid = std::isfinite(cand.reprojection_error);
    }

    if(num_candidates <= 0)
        return false;

    // IPPE_SQUARE expects 2 valid solutions; if fewer, do not write pose_estimate
    if(num_candidates < 2)
        return false;

    float ambiguity_ratio = -1.0f;
    const int best = ResolvePoseAmbiguity(estimate.candidates,
                                          num_candidates,
                                          prediction,
                                          observation.tag_id,
                                          ambiguity_ratio);
    if(best < 0)
    {
        // Candidates exist but are unresolved: still write the estimate, and keep selected_candidate at -1
        estimate.ambiguity_ratio = ambiguity_ratio;
        observation.pose_estimate = estimate;
        return false;
    }

    estimate.selected_candidate = best;
    estimate.ambiguity_ratio = ambiguity_ratio;
    observation.pose_estimate = estimate;
    return true;
}

namespace {

// Transform both candidates of source by T_target_source into the target camera frame and compute RMSE against target_corners.
// T_target_source: P_target = T_target_source * P_source
bool CrossReprojDisambiguate(xreal_tag::TagObservation &source_obs,
                             const std::array<cv::Point2f, 4> &target_corners,
                             const Sophus::SE3f &T_target_source,
                             GeometricCamera *cam_target,
                             double tag_size,
                             double tau_rho,
                             const char *direction_label,
                             bool log_fail)
{
    if(!cam_target || tag_size <= 0.0)
        return false;
    if(!source_obs.IsDetectValid())
        return false;
    if(!source_obs.pose_estimate.has_value())
        return false;

    xreal_tag::TagPoseEstimate &est = *source_obs.pose_estimate;
    // Already disambiguated: return immediately and do not log fail again
    if(est.selected_candidate >= 0)
        return false;
    if(!est.candidates[0].valid || !est.candidates[1].valid)
        return false;

    std::vector<cv::Point3f> object_pts;
    BuildSquareObjectPoints(tag_size, object_pts);

    double e[2] = {std::numeric_limits<double>::infinity(),
                   std::numeric_limits<double>::infinity()};

    for(int i = 0; i < 2; ++i)
    {
        const Sophus::SE3f &T_cs_t = est.candidates[i].t_ct;
        double sum_sq = 0.0;
        bool ok = true;
        for(int k = 0; k < 4; ++k)
        {
            const Eigen::Vector3f P_t(object_pts[k].x, object_pts[k].y,
                                     object_pts[k].z);
            // Source camera frame: P_cs = T_cs_t * P_t
            const Eigen::Vector3f P_cs = T_cs_t * P_t;
            if(P_cs.z() <= 0.0f)
            {
                ok = false;
                break;
            }
            // Target camera frame: P_ct = T_target_source * P_cs
            const Eigen::Vector3f P_ct = T_target_source * P_cs;
            if(P_ct.z() <= 0.0f)
            {
                ok = false;
                break;
            }
            const cv::Point2f uv =
                cam_target->project(cv::Point3f(P_ct.x(), P_ct.y(), P_ct.z()));
            const double dx =
                static_cast<double>(uv.x - target_corners[k].x);
            const double dy =
                static_cast<double>(uv.y - target_corners[k].y);
            sum_sq += dx * dx + dy * dy;
        }
        if(ok)
            e[i] = std::sqrt(sum_sq / 4.0);
    }

    if(!std::isfinite(e[0]) && !std::isfinite(e[1]))
        return false;

    int i_better = 0;
    int i_worse = 1;
    if(e[1] < e[0])
    {
        i_better = 1;
        i_worse = 0;
    }

    if(!std::isfinite(e[i_better]))
        return false;

    constexpr double kErrorEps = 1e-9;
    const double e_better = e[i_better];
    const double e_worse =
        std::isfinite(e[i_worse]) ? e[i_worse]
                                  : std::numeric_limits<double>::infinity();
    const double rho = (e_worse + kErrorEps) / (e_better + kErrorEps);

    if(!(rho > tau_rho) || !std::isfinite(rho))
    {
        // Log only observations that are still unresolved; the caller clears log_fail when the other eye of the same ID is already resolved
        if(log_fail && est.selected_candidate < 0)
        {
            std::cout << "[TagStereo] fail tag_id=" << source_obs.tag_id
                      << " dir=" << direction_label
                      << " reason=stereo_ratio_weak"
                      << " e0=" << e[0] << " e1=" << e[1]
                      << " rho=" << rho << " tau=" << tau_rho
                      << " mono_ratio=" << est.ambiguity_ratio
                      << std::endl;
        }
        return false;
    }

    est.selected_candidate = i_better;
    est.ambiguity_ratio = static_cast<float>(rho);

    if(log_fail)  // CrossReproj also uses log_fail to gate ok; the caller has already folded in whether a log line is worthwhile
    {
        std::cout << "[TagStereo] ok tag_id=" << source_obs.tag_id
                  << " dir=" << direction_label
                  << " selected=" << i_better
                  << " e_better=" << e_better
                  << " e_worse=" << e_worse
                  << " stereo_ratio=" << rho
                  << std::endl;
    }
    return true;
}

}  // namespace

bool DisambiguateWithStereo(xreal_tag::TagObservation &left_obs,
                            xreal_tag::TagObservation &right_obs,
                            const Sophus::SE3f &T_lr,
                            GeometricCamera *cam_left,
                            GeometricCamera *cam_right,
                            double tag_size,
                            double tau_rho,
                            bool log_stereo)
{
    if(!cam_left || !cam_right || tag_size <= 0.0)
        return false;
    if(!left_obs.IsDetectValid() || !right_obs.IsDetectValid())
        return false;
    if(left_obs.tag_id < 0 || left_obs.tag_id != right_obs.tag_id)
        return false;

    // Either camera is already resolved, or the caller disabled logging (for example the Tag is already in the map): do not spam stereo fail/ok
    const bool any_resolved =
        left_obs.IsAmbiguityResolved() || right_obs.IsAmbiguityResolved();
    const bool log_fail = log_stereo && !any_resolved;

    // 1) L->R: disambiguate the left camera (P_right = T_rl * P_left)
    if(!left_obs.IsAmbiguityResolved() && left_obs.pose_estimate.has_value())
    {
        if(CrossReprojDisambiguate(left_obs, right_obs.corners_raw,
                                   T_lr.inverse(), cam_right, tag_size, tau_rho,
                                   "L2R", log_fail))
            return true;
    }

    // 2) Left failed, then R->L: disambiguate the right camera (P_left = T_lr * P_right)
    if(!right_obs.IsAmbiguityResolved() && right_obs.pose_estimate.has_value())
    {
        if(CrossReprojDisambiguate(right_obs, left_obs.corners_raw, T_lr,
                                   cam_left, tag_size, tau_rho, "R2L",
                                   log_fail))
            return true;
    }

    return false;
}

}  // namespace xreal_tag
}  // namespace ORB_SLAM3

#endif  // HAS_APRILTAG

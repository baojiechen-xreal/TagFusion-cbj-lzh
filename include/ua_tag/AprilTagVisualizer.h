#ifndef UA_TAG_APRILTAG_VISUALIZER_H
#define UA_TAG_APRILTAG_VISUALIZER_H

#include "ua_tag/TagFrameData.h"

#include <opencv2/core/core.hpp>
#include <string>

namespace ORB_SLAM3
{
namespace xreal_tag
{

// Draw the observations of one camera on a single image (default left / mono)
cv::Mat DrawTags(const cv::Mat &image, const xreal_tag::TagFrameData &frame_data,
                 xreal_tag::CameraId camera_id = xreal_tag::CameraId::LEFT_OR_MONO);

// Overlay each Tag on an existing BGR image (imageScale matches FrameDrawer: corner coordinates are divided by scale)
void OverlayTags(cv::Mat &image, const xreal_tag::TagFrameData &frame_data,
                 xreal_tag::CameraId camera_id = xreal_tag::CameraId::LEFT_OR_MONO,
                 float imageScale = 1.f);

// Draw the left and right images separately, then concatenate them horizontally (synchronized side-by-side view)
cv::Mat DrawTagsStereo(const cv::Mat &imLeft, const cv::Mat &imRight,
                       const xreal_tag::TagFrameData &frame_data);

// Mono / left only: draw and save
bool SaveTagsVis(const cv::Mat &image,
                 const xreal_tag::TagFrameData &frame_data,
                 const std::string &savePath);

// Stereo: save the left and right detection frames side by side
bool SaveTagsVis(const cv::Mat &imLeft, const cv::Mat &imRight,
                 const xreal_tag::TagFrameData &frame_data,
                 const std::string &savePath);

// Ensure the output directory exists; return true on success
bool EnsureDir(const std::string &dirPath);

}  // namespace xreal_tag
}  // namespace ORB_SLAM3

#endif

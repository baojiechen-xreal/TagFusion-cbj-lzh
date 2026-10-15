#ifndef UA_TAG_TAG_POSE_LOGGER_H
#define UA_TAG_TAG_POSE_LOGGER_H

#include "ua_tag/TagObservation.h"

#include <fstream>
#include <string>
#include <vector>

namespace ORB_SLAM3
{
namespace xreal_tag
{

// Write the pose fields of each Tag in every frame to CSV (one row per tag)
class TagPoseLogger
{
public:
    explicit TagPoseLogger(const std::string &csv_path);
    ~TagPoseLogger();

    TagPoseLogger(const TagPoseLogger &) = delete;
    TagPoseLogger &operator=(const TagPoseLogger &) = delete;

    bool IsOpen() const { return file_.is_open(); }

    // Record the pose of every Tag in one frame; safe to call when nothing was detected (writes no row)
    void LogFrame(size_t frame_id,
                  double timestamp,
                  const std::vector<xreal_tag::TagObservation> &tags);

private:
    void WriteHeader();
    void WriteCandidate(const xreal_tag::TagPoseCandidate &cand, bool valid);

    std::ofstream file_;
};

}  // namespace xreal_tag
}  // namespace ORB_SLAM3

#endif

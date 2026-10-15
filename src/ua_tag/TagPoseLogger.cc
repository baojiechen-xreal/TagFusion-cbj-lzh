#include "ua_tag/TagPoseLogger.h"

#include <iomanip>

namespace ORB_SLAM3
{
namespace xreal_tag
{

TagPoseLogger::TagPoseLogger(const std::string &csv_path)
    : file_(csv_path.c_str(), std::ios::out | std::ios::trunc)
{
    if(file_.is_open())
    {
        file_ << std::setprecision(17);
        WriteHeader();
    }
}

TagPoseLogger::~TagPoseLogger()
{
    if(file_.is_open())
        file_.close();
}

void TagPoseLogger::WriteHeader()
{
    file_
        << "frame_id,timestamp,tag_id,"
        << "has_pose_estimate,selected_candidate,ambiguity_ratio,"
        // candidate 0
        << "c0_valid,c0_err,"
        << "c0_tx,c0_ty,c0_tz,"
        << "c0_r00,c0_r01,c0_r02,"
        << "c0_r10,c0_r11,c0_r12,"
        << "c0_r20,c0_r21,c0_r22,"
        // candidate 1
        << "c1_valid,c1_err,"
        << "c1_tx,c1_ty,c1_tz,"
        << "c1_r00,c1_r01,c1_r02,"
        << "c1_r10,c1_r11,c1_r12,"
        << "c1_r20,c1_r21,c1_r22\n";
}

void TagPoseLogger::WriteCandidate(const xreal_tag::TagPoseCandidate &cand, bool valid)
{
    if(!valid)
    {
        // 14 fields: valid + err + t(3) + R(9)
        file_ << "0,,,,,,,,,,,,,";
        return;
    }

    const Eigen::Matrix3f R = cand.t_ct.rotationMatrix();
    const Eigen::Vector3f t = cand.t_ct.translation();

    file_ << "1,"
          << cand.reprojection_error << ","
          << t[0] << "," << t[1] << "," << t[2] << ","
          << R(0, 0) << "," << R(0, 1) << "," << R(0, 2) << ","
          << R(1, 0) << "," << R(1, 1) << "," << R(1, 2) << ","
          << R(2, 0) << "," << R(2, 1) << "," << R(2, 2);
}

void TagPoseLogger::LogFrame(size_t frame_id,
                             double timestamp,
                             const std::vector<xreal_tag::TagObservation> &tags)
{
    if(!file_.is_open())
        return;

    for(size_t i = 0; i < tags.size(); ++i)
    {
        const xreal_tag::TagObservation &tag = tags[i];
        const bool has_pose = tag.pose_estimate.has_value();
        const int selected = has_pose ? tag.pose_estimate->selected_candidate : -1;
        const float ambiguity_ratio = has_pose ? tag.pose_estimate->ambiguity_ratio : -1.0f;

        file_ << frame_id << ","
              << timestamp << ","
              << tag.tag_id << ","
              << (has_pose ? 1 : 0) << ","
              << selected << ","
              << ambiguity_ratio << ",";

        if(has_pose)
        {
            WriteCandidate(tag.pose_estimate->candidates[0],
                           tag.pose_estimate->candidates[0].valid);
            file_ << ",";
            WriteCandidate(tag.pose_estimate->candidates[1],
                           tag.pose_estimate->candidates[1].valid);
        }
        else
        {
            WriteCandidate(xreal_tag::TagPoseCandidate{}, false);
            file_ << ",";
            WriteCandidate(xreal_tag::TagPoseCandidate{}, false);
        }
        file_ << "\n";
    }
}

}  // namespace xreal_tag
}  // namespace ORB_SLAM3

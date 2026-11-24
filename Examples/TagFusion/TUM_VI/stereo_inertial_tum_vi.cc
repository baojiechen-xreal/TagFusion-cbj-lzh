/**
* Stereo-Inertial TagFusion runner for a TUM-VI mav0 sequence.
*
* Layout:
*   <sequence>/mav0/cam0/data.csv + data/<timestamp>.png
*   <sequence>/mav0/cam1/data.csv + data/<timestamp>.png
*   <sequence>/mav0/imu0/data.csv
*
* Images are 16-bit grayscale PNG. They are loaded with IMREAD_GRAYSCALE
* (8-bit on this OpenCV) and then CLAHE(3.0, 8x8), matching the official
* stereo_inertial_tum_vi example. Timestamps are seconds.
*
* Frames timestamped before the first IMU sample are dropped. room1's IMU
* starts first, so nothing is dropped; room4/room5's first image is earlier
* than the first IMU sample and is dropped.
*
* The trajectory is the left-camera TUM file from SaveTrajectoryTUM (seconds).
* Do not convert it to the body frame here.
*
* Usage:
*   ./stereo_inertial_tum_vi vocabulary settings sequence_dir [trajectory_name]
*/

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "ImuTypes.h"
#include "System.h"

using namespace std;

namespace {

bool FileExists(const string &path)
{
    struct stat st;
    return stat(path.c_str(), &st) == 0 && S_ISREG(st.st_mode);
}

bool DirExists(const string &path)
{
    struct stat st;
    return stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

string StripTrailingSlash(string path)
{
    while(!path.empty() && (path.back() == '/' || path.back() == '\\'))
        path.pop_back();
    return path;
}

string Trim(const string &text)
{
    size_t begin = 0;
    while(begin < text.size() &&
          isspace(static_cast<unsigned char>(text[begin])))
        ++begin;
    size_t end = text.size();
    while(end > begin && isspace(static_cast<unsigned char>(text[end - 1])))
        --end;
    return text.substr(begin, end - begin);
}

vector<string> SplitComma(const string &line)
{
    vector<string> fields;
    string current;
    for(char c : line)
    {
        if(c == ',')
        {
            fields.push_back(Trim(current));
            current.clear();
        }
        else if(c != '\r')
        {
            current.push_back(c);
        }
    }
    fields.push_back(Trim(current));
    return fields;
}

bool LoadImagesTUMVI(const string &seqPath,
                     vector<string> &vstrImageLeft,
                     vector<string> &vstrImageRight,
                     vector<double> &vTimeStamps)
{
    vstrImageLeft.clear();
    vstrImageRight.clear();
    vTimeStamps.clear();

    const string root = StripTrailingSlash(seqPath);
    const string leftDir = root + "/mav0/cam0/data";
    const string rightDir = root + "/mav0/cam1/data";
    const string timesPath = root + "/mav0/cam0/data.csv";

    if(!DirExists(leftDir) || !DirExists(rightDir))
    {
        cerr << "ERROR: Cannot find mav0/cam0/data and mav0/cam1/data under "
             << root << endl;
        return false;
    }
    if(!FileExists(timesPath))
    {
        cerr << "ERROR: Cannot find " << timesPath << endl;
        return false;
    }

    ifstream fTimes(timesPath.c_str());
    if(!fTimes.is_open())
    {
        cerr << "ERROR: Cannot open " << timesPath << endl;
        return false;
    }

    string line;
    while(getline(fTimes, line))
    {
        if(line.empty() || line[0] == '#')
            continue;

        const vector<string> fields = SplitComma(line);
        if(fields.size() < 2 || fields[0].empty() || fields[1].empty())
            continue;

        const double t = static_cast<double>(stoll(fields[0])) * 1e-9;
        vTimeStamps.push_back(t);
        vstrImageLeft.push_back(leftDir + "/" + fields[1]);
        vstrImageRight.push_back(rightDir + "/" + fields[1]);
    }

    cout << "Timestamps: " << timesPath << " (" << vTimeStamps.size() << ")"
         << endl;
    cout << "Left:       " << leftDir << endl;
    cout << "Right:      " << rightDir << endl;
    return !vTimeStamps.empty();
}

bool LoadIMU(const string &imuPath,
             vector<double> &vTimeStamps,
             vector<cv::Point3f> &vAcc,
             vector<cv::Point3f> &vGyro)
{
    vTimeStamps.clear();
    vAcc.clear();
    vGyro.clear();

    ifstream fImu(imuPath.c_str());
    if(!fImu.is_open())
    {
        cerr << "ERROR: Cannot open IMU file: " << imuPath << endl;
        return false;
    }

    string line;
    while(getline(fImu, line))
    {
        if(line.empty() || line[0] == '#')
            continue;

        const vector<string> fields = SplitComma(line);
        // ns, gx, gy, gz, ax, ay, az
        if(fields.size() < 7)
            continue;

        const double t = static_cast<double>(stoll(fields[0])) * 1e-9;
        const float gx = stof(fields[1]);
        const float gy = stof(fields[2]);
        const float gz = stof(fields[3]);
        const float ax = stof(fields[4]);
        const float ay = stof(fields[5]);
        const float az = stof(fields[6]);

        vTimeStamps.push_back(t);
        vGyro.push_back(cv::Point3f(gx, gy, gz));
        vAcc.push_back(cv::Point3f(ax, ay, az));
    }
    return !vTimeStamps.empty();
}

// IMREAD_GRAYSCALE already returns CV_8UC1 for these 16-bit PNGs. If a build
// leaves the image at 16-bit, keep the high byte, which is what this OpenCV
// does inside IMREAD_GRAYSCALE.
cv::Mat LoadGray8(const string &path)
{
    cv::Mat im = cv::imread(path, cv::IMREAD_GRAYSCALE);
    if(im.empty() || im.type() == CV_8UC1)
        return im;

    if(im.channels() != 1)
    {
        cv::Mat gray;
        cv::cvtColor(im, gray, cv::COLOR_BGR2GRAY);
        im = gray;
    }
    if(im.type() == CV_16UC1)
    {
        cv::Mat gray8(im.rows, im.cols, CV_8UC1);
        for(int y = 0; y < im.rows; ++y)
        {
            const uint16_t *src = im.ptr<uint16_t>(y);
            uint8_t *dst = gray8.ptr<uint8_t>(y);
            for(int x = 0; x < im.cols; ++x)
                dst[x] = static_cast<uint8_t>(src[x] >> 8);
        }
        return gray8;
    }

    cv::Mat gray8;
    im.convertTo(gray8, CV_8U);
    return gray8;
}

void ApplyFrameLimitAndStride(const string &settingsPath,
                              vector<string> &vstrImageLeft,
                              vector<string> &vstrImageRight,
                              vector<double> &vTimestampsCam)
{
    cv::FileStorage fsSettings(settingsPath, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
        return;

    int maxFrames = 0;
    const cv::FileNode maxNode = fsSettings["System.maxFrames"];
    if(!maxNode.empty())
        maxFrames = static_cast<int>(maxNode);
    if(maxFrames > 0 && maxFrames < static_cast<int>(vstrImageLeft.size()))
    {
        cout << "System.maxFrames=" << maxFrames << "  (using first "
             << maxFrames << " / " << vstrImageLeft.size() << " pairs)"
             << endl;
        vstrImageLeft.resize(static_cast<size_t>(maxFrames));
        vstrImageRight.resize(static_cast<size_t>(maxFrames));
        vTimestampsCam.resize(static_cast<size_t>(maxFrames));
    }

    int frameStride = 1;
    const cv::FileNode strideNode = fsSettings["System.frame_stride"];
    if(!strideNode.empty())
        frameStride = static_cast<int>(strideNode);
    if(frameStride < 1)
    {
        cerr << "ERROR: System.frame_stride must be an integer >= 1, got "
             << frameStride << endl;
        exit(1);
    }
    if(frameStride > 1)
    {
        const size_t nSrc = vstrImageLeft.size();
        vector<string> leftKept, rightKept;
        vector<double> timesKept;
        leftKept.reserve((nSrc + static_cast<size_t>(frameStride) - 1) /
                         static_cast<size_t>(frameStride));
        for(size_t i = 0; i < nSrc; i += static_cast<size_t>(frameStride))
        {
            leftKept.push_back(vstrImageLeft[i]);
            rightKept.push_back(vstrImageRight[i]);
            timesKept.push_back(vTimestampsCam[i]);
        }
        vstrImageLeft.swap(leftKept);
        vstrImageRight.swap(rightKept);
        vTimestampsCam.swap(timesKept);
        cout << "System.frame_stride=" << frameStride << "  (kept "
             << vstrImageLeft.size() << " / " << nSrc
             << " pairs; IMU between kept frames is still integrated)"
             << endl;
    }
    else
    {
        cout << "System.frame_stride=1  (kept all " << vstrImageLeft.size()
             << " pairs)" << endl;
    }
}

// Returns the number of leading frames removed. The caller must still have
// at least one frame left.
size_t DropFramesBeforeFirstImu(vector<string> &vstrImageLeft,
                                vector<string> &vstrImageRight,
                                vector<double> &vTimestampsCam,
                                double firstImuTime)
{
    size_t drop = 0;
    while(drop < vTimestampsCam.size() && vTimestampsCam[drop] < firstImuTime)
        ++drop;
    if(drop == 0)
        return 0;

    vstrImageLeft.erase(vstrImageLeft.begin(),
                        vstrImageLeft.begin() + static_cast<long>(drop));
    vstrImageRight.erase(vstrImageRight.begin(),
                         vstrImageRight.begin() + static_cast<long>(drop));
    vTimestampsCam.erase(vTimestampsCam.begin(),
                         vTimestampsCam.begin() + static_cast<long>(drop));
    return drop;
}

}  // namespace

int main(int argc, char **argv)
{
    if(argc < 4)
    {
        cerr << endl
             << "Usage: ./stereo_inertial_tum_vi path_to_vocabulary "
                "path_to_settings path_to_sequence [trajectory_file_name]"
             << endl;
        cerr << "Example:" << endl;
        cerr << "  ./stereo_inertial_tum_vi Vocabulary/ORBvoc.txt "
                "Examples/TagFusion/TUM_VI/TUM-VI_room_kfdense.yaml "
                "/home/yldu/datasets/TUM_VI/dataset-room1_512_16 room1"
             << endl;
        return 1;
    }

    const bool bFileName = (argc >= 5);
    if(bFileName)
        cout << "file name: " << argv[4] << endl;

    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestampsCam;
    cout << "Loading TUM-VI stereo sequence from " << argv[3] << " ..."
         << endl;
    if(!LoadImagesTUMVI(string(argv[3]), vstrImageLeft, vstrImageRight,
                        vTimestampsCam))
    {
        cerr << "ERROR: No stereo pairs loaded from " << argv[3] << endl;
        return 1;
    }
    cout << "LOADED " << vstrImageLeft.size() << " stereo pairs!" << endl;

    ApplyFrameLimitAndStride(string(argv[2]), vstrImageLeft, vstrImageRight,
                             vTimestampsCam);

    const string imuPath = StripTrailingSlash(string(argv[3])) +
                           "/mav0/imu0/data.csv";
    if(!FileExists(imuPath))
    {
        cerr << "ERROR: Cannot find " << imuPath << endl;
        return 1;
    }

    vector<double> vTimestampsImu;
    vector<cv::Point3f> vAcc, vGyro;
    cout << "Loading IMU from " << imuPath << " ..." << endl;
    if(!LoadIMU(imuPath, vTimestampsImu, vAcc, vGyro))
    {
        cerr << "ERROR: No IMU samples loaded from " << imuPath << endl;
        return 1;
    }
    cout << "LOADED " << vTimestampsImu.size() << " IMU samples!" << endl;

    if(vTimestampsImu.size() >= 2)
    {
        const double dt = (vTimestampsImu.back() - vTimestampsImu.front()) /
                          static_cast<double>(vTimestampsImu.size() - 1);
        const double freq = (dt > 0.0) ? (1.0 / dt) : 0.0;
        cout << "IMU mean dt=" << dt << " s  (~" << freq << " Hz)" << endl;
    }

    const double camMinusImu = vTimestampsCam.front() - vTimestampsImu.front();
    cout << "First camera minus first IMU: " << camMinusImu << " s" << endl;
    const size_t nDropped = DropFramesBeforeFirstImu(
        vstrImageLeft, vstrImageRight, vTimestampsCam, vTimestampsImu.front());
    if(vTimestampsCam.empty())
    {
        cerr << "ERROR: Every camera frame is before the first IMU sample"
             << endl;
        return 1;
    }
    if(nDropped > 0)
    {
        cout << "Dropped " << nDropped
             << " stereo frames that start before the first IMU sample"
             << endl;
    }
    else
    {
        cout << "Dropped 0 stereo frames before the first IMU sample"
             << endl;
    }
    cout << "First used camera timestamp: " << vTimestampsCam.front() << endl;
    cout << "Last camera timestamp:       " << vTimestampsCam.back() << endl;

    int first_imu = 0;
    while(first_imu < static_cast<int>(vTimestampsImu.size()) &&
          vTimestampsImu[first_imu] <= vTimestampsCam.front())
        ++first_imu;
    if(first_imu > 0)
        --first_imu;
    cout << "First IMU index used: " << first_imu
         << " t=" << vTimestampsImu[first_imu] << endl;

    cout << endl << "-------" << endl;
    cout.precision(17);

    const auto wallStart = std::chrono::steady_clock::now();
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_STEREO,
                           false);
    const float imageScale = SLAM.GetImageScale();
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));

    cv::Mat imLeft, imRight;
    size_t nProcessed = 0;
    bool reachedEnd = false;
    for(size_t ni = 0; ni < vstrImageLeft.size(); ni++)
    {
        imLeft = LoadGray8(vstrImageLeft[ni]);
        imRight = LoadGray8(vstrImageRight[ni]);
        const double tframe = vTimestampsCam[ni];

        if(imLeft.empty() || imLeft.type() != CV_8UC1)
        {
            cerr << endl << "Failed to load 8-bit left image at: "
                 << vstrImageLeft[ni] << endl;
            return 1;
        }
        if(imRight.empty() || imRight.type() != CV_8UC1)
        {
            cerr << endl << "Failed to load 8-bit right image at: "
                 << vstrImageRight[ni] << endl;
            return 1;
        }

        if(imageScale != 1.f)
        {
            const int width = static_cast<int>(imLeft.cols * imageScale);
            const int height = static_cast<int>(imLeft.rows * imageScale);
            cv::resize(imLeft, imLeft, cv::Size(width, height));
            cv::resize(imRight, imRight, cv::Size(width, height));
        }

        clahe->apply(imLeft, imLeft);
        clahe->apply(imRight, imRight);

        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        if(ni > 0)
        {
            while(first_imu < static_cast<int>(vTimestampsImu.size()) &&
                  vTimestampsImu[first_imu] <= vTimestampsCam[ni])
            {
                vImuMeas.emplace_back(vAcc[first_imu].x, vAcc[first_imu].y,
                                      vAcc[first_imu].z, vGyro[first_imu].x,
                                      vGyro[first_imu].y, vGyro[first_imu].z,
                                      vTimestampsImu[first_imu]);
                ++first_imu;
            }
        }

#ifdef COMPILEDWITHC11
        const std::chrono::steady_clock::time_point t1 =
            std::chrono::steady_clock::now();
#else
        const std::chrono::monotonic_clock::time_point t1 =
            std::chrono::monotonic_clock::now();
#endif

        SLAM.TrackStereo(imLeft, imRight, tframe, vImuMeas);

#ifdef COMPILEDWITHC11
        const std::chrono::steady_clock::time_point t2 =
            std::chrono::steady_clock::now();
#else
        const std::chrono::monotonic_clock::time_point t2 =
            std::chrono::monotonic_clock::now();
#endif

        const double ttrack =
            std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
                .count();

        double T = 0;
        if(ni + 1 < vTimestampsCam.size())
            T = vTimestampsCam[ni + 1] - tframe;
        else if(ni > 0)
            T = tframe - vTimestampsCam[ni - 1];

        if(ttrack < T)
            usleep(static_cast<useconds_t>((T - ttrack) * 1e6));

        nProcessed = ni + 1;
        if(ni + 1 == vstrImageLeft.size())
            reachedEnd = true;
    }

    SLAM.Shutdown();

    if(bFileName)
    {
        const string kf_file = "kf_" + string(argv[4]) + ".txt";
        const string f_file = "f_" + string(argv[4]) + ".txt";
        SLAM.SaveTrajectoryTUM(f_file);
        SLAM.SaveKeyFrameTrajectoryTUM(kf_file);
    }
    else
    {
        SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    }

    const double wallSeconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      wallStart)
            .count();
    cout << "-------" << endl;
    cout << "Reached sequence end: " << (reachedEnd ? "yes" : "no") << endl;
    cout << "Frames processed: " << nProcessed << " / " << vstrImageLeft.size()
         << endl;
    cout << "Frames dropped before IMU: " << nDropped << endl;
    cout << "Wall time s: " << wallSeconds << endl;

    return reachedEnd ? 0 : 1;
}

#ifndef JETSONCAR_LIBRARIES_ZED_H
#define JETSONCAR_LIBRARIES_ZED_H

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include <string>
#include <numeric>
#include <cmath>
#include <math.h>
#include <float.h>

#include <thread>
#include <mutex>
#include <memory> // shared pointer

#include <sl/Camera.hpp> // ZED include

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#define deg2rad(angleDegrees) ((angleDegrees) * M_PI / 180.0)
#define rad2deg(angleRadians) ((angleRadians) * 180.0 / M_PI)

// OBS: This library is for the original (first) ZED camera
class ZED
{

public:
    typedef struct Pose {
        double timestamp;
        Eigen::Affine3f pose;
    } Pose;

public:
    ZED();
    ~ZED();

    void Connect();
    void Disconnect();

    void ResetOrigin();

    cv::Mat GetLeftImage(bool rectified = false);
    cv::Mat GetRightImage(bool rectified = false);
    cv::Mat GetDepthImage();

    Pose GetPose();
    Eigen::Vector3f GetPoseTranslation();
    static Eigen::Vector3f GetPoseTranslation(Eigen::Affine3f pose);
    Eigen::Vector3f GetPoseEulerRPY(bool degrees = false);
    static Eigen::Vector3f GetPoseEulerRPY(Eigen::Affine3f pose, bool degrees = false);

    void RegisterCallback_Pose(const std::function<void(Pose&)> &callback);
    void RegisterCallback_LeftImage(const std::function<void(cv::Mat&)> &callback, bool rectified = false);
    void RegisterCallback_RightImage(const std::function<void(cv::Mat&)> &callback, bool rectified = false);
    void ClearCallbacks();

private:
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    sl::Camera zed;
    bool connected_{false};

    bool shouldExit{false};
    std::unique_ptr<std::thread> pollThread{nullptr};
    std::mutex mutex; // mutex guarding the measurements/samples below

    // Store past samples
    std::shared_ptr<sl::Mat> left_image_raw{nullptr};
    std::shared_ptr<sl::Mat> left_image_rectified{nullptr};
    std::shared_ptr<sl::Mat> right_image_raw{nullptr};
    std::shared_ptr<sl::Mat> right_image_rectified{nullptr};

    std::shared_ptr<sl::Mat> depth_map{nullptr};
    std::shared_ptr<sl::Mat> depth_color{nullptr};

    // Frames:
    //   W = World frame, z-axis up
    //   M = Map frame, z-axis forward, y-axis down
    //   RP = ZED Pose frame, z-axis forward, y-axis down
    //   P = Pose frame, z-axis up, x-axis forward
    //   LEFT = Left camera frame
    //   RIGHT = Right camera frame
    Eigen::Affine3f M_T_RP{Eigen::Affine3f::Identity()}; // Current ZED pose (reported by the ZED camera) in the ZED map frame
    Eigen::Affine3f W_T_P{Eigen::Affine3f::Identity()}; // Current Pose defined in the world frame with z-axis up and x-axis forward
    Eigen::Affine3f inv_M_T_RPinit{Eigen::Affine3f::Identity()}; // used to store desired origin
    Eigen::Affine3f W_T_M{Eigen::AngleAxis<float>(-M_PI_2, Eigen::Vector3f(0,0,1))*Eigen::AngleAxis<float>(-M_PI_2, Eigen::Vector3f(1,0,0))}; // correction frame to change ZED pose into World frame
    Eigen::Affine3f P_T_RP{Eigen::AngleAxis<float>(-M_PI_2, Eigen::Vector3f(0,0,1))*Eigen::AngleAxis<float>(-M_PI_2, Eigen::Vector3f(1,0,0))};
    Eigen::Affine3f LEFT_T_RP{Eigen::Affine3f::Identity()}; // Extrinsics (calibration) from Realsense pose to Left camera
    Eigen::Affine3f RIGHT_T_RP{Eigen::Affine3f::Identity()}; // Extrinsics (calibration) from Realsense pose to Right camera
    Eigen::Affine3f W_T_LEFT{Eigen::Affine3f::Identity()}; // Left Camera frame in World space
    Eigen::Affine3f W_T_RIGHT{Eigen::Affine3f::Identity()}; // Right Camera frame in World space

    // Intrinsics
    sl::CameraParameters IntrinsicsLeft;
    sl::CameraParameters IntrinsicsRight;
    size_t ImageWidth, ImageHeight;
    cv::Mat K_left, K_right;
    cv::Mat D_left, D_right;

    cv::Mat R_left, R_right; // Extrinsics between the cameras. Left rotation is set to identity.
    cv::Mat P_left, P_right; // Left and right projection matrices with the right projection matrix being shifted according to the baseine.

    // Rectification maps
    cv::Mat LeftMap1, LeftMap2;
    cv::Mat RightMap1, RightMap2;

    // Latest estimate
    Pose latestPose;

private:
    struct {
        std::vector<std::function<void(Pose&)>> pose;
        std::vector<std::pair<std::function<void(cv::Mat&)>, bool>> left_image;
        std::vector<std::pair<std::function<void(cv::Mat&)>, bool>> right_image;
    } callbacks;

private:
    sl::InitParameters Configure();

    void PollThread();

    void LoadCameraCalibrations();
    cv::Mat GetImage(sl::Mat &frame, bool left = false, bool rectified = false);

    void ResetInternalEstimator();

    static Eigen::Matrix<float, 4, 4> GetTransform(sl::Transform tf);
    static Eigen::Matrix<float, 4, 4> GetTransform(sl::Pose pose);

    static Eigen::Vector3f GetEulerAnglesZYX(Eigen::Matrix<float, 3, 3> rotm);
    static Eigen::Vector3f GetEulerAnglesZYX(Eigen::Matrix<float, 4, 4> transform);

    //static cv::Mat getOpenCVframe(const rs2::frame &frame);
    static cv::Mat toOpenCV(const sl::Mat& frame);

};

#endif

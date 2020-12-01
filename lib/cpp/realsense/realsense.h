#ifndef JETSONCAR_LIBRARIES_REALSENSE_H
#define JETSONCAR_LIBRARIES_REALSENSE_H

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

#include <librealsense2/rs.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#define deg2rad(angleDegrees) ((angleDegrees) * M_PI / 180.0)
#define rad2deg(angleRadians) ((angleRadians) * 180.0 / M_PI)

// OBS: This library is for the Realsense T265 only!
class Realsense
{

public:
    typedef struct SensorMeasurement {
        double time;
        Eigen::Vector3f meas;
    } SensorMeasurement;

public:
    Realsense();
    ~Realsense();

    bool Connect(bool Callback_nPollThread = true);
    void Disconnect();

    bool DeviceAvailable();
    void WaitForDevice();

    void ResetOrigin();

    cv::Mat GetLeftImage(bool rectified);
    cv::Mat GetRightImage(bool rectified);

    Eigen::Affine3f GetPose();
    Eigen::Vector3f GetPoseTranslation();
    static Eigen::Vector3f GetPoseTranslation(Eigen::Affine3f pose);
    Eigen::Vector3f GetPoseEulerRPY(bool degrees = false);
    static Eigen::Vector3f GetPoseEulerRPY(Eigen::Affine3f pose, bool degrees = false);

    SensorMeasurement GetAccelerometer();
    SensorMeasurement GetGyroscope();

    void RegisterCallback_Accelerometer(const std::function<void(SensorMeasurement)> &callback);
    void RegisterCallback_Gyroscope(const std::function<void(SensorMeasurement)> &callback);
    void RegisterCallback_Pose(const std::function<void(Eigen::Affine3f)> &callback);
    void RegisterCallback_LeftImage(const std::function<void(cv::Mat&)> &callback, bool rectified = false);
    void RegisterCallback_RightImage(const std::function<void(cv::Mat&)> &callback, bool rectified = false);
    void ClearCallbacks();

    // Misc test functions
    void Test_StreamingProfile();
    void Test_AllIntrinsics();
    void Test_IntrinsicsExtrinsics();
    void Test_UndistortAndDisparity();
    void Test_PoseProjection();

private:
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipeline;
    bool connected_{false};

    bool shouldExit{false};
    std::unique_ptr<std::thread> pollThread{nullptr};
    std::mutex mutex; // mutex guarding the measurements/samples below

    // Store past samples
    std::shared_ptr<rs2::video_frame> left_image{nullptr};
    std::shared_ptr<rs2::video_frame> right_image{nullptr};

    std::shared_ptr<rs2::motion_frame> accel_measurement{nullptr};
    std::shared_ptr<rs2::motion_frame> gyro_measurement{nullptr};

    // Frames:
    //   W = World frame, z-axis up
    //   M = Map frame (internal to T265), y-axis up
    //   RP = Realse Pose frame, z-axis backwards
    //   P = Pose frame, z-axis up, x-axis forward
    //   LEFT = Left camera frame
    //   RIGHT = Right camera frame
    Eigen::Affine3f M_T_RP{Eigen::Affine3f::Identity()}; // Current Realsense pose (reported by the realsense camera) in the Realsense map frame
    Eigen::Affine3f W_T_P{Eigen::Affine3f::Identity()}; // Current Pose defined in the world frame with z-axis up and x-axis forward
    Eigen::Affine3f inv_M_T_RPinit{Eigen::Affine3f::Identity()}; // used to store desired origin
    Eigen::Affine3f W_T_M{Eigen::AngleAxis<float>(-M_PI_2, Eigen::Vector3f(0,0,1))*Eigen::AngleAxis<float>(M_PI_2, Eigen::Vector3f(1,0,0))}; // correction frame to change Realsense pose into World frame
    Eigen::Affine3f P_T_RP{Eigen::AngleAxis<float>(-M_PI_2, Eigen::Vector3f(0,0,1))*Eigen::AngleAxis<float>(M_PI_2, Eigen::Vector3f(1,0,0))};
    Eigen::Affine3f LEFT_T_RP{Eigen::Affine3f::Identity()}; // Extrinsics (calibration) from Realsense pose to Left camera
    Eigen::Affine3f RIGHT_T_RP{Eigen::Affine3f::Identity()}; // Extrinsics (calibration) from Realsense pose to Right camera
    Eigen::Affine3f ACCEL_T_RP{Eigen::Affine3f::Identity()}; // Extrinsics (calibration) from Realsense pose to Accelerometer
    Eigen::Affine3f GYRO_T_RP{Eigen::Affine3f::Identity()}; // Extrinsics (calibration) from Realsense pose to Gyroscope
    Eigen::Affine3f W_T_LEFT{Eigen::Affine3f::Identity()}; // Left Camera frame in World space
    Eigen::Affine3f W_T_RIGHT{Eigen::Affine3f::Identity()}; // Right Camera frame in World space

    // Intrinsics
    rs2_intrinsics IntrinsicsLeft;
    rs2_intrinsics IntrinsicsRight;
    size_t ImageWidth, ImageHeight;
    cv::Mat K_left, K_right;
    cv::Mat D_left, D_right;

    cv::Mat R_left, R_right; // Extrinsics between the cameras. Left rotation is set to identity.
    cv::Mat P_left, P_right; // Left and right projection matrices with the right projection matrix being shifted according to the baseine.

    // Rectification maps
    cv::Mat LeftMap1, LeftMap2;
    cv::Mat RightMap1, RightMap2;

    // IMU intrinsics (includes scale, bias, cross axis sensitivity and variances)
    rs2_motion_device_intrinsic accel_intrinsics;
    rs2_motion_device_intrinsic gyro_intrinsics;

private:
    struct {
        std::vector<std::function<void(SensorMeasurement)>> accel;
        std::vector<std::function<void(SensorMeasurement)>> gyro;
        std::vector<std::function<void(Eigen::Affine3f)>> pose;
        std::vector<std::pair<std::function<void(cv::Mat&)>, bool>> left_image;
        std::vector<std::pair<std::function<void(cv::Mat&)>, bool>> right_image;
    } callbacks;

private:
    rs2::config Configure();

    void PollThread();
    void ProcessFrame(const rs2::frame &frame);

    void LoadCameraCalibrations();
    cv::Mat GetImage(rs2::video_frame &frame, bool rectified = false);
    bool VerifyIMUsupport() const;

    void ResetInternalEstimator();

    static Eigen::Matrix<float, 4, 4> GetTransform(rs2_pose& pose_data);
    static Eigen::Matrix<float, 4, 4> GetTransform(rs2_extrinsics& extrinsics);

    static Eigen::Vector3f GetEulerAnglesZYX(Eigen::Matrix<float, 3, 3> rotm);
    static Eigen::Vector3f GetEulerAnglesZYX(Eigen::Matrix<float, 4, 4> transform);

    static cv::Mat getOpenCVframe(const rs2::frame &frame);
    static cv::Mat toOpenCV(const rs2::video_frame& frame);
    static rs2::video_frame fromOpenCV(const cv::Mat& image, const rs2::video_frame& frame_for_profile, rs2::frame_source& src);

    static rs2_quaternion quaternion_exp(rs2_vector v);
    static rs2_quaternion quaternion_multiply(rs2_quaternion a, rs2_quaternion b);
    static rs2_pose predict_pose(rs2_pose & pose, float dt_s);

};

#endif

#include "realsense.h"
#include "api_helper.hpp"

Realsense::Realsense()
{

}

Realsense::~Realsense()
{
    if (connected_) {
        Disconnect();
    }
}

bool Realsense::Connect(bool Callback_nPollThread)
{
    if (connected_) return true;

    if (!DeviceAvailable()) return false;
    if (!VerifyIMUsupport()) return false;

    const auto cfg = Configure();

    // Callback_nPollThread defines whether to use the built-in Callback functionality of the Realsense SDK or
    // to spawn a processing thread in this library
    if (Callback_nPollThread) {
        // Start pipeline with chosen configurationand registered callbacks
        auto pipeline_profile = pipeline.start(cfg, std::bind(&Realsense::ProcessFrame, this, std::placeholders::_1));
        // Alternatively with
        /*pipeline.start(cfg, [&](const rs2::frame& frame) {
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (rs2::frameset fs = frame.as<rs2::frameset>())
            {
                // With callbacks, all synchronized stream will arrive in a single frameset
                for (const rs2::frame& f : fs)
                    counters[f.get_profile().unique_id()]++;
            }
            else
            {
                // Stream that bypass synchronization (such as IMU) will produce single frames
                counters[frame.get_profile().unique_id()]++;
            }
        });*/
        if (!pipeline_profile) return false;
    } else {
        auto pipeline_profile = pipeline.start(cfg);
        if (!pipeline_profile) return false;
    }

    // Get camera calibrations
    //auto fisheye_stream     = pipe_profile.get_stream(RS2_STREAM_FISHEYE, 1); // 1 = left fisheye lens of T265
    //auto fisheye_intrinsics = fisheye_stream.as<rs2::video_stream_profile>().get_intrinsics();
    //auto body_fisheye_extr  = fisheye_stream.get_extrinsics_to(pipe_profile.get_stream(RS2_STREAM_POSE));
    LoadCameraCalibrations();

    // Process the incoming data streams

    if (!Callback_nPollThread) {
        // Start poll thread
        pollThread = std::make_unique<std::thread>(std::bind(&Realsense::PollThread, this));
    }

    connected_ = true;
    return true;
}

rs2::config Realsense::Configure()
{
    // Set options before configuring the pipeline
    auto sensor = get_sensor(get_device());
    sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);       // Enable auto exposure
    sensor.set_option(RS2_OPTION_ENABLE_MAPPING, 1);             // Use an on device map (recommended)
    sensor.set_option(RS2_OPTION_ENABLE_RELOCALIZATION, 1);      // Use appearance based relocalization (depends on mapping)
    sensor.set_option(RS2_OPTION_ENABLE_POSE_JUMPING, 1);        // Allow pose jumping (depends on mapping)
    sensor.set_option(RS2_OPTION_ENABLE_DYNAMIC_CALIBRATION, 1); // Enable dynamic calibration (recommended)
    sensor.set_option(RS2_OPTION_ENABLE_MAP_PRESERVATION, 0);    // Preserve the map from the previous run as if it was loaded

    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    // Add pose stream
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);

    // Enable both image streams
    // Note: It is not currently possible to enable only one
    cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
    cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);

    // Add streams of gyro and accelerometer to configuration
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

    return cfg;
}

bool Realsense::DeviceAvailable()
{
    // See https://github.com/IntelRealSense/librealsense/blob/master/examples/sensor-control/api_how_to.h
    // And see api_helper.hpp
    //
    // First, create a rs2::context.
    // The context represents the current platform with respect to connected devices
    rs2::context ctx;

    // Using the context we can get all connected devices in a device list
    rs2::device_list devices = ctx.query_devices();

    return (devices.size() > 0);
}

void Realsense::WaitForDevice()
{
    // First, create a rs2::context.
    // The context represents the current platform with respect to connected devices
    rs2::context ctx;

    //To help with the boilerplate code of waiting for a device to connect
    //The SDK provides the rs2::device_hub class
    rs2::device_hub device_hub(ctx);

    //Using the device_hub we can block the program until a device connects
    device_hub.wait_for_device();
}

void Realsense::Disconnect()
{
    if (!connected_) return;

    if (pollThread) {
        shouldExit = true;
        pollThread->join();
        pollThread.reset();
    }

    pipeline.stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(200)); // allow some time to wait for the device to stop properly (see https://github.com/IntelRealSense/librealsense/issues/4642)

    connected_ = false;
}

void Realsense::PollThread()
{
    while (!shouldExit)
    {
        // Wait for the next set of frames from the camera
        auto frames = pipeline.wait_for_frames(500);
        // Process all the frames
        for (const rs2::frame& frame : frames) {
            ProcessFrame(frame);
        }
    }
}

void Realsense::ProcessFrame(const rs2::frame &frame)
{
    // Get a frame from the pose stream
    //auto frame = frames.first_or_default(RS2_STREAM_POSE);

    //auto fisheye_frame = frames.get_fisheye_frame(fisheye_sensor_idx);
    //auto frame_number  = fisheye_frame.get_frame_number();
    //auto camera_pose   = frames.get_pose_frame().get_pose_data();

    // With callbacks, all synchronized stream will arrive in a single frameset
    auto frameset = frame.as<rs2::frameset>();
    if (frameset) {
        for (const rs2::frame& frame : frameset) {
            ProcessFrame(frame);
        }
        return;
    }

    {
        std::lock_guard<std::mutex> lock(mutex);

        // Cast the frame that arrived to motion frame
        auto motion = frame.as<rs2::motion_frame>();
        // If casting succeeded and the arrived frame is from gyro stream
        if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
        {
            // Get the timestamp of the current frame
            double ts = motion.get_timestamp();
            // Get gyro measures
            //gyro_measurement = motion.get_motion_data();
            if (!gyro_measurement)
                gyro_measurement = std::make_shared<rs2::motion_frame>(motion);
            else
                *gyro_measurement = motion;

            if (callbacks.gyro.size() > 0) {
                auto meas = GetGyroscope();
                for (auto &cb : callbacks.gyro)
                    cb(meas);
            }

            return;
        }
        // If casting succeeded and the arrived frame is from accelerometer stream
        if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
        {
            // Get accelerometer measures
            //accel_measurement = motion.get_motion_data();
            if (!accel_measurement)
                accel_measurement = std::make_shared<rs2::motion_frame>(motion);
            else
                *accel_measurement = motion;

            if (callbacks.accel.size() > 0) {
                auto meas = GetAccelerometer();
                for (auto &cb : callbacks.accel)
                    cb(meas);
            }

            return;
        }

        auto fisheye = frame.as<rs2::video_frame>();
        if (fisheye && fisheye.get_profile().stream_type() == RS2_STREAM_FISHEYE) {
            if (fisheye.get_profile().stream_index() == 1) {
                if (!left_image)
                    left_image = std::make_shared<rs2::video_frame>(fisheye);
                else
                    *left_image = fisheye;

                if (callbacks.left_image.size() > 0) {
                    for (auto &cb : callbacks.left_image) {
                        auto image = GetImage(*left_image, cb.second);
                        cb.first(image);
                    }
                }

                return;
            } else if (fisheye.get_profile().stream_index() == 2) {
                if (!right_image)
                    right_image = std::make_shared<rs2::video_frame>(fisheye);
                else
                    *right_image = fisheye;

                if (callbacks.right_image.size() > 0) {
                    for (auto &cb : callbacks.right_image) {
                        auto image = GetImage(*right_image, cb.second);
                        cb.first(image);
                    }
                }

                return;
            }
        }

        // Cast the frame to pose_frame and get its data
        auto pose = frame.as<rs2::pose_frame>();
        if (pose) {
            auto pose_data = pose.get_pose_data();
            // Calculate current transformation matrix
            M_T_RP = GetTransform(pose_data);

            // Transform into desired frames
            W_T_P = W_T_M * inv_M_T_RPinit * M_T_RP * P_T_RP.inverse();
            W_T_LEFT = W_T_M * inv_M_T_RPinit * M_T_RP * LEFT_T_RP.inverse();
            W_T_RIGHT = W_T_M * inv_M_T_RPinit * M_T_RP * RIGHT_T_RP.inverse();

            if (callbacks.pose.size() > 0) {
                for (auto &cb : callbacks.pose)
                    cb(W_T_P);
            }

            return;
        }
    }
}

void Realsense::RegisterCallback_Accelerometer(const std::function<void(SensorMeasurement)> &callback)
{
    callbacks.accel.emplace_back(callback);
}
void Realsense::RegisterCallback_Gyroscope(const std::function<void(SensorMeasurement)> &callback)
{
    callbacks.gyro.emplace_back(callback);
}
void Realsense::RegisterCallback_Pose(const std::function<void(Eigen::Affine3f)> &callback)
{
    callbacks.pose.emplace_back(callback);
}
void Realsense::RegisterCallback_LeftImage(const std::function<void(cv::Mat&)> &callback, bool rectified)
{
    callbacks.left_image.emplace_back(callback, rectified);
}
void Realsense::RegisterCallback_RightImage(const std::function<void(cv::Mat&)> &callback, bool rectified)
{
    callbacks.right_image.emplace_back(callback, rectified);
}
void Realsense::ClearCallbacks()
{
    callbacks.accel.clear();
    callbacks.gyro.clear();
    callbacks.pose.clear();
    callbacks.left_image.clear();
    callbacks.right_image.clear();
}

void Realsense::LoadCameraCalibrations()
{
    auto pipeline_profiles = pipeline.get_active_profile();

    // Load Extrinsics
    auto left_camera = pipeline_profiles.get_stream(RS2_STREAM_FISHEYE, 1).as<rs2::video_stream_profile>();
    auto right_camera = pipeline_profiles.get_stream(RS2_STREAM_FISHEYE, 2).as<rs2::video_stream_profile>();
    auto pose_profile = pipeline_profiles.get_stream(RS2_STREAM_POSE).as<rs2::pose_stream_profile>();

    // Extract pose to camera extrinsincs
    auto pose_to_leftcam_extrinsics = pose_profile.get_extrinsics_to(left_camera);
    LEFT_T_RP = GetTransform(pose_to_leftcam_extrinsics);
    auto pose_to_rightcam_extrinsics = pose_profile.get_extrinsics_to(right_camera);
    RIGHT_T_RP = GetTransform(pose_to_rightcam_extrinsics);

    // Load Camera intrinsics
    IntrinsicsLeft = left_camera.get_intrinsics();
    IntrinsicsRight = right_camera.get_intrinsics();

    ImageWidth = IntrinsicsLeft.width;
    ImageHeight = IntrinsicsLeft.height;

    K_left = cv::Mat(3, 3, CV_32F, cv::Scalar(0.0));
    K_left.at<float>(0, 0) = IntrinsicsLeft.fx;
    K_left.at<float>(0, 2) = IntrinsicsLeft.ppx;
    K_left.at<float>(1, 1) = IntrinsicsLeft.fy;
    K_left.at<float>(1, 2) = IntrinsicsLeft.ppy;
    K_left.at<float>(2, 2) = 1;

    K_right = cv::Mat(3, 3, CV_32F, cv::Scalar(0.0));
    K_right.at<float>(0, 0) = IntrinsicsRight.fx;
    K_right.at<float>(0, 2) = IntrinsicsRight.ppx;
    K_right.at<float>(1, 1) = IntrinsicsRight.fy;
    K_right.at<float>(1, 2) = IntrinsicsRight.ppy;
    K_right.at<float>(2, 2) = 1;

    D_left = cv::Mat(1, 4, CV_32F, IntrinsicsLeft.coeffs);
    D_right = cv::Mat(1, 4, CV_32F, IntrinsicsRight.coeffs);

    // Get the relative extrinsics between the left and right camera
    auto LEFT_T_RIGHT = left_camera.get_extrinsics_to(right_camera);

    cv::Mat R = cv::Mat(3, 3, CV_32F, LEFT_T_RIGHT.rotation);
    cv::Mat T = cv::Mat(3, 1, CV_32F, LEFT_T_RIGHT.translation);

    // We set the left rotation to identity and the right rotation
    // the rotation between the cameras
    R_left = cv::Mat::eye(3, 3, CV_32F);
    R_right = R;

    // Compute rectification maps based on a desired FOV in the rectified image
    const float rectified_FOV_rad = deg2rad(120);  // 120 degree desired fov
    const int rectified_size_px = std::min(ImageWidth, ImageHeight);    // output resolution
    const float rectified_focal_px = rectified_size_px/2 / tanf(rectified_FOV_rad/2);
    const cv::Size rectified_size(rectified_size_px, rectified_size_px);

    // Construct the left and right projection matrices, the only difference is
    // that the right projection matrix should have a shift along the x axis of
    // baseline*focal_length
    P_left = cv::Mat(3, 4, CV_32F, 0.0f);
    P_left.at<float>(0,0) = rectified_focal_px;
    P_left.at<float>(0,2) = (rectified_size_px - 1)/2;
    P_left.at<float>(1,1) = rectified_focal_px;
    P_left.at<float>(1,2) = (rectified_size_px - 1)/2;
    P_left.at<float>(2,2) = 1.0f;
    P_right = P_left;
    P_right.at<float>(0, 3) = T.at<float>(0) * rectified_focal_px;

    // Create an undistortion map for the left and right camera which applies the
    // rectification and undoes the camera distortion. This only has to be done
    // once
    cv::Mat _lm1, _lm2, lm1, lm2;
    cv::fisheye::initUndistortRectifyMap(K_left, D_left, R_left, P_left, rectified_size, CV_32FC1, _lm1, _lm2);
    cv::convertMaps(_lm1, _lm2, LeftMap1, LeftMap2, CV_16SC2);
    cv::Mat _rm1, _rm2, rm1, rm2;
    cv::fisheye::initUndistortRectifyMap(K_right, D_right, R_right, P_right, rectified_size, CV_32FC1, _rm1, _rm2);
    cv::convertMaps(_rm1, _rm2, RightMap1, RightMap2, CV_16SC2);

    // Get IMU intrinsics
    auto accel = pipeline_profiles.get_stream(RS2_STREAM_ACCEL).as<rs2::motion_stream_profile>();
    auto gyro = pipeline_profiles.get_stream(RS2_STREAM_GYRO).as<rs2::motion_stream_profile>();
    accel_intrinsics = accel.get_motion_intrinsics();
    gyro_intrinsics = gyro.get_motion_intrinsics();

    // Extract IMU to pose extrinsincs
    auto pose_to_accel_extrinsics = pose_profile.get_extrinsics_to(accel);
    auto pose_to_gyro_extrinsics = pose_profile.get_extrinsics_to(gyro);
    ACCEL_T_RP = GetTransform(pose_to_leftcam_extrinsics);
    GYRO_T_RP = GetTransform(pose_to_gyro_extrinsics);
}

void Realsense::ResetOrigin()
{
    // Reset the origin to the current pose
    // M_T_RP = inv(M_T_RPinit) * M_T_RP;
    inv_M_T_RPinit = M_T_RP.inverse();
}

cv::Mat Realsense::GetImage(rs2::video_frame &frame, bool rectified)
{
    auto image = toOpenCV(frame);

    rs2_intrinsics intrinsics = frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
    if (rectified && intrinsics.model == RS2_DISTORTION_KANNALA_BRANDT4) {
        // See also rs2_project_point_to_pixel and rs2_deproject_pixel_to_point in rsutil.h
        // cv::remap is similar to cv::fisheye::undistortImage, although undistortImage does not work
        cv::Mat image_rectified;
        if (frame.get_profile().stream_index() == 1) { // left camera
            cv::remap(image, image_rectified, LeftMap1, LeftMap2, cv::INTER_LINEAR);
            return image_rectified;
        } else if (frame.get_profile().stream_index() == 2) {
            cv::remap(image, image_rectified, RightMap1, RightMap2, cv::INTER_LINEAR);
            return image_rectified;
        }
    }

    return image;
}

cv::Mat Realsense::GetLeftImage(bool rectified)
{
    if (left_image)
        return GetImage(*left_image, rectified);
    else
        return cv::Mat();
}

cv::Mat Realsense::GetRightImage(bool rectified)
{
    if (right_image)
        return GetImage(*right_image, rectified);
    else
        return cv::Mat();
}

Eigen::Affine3f Realsense::GetPose()
{
    return W_T_P;
}

Eigen::Vector3f Realsense::GetPoseTranslation()
{
    return W_T_P.translation().matrix();
}

Eigen::Vector3f Realsense::GetPoseTranslation(Eigen::Affine3f pose)
{
    return pose.translation().matrix();
}

Eigen::Vector3f Realsense::GetPoseEulerRPY(bool degrees)
{
    return GetPoseEulerRPY(W_T_P, degrees);
}

Eigen::Vector3f Realsense::GetPoseEulerRPY(Eigen::Affine3f pose, bool degrees)
{
    Eigen::Vector3f euler_ZYX = GetEulerAnglesZYX(pose.matrix()); // The output format is [yaw, pitch, roll]
    if (degrees)
        return Eigen::Vector3f(rad2deg(euler_ZYX(2)), rad2deg(euler_ZYX(1)), rad2deg(euler_ZYX(0))); // Swap it to make the output [roll, pitch, yaw]
    else
        return Eigen::Vector3f(euler_ZYX(2), euler_ZYX(1), euler_ZYX(0)); // Swap it to make the output [roll, pitch, yaw]
}


Realsense::SensorMeasurement Realsense::GetAccelerometer()
{
    SensorMeasurement measurement;
    if (!accel_measurement) return measurement;

    measurement.time = accel_measurement->get_timestamp();

    auto data = accel_measurement->get_motion_data();
    measurement.meas(0) = data.x;
    measurement.meas(1) = data.y;
    measurement.meas(2) = data.z;

    return measurement;
}

Realsense::SensorMeasurement Realsense::GetGyroscope()
{
    SensorMeasurement measurement;
    if (!gyro_measurement) return measurement;

    measurement.time = gyro_measurement->get_timestamp();

    auto data = gyro_measurement->get_motion_data();
    measurement.meas(0) = data.x;
    measurement.meas(1) = data.y;
    measurement.meas(2) = data.z;

    return measurement;
}

bool Realsense::VerifyIMUsupport() const
{
    rs2::context ctx;
    bool found_gyro = false;
    bool found_accel = false;

    for (auto dev : ctx.query_devices())
    {
        // The same device should support gyro and accel
        found_gyro = false;
        found_accel = false;
        for (auto sensor : dev.query_sensors())
        {
            for (auto profile : sensor.get_stream_profiles())
            {
                if (profile.stream_type() == RS2_STREAM_GYRO)
                    found_gyro = true;

                if (profile.stream_type() == RS2_STREAM_ACCEL)
                    found_accel = true;
            }
        }
        if (found_gyro && found_accel)
            break;
    }
    return found_gyro && found_accel;
}

// Calculates transformation matrix based on pose data from the device
Eigen::Matrix<float, 4, 4> Realsense::GetTransform(rs2_pose& pose_data)
{
    Eigen::Matrix<float, 4, 4, Eigen::ColMajor> mat;
    auto q = pose_data.rotation;
    auto t = pose_data.translation;
    // Convert the quaternion into a rotation matrix, stored in column-major format
    mat.data()[0] = 1 - 2 * q.y*q.y - 2 * q.z*q.z;   mat.data()[4] = 2 * q.x*q.y - 2 * q.z*q.w;     mat.data()[8] = 2 * q.x*q.z + 2 * q.y*q.w;        mat.data()[12] = t.x;
    mat.data()[1] = 2 * q.x*q.y + 2 * q.z*q.w;       mat.data()[5] = 1 - 2 * q.x*q.x - 2 * q.z*q.z; mat.data()[9] = 2 * q.y*q.z - 2 * q.x*q.w;        mat.data()[13] = t.y;
    mat.data()[2] = 2 * q.x*q.z - 2 * q.y*q.w;       mat.data()[6] = 2 * q.y*q.z + 2 * q.x*q.w;     mat.data()[10] = 1 - 2 * q.x*q.x - 2 * q.y*q.y;   mat.data()[14] = t.z;
    mat.data()[3] = 0.0f;                            mat.data()[7] = 0.0f;                          mat.data()[11] = 0.0f;                            mat.data()[15] = 1.0f;
    return mat;
}

// Calculates transformation matrix based on pose data from the device
Eigen::Matrix<float, 4, 4> Realsense::GetTransform(rs2_extrinsics& extrinsics)
{
    Eigen::Matrix<float, 4, 4, Eigen::ColMajor> mat;
    auto R = extrinsics.rotation;
    auto t = extrinsics.translation;
    // Set the matrix as column-major for convenient work with OpenGL and rotate by 180 degress (by negating 1st and 3rd columns)
    mat.data()[0] = R[0]; mat.data()[4] = R[3]; mat.data()[8] = R[6];  mat.data()[12] = t[0];
    mat.data()[1] = R[1]; mat.data()[5] = R[4]; mat.data()[9] = R[7];  mat.data()[13] = t[1];
    mat.data()[2] = R[2]; mat.data()[6] = R[5]; mat.data()[10] = R[8]; mat.data()[14] = t[2];
    mat.data()[3] = 0.0f; mat.data()[7] = 0.0f; mat.data()[11] = 0.0f; mat.data()[15] = 1.0f;
    return mat;
}

cv::Mat Realsense::toOpenCV(const rs2::video_frame& frame)
{
    const int w = frame.get_width();
    const int h = frame.get_height();

    const int bits_per_pixel = frame.get_bits_per_pixel();
    if (bits_per_pixel == 8)
        return cv::Mat(cv::Size(w, h), CV_8UC1, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
    else if (bits_per_pixel == 24)
        return cv::Mat(cv::Size(w, h), CV_8UC3, (void*)frame.get_data(), cv::Mat::AUTO_STEP);

    return cv::Mat();
}

rs2::video_frame Realsense::fromOpenCV(const cv::Mat& image, const rs2::video_frame& frame_for_profile, rs2::frame_source& src)
{
    const int w = image.cols;
    const int h = image.rows;

    // Allocate new frame. Copy all missing data from f.
    // This assumes the output is same resolution and format
    // if not true, need to specify parameters explicitly
    auto res = src.allocate_video_frame(frame_for_profile.get_profile(), frame_for_profile, 0, w, h);

    // copy from cv --> frame
    memcpy((void*)res.get_data(), image.data, w * h * 2);

    // Send the resulting frame to the output queue
    src.frame_ready(res);
}

Eigen::Vector3f Realsense::GetEulerAnglesZYX(Eigen::Matrix<float, 3, 3> rotm)
{
    Eigen::Vector3f eul; // yaw, pitch, roll

    // convention used by (*) and (**).
    // note: the final orientation is the same as in XYZ order about fixed axes ...
    if (rotm(2,0) < 1) {
        if (rotm(2,0) > -1) { // case 1: if r31 ~= ±1
            // Solution with positive sign. It limits the range of the values
            // of theta_y to (-pi/2, pi/2):
            eul(0) = atan2f(rotm(1,0), rotm(0,0)); // theta_z
            eul(1) = asinf(-rotm(2,0));            // theta_y
            eul(2) = atan2f(rotm(2,1), rotm(2,2)); // theta_x
        } else { // case 2: if r31 = -1
            // theta_x and theta_z are linked --> Gimbal lock:
            // There are infinity number of solutions for theta_x - theta_z = atan2(-r23, r22).
            // To find a solution, set theta_x = 0 by convention.
            eul(0) = -atan2f(-rotm(1,2), rotm(1,1));
            eul(1) = M_PI / 2.f;
            eul(2) = 0;
        }
    } else { // case 3: if r31 = 1
        // Gimbal lock: There is not a unique solution for
        //   theta_x + theta_z = atan2(-r23, r22), by convention, set theta_x = 0.
        eul(0) = atan2(-rotm(1,2), rotm(1,1));
        eul(1) = -M_PI / 2.f;
        eul(2) = 0;
    }

    return eul;
}

Eigen::Vector3f Realsense::GetEulerAnglesZYX(Eigen::Matrix<float, 4, 4> transform)
{
    return GetEulerAnglesZYX(static_cast<Eigen::Matrix<float, 3, 3>>(transform.block<3,3>(0,0).matrix()));
}

rs2_quaternion Realsense::quaternion_exp(rs2_vector v)
{
    float x = v.x/2, y = v.y/2, z = v.z/2, th2, th = sqrtf(th2 = x*x + y*y + z*z);
    float c = cosf(th), s = th2 < sqrtf(120*FLT_EPSILON) ? 1-th2/6 : sinf(th)/th;
    rs2_quaternion Q = { s*x, s*y, s*z, c };
    return Q;
}

rs2_quaternion Realsense::quaternion_multiply(rs2_quaternion a, rs2_quaternion b)
{
    rs2_quaternion Q = {
            a.x * b.w + a.w * b.x - a.z * b.y + a.y * b.z,
            a.y * b.w + a.z * b.x + a.w * b.y - a.x * b.z,
            a.z * b.w - a.y * b.x + a.x * b.y + a.w * b.z,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    };
    return Q;
}

rs2_pose Realsense::predict_pose(rs2_pose & pose, float dt_s)
{
    rs2_pose P = pose;
    P.translation.x = dt_s * (dt_s/2 * pose.acceleration.x + pose.velocity.x) + pose.translation.x;
    P.translation.y = dt_s * (dt_s/2 * pose.acceleration.y + pose.velocity.y) + pose.translation.y;
    P.translation.z = dt_s * (dt_s/2 * pose.acceleration.z + pose.velocity.z) + pose.translation.z;
    rs2_vector W = {
            dt_s * (dt_s/2 * pose.angular_acceleration.x + pose.angular_velocity.x),
            dt_s * (dt_s/2 * pose.angular_acceleration.y + pose.angular_velocity.y),
            dt_s * (dt_s/2 * pose.angular_acceleration.z + pose.angular_velocity.z),
    };
    P.rotation = quaternion_multiply(quaternion_exp(W), pose.rotation);
    return P;
}

void Realsense::ResetInternalEstimator()
{
    pipeline.stop();
    std::this_thread::sleep_for( std::chrono::seconds( 1 ) ); // wait a little bit
    pipeline.start();
}

void Realsense::Test_StreamingProfile()
{
    rs2::device dev = get_device();
    rs2::sensor sensor = get_sensor(dev);
    rs2::stream_profile profile = get_streaming_profile(sensor, 0);

    test_streaming_profile(sensor, profile);
}

void Realsense::Test_AllIntrinsics()
{
    pipeline.start(Configure());

    auto profiles = pipeline.get_active_profile().get_streams();
    for (auto profile : profiles)
        get_intrinsics(profile);

    Disconnect();
}

void Realsense::Test_IntrinsicsExtrinsics()
{
    pipeline.start(Configure());

    list_streaming_profiles(pipeline.get_active_profile().get_streams());

    auto pipeline_profiles = pipeline.get_active_profile();
    auto fisheye_stream     = pipeline_profiles.get_stream(RS2_STREAM_FISHEYE, 1); // Left camera
    auto fisheye_intrinsics = fisheye_stream.as<rs2::video_stream_profile>().get_intrinsics();
    auto body_fisheye_extr  = fisheye_stream.get_extrinsics_to(pipeline_profiles.get_stream(RS2_STREAM_FISHEYE, 2));

    print_intrinsics(fisheye_intrinsics);
    std::cout << std::endl;
    print_extrinsics(body_fisheye_extr);

    Disconnect();
}

void Realsense::Test_UndistortAndDisparity()
{
    pipeline.start(Configure());

    // Undistortion of the fish-eye camera, see: https://docs.opencv.org/3.1.0/db/d58/group__calib3d__fisheye.html#gsc.tab=0

    // Configure the OpenCV stereo algorithm. See
    // https://docs.opencv.org/3.4/d2/d85/classcv_1_1StereoSGBM.html for a
    // description of the parameters
    const int window_size = 5;
    const int min_disp = 0;
    // must be divisible by 16
    const int num_disp = 80 - min_disp;
    const int max_disp = min_disp + num_disp;
    auto stereo = cv::StereoSGBM::create(min_disp, num_disp, 15, 8*3*window_size*window_size, 32*3*window_size*window_size, 1, 0, 10, 100, 2);

    auto pipeline_profiles = pipeline.get_active_profile();
    auto left_camera = pipeline_profiles.get_stream(RS2_STREAM_FISHEYE, 1).as<rs2::video_stream_profile>();
    auto right_camera = pipeline_profiles.get_stream(RS2_STREAM_FISHEYE, 2).as<rs2::video_stream_profile>();
    auto left_intrinsics = left_camera.get_intrinsics();
    auto right_intrinsics = right_camera.get_intrinsics();

    cv::Mat K_left(3, 3, CV_32F, cv::Scalar(0.0));
    K_left.at<float>(0, 0) = left_intrinsics.fx;
    K_left.at<float>(0, 2) = left_intrinsics.ppx;
    K_left.at<float>(1, 1) = left_intrinsics.fy;
    K_left.at<float>(1, 2) = left_intrinsics.ppy;
    K_left.at<float>(2, 2) = 1;

    cv::Mat K_right(3, 3, CV_32F, cv::Scalar(0.0));
    K_right.at<float>(0, 0) = right_intrinsics.fx;
    K_right.at<float>(0, 2) = right_intrinsics.ppx;
    K_right.at<float>(1, 1) = right_intrinsics.fy;
    K_right.at<float>(1, 2) = right_intrinsics.ppy;
    K_right.at<float>(2, 2) = 1;

    cv::Mat D_left = cv::Mat(1, 4, CV_32F, left_intrinsics.coeffs);
    cv::Mat D_right = cv::Mat(1, 4, CV_32F, right_intrinsics.coeffs);

    int width = left_intrinsics.width;
    int height = left_intrinsics.height;

    // Get the relative extrinsics between the left and right camera
    auto extrinsics = left_camera.get_extrinsics_to(right_camera);

    cv::Mat R = cv::Mat(3, 3, CV_32F, extrinsics.rotation);
    cv::Mat T = cv::Mat(3, 1, CV_32F, extrinsics.translation);

//    // We need to determine what focal length our undistorted images should have
//    // in order to set up the camera matrices for initUndistortRectifyMap.  We
//    // could use stereoRectify, but here we show how to derive these projection
//    // matrices from the calibration and a desired height and field of view
//
//    // We calculate the undistorted focal length:
//    //
//    //         h
//    // -----------------
//    //  \      |      /
//    //    \    | f  /
//    //     \   |   /
//    //      \ fov /
//    //        \|/
    float stereo_fov_rad = 140 * (M_PI/180);  // 90 degree desired fov
    int stereo_height_px = 600;         // 300x300 pixel stereo output
    float stereo_focal_px = stereo_height_px/2 / tanf(stereo_fov_rad/2);
//
//    // We set the left rotation to identity and the right rotation
//    // the rotation between the cameras
    cv::Mat R_left = cv::Mat::eye(3, 3, CV_32F);
    cv::Mat R_right = R;
//
//    // The stereo algorithm needs max_disp extra pixels in order to produce valid
//    // disparity on the desired output region. This changes the width, but the
//    // center of projection should be on the center of the cropped image
    int stereo_width_px = stereo_height_px + max_disp;
    cv::Size stereo_size(stereo_width_px, stereo_height_px);
    float stereo_cx = (stereo_height_px - 1)/2 + max_disp;
    float stereo_cy = (stereo_height_px - 1)/2;
//
//    // Construct the left and right projection matrices, the only difference is
//    // that the right projection matrix should have a shift along the x axis of
//    // baseline*focal_length
    cv::Mat P_left(3, 4, CV_32F, 0.0f);
    P_left.at<float>(0,0) = stereo_focal_px;
    P_left.at<float>(0,2) = stereo_cx;
    P_left.at<float>(1,1) = stereo_focal_px;
    P_left.at<float>(1,2) = stereo_cy;
    P_left.at<float>(2,2) = 1.0f;
    cv::Mat P_right = P_left;
    P_right.at<float>(0, 3) = T.at<float>(0) * stereo_focal_px;

    // Create an undistortion map for the left and right camera which applies the
    // rectification and undoes the camera distortion. This only has to be done
    // once
    cv::Mat _lm1, _lm2, lm1, lm2;
    cv::fisheye::initUndistortRectifyMap(K_left, D_left, R_left, P_left, stereo_size, CV_32FC1, _lm1, _lm2);
    cv::convertMaps(_lm1, _lm2, lm1, lm2, CV_16SC2);
    cv::Mat _rm1, _rm2, rm1, rm2;
    cv::fisheye::initUndistortRectifyMap(K_right, D_right, R_right, P_right, stereo_size, CV_32FC1, _rm1, _rm2);
    cv::convertMaps(_rm1, _rm2, rm1, rm2, CV_16SC2);

    while (1) {
        auto frames = pipeline.wait_for_frames(2000);
        std::cout << "Received a frameset consisting of " << frames.size() << " frames" << std::endl;

        cv::Mat left, right;
        cv::Mat left_undistorted, right_undistorted;

        for (const auto &frame : frames) {
            if (frame.get_profile().stream_type() == RS2_STREAM_FISHEYE &&
                frame.get_profile().stream_index() == 1) { // left fisheye
                auto image_frame = frame.as<rs2::video_frame>();
                left = toOpenCV(image_frame);

                rs2_intrinsics intrinsics = frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
                if (intrinsics.model == RS2_DISTORTION_KANNALA_BRANDT4) {
                    // See also rs2_project_point_to_pixel and rs2_deproject_pixel_to_point in rsutil.h
                    cv::remap(left, left_undistorted, lm1, lm2, cv::INTER_LINEAR); // similar to cv::fisheye::undistortImage, although undistortImage does not work
                }

            } else if (frame.get_profile().stream_type() == RS2_STREAM_FISHEYE &&
                       frame.get_profile().stream_index() == 2) { // right fisheye
                auto image_frame = frame.as<rs2::video_frame>();
                right = toOpenCV(image_frame);

                rs2_intrinsics intrinsics = frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
                if (intrinsics.model == RS2_DISTORTION_KANNALA_BRANDT4) {
                    // See also rs2_project_point_to_pixel and rs2_deproject_pixel_to_point in rsutil.h
                    cv::remap(right, right_undistorted, rm1, rm2, cv::INTER_LINEAR); // similar to cv::fisheye::undistortImage, although undistortImage does not work
                }
            }
        }

        // compute the disparity on the center of the frames and convert it to a pixel disparity (divide by DISP_SCALE=16)
        cv::Mat disparity;
        stereo->compute(left_undistorted, right_undistorted, disparity);
        cv::Mat disparity_F32;
        disparity.convertTo(disparity_F32, CV_32F, 1.0/16.0);

        // convert disparity to 0-255 and color it
        cv::Mat disp_vis;
        disparity_F32.convertTo(disp_vis, -1, 255.0/num_disp, -255.0*min_disp/num_disp); // disp_vis = 255*(disparity - min_disp)/ num_disp
        cv::convertScaleAbs(disp_vis, disp_vis); // disp_vis = abs(disp_vis)
        cv::Mat disp_color;
        cv::applyColorMap(disp_vis, disp_color, cv::COLORMAP_JET);

        cv::Mat top, bottom;
        cv::hconcat(right, left, top);
        cv::hconcat(right_undistorted, left_undistorted, bottom);

        cv::namedWindow("Fisheye", cv::WINDOW_NORMAL);
        cv::imshow("Fisheye", top);

        cv::namedWindow("Undistorted", cv::WINDOW_NORMAL);
        cv::imshow("Undistorted", bottom);

        cv::namedWindow("Disparity", cv::WINDOW_NORMAL);
        cv::imshow("Disparity", disp_color);

        cv::waitKey(10);
    }

    Disconnect();
}


void Realsense::Test_PoseProjection()
{
    pipeline.start(Configure());

    // Undistortion of the fish-eye camera, see: https://docs.opencv.org/3.1.0/db/d58/group__calib3d__fisheye.html#gsc.tab=0

    // Configure the OpenCV stereo algorithm. See
    // https://docs.opencv.org/3.4/d2/d85/classcv_1_1StereoSGBM.html for a
    // description of the parameters
    const int window_size = 5;
    const int min_disp = 0;
    // must be divisible by 16
    const int num_disp = 80 - min_disp;
    const int max_disp = min_disp + num_disp;

    auto pipeline_profiles = pipeline.get_active_profile();
    auto left_camera = pipeline_profiles.get_stream(RS2_STREAM_FISHEYE, 1).as<rs2::video_stream_profile>();
    auto right_camera = pipeline_profiles.get_stream(RS2_STREAM_FISHEYE, 2).as<rs2::video_stream_profile>();
    auto pose_profile = pipeline_profiles.get_stream(RS2_STREAM_POSE).as<rs2::pose_stream_profile>();
    auto left_intrinsics = left_camera.get_intrinsics();
    auto right_intrinsics = right_camera.get_intrinsics();

    cv::Mat K_left(3, 3, CV_32F, cv::Scalar(0.0));
    K_left.at<float>(0, 0) = left_intrinsics.fx;
    K_left.at<float>(0, 2) = left_intrinsics.ppx;
    K_left.at<float>(1, 1) = left_intrinsics.fy;
    K_left.at<float>(1, 2) = left_intrinsics.ppy;
    K_left.at<float>(2, 2) = 1;

    cv::Mat K_right(3, 3, CV_32F, cv::Scalar(0.0));
    K_right.at<float>(0, 0) = right_intrinsics.fx;
    K_right.at<float>(0, 2) = right_intrinsics.ppx;
    K_right.at<float>(1, 1) = right_intrinsics.fy;
    K_right.at<float>(1, 2) = right_intrinsics.ppy;
    K_right.at<float>(2, 2) = 1;

    cv::Mat D_left = cv::Mat(1, 4, CV_32F, left_intrinsics.coeffs);
    cv::Mat D_right = cv::Mat(1, 4, CV_32F, right_intrinsics.coeffs);

    int width = left_intrinsics.width;
    int height = left_intrinsics.height;

    // Get the relative extrinsics between the left and right camera
    auto extrinsics = left_camera.get_extrinsics_to(right_camera);

    cv::Mat R = cv::Mat(3, 3, CV_32F, extrinsics.rotation);
    cv::Mat T = cv::Mat(3, 1, CV_32F, extrinsics.translation);

    // Extract pose to camera extrinsincs
    auto pose_to_leftcam_extrinsics = pose_profile.get_extrinsics_to(left_camera);
    Eigen::Matrix<float, 4, 4> pose_to_leftcam_extrinsics_eigen = GetTransform(pose_to_leftcam_extrinsics);
    cv::Mat pose_to_leftcam_extrinsics_cv(4, 4, CV_32F);
    cv::eigen2cv(pose_to_leftcam_extrinsics_eigen, pose_to_leftcam_extrinsics_cv);
    cv::Affine3f pose_to_leftcam_extrinsics_cv_affine(pose_to_leftcam_extrinsics_cv);

    Eigen::Matrix<float, 4, 4> C_T_P = pose_to_leftcam_extrinsics_eigen;
    std::cout << C_T_P << std::endl << std::endl;

//    // We need to determine what focal length our undistorted images should have
//    // in order to set up the camera matrices for initUndistortRectifyMap.  We
//    // could use stereoRectify, but here we show how to derive these projection
//    // matrices from the calibration and a desired height and field of view
//
//    // We calculate the undistorted focal length:
//    //
//    //         h
//    // -----------------
//    //  \      |      /
//    //    \    | f  /
//    //     \   |   /
//    //      \ fov /
//    //        \|/
    float stereo_fov_rad = 140 * (M_PI/180);  // 90 degree desired fov
    int stereo_height_px = 600;         // 300x300 pixel stereo output
    float stereo_focal_px = stereo_height_px/2 / tanf(stereo_fov_rad/2);
//
//    // We set the left rotation to identity and the right rotation
//    // the rotation between the cameras
    cv::Mat R_left = cv::Mat::eye(3, 3, CV_32F);
    cv::Mat R_right = R;
//
//    // The stereo algorithm needs max_disp extra pixels in order to produce valid
//    // disparity on the desired output region. This changes the width, but the
//    // center of projection should be on the center of the cropped image
    int stereo_width_px = stereo_height_px + max_disp;
    cv::Size stereo_size(stereo_width_px, stereo_height_px);
    float stereo_cx = (stereo_height_px - 1)/2 + max_disp;
    float stereo_cy = (stereo_height_px - 1)/2;
//
//    // Construct the left and right projection matrices, the only difference is
//    // that the right projection matrix should have a shift along the x axis of
//    // baseline*focal_length
    cv::Mat P_left(3, 4, CV_32F, 0.0f);
    P_left.at<float>(0,0) = stereo_focal_px;
    P_left.at<float>(0,2) = stereo_cx;
    P_left.at<float>(1,1) = stereo_focal_px;
    P_left.at<float>(1,2) = stereo_cy;
    P_left.at<float>(2,2) = 1.0f;
    cv::Mat P_right = P_left;
    P_right.at<float>(0, 3) = T.at<float>(0) * stereo_focal_px;

    // Create an undistortion map for the left and right camera which applies the
    // rectification and undoes the camera distortion. This only has to be done
    // once
    cv::Mat _lm1, _lm2, lm1, lm2;
    cv::fisheye::initUndistortRectifyMap(K_left, D_left, R_left, P_left, stereo_size, CV_32FC1, _lm1, _lm2);
    cv::convertMaps(_lm1, _lm2, lm1, lm2, CV_16SC2);
    cv::Mat _rm1, _rm2, rm1, rm2;
    cv::fisheye::initUndistortRectifyMap(K_right, D_right, R_right, P_right, stereo_size, CV_32FC1, _rm1, _rm2);
    cv::convertMaps(_rm1, _rm2, rm1, rm2, CV_16SC2);

    bool InitPoseSet = false;
    Eigen::Matrix<float, 4, 4> W_T_Cinit;
    Eigen::Matrix<float, 4, 4> W_T_P;
    Eigen::Matrix<float, 4, 4> W_T_C;

    while (1) {
        auto frames = pipeline.wait_for_frames(2000);
        //std::cout << "Received a frameset consisting of " << frames.size() << " frames" << std::endl;

        cv::Mat left, right;
        cv::Mat left_undistorted, right_undistorted;

        for (const auto &frame : frames) {
            if (frame.get_profile().stream_type() == RS2_STREAM_FISHEYE &&
                frame.get_profile().stream_index() == 1) { // left fisheye
                auto image_frame = frame.as<rs2::video_frame>();
                left = toOpenCV(image_frame);

                rs2_intrinsics intrinsics = frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
                if (intrinsics.model == RS2_DISTORTION_KANNALA_BRANDT4) {
                    // See also rs2_project_point_to_pixel and rs2_deproject_pixel_to_point in rsutil.h
                    cv::remap(left, left_undistorted, lm1, lm2, cv::INTER_LINEAR); // similar to cv::fisheye::undistortImage, although undistortImage does not work
                }

            }
            else if (frame.get_profile().stream_type() == RS2_STREAM_FISHEYE &&
                       frame.get_profile().stream_index() == 2) { // right fisheye
                auto image_frame = frame.as<rs2::video_frame>();
                right = toOpenCV(image_frame);

                rs2_intrinsics intrinsics = frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
                if (intrinsics.model == RS2_DISTORTION_KANNALA_BRANDT4) {
                    // See also rs2_project_point_to_pixel and rs2_deproject_pixel_to_point in rsutil.h
                    cv::remap(right, right_undistorted, rm1, rm2, cv::INTER_LINEAR); // similar to cv::fisheye::undistortImage, although undistortImage does not work
                }
            }
            else if (frame.get_profile().stream_type() == RS2_STREAM_POSE) {
                auto pose_frame = frame.as<rs2::pose_frame>();
                rs2_pose pose_data = pose_frame.get_pose_data();
                W_T_P = Eigen::Affine3f(Eigen::AngleAxis<float>(M_PI_2, Eigen::Vector3f(1,0,0))).matrix() * GetTransform(pose_data);
                W_T_C = W_T_P * C_T_P.inverse();

                if (!InitPoseSet) {
                    W_T_Cinit = W_T_C;
                    InitPoseSet = true;
                }
            }
        }

        // Compute displayed pose location in current camera frame
        auto C_T_Cinit = W_T_C.inverse() * W_T_Cinit;
        Eigen::Matrix<float, 3, 1> position_in_cam = C_T_Cinit.block<3,1>(0,3).matrix();
        std::cout << W_T_P << std::endl << std::endl;

        //std::cout << position_in_cam.transpose() << std::endl << std::endl;
        cv::Mat pose_visualized;
        cv::cvtColor(left, pose_visualized, cv::COLOR_GRAY2BGR);
        if (position_in_cam(2) > 0) {
            // Project position into camera frame
            cv::Mat ImagePoints;
            //cv::Mat pointToProject();
            std::vector<cv::Point3f> points;
            points.emplace_back(position_in_cam(0), position_in_cam(1), position_in_cam(2));
            cv::fisheye::projectPoints(points,
                                       ImagePoints,
                                       cv::Affine3f(), // pose_to_leftcam_extrinsics_cv_affine
                                       K_left, D_left);

            cv::circle(pose_visualized, ImagePoints.at<cv::Point2f>(0), 3, cv::Scalar(0, 0, 255));
        }

        cv::Mat top, bottom;
        cv::hconcat(right, left, top);
        cv::hconcat(right_undistorted, left_undistorted, bottom);

        cv::namedWindow("Fisheye", cv::WINDOW_NORMAL);
        cv::imshow("Fisheye", top);

        cv::namedWindow("Undistorted", cv::WINDOW_NORMAL);
        cv::imshow("Undistorted", bottom);

        if (pose_visualized.rows > 0) {
            cv::namedWindow("Pose", cv::WINDOW_NORMAL);
            cv::imshow("Pose", pose_visualized);
        }

        cv::waitKey(10);
    }

    Disconnect();
}

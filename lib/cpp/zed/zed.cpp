#include "zed.h"

ZED::ZED()
{
    left_image_raw = std::make_shared<sl::Mat>();
    left_image_rectified = std::make_shared<sl::Mat>();
    right_image_raw = std::make_shared<sl::Mat>();
    right_image_rectified = std::make_shared<sl::Mat>();

    depth_map = std::make_shared<sl::Mat>();
    depth_color = std::make_shared<sl::Mat>();
}

ZED::~ZED()
{
    if (connected_) {
        Disconnect();
    }
}

void ZED::Connect()
{
    if (connected_) return;

    const auto cfg = Configure();

    // Open the camera
    sl::ERROR_CODE zed_open_state = zed.open(cfg);
    if (zed_open_state != sl::ERROR_CODE::SUCCESS) {
        std::cout << "[ZED] Error: Camera Open, " << sl::toString(zed_open_state) << std::endl;
        return;
    }

    // Get camera calibrations
    LoadCameraCalibrations();

    // Set parameters for Positional Tracking
    sl::PositionalTrackingParameters positional_tracking_param;
    positional_tracking_param.enable_area_memory = true;

    // Enable Positional Tracking
    auto returned_state = zed.enablePositionalTracking(positional_tracking_param);
    if (returned_state != sl::ERROR_CODE::SUCCESS) {
        std::cout << "[ZED] Error: Positionnal tracking failed, " << sl::toString(returned_state) << std::endl;
        zed.close();
        return;
    }

    // Process the incoming data streams through Poll thread
    pollThread = std::make_unique<std::thread>(std::bind(&ZED::PollThread, this));

    connected_ = true;
}

sl::InitParameters ZED::Configure()
{
    // Set configuration parameters for the ZED
    sl::InitParameters init_parameters;

    //init_parameters.camera_resolution = sl::RESOLUTION::HD720;
    init_parameters.depth_mode = sl::DEPTH_MODE::ULTRA;
    init_parameters.coordinate_units = sl::UNIT::METER; // Use meter units (for depth measurements)
    init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::IMAGE;
    init_parameters.depth_minimum_distance = 0.3;

    init_parameters.sdk_verbose = true;

    return init_parameters;
}

void ZED::Disconnect()
{
    if (!connected_) return;

    if (pollThread) {
        shouldExit = true;
        pollThread->join();
        pollThread.reset();
    }

    zed.disablePositionalTracking();
    //zed.disableRecording();
    zed.close();
    std::this_thread::sleep_for(std::chrono::milliseconds(200)); // allow some time to wait for the device to stop properly

    connected_ = false;
}

void ZED::PollThread()
{
    sl::RuntimeParameters runtime_param;
    runtime_param.sensing_mode = sl::SENSING_MODE::STANDARD; // Use STANDARD sensing mode

    while (!shouldExit)
    {
        // Wait for the next set of frames from the camera
        if (zed.grab(runtime_param) == sl::ERROR_CODE::SUCCESS) {
            std::lock_guard<std::mutex> lock(mutex);

            // Get the position of the camera in a fixed reference frame (the World Frame)
            sl::Pose pose;
            sl::POSITIONAL_TRACKING_STATE tracking_state = zed.getPosition(pose, sl::REFERENCE_FRAME::WORLD);
            //sl::POSITIONAL_TRACKING_STATE tracking_state = zed.getPosition(pose, sl::REFERENCE_FRAME::CAMERA); // odometry (since last camera frame

            if (tracking_state == sl::POSITIONAL_TRACKING_STATE::OK) {
                // Calculate current transformation matrix
                M_T_RP = GetTransform(pose);

                // Transform into desired frames
                W_T_P = W_T_M * inv_M_T_RPinit * M_T_RP * P_T_RP.inverse();
                W_T_LEFT = W_T_M * inv_M_T_RPinit * M_T_RP * LEFT_T_RP.inverse();
                W_T_RIGHT = W_T_M * inv_M_T_RPinit * M_T_RP * RIGHT_T_RP.inverse();

                latestPose.timestamp = double(pose.timestamp.getNanoseconds() / 1000000000.0);
                latestPose.pose = W_T_P;
                if (callbacks.pose.size() > 0) {
                    for (auto &cb : callbacks.pose)
                        cb(latestPose);
                }
            }

            // Get the left image
            zed.retrieveImage(*left_image_raw, sl::VIEW::LEFT_UNRECTIFIED);
            zed.retrieveImage(*left_image_rectified, sl::VIEW::LEFT);
            if (callbacks.left_image.size() > 0) {
                for (auto &cb : callbacks.left_image) {
                    if (cb.second) { // provide rectified
                        auto image = GetImage(*left_image_rectified);
                        cb.first(image);
                    } else { // provide raw/unrectified
                        auto image = GetImage(*left_image_raw);
                        cb.first(image);
                    }
                }
            }

            // Get the right image
            zed.retrieveImage(*right_image_raw, sl::VIEW::RIGHT_UNRECTIFIED);
            zed.retrieveImage(*right_image_rectified, sl::VIEW::RIGHT);
            if (callbacks.right_image.size() > 0) {
                for (auto &cb : callbacks.right_image) {
                    if (cb.second) { // provide rectified
                        auto image = GetImage(*right_image_rectified);
                        cb.first(image);
                    } else { // provide raw/unrectified
                        auto image = GetImage(*right_image_raw);
                        cb.first(image);
                    }
                }
            }

            zed.retrieveMeasure(*depth_map, sl::MEASURE::DEPTH, sl::MEM::CPU); // Get the depth map
            zed.retrieveImage(*depth_color, sl::VIEW::DEPTH); // Get the colored depth image
            /*// Read a depth value
            float centerDepth = 0;
            depthMap.getValue<float>(x, y, &centerDepth, sl::MEM::CPU); // each depth map pixel is a float value
            if (isnormal(centerDepth)) { // + Inf is "too far", -Inf is "too close", Nan is "unknown/occlusion"
                std::cout << "Depth value at center: " << centerDepth << " " << init_params.coordinate_units << std::endl;
            }*/

            sl::Mat pointCloud;
            zed.retrieveMeasure(pointCloud, sl::MEASURE::XYZRGBA, sl::MEM::CPU);// Get the point cloud
            /*// Read a point cloud value
            sl::float4 pcValue;
            pointCloud.getValue<sl::float4>(x, y, &pcValue); // each point cloud pixel contains 4 floats, so we are using a sl::float4
            if (isnormal(pcValue.z)) {
                std::cout << "Point cloud coordinates at center: X=" << pcValue.x << ", Y=" << pcValue.y << ", Z=" << pcValue.z << std::endl;
                unsigned char color[sizeof(float)];
                memcpy(color, &pcValue[3], sizeof(float));
                std::cout << "Point cloud color at center: R=" << (int)color[0] << ", G=" << (int)color[1] << ", B=" << (int)color[2] << std::endl;
            }*/

        }
    }
}

void ZED::RegisterCallback_Pose(const std::function<void(Pose&)> &callback)
{
    callbacks.pose.emplace_back(callback);
}
void ZED::RegisterCallback_LeftImage(const std::function<void(cv::Mat&)> &callback, bool rectified)
{
    callbacks.left_image.emplace_back(callback, rectified);
}
void ZED::RegisterCallback_RightImage(const std::function<void(cv::Mat&)> &callback, bool rectified)
{
    callbacks.right_image.emplace_back(callback, rectified);
}
void ZED::ClearCallbacks()
{
    callbacks.pose.clear();
    callbacks.left_image.clear();
    callbacks.right_image.clear();
}

void ZED::LoadCameraCalibrations()
{
    auto camera_info = zed.getCameraInformation();

    // Load Camera intrinsics
    IntrinsicsLeft = camera_info.camera_configuration.calibration_parameters_raw.left_cam;
    IntrinsicsRight = camera_info.camera_configuration.calibration_parameters_raw.right_cam;

    ImageWidth = camera_info.camera_configuration.calibration_parameters_raw.left_cam.image_size.width;
    ImageHeight = camera_info.camera_configuration.calibration_parameters_raw.left_cam.image_size.height;

    // Extract pose to camera extrinsincs
    // Get the relative extrinsics between the left and right camera
    auto LEFT_T_RIGHT = camera_info.camera_configuration.calibration_parameters_raw.stereo_transform;
    LEFT_T_RP = Eigen::Affine3f::Identity(); // left camera is origin of pose
    RIGHT_T_RP = Eigen::Affine3f(GetTransform(LEFT_T_RIGHT).inverse()) * LEFT_T_RP;

    K_left = cv::Mat(3, 3, CV_32F, cv::Scalar(0.0));
    K_left.at<float>(0, 0) = IntrinsicsLeft.fx;
    K_left.at<float>(0, 2) = IntrinsicsLeft.cx;
    K_left.at<float>(1, 1) = IntrinsicsLeft.fy;
    K_left.at<float>(1, 2) = IntrinsicsLeft.cy;
    K_left.at<float>(2, 2) = 1;

    K_right = cv::Mat(3, 3, CV_32F, cv::Scalar(0.0));
    K_right.at<float>(0, 0) = IntrinsicsRight.fx;
    K_right.at<float>(0, 2) = IntrinsicsRight.cx;
    K_right.at<float>(1, 1) = IntrinsicsRight.fy;
    K_right.at<float>(1, 2) = IntrinsicsRight.cy;
    K_right.at<float>(2, 2) = 1;

    D_left = cv::Mat(1, 5, CV_32F, IntrinsicsLeft.disto);
    D_right = cv::Mat(1, 5, CV_32F, IntrinsicsRight.disto);

    cv::Mat R_ = cv::Mat(3, 3, CV_32F, LEFT_T_RIGHT.getRotationMatrix().r);
    cv::Mat R = R_.t();
    cv::Mat T = cv::Mat(3, 1, CV_32F, LEFT_T_RIGHT.getTranslation().v);

    // We set the left rotation to identity and the right rotation
    // the rotation between the cameras
    R_left = cv::Mat::eye(3, 3, CV_32F);
    R_right = R;

    // Construct the left and right projection matrices, the only difference is
    // that the right projection matrix should have a shift along the x axis of
    // baseline*focal_length
    const cv::Size rectified_size(ImageWidth, ImageHeight);
    P_left = cv::Mat(3, 4, CV_32F, 0.0f);
    P_left.at<float>(0,0) = IntrinsicsLeft.fx;
    P_left.at<float>(0,2) = IntrinsicsLeft.cx;
    P_left.at<float>(1,1) = IntrinsicsLeft.fy;
    P_left.at<float>(1,2) = IntrinsicsLeft.cy;
    P_left.at<float>(2,2) = 1.0f;
    P_right = P_left;
    P_right.at<float>(0, 3) = T.at<float>(0) * IntrinsicsLeft.fx;

    // Create an undistortion map for the left and right camera which applies the
    // rectification and undoes the camera distortion. This only has to be done
    // once
    cv::Mat _lm1, _lm2, lm1, lm2;
    cv::initUndistortRectifyMap(K_left, D_left, R_left, P_left, rectified_size, CV_32FC1, _lm1, _lm2);
    cv::convertMaps(_lm1, _lm2, LeftMap1, LeftMap2, CV_16SC2);
    cv::Mat _rm1, _rm2, rm1, rm2;
    cv::initUndistortRectifyMap(K_right, D_right, R_right, P_right, rectified_size, CV_32FC1, _rm1, _rm2);
    cv::convertMaps(_rm1, _rm2, RightMap1, RightMap2, CV_16SC2);
}

void ZED::ResetOrigin()
{
    // Reset the origin to the current pose
    // M_T_RP = inv(M_T_RPinit) * M_T_RP;
    inv_M_T_RPinit = M_T_RP.inverse();
}

cv::Mat ZED::GetImage(sl::Mat &frame, bool left, bool rectify_manually)
{
    auto image = toOpenCV(frame);

    if (rectify_manually) {
        // cv::remap is similar to cv::undistortImage but with a precomputed map
        cv::Mat image_rectified;
        if (left) { // left camera
            cv::remap(image, image_rectified, LeftMap1, LeftMap2, cv::INTER_LINEAR);
            return image_rectified;
        } else {
            cv::remap(image, image_rectified, RightMap1, RightMap2, cv::INTER_LINEAR);
            return image_rectified;
        }
    }

    return image;
}

cv::Mat ZED::GetLeftImage(bool rectified)
{
    if (rectified && left_image_rectified->getWidth() > 0)
        return GetImage(*left_image_rectified);
        //return GetImage(*left_image_raw, true, true); // rectify manually
    else if (!rectified && left_image_raw->getWidth() > 0)
        return GetImage(*left_image_raw);

    return cv::Mat();
}

cv::Mat ZED::GetRightImage(bool rectified)
{
    if (rectified && right_image_rectified->getWidth() > 0)
        return GetImage(*right_image_rectified);
        //return GetImage(*right_image_rectified, false, true); // rectify manually
    else if (!rectified && right_image_raw->getWidth() > 0)
        return GetImage(*right_image_raw);

    return cv::Mat();
}

cv::Mat ZED::GetDepthImage() // comes in the left rectified frame
{
    if (depth_color->getWidth() > 0)
        return GetImage(*depth_color);

    return cv::Mat();
}

ZED::Pose ZED::GetPose()
{
    return latestPose;
}

Eigen::Vector3f ZED::GetPoseTranslation()
{
    return W_T_P.translation().matrix();
}

Eigen::Vector3f ZED::GetPoseTranslation(Eigen::Affine3f pose)
{
    return pose.translation().matrix();
}

Eigen::Vector3f ZED::GetPoseEulerRPY(bool degrees)
{
    return GetPoseEulerRPY(W_T_P, degrees);
}

Eigen::Vector3f ZED::GetPoseEulerRPY(Eigen::Affine3f pose, bool degrees)
{
    Eigen::Vector3f euler_ZYX = GetEulerAnglesZYX(pose.matrix()); // The output format is [yaw, pitch, roll]
    if (degrees)
        return Eigen::Vector3f(rad2deg(euler_ZYX(2)), rad2deg(euler_ZYX(1)), rad2deg(euler_ZYX(0))); // Swap it to make the output [roll, pitch, yaw]
    else
        return Eigen::Vector3f(euler_ZYX(2), euler_ZYX(1), euler_ZYX(0)); // Swap it to make the output [roll, pitch, yaw]
}

// Calculates transformation matrix based on pose data from the device
Eigen::Matrix<float, 4, 4> ZED::GetTransform(sl::Transform tf)
{
   return Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>>(tf.m);
}

Eigen::Matrix<float, 4, 4> ZED::GetTransform(sl::Pose pose)
{
    return GetTransform(pose.pose_data);
}


cv::Mat ZED::toOpenCV(const sl::Mat& frame)
{
    const int w = frame.getWidth();
    const int h = frame.getHeight();

    if (frame.getChannels() == 4)
        return cv::Mat(h, w, CV_8UC4, frame.getPtr<uint8_t>(), frame.getStepBytes());

    return cv::Mat();
}

Eigen::Vector3f ZED::GetEulerAnglesZYX(Eigen::Matrix<float, 3, 3> rotm)
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

Eigen::Vector3f ZED::GetEulerAnglesZYX(Eigen::Matrix<float, 4, 4> transform)
{
    return GetEulerAnglesZYX(static_cast<Eigen::Matrix<float, 3, 3>>(transform.block<3,3>(0,0).matrix()));
}

void ZED::ResetInternalEstimator()
{
    sl::Pose identity;
    zed.resetPositionalTracking(identity.pose_data);
}
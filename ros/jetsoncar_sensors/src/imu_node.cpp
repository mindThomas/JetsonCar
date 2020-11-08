#include <ros/ros.h>
#include "imu/imu.h"

#include "std_msgs/String.h"

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/Temperature.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
/*#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>*/
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <fstream>
#include <boost/filesystem.hpp>

#include "utils/utils.h"


class IMUNode {
private:
    IMU imu;

    bool publish_tf_;
    bool include_standard_deviations_;
    bool enable_position_fusion_;
    bool enable_heading_fusion_;
    int odom_downsample_rate_;
    int odom_sample_count_{0};
    float odom_position_stddev_m_;
    float odom_heading_stddev_deg_;

    ros::Time imu_t0;
    IMU::Orientation est_orientation_;
    IMU::QuaternionStdDev est_quaternion_stddev_;
    IMU::StateEstimate est_acceleration_;
    IMU::StateEstimate est_angular_velocity_;
    IMU::StateEstimate est_body_velocity_;
    IMU::StateEstimate est_position_NED_;
    IMU::CombinedSensorsMeasurement raw_;

    ros::NodeHandle nh_;
    ros::NodeHandle nParam_;
    std::string world_frame_;
    std::string imu_frame_;
    std::string position_imu_frame_; // mainly for visualization

    // Publishers
    ros::Publisher pub_imu_;
    ros::Publisher pub_imu_pose_;
    ros::Publisher pub_imu_raw_;
    ros::Publisher pub_magnetic_field_;

    // Subscribers
    ros::Subscriber sub_odom_in_;

    tf::TransformBroadcaster tf_broadcaster_;
    tf::TransformListener tf_listener_;

    // Logging
    bool loggingEnabled{false};
    std::ofstream log_raw;
    std::ofstream log_estimate;
    std::ofstream log_angular_velocity;
    std::ofstream log_odom_pose;

private:
#if 0
    void publish_imu_msg() {
        sensor_msgs::Imu imu_msg;
        geometry_msgs::Quaternion quaternion_msg;
        float orientation_covariance[9] = {pow((quaternion_std_packet_.standard_deviation[0]), 2.0), 0, 0,
                                           0, pow((quaternion_std_packet_.standard_deviation[1]), 2.0), 0,
                                           0, 0, pow((quaternion_std_packet_.standard_deviation[2]), 2.0)};
        geometry_msgs::Vector3 angular_velocity;

        geometry_msgs::Vector3 linear_acceleration;

        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = frame_id_;

        imu_msg.orientation.w = quaternion_packet_.orientation[0];
        imu_msg.orientation.x = quaternion_packet_.orientation[1];
        imu_msg.orientation.y = quaternion_packet_.orientation[2];
        imu_msg.orientation.z = quaternion_packet_.orientation[3];
        imu_msg.orientation_covariance[0] = orientation_covariance[0];
        imu_msg.orientation_covariance[4] = orientation_covariance[4];
        imu_msg.orientation_covariance[8] = orientation_covariance[8];

        imu_msg.angular_velocity.x = angular_velocity_packet_.angular_velocity[0];
        imu_msg.angular_velocity.y = angular_velocity_packet_.angular_velocity[1];
        imu_msg.angular_velocity.z = angular_velocity_packet_.angular_velocity[2];
        imu_msg.angular_velocity_covariance[0] = -1;

        imu_msg.linear_acceleration.x = acceleration_packet_.acceleration[0];
        imu_msg.linear_acceleration.y = acceleration_packet_.acceleration[1];
        imu_msg.linear_acceleration.z = acceleration_packet_.acceleration[2];
        imu_msg.linear_acceleration_covariance[0] = -1;

        imu_pub_.publish(imu_msg);
    }
    void publish_imu_orienation_tf() {
        tf::Quaternion q_attitude;
        q_attitude.setW(quaternion_packet_.orientation[0]);
        q_attitude.setX(quaternion_packet_.orientation[2]);
        q_attitude.setY(quaternion_packet_.orientation[1]);
        q_attitude.setZ(-quaternion_packet_.orientation[3]);

        tf::Transform attitudeTf;
        attitudeTf.setIdentity();
        attitudeTf.setRotation(q_attitude);

        /* Send transfrom from "heading" frame to "base_link" frame */
        geometry_msgs::TransformStamped tf_tilt_msg;
        tf_tilt_msg.header.frame_id = "heading";
        tf_tilt_msg.child_frame_id = "base_link";
        tf_tilt_msg.header.stamp = ros::Time::now();
        tf_tilt_msg.transform.translation.x = 0;
        tf_tilt_msg.transform.translation.y = 0;
        tf_tilt_msg.transform.translation.z = 0.0;
        tf_tilt_msg.transform.rotation.w = q_attitude.w();
        tf_tilt_msg.transform.rotation.x = q_attitude.x();
        tf_tilt_msg.transform.rotation.y = q_attitude.y();
        tf_tilt_msg.transform.rotation.z = q_attitude.z();
        tf_broadcaster_.sendTransform(tf_tilt_msg);

        /* Send odometry message */
        //   This represents an estimate of a position and velocity in free space.
        //   The pose in this message should be specified in the coordinate frame given by header.frame_id
        //   The twist in this message should be specified in the coordinate frame given by the child_frame_id
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "imu";
        odom_msg.pose.pose.position.x = 0; // inertial frame position
        odom_msg.pose.pose.position.y = 0;
        odom_msg.pose.pose.position.z = 0;
        odom_msg.pose.pose.orientation.w = q_attitude.w();
        odom_msg.pose.pose.orientation.x = q_attitude.x();
        odom_msg.pose.pose.orientation.y = q_attitude.y();
        odom_msg.pose.pose.orientation.z = q_attitude.z();
        odom_msg.twist.twist.linear.x = 0; // body frame velocity
        odom_msg.twist.twist.linear.y = 0;
        odom_msg.twist.twist.linear.z = 0;
        odom_msg.twist.twist.angular.x = angular_velocity_packet_.angular_velocity[0];
        odom_msg.twist.twist.angular.y = angular_velocity_packet_.angular_velocity[1];
        odom_msg.twist.twist.angular.z = angular_velocity_packet_.angular_velocity[2];
        pub_odom_.publish(odom_msg);
    }
    void publish_imu_raw_msg() {
        sensor_msgs::Imu imu_msg;
        geometry_msgs::Vector3 angular_velocity;
        geometry_msgs::Vector3 linear_acceleration;

        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = frame_id_;

        imu_msg.orientation_covariance[0] = -1;

        imu_msg.angular_velocity.x = raw_sensors_packet_.gyroscopes[0];
        imu_msg.angular_velocity.y = raw_sensors_packet_.gyroscopes[1];
        imu_msg.angular_velocity.z = raw_sensors_packet_.gyroscopes[2];
        imu_msg.angular_velocity_covariance[0] = -1;

        imu_msg.linear_acceleration.x = raw_sensors_packet_.accelerometers[0];
        imu_msg.linear_acceleration.y = raw_sensors_packet_.accelerometers[1];
        imu_msg.linear_acceleration.z = raw_sensors_packet_.accelerometers[2];
        imu_msg.linear_acceleration_covariance[0] = -1;

        imu_raw_pub_.publish(imu_msg);
    }
    void publish_magnetics_msg() {
        sensor_msgs::MagneticField magnetic_field_msg;

        magnetic_field_msg.header.stamp = ros::Time::now();
        magnetic_field_msg.header.frame_id = frame_id_;

        magnetic_field_msg.magnetic_field.x = raw_sensors_packet_.magnetometers[0] * (1.0e-7);
        magnetic_field_msg.magnetic_field.y = raw_sensors_packet_.magnetometers[1] * (1.0e-7);
        magnetic_field_msg.magnetic_field.z = raw_sensors_packet_.magnetometers[2] * (1.0e-7);

        magnetic_field_pub_.publish(magnetic_field_msg);
    }
#endif

    void RawSensorsUpdate(IMU::CombinedSensorsMeasurement meas)
    {
        auto time = imu_t0 + ros::Duration(meas.timestamp);

        if (log_raw.is_open()) {
            log_raw << time.toNSec() << "\t";
            log_raw << std::setprecision(10) << meas.timestamp << "\t";
            log_raw << std::setprecision(10) << meas.accelerometer[0] << "\t";
            log_raw << std::setprecision(10) << meas.accelerometer[1] << "\t";
            log_raw << std::setprecision(10) << meas.accelerometer[2] << "\t";
            log_raw << std::setprecision(10) << meas.gyroscope[0] << "\t";
            log_raw << std::setprecision(10) << meas.gyroscope[1] << "\t";
            log_raw << std::setprecision(10) << meas.gyroscope[2] << "\t";
            log_raw << std::setprecision(10) << meas.magnetometer[0] << "\t";
            log_raw << std::setprecision(10) << meas.magnetometer[1] << "\t";
            log_raw << std::setprecision(10) << meas.magnetometer[2];
            log_raw << std::endl;
        }

        raw_ = meas;

        // Publish IMU message
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = imu_t0 + ros::Duration(meas.timestamp);
        imu_msg.header.frame_id = imu_frame_;

        imu_msg.orientation_covariance[0] = -1;

        imu_msg.angular_velocity.x = meas.gyroscope[0];
        imu_msg.angular_velocity.y = meas.gyroscope[1];
        imu_msg.angular_velocity.z = meas.gyroscope[2];
        imu_msg.angular_velocity_covariance[0] = -1;

        imu_msg.linear_acceleration.x = meas.accelerometer[0];
        imu_msg.linear_acceleration.y = meas.accelerometer[1];
        imu_msg.linear_acceleration.z = meas.accelerometer[2];
        imu_msg.linear_acceleration_covariance[0] = -1;

        pub_imu_raw_.publish(imu_msg);

        // Publish magnetometer message
        sensor_msgs::MagneticField magnetic_field_msg;

        magnetic_field_msg.header.stamp = ros::Time::now();
        magnetic_field_msg.header.frame_id = imu_frame_;

        magnetic_field_msg.magnetic_field.x = meas.magnetometer[0] * (1.0e-7); // milliGauss to Tesla
        magnetic_field_msg.magnetic_field.y = meas.magnetometer[1] * (1.0e-7);
        magnetic_field_msg.magnetic_field.z = meas.magnetometer[2] * (1.0e-7);

        pub_magnetic_field_.publish(magnetic_field_msg);
    }

    void PublishEstimateIfReady()
    {
        if (est_orientation_.timestamp > 0 &&
           (est_orientation_.timestamp == est_quaternion_stddev_.timestamp || !include_standard_deviations_) &&
            est_orientation_.timestamp == est_angular_velocity_.timestamp &&
           (est_orientation_.timestamp == est_body_velocity_.timestamp || !enable_position_fusion_) &&
           (est_orientation_.timestamp == est_position_NED_.timestamp || !enable_position_fusion_))
        {
            auto time = imu_t0 + ros::Duration(est_orientation_.timestamp);

            tf::Quaternion q_imu;
            q_imu.setX(est_orientation_.quaternion[0]);
            q_imu.setY(est_orientation_.quaternion[1]);
            q_imu.setZ(est_orientation_.quaternion[2]);
            q_imu.setW(est_orientation_.quaternion[3]);

            if (fabs(q_imu.length() - 1.0) > 0.01) {
                ROS_WARN_STREAM("[IMU] Internal estimator not ready yet");
                return; // error
            }

            tf::Quaternion q_flip({1,0,0}, deg2rad(180));
            tf::Quaternion q_orientation = q_flip * q_imu; // apply this flip to convert the quaternion estimate from a NED frame into the world NWU frame

            auto rpy = IMU::GetPoseEulerRPY(est_orientation_.quaternion, true);
            ROS_DEBUG_STREAM("RPY = " << rpy[0] << ", " << rpy[1] << ", " << rpy[2]);

            /* Send transfrom from "world" frame to "imu" frame */
            if (publish_tf_) {
                geometry_msgs::TransformStamped tf_imu_msg;
                tf_imu_msg.header.frame_id = world_frame_;
                tf_imu_msg.child_frame_id = imu_frame_;
                tf_imu_msg.header.stamp = time;
                tf_imu_msg.transform.translation.x = 0;
                tf_imu_msg.transform.translation.y = 0;
                tf_imu_msg.transform.translation.z = 0;
                tf_imu_msg.transform.rotation.x = q_orientation.x();
                tf_imu_msg.transform.rotation.y = q_orientation.y();
                tf_imu_msg.transform.rotation.z = q_orientation.z();
                tf_imu_msg.transform.rotation.w = q_orientation.w();
                tf_broadcaster_.sendTransform(tf_imu_msg);
            }

            // Publish IMU estimates
            sensor_msgs::Imu imu_msg;
            geometry_msgs::Quaternion quaternion_msg;
            // Factor of 2 since the quaternion elements are = sin(1/2*angle)
            imu_msg.header.stamp = time;
            imu_msg.header.frame_id = imu_frame_;

            imu_msg.orientation.x = q_orientation.x();
            imu_msg.orientation.y = q_orientation.y();
            imu_msg.orientation.z = q_orientation.z();
            imu_msg.orientation.w = q_orientation.w();
            imu_msg.orientation_covariance[0] = -1;

            if (include_standard_deviations_) {
                double orientation_covariance[9] = {pow((2.0 * est_quaternion_stddev_.stddev[0]), 2.0), 0, 0,
                                                    0, pow((2.0 * est_quaternion_stddev_.stddev[1]), 2.0), 0,
                                                    0, 0, pow((2.0 * est_quaternion_stddev_.stddev[2]), 2.0)};
                imu_msg.orientation_covariance[0] = orientation_covariance[0];
                imu_msg.orientation_covariance[4] = orientation_covariance[4];
                imu_msg.orientation_covariance[8] = orientation_covariance[8];
            }

            imu_msg.angular_velocity.x = est_angular_velocity_.estimate[0];
            imu_msg.angular_velocity.y = est_angular_velocity_.estimate[1];
            imu_msg.angular_velocity.z = est_angular_velocity_.estimate[2];
            imu_msg.angular_velocity_covariance[0] = -1;

            // est_acceleration_.estimate doesn't contain any acceleration if the internal position estimator within the IMU isn't running
            /*
            imu_msg.linear_acceleration.x = est_acceleration_.estimate[0];
            imu_msg.linear_acceleration.y = est_acceleration_.estimate[1];
            imu_msg.linear_acceleration.z = est_acceleration_.estimate[2];
            */
            // Instead we correct the accelerometer measurement with the orientation quaternion and report that as the linear acceleration
            /*auto accelerometer = tf::Vector3(raw_.accelerometer[0], raw_.accelerometer[1], raw_.accelerometer[2]);
            auto acceleration = accelerometer - tf::quatRotate(q_imu.inverse(), {0,0,-gravity_});
            imu_msg.linear_acceleration.x = acceleration.x();
            imu_msg.linear_acceleration.y = acceleration.y();
            imu_msg.linear_acceleration.z = acceleration.z();*/
            imu_msg.linear_acceleration_covariance[0] = -1;

            pub_imu_.publish(imu_msg);

            // Send position and velocity estimates
            if (enable_position_fusion_) {
                geometry_msgs::TransformStamped tf_imu_msg;
                tf_imu_msg.header.frame_id = world_frame_;
                tf_imu_msg.child_frame_id = position_imu_frame_;
                tf_imu_msg.header.stamp = ros::Time::now();
                tf_imu_msg.transform.translation.x = est_position_NED_.estimate[0]; // From NED to ENU
                tf_imu_msg.transform.translation.y = -est_position_NED_.estimate[1];
                tf_imu_msg.transform.translation.z = -est_position_NED_.estimate[2];
                tf_imu_msg.transform.rotation.x = q_orientation.x();
                tf_imu_msg.transform.rotation.y = q_orientation.y();
                tf_imu_msg.transform.rotation.z = q_orientation.z();
                tf_imu_msg.transform.rotation.w = q_orientation.w();
                tf_broadcaster_.sendTransform(tf_imu_msg);

                /* Send IMU Pose odometry message */
                //   This represents an estimate of a position and velocity in free space.
                //   The pose in this message should be specified in the coordinate frame given by header.frame_id
                //   The twist in this message should be specified in the coordinate frame given by the child_frame_id
                nav_msgs::Odometry odom_msg;
                odom_msg.header.stamp = ros::Time::now();
                odom_msg.header.frame_id = world_frame_;
                odom_msg.child_frame_id = position_imu_frame_;
                odom_msg.pose.pose.position.x = est_position_NED_.estimate[0]; // From NED to ENU
                odom_msg.pose.pose.position.y = -est_position_NED_.estimate[1];
                odom_msg.pose.pose.position.z = -est_position_NED_.estimate[2];
                odom_msg.pose.pose.orientation.w = q_orientation.w();
                odom_msg.pose.pose.orientation.x = q_orientation.x();
                odom_msg.pose.pose.orientation.y = q_orientation.y();
                odom_msg.pose.pose.orientation.z = q_orientation.z();
                odom_msg.twist.twist.linear.x = est_body_velocity_.estimate[0]; // body frame velocity
                odom_msg.twist.twist.linear.y = est_body_velocity_.estimate[1]; // OBS! Needs to be converted
                odom_msg.twist.twist.linear.z = est_body_velocity_.estimate[2];
                odom_msg.twist.twist.angular.x = est_angular_velocity_.estimate[0]; // these are already body angular velocities
                odom_msg.twist.twist.angular.y = est_angular_velocity_.estimate[1];
                odom_msg.twist.twist.angular.z = est_angular_velocity_.estimate[2];
                pub_imu_pose_.publish(odom_msg);
            }

            // Invalidate stored estimates
            est_orientation_.timestamp = 0;
            est_quaternion_stddev_.timestamp = 0;
            est_angular_velocity_.timestamp = 0;
            est_body_velocity_.timestamp = 0;
            est_position_NED_.timestamp = 0;
        }
    }

    void OrientationUpdate(IMU::Orientation orientation)
    {
        auto time = imu_t0 + ros::Duration(orientation.timestamp);

        if (log_estimate.is_open()) {
            log_estimate << time.toNSec() << "\t";
            log_estimate << std::setprecision(10) << orientation.timestamp << "\t";
            log_estimate << std::setprecision(10) << orientation.quaternion[3] << "\t"; // w
            log_estimate << std::setprecision(10) << orientation.quaternion[0] << "\t"; // x
            log_estimate << std::setprecision(10) << orientation.quaternion[1] << "\t"; // y
            log_estimate << std::setprecision(10) << orientation.quaternion[2];         // z
            log_estimate << std::endl;
        }

        est_orientation_ = orientation;
        PublishEstimateIfReady();
    }

    void QuaternionStdDevUpdate(IMU::QuaternionStdDev stddev)
    {
        est_quaternion_stddev_ = stddev;
        PublishEstimateIfReady();
    }

    void AccelerationUpdate(IMU::StateEstimate estimate)
    {
        est_acceleration_ = estimate;
        PublishEstimateIfReady();
    }

    void AngularVelocityUpdate(IMU::StateEstimate estimate)
    {
        auto time = imu_t0 + ros::Duration(estimate.timestamp);

        if (log_angular_velocity.is_open()) {
            log_angular_velocity << time.toNSec() << "\t";
            log_angular_velocity << std::setprecision(10) << estimate.timestamp << "\t";
            log_angular_velocity << std::setprecision(10) << estimate.estimate[0] << "\t";
            log_angular_velocity << std::setprecision(10) << estimate.estimate[1] << "\t";
            log_angular_velocity << std::setprecision(10) << estimate.estimate[2];
            log_angular_velocity << std::endl;
        }

        est_angular_velocity_ = estimate;
        PublishEstimateIfReady();
    }

    void BodyVelocityUpdate(IMU::StateEstimate estimate)
    {
        est_body_velocity_ = estimate;
        PublishEstimateIfReady();
    }

    void PositionNEDUpdate(IMU::StateEstimate estimate)
    {
        est_position_NED_ = estimate;
        PublishEstimateIfReady();
    }

    void odom_in_callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        if (log_odom_pose.is_open()) {
            log_odom_pose << msg->header.stamp.toNSec() << "\t";
            log_odom_pose << std::setprecision(10) << msg->pose.pose.orientation.w << "\t";
            log_odom_pose << std::setprecision(10) << msg->pose.pose.orientation.x << "\t";
            log_odom_pose << std::setprecision(10) << msg->pose.pose.orientation.y << "\t";
            log_odom_pose << std::setprecision(10) << msg->pose.pose.orientation.z << "\t";
            log_odom_pose << std::setprecision(10) << msg->pose.pose.position.x << "\t";
            log_odom_pose << std::setprecision(10) << msg->pose.pose.position.y << "\t";
            log_odom_pose << std::setprecision(10) << msg->pose.pose.position.z << "\t";
            log_odom_pose << std::setprecision(10) << msg->twist.twist.linear.x << "\t";
            log_odom_pose << std::setprecision(10) << msg->twist.twist.linear.y << "\t";
            log_odom_pose << std::setprecision(10) << msg->twist.twist.linear.z << "\t";
            log_odom_pose << std::setprecision(10) << msg->twist.twist.angular.x << "\t";
            log_odom_pose << std::setprecision(10) << msg->twist.twist.angular.y << "\t";
            log_odom_pose << std::setprecision(10) << msg->twist.twist.angular.z;
            log_odom_pose << std::endl;
        }

        odom_sample_count_++;
        if (odom_sample_count_ % odom_downsample_rate_ != 0) return;

        tf::Pose W_T_realsense;
        tf::poseMsgToTF(msg->pose.pose, W_T_realsense);

        tf::StampedTransform imu_T_realsense;
        try {
            tf_listener_.lookupTransform(imu_frame_, msg->child_frame_id, msg->header.stamp, imu_T_realsense);

            // Note: The part below could also have been made smarter simply by looking up the imu_frame position in the world frame and reporting that
            auto imu_orientation = imu_T_realsense.getRotation() * W_T_realsense.getRotation();

            auto realsense_position_NED = tf::Vector3(W_T_realsense.getOrigin().x(),
                                                      -W_T_realsense.getOrigin().y(),
                                                      -W_T_realsense.getOrigin().z());

            auto W_T_imu = W_T_realsense * imu_T_realsense.inverse();

            auto imu_position_NED = tf::Vector3(W_T_imu.getOrigin().x(),
                                               -W_T_imu.getOrigin().y(),
                                               -W_T_imu.getOrigin().z());

            // Compute heading from received Realsense orientation quaternion
            tf::Vector3 xVector = tf::quatRotate(imu_orientation, tf::Vector3(1,0,0));
            float heading = atan2(xVector.y(), xVector.x());

            imu.SetHeading(heading, deg2rad(odom_heading_stddev_deg_));
            imu.SetPositionNED(imu_position_NED.x(), imu_position_NED.y(), imu_position_NED.z(), odom_position_stddev_m_);

            ROS_DEBUG("X,Y,Z - Heading = %f, %f, %f - %f", imu_position_NED.x(), imu_position_NED.y(), imu_position_NED.z(), rad2deg(heading));
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s", ex.what());
        }
    }

    void OpenLogFiles()
    {
        // Prepare logs folder
        if (!boost::filesystem::is_directory(boost::filesystem::path(std::string(getenv("HOME")) + "/logs"))) {
            if (boost::filesystem::exists(boost::filesystem::path(std::string(getenv("HOME")) + "/logs"))) {
                printf("Log path (~/logs) already exists but without write permissions\n");
                return;
            } else {
                if (!boost::filesystem::create_directory(boost::filesystem::path(std::string(getenv("HOME")) + "/logs"))) {
                    printf("Could not create Log folder (~/logs)\n");
                    return;
                }
                else
                    printf("Successfully created log folder (~/logs)\n");
            }
        }


        std::string logTimestamp = utils::GetLogFormattedTimestamp(utils::utime());
        log_raw.open(std::string(getenv("HOME")) + "/logs/" + logTimestamp + "_imu_raw.txt", std::ofstream::trunc);
        if (!log_raw.is_open()) {
            std::cout << "Could not create " << logTimestamp + "_imu_accel.txt" << std::endl;
        }

        log_estimate.open(std::string(getenv("HOME")) + "/logs/" + logTimestamp + "_imu_estimate.txt", std::ofstream::trunc);
        if (!log_estimate.is_open()) {
            std::cout << "Could not create " << logTimestamp + "_imu_estimate.txt" << std::endl;
        }

        log_angular_velocity.open(std::string(getenv("HOME")) + "/logs/" + logTimestamp + "_imu_angular_velocity.txt", std::ofstream::trunc);
        if (!log_angular_velocity.is_open()) {
            std::cout << "Could not create " << logTimestamp + "_imu_angular_velocity.txt" << std::endl;
        }

        log_odom_pose.open(std::string(getenv("HOME")) + "/logs/" + logTimestamp + "_odom_pose.txt", std::ofstream::trunc);
        if (!log_odom_pose.is_open()) {
            std::cout << "Could not create " << logTimestamp + "_odom_pose.txt" << std::endl;
        }

        loggingEnabled = true;
    }

    void CloseLogFiles()
    {
        log_raw.close();
        log_estimate.close();
        log_odom_pose.close();

        loggingEnabled = false;
    }

public:
    IMUNode(ros::NodeHandle nh, ros::NodeHandle nParam, std::string world_frame, std::string imu_frame, std::string position_imu_frame,  bool enable_position_fusion, bool enable_heading_fusion, float odom_position_stddev_m, float odom_heading_stddev_deg, bool publish_tf, std::string log_path, int odom_downsample_rate = 1)
            : include_standard_deviations_{false}
            , enable_position_fusion_{enable_position_fusion}
            , enable_heading_fusion_{enable_heading_fusion}
            , odom_downsample_rate_{odom_downsample_rate}
            , odom_position_stddev_m_{odom_position_stddev_m}
            , odom_heading_stddev_deg_{odom_heading_stddev_deg}
            , nh_(nh)
            , nParam_(nParam)
            , world_frame_{world_frame}
            , imu_frame_{imu_frame}
            , position_imu_frame_{position_imu_frame}
            , publish_tf_{publish_tf}
            , pub_imu_{nh.advertise<sensor_msgs::Imu>("imu/est", 10)}
            , pub_imu_pose_{nh.advertise<nav_msgs::Odometry>("imu/pose", 10)}
            , pub_imu_raw_{nh.advertise<sensor_msgs::Imu>("imu/raw", 10)}
            , pub_magnetic_field_{nh.advertise<sensor_msgs::MagneticField>("imu/mag", 10)}
            , sub_odom_in_{nh.subscribe("odom_in", 50, &IMUNode::odom_in_callback, this)}
    {
        imu.RegisterCallback_RawSensors(std::bind(&IMUNode::RawSensorsUpdate, this, std::placeholders::_1));
        imu.RegisterCallback_Orientation(std::bind(&IMUNode::OrientationUpdate, this, std::placeholders::_1));
        imu.RegisterCallback_AngularVelocity(std::bind(&IMUNode::AngularVelocityUpdate, this, std::placeholders::_1));

        if (enable_position_fusion_)
            imu.RegisterCallback_Acceleration(std::bind(&IMUNode::AccelerationUpdate, this, std::placeholders::_1));

        imu.LoadLog(log_path);
    }
    IMUNode(ros::NodeHandle nh, ros::NodeHandle nParam, std::string world_frame, std::string imu_frame, std::string position_imu_frame, bool enable_position_fusion, bool enable_heading_fusion, float odom_position_stddev_m, float odom_heading_stddev_deg, bool publish_tf, int raw_data_rate = 200, int estimate_data_rate = 100, int odom_downsample_rate = 1, bool include_standard_deviations = false, std::string port_name = "/dev/ttyUSB0", uint32_t baud_rate = 460800)
            : imu(port_name, baud_rate)
            , include_standard_deviations_{include_standard_deviations}
            , enable_position_fusion_{enable_position_fusion}
            , enable_heading_fusion_{enable_heading_fusion}
            , odom_downsample_rate_{odom_downsample_rate}
            , odom_position_stddev_m_{odom_position_stddev_m}
            , odom_heading_stddev_deg_{odom_heading_stddev_deg}
            , nh_(nh)
            , nParam_(nParam)
            , world_frame_{world_frame}
            , imu_frame_{imu_frame}
            , position_imu_frame_{position_imu_frame}
            , publish_tf_{publish_tf}
            , pub_imu_{nh.advertise<sensor_msgs::Imu>("imu/est", 10)}
            , pub_imu_pose_{nh.advertise<nav_msgs::Odometry>("imu/pose", 10)}
            , pub_imu_raw_{nh.advertise<sensor_msgs::Imu>("imu/raw", 10)}
            , pub_magnetic_field_{nh.advertise<sensor_msgs::MagneticField>("imu/mag", 10)}
            , sub_odom_in_{nh.subscribe("imu/odom_in", 50, &IMUNode::odom_in_callback, this)} // odom_in
    {
        if (raw_data_rate < 0) raw_data_rate = 200;
        if (estimate_data_rate < 0) estimate_data_rate = 200;

        imu.Connect();
        imu.Configure(IMU::OutputType::RawAndIndividualEstimates, raw_data_rate, estimate_data_rate, enable_heading_fusion_, enable_position_fusion_, include_standard_deviations_);
        imu.SynchronizeTime(); // synchronize time again manually
        imu_t0 = ros::Time().now();

        imu.RegisterCallback_RawSensors(std::bind(&IMUNode::RawSensorsUpdate, this, std::placeholders::_1));
        imu.RegisterCallback_Orientation(std::bind(&IMUNode::OrientationUpdate, this, std::placeholders::_1));
        imu.RegisterCallback_AngularVelocity(std::bind(&IMUNode::AngularVelocityUpdate, this, std::placeholders::_1));

        if (include_standard_deviations_)
            imu.RegisterCallback_QuaternionStdDev(std::bind(&IMUNode::QuaternionStdDevUpdate, this, std::placeholders::_1));

        if (enable_position_fusion_) {
            imu.RegisterCallback_Acceleration(std::bind(&IMUNode::AccelerationUpdate, this, std::placeholders::_1));
            imu.RegisterCallback_Velocity(std::bind(&IMUNode::BodyVelocityUpdate, this, std::placeholders::_1));
            imu.RegisterCallback_PositionNED(std::bind(&IMUNode::PositionNEDUpdate, this, std::placeholders::_1));
        }
    }

    ~IMUNode()
    {
        CloseLogFiles();
    }

    void StartLogging()
    {
        if (loggingEnabled) return;
        OpenLogFiles();
        imu.RecordANPPLog();
    }

    void StopLogging()
    {
        if (!loggingEnabled) return;
        CloseLogFiles();
        imu.StopANPPLog();
    }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu_node");
    ros::NodeHandle nh;
    ros::NodeHandle nParam("~"); // default/current namespace node handle

    int raw_data_rate = 100; // Hz
    if (!nParam.getParam("raw_data_rate", raw_data_rate)) {
        ROS_WARN_STREAM("IMU Raw data rate not set (Parameter: raw_data_rate). Defaults to: " << raw_data_rate);
    }

    int estimate_data_rate = 100; // Hz
    if (!nParam.getParam("estimate_data_rate", estimate_data_rate)) {
        ROS_WARN_STREAM("IMU Estimate data rate not set (Parameter: estimate_data_rate). Defaults to: " << estimate_data_rate);
    }

    bool publish_tf = true;
    if (!nParam.getParam("publish_tf", publish_tf)) {
        ROS_WARN_STREAM("Publish Transform not set (Parameter: publish_tf). Defaults to: " << publish_tf);
    }

    std::string world_frame = "world";
    if (!nParam.getParam("world_frame", world_frame)) {
        ROS_WARN_STREAM("World frame not set (Parameter: world_frame). Defaults to: " << world_frame);
    }

    std::string imu_frame = "imu_link";
    if (!nParam.getParam("imu_frame", imu_frame)) {
        ROS_WARN_STREAM("IMU frame not set (Parameter: imu_frame). Defaults to: " << imu_frame);
    }

    std::string position_imu_frame = "imu";
    if (!nParam.getParam("position_imu_frame", position_imu_frame)) {
        ROS_WARN_STREAM("IMU publish frame not set (Parameter: position_imu_frame). Defaults to: " << position_imu_frame);
    }

    bool enable_position_fusion = false;
    if (!nParam.getParam("enable_position_fusion", enable_position_fusion)) {
        ROS_WARN_STREAM("IMU Position fusion flag not set (Parameter: enable_position_fusion). Defaults to: " << enable_position_fusion);
    }

    bool enable_heading_fusion = false;
    if (!nParam.getParam("enable_heading_fusion", enable_heading_fusion)) {
        ROS_WARN_STREAM("IMU Heading fusion flag not set (Parameter: enable_heading_fusion). Defaults to: " << enable_heading_fusion);
    }

    int odom_downsample_rate = 10; // number of samples to skip + 1
    if (!nParam.getParam("odom_downsample_rate", odom_downsample_rate)) {
        ROS_WARN_STREAM("IMU Odometry downsample rate not set (Parameter: odom_downsample_rate). Defaults to: " << odom_downsample_rate);
    }

    float odom_position_stddev_m = 0.1; // m
    if (!nParam.getParam("odom_position_stddev_m", odom_position_stddev_m)) {
        ROS_WARN_STREAM("IMU Odometry position standard deviation not set (Parameter: odom_position_stddev_m). Defaults to: " << odom_position_stddev_m);
    }

    float odom_heading_stddev_deg = 1; // deg
    if (!nParam.getParam("odom_heading_stddev_deg", odom_heading_stddev_deg)) {
        ROS_WARN_STREAM("IMU Odometry heading standard deviation not set (Parameter: odom_heading_stddev_deg). Defaults to: " << odom_heading_stddev_deg);
    }

    try {
        IMUNode node(nh, nParam, world_frame, imu_frame, position_imu_frame, enable_position_fusion, enable_heading_fusion, odom_position_stddev_m, odom_heading_stddev_deg, publish_tf, raw_data_rate, estimate_data_rate, odom_downsample_rate);
        //IMUNode node(nh, nParam, world_frame, imu_frame, position_imu_frame, enable_position_fusion, enable_heading_fusion, odom_position_stddev_m, odom_heading_stddev_deg, publish_tf, "SpatialLog_20-06-17_15-56-07.anpp");
        node.StartLogging();
        ros::spin();
    } catch(std::exception& e){
        ROS_FATAL_STREAM("Exception thrown: " << e.what());
    }
}

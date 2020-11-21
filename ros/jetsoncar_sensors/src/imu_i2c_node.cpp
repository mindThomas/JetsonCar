#include <ros/ros.h>

#include "i2c_devices/ADXL345.h"
#include "i2c_devices/ITG3200.h"
#include "i2c_devices/HMC58X3.h"

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
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

#include <fstream>
#include <boost/filesystem.hpp>

#include "utils/utils.h"

class IMUNode {
private:
    // Sensors
    ADXL345 acc;
    ITG3200 gyro;
    HMC58X3 mag;

    // Sensor parameters
    const float g = 9.82; // 1g -> m/s
    float accelScale_;
    const float deg2rad = M_PI / 180.0f;
    float gyroScale_;
    float magScale_;

    // ROS specific
    ros::NodeHandle nh_;
    ros::NodeHandle nParam_;
    int poll_rate_;
    ros::Timer timerPublish_;

    // Frames
    std::string world_frame_;
    std::string imu_frame_;

    // Publishers
    ros::Publisher pub_imu_raw_;
    ros::Publisher pub_magnetic_field_;

    // Logging
    bool loggingEnabled{false};
    std::ofstream log_raw;

    typedef struct {
        float accelerometer[3]; // m/s^2
        float gyroscope[3];     // rad/s
        float magnetometer[3];  // milli Gauss
    } SensorMeasurements;

private:
    void RawSensorsUpdate(SensorMeasurements meas)
    {
        auto time = ros::Time().now();

        /*if (log_raw.is_open()) {
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
        }*/

        // Publish IMU message
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = time;
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

    void poll()
    {
        SensorMeasurements meas = {0};
        int16_t xa, ya, za;
        float xg, yg, zg, temperature;
        float xm, ym, zm;

        // Accelerometer Reading
        acc.readAccel(&xa, &ya, &za);         // Read the accelerometer values and store them in variables declared above x,y,z
        meas.accelerometer[0] = accelScale_ * xa;
        meas.accelerometer[1] = accelScale_ * ya;
        meas.accelerometer[2] = accelScale_ * za;
        ROS_DEBUG("Acc\tX,Y,Z = %04.1f,\t %04.1f,\t %04.1f", meas.accelerometer[0], meas.accelerometer[1], meas.accelerometer[2]);

        // Gyroscope Reading
        if (gyro.isRawDataReady()) {
            //gyro.readTemp(&temperature);
            gyro.readGyro(&xg, &yg, &zg);
            meas.gyroscope[0] = gyroScale_ * xg;
            meas.gyroscope[1] = gyroScale_ * yg;
            meas.gyroscope[2] = gyroScale_ * zg;
            ROS_DEBUG("Gyro\tX,Y,Z = %04.1f,\t %04.1f,\t %04.1f", meas.gyroscope[0], meas.gyroscope[1], meas.gyroscope[2]);
        }

        // Magnetometer reading
        mag.getValues(&xm, &ym, &zm);
        meas.magnetometer[0] = magScale_ * xm;
        meas.magnetometer[1] = magScale_ * ym;
        meas.magnetometer[2] = magScale_ * zm;
        ROS_DEBUG("Mag\tX,Y,Z = %04.1f,\t %04.1f,\t %04.1f", meas.magnetometer[0], meas.magnetometer[1], meas.magnetometer[2]);

        RawSensorsUpdate(meas);
    }

public:
    IMUNode(ros::NodeHandle nh, ros::NodeHandle nParam, std::string world_frame, std::string imu_frame, int poll_rate)
        : nh_(nh)
        , nParam_(nParam)
        , poll_rate_{poll_rate}
        , world_frame_{world_frame}
        , imu_frame_{imu_frame}
        , pub_imu_raw_{nh.advertise<sensor_msgs::Imu>("imu/raw", 10)}
        , pub_magnetic_field_{nh.advertise<sensor_msgs::MagneticField>("imu/mag", 10)}
    {
        /* Configure IMU sensors */
        auto configure_accelerometer = [&]() {
            // Accelerometer
            acc.powerOn();
            acc.setRangeSetting(2);           // Give the range settings
            accelScale_ = g * 2.0f / (float) std::pow(2, 9); // convert 10-bit to m/s
            // Accepted values are 2g, 4g, 8g or 16g
            // Higher Values = Wider Measurement Range
            // Lower Values = Greater Sensitivity
        };

        auto configure_gyroscope = [&]() {
            // Gyroscope
            // Use ITG3200_ADDR_AD0_HIGH or ITG3200_ADDR_AD0_LOW as the ITG3200 address
            // depending on how AD0 is connected on your breakout board, check its schematics for details
            gyro.init(ITG3200_ADDR_AD0_LOW);
            // Configure full range (+/- 2000 deg/s) and internal LPF filter setting
            gyroScale_ = deg2rad; // convert deg/s to rad/s
            //gyro.setFilterBW(BW256_SR8); // 256Khz BW and 8Khz SR
            gyro.setFilterBW(BW042_SR1); // 42 Hz BW and 1Khz SR

            ROS_INFO("Gyro: Calibrating for 5 seconds. Please wait...");
            int sample_time_ms = 1000 / poll_rate_;
            gyro.zeroCalibrate(5 * poll_rate_, sample_time_ms);
            ROS_INFO("Gyro: Finished calibrating");
        };

        auto configure_magnetometer = [&]() {
            // Magnetometer
            mag.init();
            ROS_INFO("Magnetometer: Calibrating, please wait...");
            mag.calibrate(0, 64);
            ROS_INFO("Magnetometer: Finished calibrating");
            mag.setMode(0); // set 75 Hz sample frequency, averaging of 8 samples and no artificial bias.
            magScale_ = 1000; // Gauss to milliGauss
        };

        // Call the 3 functions in parallel
        #pragma omp parallel // default(none) shared(arg)
        #pragma omp single
        {
            #pragma omp task
            configure_accelerometer();
            #pragma omp task
            configure_gyroscope();
            #pragma omp task
            configure_magnetometer();
        }

        /* Create a Poll timer for publishing the measurements */
        timerPublish_ = nh.createTimer(ros::Duration(1.0 / poll_rate_), std::bind(&IMUNode::poll, this));
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu_i2c_node");
    ros::NodeHandle nh;
    ros::NodeHandle nParam("~"); // default/current namespace node handle

    // Enable debug verbosity by default
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
        ros::console::notifyLoggerLevelsChanged();

    int poll_rate = 100; // Hz
    if (!nParam.getParam("poll_rate", poll_rate)) {
        ROS_WARN_STREAM("IMU Raw data rate not set (Parameter: poll_rate). Defaults to: " << poll_rate);
    }

    std::string world_frame = "world";
    if (!nParam.getParam("world_frame", world_frame)) {
        ROS_WARN_STREAM("World frame not set (Parameter: world_frame). Defaults to: " << world_frame);
    }

    std::string imu_frame = "imu_link";
    if (!nParam.getParam("imu_frame", imu_frame)) {
        ROS_WARN_STREAM("IMU frame not set (Parameter: imu_frame). Defaults to: " << imu_frame);
    }

    try {
        IMUNode node(nh, nParam, world_frame, imu_frame, poll_rate);
        ros::spin();
    } catch(std::exception& e){
        ROS_FATAL_STREAM("Exception thrown: " << e.what());
    }

	/*
    ros::Rate loop_rate(poll_rate);
    while (ros::ok())
    {
        node.poll();
        ros::spinOnce();
        loop_rate.sleep();
    }
	*/
}
#include "loggertab.h"
#include "ui_loggertab.h"

#ifdef WITH_ZED_CAMERA
LoggerTab::LoggerTab(IMU * imu_, Realsense * realsense_, ZED * zed_, QWidget *parent) :
#else
LoggerTab::LoggerTab(IMU * imu_, Realsense * realsense_, QWidget *parent) :
#endif
    QWidget(parent),
    ui(new Ui::LoggerTab),
    imu(imu_),
    realsense(realsense_)
    #ifdef WITH_ZED_CAMERA
    , zed(zed_)
    #endif
{
    ui->setupUi(this);

    imu->RegisterCallback_Accelerometer(std::bind(&LoggerTab::AccelerometerUpdate, this, std::placeholders::_1));
    imu->RegisterCallback_Gyroscope(std::bind(&LoggerTab::GyroscopeUpdate, this, std::placeholders::_1));
    imu->RegisterCallback_Magnetometer(std::bind(&LoggerTab::MagnetometerUpdate, this, std::placeholders::_1));
    imu->RegisterCallback_Orientation(std::bind(&LoggerTab::OrientationUpdate, this, std::placeholders::_1));
    imu->RegisterCallback_Estimate(std::bind(&LoggerTab::EstimateUpdate, this, std::placeholders::_1));
    imu->RegisterCallback_Acceleration(std::bind(&LoggerTab::AccelerationUpdate, this, std::placeholders::_1));
    imu->RegisterCallback_AngularVelocity(std::bind(&LoggerTab::AngularVelocityUpdate, this, std::placeholders::_1));
    imu->RegisterCallback_PositionFusion(std::bind(&LoggerTab::PositionFusionUpdate, this, std::placeholders::_1));

    realsense->RegisterCallback_Pose(std::bind(&LoggerTab::RealsensePoseUpdate, this, std::placeholders::_1));
}

LoggerTab::~LoggerTab()
{
    CloseLogFiles();
    delete ui;
}

void LoggerTab::OpenLogFiles()
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
    log_imu_accel.open(std::string(getenv("HOME")) + "/logs/" + logTimestamp + "_imu_accel.txt", std::ofstream::trunc);
    if (!log_imu_accel.is_open()) {
        std::cout << "Could not create " << logTimestamp + "_imu_accel.txt" << std::endl;
    }

    log_imu_gyro.open(std::string(getenv("HOME")) + "/logs/" + logTimestamp + "_imu_gyro.txt", std::ofstream::trunc);
    if (!log_imu_gyro.is_open()) {
        std::cout << "Could not create " << logTimestamp + "_imu_gyro.txt" << std::endl;
    }

    log_imu_mag.open(std::string(getenv("HOME")) + "/logs/" + logTimestamp + "_imu_mag.txt", std::ofstream::trunc);
    if (!log_imu_mag.is_open()) {
        std::cout << "Could not create " << logTimestamp + "_imu_mag.txt" << std::endl;
    }

    log_imu_estimate.open(std::string(getenv("HOME")) + "/logs/" + logTimestamp + "_imu_estimate.txt", std::ofstream::trunc);
    if (!log_imu_estimate.is_open()) {
        std::cout << "Could not create " << logTimestamp + "_imu_estimate.txt" << std::endl;
    }

    log_realsense_pose.open(std::string(getenv("HOME")) + "/logs/" + logTimestamp + "_realsense_pose.txt", std::ofstream::trunc);
    if (!log_realsense_pose.is_open()) {
        std::cout << "Could not create " << logTimestamp + "_realsense_pose.txt" << std::endl;
    }

    loggingEnabled = true;
}

void LoggerTab::CloseLogFiles()
{
    log_imu_accel.close();
    log_imu_gyro.close();
    log_imu_mag.close();
    log_imu_estimate.close();
    log_realsense_pose.close();

    loggingEnabled = false;
}

void LoggerTab::on_start_clicked()
{
    if (loggingEnabled) return;
    OpenLogFiles();
}

void LoggerTab::on_stop_clicked()
{
    if (!loggingEnabled) return;
    CloseLogFiles();
}

void LoggerTab::AccelerometerUpdate(IMU::SensorMeasurement accel)
{
    if (log_imu_accel.is_open()) {
        log_imu_accel << utils::utime() << "\t";
        log_imu_accel << std::setprecision(10) << accel.timestamp << "\t";
        log_imu_accel << std::setprecision(10) << accel.measurement[0] << "\t";
        log_imu_accel << std::setprecision(10) << accel.measurement[1] << "\t";
        log_imu_accel << std::setprecision(10) << accel.measurement[2];
        log_imu_accel << std::endl;
    }
}

void LoggerTab::GyroscopeUpdate(IMU::SensorMeasurement gyro)
{
    if (log_imu_gyro.is_open()) {
        log_imu_gyro << utils::utime() << "\t";
        log_imu_gyro << std::setprecision(10) << gyro.timestamp << "\t";
        log_imu_gyro << std::setprecision(10) << gyro.measurement[0] << "\t";
        log_imu_gyro << std::setprecision(10) << gyro.measurement[1] << "\t";
        log_imu_gyro << std::setprecision(10) << gyro.measurement[2];
        log_imu_gyro << std::endl;
    }
}

void LoggerTab::MagnetometerUpdate(IMU::SensorMeasurement mag)
{
    if (log_imu_mag.is_open()) {
        log_imu_mag << utils::utime() << "\t";
        log_imu_mag << std::setprecision(10) << mag.timestamp << "\t";
        log_imu_mag << std::setprecision(10) << mag.measurement[0] << "\t";
        log_imu_mag << std::setprecision(10) << mag.measurement[1] << "\t";
        log_imu_mag << std::setprecision(10) << mag.measurement[2];
        log_imu_mag << std::endl;
    }
}

void LoggerTab::OrientationUpdate(IMU::Orientation orientation)
{
    if (log_imu_estimate.is_open()) {
        log_imu_estimate << utils::utime() << "\t";
        log_imu_estimate << std::setprecision(10) << orientation.timestamp << "\t";
        log_imu_estimate << std::setprecision(10) << orientation.quaternion[3] << "\t"; // w
        log_imu_estimate << std::setprecision(10) << orientation.quaternion[0] << "\t"; // x
        log_imu_estimate << std::setprecision(10) << orientation.quaternion[1] << "\t"; // y
        log_imu_estimate << std::setprecision(10) << orientation.quaternion[2];         // z
        log_imu_estimate << std::endl;
    }
}

void LoggerTab::EstimateUpdate(IMU::Estimate estimate)
{

}

void LoggerTab::AccelerationUpdate(IMU::StateEstimate estimate)
{

}

void LoggerTab::AngularVelocityUpdate(IMU::StateEstimate estimate)
{

}

void LoggerTab::RealsensePoseUpdate(Eigen::Affine3f pose)
{
    if (log_realsense_pose.is_open()) {
        Eigen::Quaternionf quaternion(pose.rotation());
        log_realsense_pose << utils::utime() << "\t";
        log_realsense_pose << std::setprecision(10) << quaternion.w() << "\t";
        log_realsense_pose << std::setprecision(10) << quaternion.x() << "\t";
        log_realsense_pose << std::setprecision(10) << quaternion.y() << "\t";
        log_realsense_pose << std::setprecision(10) << quaternion.z() << "\t";
        log_realsense_pose << std::setprecision(10) << pose.translation().x() << "\t";
        log_realsense_pose << std::setprecision(10) << pose.translation().y() << "\t";
        log_realsense_pose << std::setprecision(10) << pose.translation().z();
        log_realsense_pose << std::endl;
    }
}

void LoggerTab::PositionFusionUpdate(IMU::PositionFusion estimate)
{

}

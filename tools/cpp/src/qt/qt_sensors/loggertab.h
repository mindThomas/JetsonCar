#ifndef LOGGERTAB_H
#define LOGGERTAB_H

#include <QWidget>

#include "imu_spatial/imu_spatial.h"
#include "realsense/realsense.h"

#ifdef WITH_ZED_CAMERA
#include "zed/zed.h"
#endif

#include <jetsoncar_utils/jetsoncar_utils.h>
#include <fstream>
#include <boost/filesystem.hpp>

namespace Ui {
class LoggerTab;
}

class LoggerTab : public QWidget
{
    Q_OBJECT

public:
    #ifdef WITH_ZED_CAMERA
    explicit LoggerTab(IMU * imu_, Realsense * realsense_, ZED * zed_, QWidget *parent = nullptr);
    #else
    explicit LoggerTab(IMU * imu_, Realsense * realsense_, QWidget *parent = nullptr);
    #endif
    ~LoggerTab();

private slots:
    void on_start_clicked();
    void on_stop_clicked();

private:
    Ui::LoggerTab *ui;

    IMU * imu;
    Realsense * realsense;

    #ifdef WITH_ZED_CAMERA
    ZED * zed;
    #endif

    bool loggingEnabled{false};
    std::ofstream log_imu_accel;
    std::ofstream log_imu_gyro;
    std::ofstream log_imu_mag;
    std::ofstream log_imu_estimate;
    std::ofstream log_realsense_pose;

    void OpenLogFiles();
    void CloseLogFiles();

    void AccelerometerUpdate(IMU::SensorMeasurement accel);
    void GyroscopeUpdate(IMU::SensorMeasurement gyro);
    void MagnetometerUpdate(IMU::SensorMeasurement mag);
    void OrientationUpdate(IMU::Orientation orientation);
    void EstimateUpdate(IMU::Estimate estimate);
    void AccelerationUpdate(IMU::StateEstimate estimate);
    void AngularVelocityUpdate(IMU::StateEstimate estimate);
    void RealsensePoseUpdate(Eigen::Affine3f pose);
    void PositionFusionUpdate(IMU::PositionFusion estimate);
};

#endif // LOGGERTAB_H

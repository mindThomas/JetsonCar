#ifndef IMUTAB_H
#define IMUTAB_H

#include <QWidget>
#include <QTimer>

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <mutex>

#include <QtWidgets/QGridLayout>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QWidget>
#include "qcustomplot.h"
#include "imusubtab.h"
#include "imu3dview.h"

#include "imu_spatial/imu_spatial.h"
#include "realsense/realsense.h"

#include "ui_imutab.h"

class ImuTab : public QWidget
{
    Q_OBJECT

public:
    explicit ImuTab(IMU * imu_, Realsense * realsense_, QWidget *parent = nullptr);
    ~ImuTab();

private:
    const uint16_t sample_rate{100};

private:        
    IMU * imu;
    Realsense * realsense;

    ImuSubTab * accelerometerTab;
    ImuSubTab * gyroscopeTab;
    ImuSubTab * magnetometerTab;
    ImuSubTab * eulerTab;
    IMU3DView * orientationTab;
    ImuSubTab * velocityTab;
    ImuSubTab * positionTab;
    ImuSubTab * positionLatLonTab;

    QCheckBox * HeadingFusionEnabled;
    QCheckBox * PositionFusionEnabled;

    void AccelerometerUpdate(IMU::SensorMeasurement accel);
    void GyroscopeUpdate(IMU::SensorMeasurement gyro);
    void MagnetometerUpdate(IMU::SensorMeasurement mag);
    void EulerRPYUpdate(IMU::Orientation orientation);
    void EstimateUpdate(IMU::Estimate estimate);
    void AccelerationUpdate(IMU::StateEstimate estimate);
    void AngularVelocityUpdate(IMU::StateEstimate estimate);
    void RealsensePoseUpdate(Eigen::Affine3f pose);
    void PositionFusionUpdate(IMU::PositionFusion estimate);
};

#endif // IMUTAB_H

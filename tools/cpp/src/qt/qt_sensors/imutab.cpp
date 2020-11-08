#include "imutab.h"

ImuTab::ImuTab(IMU * imu_, Realsense * realsense_, QWidget *parent) :
    imu(imu_),
    realsense(realsense_),
    QWidget(parent)
{
    QGridLayout * gridLayout = new QGridLayout(this);
    gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
    QTabWidget * tabWidget = new QTabWidget(parent);
    tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
    gridLayout->addWidget(tabWidget, 0, 0, 1, 1);
    tabWidget->setCurrentIndex(-1);

    {                     
        accelerometerTab = new ImuSubTab(sample_rate);
        accelerometerTab->AddPlotOption("IMU Accelerometer", "Acceleration [m/s^2]");
        accelerometerTab->AddPlotOption("IMU Acceleration estimate", "Acceleration [m/s^2]");
        accelerometerTab->AddPlotOption("Realsense Accelerometer", "Acceleration [m/s^2]");
        tabWidget->addTab(accelerometerTab, "Accelerometer");
    }

    {
        gyroscopeTab = new ImuSubTab(sample_rate);
        gyroscopeTab->AddPlotOption("IMU Gyroscope", "Angular velocity [rad/s]");
        gyroscopeTab->AddPlotOption("IMU Angular velocity estimate", "Angular velocity [rad/s]");
        gyroscopeTab->AddPlotOption("Realsense Gyroscope", "Angular velocity [rad/s]");
        gyroscopeTab->AddPlotOption("Realsense Angular velocity estimate", "Angular velocity [rad/s]");
        tabWidget->addTab(gyroscopeTab, "Gyroscope");
    }

    {
        magnetometerTab = new ImuSubTab(sample_rate);
        magnetometerTab->AddPlotOption("Magnetometer", "Magnetic field [mG]");
        tabWidget->addTab(magnetometerTab, "Magnetometer");
    }

    {
        eulerTab = new ImuSubTab(sample_rate);
        eulerTab->AddPlotOption("IMU", "Angle [deg]");
        eulerTab->AddPlotOption("IMU + Realsense", "Angle [deg]", true);
        eulerTab->ChangeLegend(0, "IMU Roll");
        eulerTab->ChangeLegend(1, "IMU Pitch");
        eulerTab->ChangeLegend(2, "IMU Yaw");
        eulerTab->ChangeLegend(3, "Realsense Roll");
        eulerTab->ChangeLegend(4, "Realsense Pitch");
        eulerTab->ChangeLegend(5, "Realsense Yaw");

        HeadingFusionEnabled = new QCheckBox(eulerTab);
        HeadingFusionEnabled->setObjectName(QString::fromUtf8("HeadingFusionEnabled"));
        HeadingFusionEnabled->setText("Heading Fusion Enabled");
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Maximum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(HeadingFusionEnabled->sizePolicy().hasHeightForWidth());
        HeadingFusionEnabled->setSizePolicy(sizePolicy);
        HeadingFusionEnabled->setChecked(false);
        eulerTab->ui->horizontalLayout->addWidget(HeadingFusionEnabled);

        tabWidget->addTab(eulerTab, "Euler");
    }

    {
        orientationTab = new IMU3DView;
        tabWidget->addTab(orientationTab, "Orientation");
    }

    {
        velocityTab = new ImuSubTab(sample_rate);
        velocityTab->AddPlotOption("Velocity", "Velocity [m/s]", true);
        velocityTab->ChangeLegend(0, "IMU X");
        velocityTab->ChangeLegend(1, "IMU Y");
        velocityTab->ChangeLegend(2, "IMU Z");
        velocityTab->ChangeLegend(3, "Realsense X");
        velocityTab->ChangeLegend(4, "Realsense Y");
        velocityTab->ChangeLegend(5, "Realsense Z");
        tabWidget->addTab(velocityTab, "Velocity");
    }

    {
        positionTab = new ImuSubTab(sample_rate);
        positionTab->AddPlotOption("Position", "Position [m]", true);
        positionTab->ChangeLegend(0, "IMU X");
        positionTab->ChangeLegend(1, "IMU Y");
        positionTab->ChangeLegend(2, "IMU Z");
        positionTab->ChangeLegend(3, "Realsense X");
        positionTab->ChangeLegend(4, "Realsense Y");
        positionTab->ChangeLegend(5, "Realsense Z");
        positionTab->ui->plotOption->hide();

        PositionFusionEnabled = new QCheckBox(positionTab);
        PositionFusionEnabled->setObjectName(QString::fromUtf8("PositionFusionEnabled"));
        PositionFusionEnabled->setText("Position Fusion Enabled");
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Maximum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(PositionFusionEnabled->sizePolicy().hasHeightForWidth());
        PositionFusionEnabled->setSizePolicy(sizePolicy);
        PositionFusionEnabled->setChecked(false);
        positionTab->ui->horizontalLayout->addWidget(PositionFusionEnabled);

        tabWidget->addTab(positionTab, "Position");
    }
    {
        positionLatLonTab = new ImuSubTab(sample_rate);
        positionLatLonTab->AddPlotOption("Position", "Lat/Lon [deg], Alt [m]", true);
        positionLatLonTab->ChangeLegend(0, "Setpoint Lat");
        positionLatLonTab->ChangeLegend(1, "Setpoint Lon");
        positionLatLonTab->ChangeLegend(2, "Setpoint Alt");
        positionLatLonTab->ChangeLegend(3, "Estimate Lat");
        positionLatLonTab->ChangeLegend(4, "Estimate Lon");
        positionLatLonTab->ChangeLegend(5, "Estimate Alt");
        positionLatLonTab->ui->plotOption->hide();
        tabWidget->addTab(positionLatLonTab, "Position Lat/Lon");
    }

    imu->Configure(IMU::OutputType::All, sample_rate);
    imu->RegisterCallback_Accelerometer(std::bind(&ImuTab::AccelerometerUpdate, this, std::placeholders::_1));
    imu->RegisterCallback_Gyroscope(std::bind(&ImuTab::GyroscopeUpdate, this, std::placeholders::_1));
    imu->RegisterCallback_Magnetometer(std::bind(&ImuTab::MagnetometerUpdate, this, std::placeholders::_1));
    imu->RegisterCallback_Orientation(std::bind(&ImuTab::EulerRPYUpdate, this, std::placeholders::_1));
    imu->RegisterCallback_Estimate(std::bind(&ImuTab::EstimateUpdate, this, std::placeholders::_1));
    imu->RegisterCallback_Acceleration(std::bind(&ImuTab::AccelerationUpdate, this, std::placeholders::_1));
    imu->RegisterCallback_AngularVelocity(std::bind(&ImuTab::AngularVelocityUpdate, this, std::placeholders::_1));
    imu->RegisterCallback_PositionFusion(std::bind(&ImuTab::PositionFusionUpdate, this, std::placeholders::_1));

    realsense->RegisterCallback_Pose(std::bind(&ImuTab::RealsensePoseUpdate, this, std::placeholders::_1));
 }

ImuTab::~ImuTab()
{
    imu->ClearCallbacks();
    imu->Disconnect();   
}

void ImuTab::AccelerometerUpdate(IMU::SensorMeasurement accel)
{
    accelerometerTab->AddMeasurement(0, accel.timestamp, accel.measurement);
}

void ImuTab::GyroscopeUpdate(IMU::SensorMeasurement gyro)
{
    gyroscopeTab->AddMeasurement(0, gyro.timestamp, gyro.measurement);
}

void ImuTab::MagnetometerUpdate(IMU::SensorMeasurement mag)
{
    magnetometerTab->AddMeasurement(0, mag.timestamp, mag.measurement);
}

void ImuTab::EulerRPYUpdate(IMU::Orientation orientation)
{
    Eigen::Vector3f RPYdeg;
    RPYdeg[0] = rad2deg(orientation.RPY[0]);
    RPYdeg[1] = rad2deg(orientation.RPY[1]);
    RPYdeg[2] = rad2deg(orientation.RPY[2]);
    eulerTab->AddMeasurement(0, orientation.timestamp, RPYdeg);
    eulerTab->AddMeasurement(1, orientation.timestamp, RPYdeg);
}

void ImuTab::EstimateUpdate(IMU::Estimate estimate)
{
    Eigen::Vector3f RPYdeg;
    RPYdeg[0] = rad2deg(estimate.RPY[0]);
    RPYdeg[1] = rad2deg(estimate.RPY[1]);
    RPYdeg[2] = rad2deg(estimate.RPY[2]);

    orientationTab->setRollPitchYaw(RPYdeg[0], RPYdeg[1], RPYdeg[2]);
    //orientationTab->setQuanternions(estimate.quaternion[3], estimate.quaternion[0], estimate.quaternion[1], estimate.quaternion[2]);

    positionTab->AddMeasurement(0, estimate.timestamp, estimate.position_NED);
}

void ImuTab::AccelerationUpdate(IMU::StateEstimate estimate)
{
    accelerometerTab->AddMeasurement(1, estimate.timestamp, estimate.estimate);
}

void ImuTab::AngularVelocityUpdate(IMU::StateEstimate estimate)
{
    gyroscopeTab->AddMeasurement(1, estimate.timestamp, estimate.estimate);
}

void ImuTab::RealsensePoseUpdate(Eigen::Affine3f pose)
{
    static size_t count = 0;
    //auto pose_IMUframe = Eigen::AngleAxis<float>(M_PI, Eigen::Vector3f(0,0,1)) * pose * Eigen::AngleAxis<float>(M_PI, Eigen::Vector3f(1,0,0));
    auto XYZ = Realsense::GetPoseTranslation(pose);
    auto RPY = Realsense::GetPoseEulerRPY(pose, true);

    auto position_NED = XYZ;
    position_NED[1] *= -1;
    position_NED[2] *= -1;

    auto RPY_NED = RPY;
    RPY_NED[0] *= -1;
    RPY_NED[2] *= -1;
    RPY_NED[2] += 180;
    if (RPY_NED[2] > 180)
        RPY_NED[2] -= 360;

    QString text;
    text.sprintf("XYZ = %+.2f, %+.2f, %+.2f", position_NED[0], position_NED[1], position_NED[2]);
    qDebug() << text;
    text.sprintf("RPY = %+.2f, %+.2f, %+.2f", RPY_NED[0], RPY_NED[1], RPY_NED[2]);
    qDebug() << text;

    if ((count++ % 100) == 0) {
        if (HeadingFusionEnabled->isChecked())
            imu->SetHeading(deg2rad(-RPY[2]), 0.1);
        if (PositionFusionEnabled->isChecked())
            imu->SetPositionNED(position_NED[0], position_NED[1], position_NED[2], 1);
    }

    eulerTab->AddSecondMeasurement(1, imu->GetLatestTimestamp(), RPY);//RPY_NED);
    positionTab->AddSecondMeasurement(0, imu->GetEstimate().timestamp, position_NED);
}

void ImuTab::PositionFusionUpdate(IMU::PositionFusion estimate)
{
    positionLatLonTab->AddMeasurement(0, estimate.timestamp, estimate.position_setpoint);
    positionLatLonTab->AddSecondMeasurement(0, estimate.timestamp, estimate.position_estimate);
}

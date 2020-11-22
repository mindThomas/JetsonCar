#ifndef JETSONCAR_LIBRARIES_IMU_SPATIAL_H
#define JETSONCAR_LIBRARIES_IMU_SPATIAL_H

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include <string>
#include <numeric>
#include <cmath>

#include <thread>
#include <array>
#include <algorithm>
#include <mutex>
#include <condition_variable>

#include <boost/asio.hpp>
#include <boost/math/quaternion.hpp>

#if defined(__linux__)
#include <linux/serial.h>
#endif

#include "imu_spatial_protocol_dynamic/an_packet_protocol.h"
#include "imu_spatial_protocol_dynamic/spatial_packets.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#define deg2rad(angleDegrees) ((angleDegrees) * M_PI / 180.0)
#define rad2deg(angleRadians) ((angleRadians) * 180.0 / M_PI)

namespace geodetic_converter {
    class GeodeticConverter;
}

class IMU
{

public:
    typedef Eigen::Vector4f Quaternion;
    typedef Eigen::Vector3f Vector;

    /*struct Quaternion {
        Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {};
        Quaternion(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {};
        Quaternion(Eigen::Vector4f &q) : w(q.w()), x(q.x()), y(q.y()), z(q.z()) {};
        Quaternion(Eigen::Vector3f &v) : w(0.0f), x(v.x()), y(v.y()), z(v.z()) {};
        float w;
        float x;
        float y;
        float z;
    };

    struct Vector {
        Vector() : x(0.0f), y(0.0f), z(0.0f) {};
        Vector(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {};
        Vector(Eigen::Vector3f &v) : x(v.x()), y(v.y()), z(v.z()) {};
        float x;
        float y;
        float z;
    };*/

    /*typedef struct Estimate {
        double timestamp;
        struct {
            Quaternion quaternion; // Orientation
            Vector RPY; // Euler angles
            Vector velocity; // v [body frame]
            Vector acceleration; // a [body frame]
            Vector angular_velocity; // omega [body frame]
            Vector angular_acceleration; // domega [body frame]
        } estimates;
        struct {
            Quaternion quaternion; // standard deviation (estimate uncertainty)
            Vector RPY; // Euler angles
            Vector velocity; // v
            Vector angular_velocity; // omega
        } standard_deviation;
    } Estimate;*/

    typedef struct Estimate {
        double timestamp;
        Quaternion quaternion; // Orientation
        Vector RPY; // Euler angles
        Vector velocity; // v [body frame]
        Vector acceleration; // a [body frame]
        Vector angular_velocity; // omega [body frame]
        Vector angular_acceleration; // domega [body frame]
        Vector position_NED;
    } Estimate;

    typedef struct PositionFusion {
        double timestamp;
        Eigen::Vector3d position_setpoint; // lat, lon, altitude
        Eigen::Vector3d position_estimate; // lat, lon, altitude
    } PositionFusion;

    typedef struct Orientation {
        double timestamp;
        Quaternion quaternion; // Orientation
        Vector RPY; // Euler angles
    } Orientation;

    typedef struct QuaternionStdDev {
        double timestamp;
        Eigen::Vector4f stddev;
    } QuaternionStdDev;

    typedef struct EulerEstimate {
        double timestamp;
        Vector RPY;
        Vector sigma; // standard deviation (estimate uncertainty)
    } EulerEstimate;

    typedef struct StateEstimate {
        double timestamp;
        Eigen::Vector3f estimate;
    } StateEstimate;

    // Sensor measurement used for Accelerometer, Gyroscope and Magnetometer measurements
    typedef struct SensorMeasurement {
        double timestamp;
        Eigen::Vector3f measurement;
    } SensorMeasurement;

    typedef struct CombinedSensorsMeasurement {
        double timestamp;
        Eigen::Vector3f accelerometer; // m/s^2
        Eigen::Vector3f gyroscope; // rad/s
        Eigen::Vector3f magnetometer; // milli Gauss
    } CombinedSensorsMeasurement;

    typedef enum OutputType : uint8_t {
        RawSensors = 0,
        Estimates = 1,
        IndividualEstimates = 2,
        RawAndIndividualEstimates = 3,
        All = 4
    } OutputType;

public:
    IMU(std::string port_name = "/dev/ttyUSB0", uint32_t baud_rate = 460800);
    ~IMU();

    void Connect();
    void Disconnect();
    void SynchronizeTime(); // resets the timestamps synchronously. The time at which this function returns the IMU timestamp is 0.
    void Configure(OutputType output = RawSensors, uint16_t RawDataRate = 200, uint16_t EstimateDataRate = 100, bool enableExternalHeadingInput = false, bool enableExternalPositionInput = false, bool includeStandardDeviations = false);

    void LoadLog(std::string log_path);
    void RecordANPPLog(std::string log_folder = "");
    void StopANPPLog();

    void SetHeading(float heading_rad, float standard_deviation_rad); // assuming z-down, positive direction is clockwise
    void SetPositionNED(float x, float y, float z, float standard_deviation_m);
    void StartStaticPositionPublisher(float x_N = 0, float y_E = 0, float z_D = 0, float standard_deviation_m = 0.5);

    Estimate GetEstimate();
    Orientation GetOrientation();
    Quaternion GetQuaternion();
    QuaternionStdDev GetQuaternionStdDev();
    CombinedSensorsMeasurement GetRawSensors();
    SensorMeasurement GetAccelerometer();
    SensorMeasurement GetGyroscope();
    SensorMeasurement GetMagnetometer();
    StateEstimate GetVelocity();
    StateEstimate GetAcceleration();
    StateEstimate GetAngularVelocity();
    StateEstimate GetAngularAcceleration();
    double GetLatestTimestamp();
    bool GetStatus();

    Eigen::Vector3f GetPoseEulerRPY(bool degrees = false);
    static Eigen::Vector3f GetPoseEulerRPY(Quaternion orientation, bool degrees = false);
    static Eigen::Vector3f GetPoseEulerRPY(Eigen::Matrix<float, 3, 3> rotation, bool degrees = false);
    static Eigen::Vector3f GetPoseEulerRPY(Eigen::Affine3f pose, bool degrees = false);
    static Eigen::Matrix<float, 3, 3> GetRotation(Quaternion& q);
    static Eigen::Matrix<float, 3, 3> GetRotation(Estimate& est);
    static Quaternion GetRotation(Vector& RPY);

    void RegisterCallback_RawSensors(const std::function<void(CombinedSensorsMeasurement)> &callback);
    void RegisterCallback_Accelerometer(const std::function<void(SensorMeasurement)> &callback);
    void RegisterCallback_Gyroscope(const std::function<void(SensorMeasurement)> &callback);
    void RegisterCallback_Magnetometer(const std::function<void(SensorMeasurement)> &callback);
    void RegisterCallback_Estimate(const std::function<void(Estimate)> &callback);
    void RegisterCallback_Orientation(const std::function<void(Orientation)> &callback);
    void RegisterCallback_QuaternionStdDev(const std::function<void(QuaternionStdDev)> &callback);
    void RegisterCallback_Velocity(const std::function<void(StateEstimate)> &callback);
    void RegisterCallback_Acceleration(const std::function<void(StateEstimate)> &callback);
    void RegisterCallback_AngularVelocity(const std::function<void(StateEstimate)> &callback);
    void RegisterCallback_AngularAcceleration(const std::function<void(StateEstimate)> &callback);
    void RegisterCallback_PositionNED(const std::function<void(StateEstimate)> &callback);
    void RegisterCallback_PositionUTM(const std::function<void(StateEstimate)> &callback);
    void RegisterCallback_PositionFusion(const std::function<void(PositionFusion)> &callback);
    void ClearCallbacks();

private:
    std::string port_name_;
    uint32_t baud_rate_;
    an_decoder_t an_decoder_;
    uint16_t tick_period_us_{1000}; // 1 ms
    bool externalHeadingInputEnabled_;
    bool enableExternalPositionInput_;

    // For recording/logging
    FILE * anpp_log_file{0};
    size_t anpp_logged_bytes{0};

    // Serial port
    boost::asio::io_service ioservice;
    boost::asio::serial_port port_ =
            boost::asio::serial_port(ioservice);
    std::thread ioservice_thread;
    std::thread position_publisher_thread;
    const bool hwflow_{false};
    bool connected_{false};

    std::unique_ptr<geodetic_converter::GeodeticConverter> g_geodetic_converter;

    // Received packages/measurements
    system_state_packet_t system_state_packet_{0};
    raw_sensors_packet_t raw_sensors_packet_{0};

    quaternion_orientation_packet_t quaternion_packet_{0};

    body_velocity_packet_t body_velocity_packet_{0};
    body_acceleration_packet_t body_acceleration_packet_{0};

    acceleration_packet_t acceleration_packet_{0};

    angular_velocity_packet_t angular_velocity_packet_{0};
    angular_acceleration_packet_t angular_acceleration_packet_{0};

    geodetic_position_packet_t geodetic_position_packet_{0};
    utm_position_packet_t utm_position_packet_{0};

    quaternion_orientation_standard_deviation_packet_t quaternion_std_packet_{0};
    euler_orientation_standard_deviation_packet_t euler_orientation_std_packet_t{0};
    velocity_standard_deviation_packet_t velocity_std_packet_{0};

    status_packet_t status_packet_{0};
    running_time_packet_t running_time_packet_{0};
    device_information_packet_t device_information_packet_{0};

    // Latest readings
    uint32_t initUnixTimeSeconds{0};
    bool timeSyncEnabled;
    std::mutex timeSyncMutex;
    std::condition_variable timeSyncCV;
    double initTimestamp{0};
    double latestTimestamp{0};
    Estimate latestEstimate{0, Quaternion::Zero(), Vector::Zero(), Vector::Zero(), Vector::Zero(), Vector::Zero(), Vector::Zero()};
    Orientation latestOrientation{0, Quaternion::Zero()};
    CombinedSensorsMeasurement latestRawSensors{0, Vector::Zero(), Vector::Zero(), Vector::Zero()};
    StateEstimate latestVelocity{0, Vector::Zero()};
    StateEstimate latestAcceleration{0, Vector::Zero()};
    StateEstimate latestAngularVelocity{0, Vector::Zero()};
    StateEstimate latestAngularAcceleration{0, Vector::Zero()};
    QuaternionStdDev latestQuaternionStdDev{0, Quaternion::Zero()};
    PositionFusion latestPositionFusion{0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    StateEstimate latestGeodeticPosition{0, Vector::Zero()};
    StateEstimate latestPositionNED{0, Vector::Zero()};
    StateEstimate latestPositionUTM{0, Vector::Zero()};

private:
    struct {
        std::vector<std::function<void(CombinedSensorsMeasurement)>> raw_sensors;
        std::vector<std::function<void(SensorMeasurement)>> accel;
        std::vector<std::function<void(SensorMeasurement)>> gyro;
        std::vector<std::function<void(SensorMeasurement)>> mag;
        std::vector<std::function<void(Estimate)>> estimate;
        std::vector<std::function<void(Orientation)>> orientation;
        std::vector<std::function<void(QuaternionStdDev)>> quaternion_stddev;
        std::vector<std::function<void(StateEstimate)>> velocity;
        std::vector<std::function<void(StateEstimate)>> acceleration;
        std::vector<std::function<void(StateEstimate)>> angular_velocity;
        std::vector<std::function<void(StateEstimate)>> angular_acceleration;
        std::vector<std::function<void(StateEstimate)>> positionNED;
        std::vector<std::function<void(StateEstimate)>> positionUTM;
        std::vector<std::function<void(PositionFusion)>> position_fusion;
    } callbacks;

    /// @brief Different ways a serial port may be flushed.
    enum flush_type
    {
        flush_receive = TCIFLUSH,
        flush_send = TCOFLUSH,
        flush_both = TCIOFLUSH
    };

private:
    int SendPackage(an_packet_t* packet);
    boost::system::error_code flush_serial_port( boost::asio::serial_port& serial_port, flush_type what);

    void SerialCallback(const boost::system::error_code &error, std::size_t bytes_transferred);
    void ProcessIncomingBytes(size_t bytes_read);
    void ReceiveNextPackage();
    void ProcessPackage(an_packet_t * an_packet);

    bool accelerometerStatus();
    bool gyroscopeStatus();
    bool magnetometerStatus();
    bool temperatureStatus();
    bool voltageStatus();
    bool filterStatus();

    void Reset();


    static Eigen::Vector3f GetEulerAnglesZYX(Eigen::Matrix<float, 3, 3> rotm);
    static Eigen::Vector3f GetEulerAnglesZYX(Eigen::Matrix<float, 4, 4> transform);

    static Quaternion quaternion_exp(Vector v);
    static Quaternion quaternion_multiply(Quaternion a, Quaternion b);
    static void quaternion_integration(boost::math::quaternion<float> &q, float gyroscopes[3], const float dt);
    static double norm( const float x[], std::size_t sz );
    static Quaternion PredictOrientation(Estimate &est, float dt_s);
    static int64_t utime();

};

#endif

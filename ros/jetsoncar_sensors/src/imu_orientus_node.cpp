#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <numeric>
#include <cmath>

#include <thread>
#include <array>
#include <algorithm>

#include <boost/asio.hpp>

#if defined(__linux__)
#include <linux/serial.h>
#endif

#include "imu_spatial_protocol_dynamic/an_packet_protocol.h"
#include "imu_spatial_protocol_dynamic/spatial_packets.h"

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/Temperature.h"
#include "geometry_msgs/Quaternion.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include <tf/tf.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>
/*#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>*/
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <easy/profiler.h>

class OrientusNode {
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Rate loop_rate_;
  std::string frame_id_;
  ros::Publisher imu_pub_;
  ros::Publisher imu_raw_pub_;
  ros::Publisher magnetic_field_pub_;
  ros::Publisher temperature_pub_;
  diagnostic_updater::Updater diagnostic_;

  //boost::asio::io_service io_service_;
  std::string port_name_;
  //boost::asio::serial_port port_;
  an_decoder_t an_decoder_;

  quaternion_orientation_standard_deviation_packet_t quaternion_std_packet_;
  acceleration_packet_t acceleration_packet_;
  quaternion_orientation_packet_t quaternion_packet_;
  angular_velocity_packet_t angular_velocity_packet_;
  raw_sensors_packet_t raw_sensors_packet_;
  status_packet_t status_packet_;
  running_time_packet_t running_time_packet_;
  device_information_packet_t device_information_packet_;
  bool quaternion_orientation_std_received_;
  bool quaternion_orientation_received_;
  bool acceleration_received_;
  bool angular_velocity_received_;
  bool raw_sensors_received_;
  bool status_received_;
  bool running_time_received_;
  bool device_information_received_;

    // Serial port
    boost::asio::io_service ioservice;
    boost::asio::serial_port port_ =
            boost::asio::serial_port(ioservice);
    std::thread ioservice_thread;
    bool hwflow_ = false;

    tf::TransformBroadcaster tf_broadcaster_;
    ros::Publisher pub_odom_;

    tf::Quaternion q_est{0.0, 0.0, 0.0, 1.0};
    int print_count{0};

public:
  OrientusNode(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(nh), pnh_(pnh), loop_rate_(1000),
      diagnostic_(nh_, pnh_), port_(ioservice),
      pub_odom_(nh.advertise<nav_msgs::Odometry>("odom", 50)) { //port_(io_service_){

    pnh_.param("port", port_name_, std::string("/dev/ttyUSB0"));
    pnh_.param("frame_id", frame_id_, std::string("imu"));

    open();

      port_.async_read_some(boost::asio::buffer(an_decoder_pointer(&an_decoder_), an_decoder_size(&an_decoder_)),
                            std::bind(&OrientusNode::processSerial, this, std::placeholders::_1, std::placeholders::_2));
    /*boost::asio::async_read(
            //port_, boost::asio::buffer(read_buffer),
            port_, boost::asio::buffer(an_decoder_pointer(&an_decoder_), std::min(an_decoder_size(&an_decoder_), size_t(10))),
          std::bind(&OrientusNode::processSerial, this, std::placeholders::_1, std::placeholders::_2));
    */
    // Start the I/O service in its own thread.
    ioservice_thread = std::thread([&] { ioservice.run(); });

    an_packet_t *an_packet;
    // Request device information
    an_packet = encode_request_packet(packet_id_device_information);
    an_packet_encode_and_send(an_packet);
    an_packet_free(&an_packet);

    // Setup packet rate timer
    packet_timer_period_packet_t packet_timer_period_packet;
    packet_timer_period_packet.permanent = 0;
    packet_timer_period_packet.packet_timer_period = 1000; // 1 ms
    an_packet = encode_packet_timer_period_packet(&packet_timer_period_packet);
    an_packet_encode_and_send(an_packet);
    an_packet_free(&an_packet);

    // Configure packet rates
    packet_periods_packet_t packet_periods_packet;
    packet_periods_packet.permanent = 0;
    packet_periods_packet.clear_existing_packets = 1;
    packet_periods_packet.packet_periods[0].packet_id = packet_id_quaternion_orientation_standard_deviation;
    packet_periods_packet.packet_periods[0].period = 10; // 100 Hz
    packet_periods_packet.packet_periods[1].packet_id = packet_id_quaternion_orientation;
    packet_periods_packet.packet_periods[1].period = 10; // 100 Hz
    packet_periods_packet.packet_periods[2].packet_id = packet_id_acceleration;
    packet_periods_packet.packet_periods[2].period = 10; // 100 Hz
    packet_periods_packet.packet_periods[3].packet_id = packet_id_angular_velocity;
    packet_periods_packet.packet_periods[3].period = 2; // 100 Hz
    packet_periods_packet.packet_periods[4].packet_id = packet_id_raw_sensors;
    packet_periods_packet.packet_periods[4].period = 2; // 500 Hz

    packet_periods_packet.packet_periods[5].packet_id = packet_id_status;
    packet_periods_packet.packet_periods[5].period = 200; // 5 Hz
    packet_periods_packet.packet_periods[6].packet_id = packet_id_running_time;
    packet_periods_packet.packet_periods[6].period = 200; // 5 Hz

    packet_periods_packet.packet_periods[7].packet_id = 0;
    an_packet = encode_packet_periods_packet(&packet_periods_packet);
    an_packet_encode_and_send(an_packet);
    an_packet_free(&an_packet);


    // Setup ROS topic
    imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu/data", 10);
    imu_raw_pub_ = nh.advertise<sensor_msgs::Imu>("imu/raw", 10);
    magnetic_field_pub_ = nh.advertise<sensor_msgs::MagneticField>("imu/mag", 10);
    temperature_pub_ = nh.advertise<sensor_msgs::Temperature>("imu/temp", 10);


    // Setup diagnostics
    diagnostic_.setHardwareID("orientus");
    diagnostic_.add("IMU Status", this, &OrientusNode::deviceStatus);
    diagnostic_.add("Accelerometer Status", this, &OrientusNode::accelerometerStatus);
    diagnostic_.add("Gyroscope Status", this, &OrientusNode::gyroscopeStatus);
    diagnostic_.add("Magnetometer Status", this, &OrientusNode::magnetometerStatus);
    diagnostic_.add("Temperature Status", this, &OrientusNode::temperatureStatus);
    diagnostic_.add("Voltage Status", this, &OrientusNode::voltageStatus);
    diagnostic_.add("Filter Status", this, &OrientusNode::filterStatus);


    an_decoder_initialise(&an_decoder_);

    quaternion_orientation_std_received_ = false;
    quaternion_orientation_received_ = false;
    acceleration_received_ = false;
    raw_sensors_received_ = false;
    status_received_ = false;
    running_time_received_ = false;
    device_information_received_ = false;
  }
  ~OrientusNode()
  {
      ioservice.stop();

      if (ioservice_thread.joinable()) {
          ioservice_thread.join();
      }
  }

  void spin() {
      ros::Time t_start = ros::Time::now();

    while (ros::ok()) {
    EASY_BLOCK("Publishing messages");
      //while(receive_next_packet()){
	if(/*quaternion_orientation_std_received_ && quaternion_orientation_received_
	   && acceleration_received_ && */angular_velocity_received_ && raw_sensors_received_) {
	  publish_imu_msg();
      quaternion_integration(q_est, angular_velocity_packet_.angular_velocity, 0.002f);
      publish_imu_orienation_tf();
	  quaternion_orientation_std_received_ = false;
	  quaternion_orientation_received_ = false;
	  acceleration_received_ = false;
	  angular_velocity_received_ = false;
	}
	if(raw_sensors_received_) {
	  publish_imu_raw_msg();
	  publish_magnetics_msg();
	  publish_temperature_msg();
      //quaternion_integration(q_est, raw_sensors_packet_.gyroscopes, 0.002f);
	  raw_sensors_received_ = false;
	}

	if(status_received_ && running_time_received_ && device_information_received_){
	  diagnostic_.update();
	  status_received_ = false;
	  running_time_received_ = false;
	}

	if (print_count++ % 200) {
        print_count = 0;
        ROS_WARN_STREAM("Integrated Gyroscope drift = " << q_est.getAngle() * 180.0 / M_PI << " deg");

        ros::Time t_now = ros::Time::now();
        ros::Duration diff = t_now - t_start;
        ROS_WARN_STREAM("Integrated Gyroscope drift rate = " << 180.0 / M_PI * q_est.getAngle() / diff.toSec() << " deg/s");

        double roll, pitch, yaw;
        tf::Matrix3x3(q_est).getRPY(roll, pitch, yaw);
        ROS_WARN_STREAM("Integrated Gyroscope Euler [deg] = " << "\n\t" <<roll*180.0/M_PI << "\n\t" << pitch*180.0/M_PI << "\n\t" << yaw*180.0/M_PI);
	}

      //}
        EASY_END_BLOCK;
        EASY_BLOCK("ROS waiting");
      ros::spinOnce();
      loop_rate_.sleep();
        EASY_END_BLOCK;
    }
  }

private:
    double norm( const float x[], std::size_t sz ) // *** const
    {
        // http://en.cppreference.com/w/cpp/algorithm/inner_product
        return std::sqrt( std::inner_product( x, x+sz, x, 0.0 ) ) ;
    }

    void quaternion_integration(tf::Quaternion &q, float gyroscopes[3], const float dt)
    {
      float omega_norm = norm(gyroscopes, 3) + 1.0e-12;

        tf::Quaternion q_delta;
        q_delta.setW(std::cos(0.5f*omega_norm*dt));
        float sinOmega = sin(0.5f*omega_norm*dt);
        q_delta.setX(sinOmega*gyroscopes[0]/omega_norm);
        q_delta.setY(sinOmega*gyroscopes[1]/omega_norm);
        q_delta.setZ(sinOmega*gyroscopes[2]/omega_norm);

        q *= q_delta;// q = q * q_delta;
    }

    void open() {
      // From https://github.com/mavlink/mavros/blob/master/libmavconn/src/serial.cpp
        try {
            port_.open(port_name_);

            // Set baudrate and 8N1 mode
            port_.set_option(boost::asio::serial_port_base::baud_rate(460800));
            port_.set_option(boost::asio::serial_port_base::character_size(8));
            port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
            port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));

#if BOOST_ASIO_VERSION >= 101200 || !defined(__linux__)
            // Flow control setting in older versions of Boost.ASIO is broken, use workaround (below) for now.
		port_.set_option(SPB::flow_control( (hwflow) ? SPB::flow_control::hardware : SPB::flow_control::none));
#elif BOOST_ASIO_VERSION < 101200 && defined(__linux__)
            // Workaround to set some options for the port manually. This is done in
            // Boost.ASIO, but until v1.12.0 (Boost 1.66) there was a bug which doesn't enable relevant
            // code. Fixed by commit: https://github.com/boostorg/asio/commit/619cea4356
            {
                int fd = port_.native_handle();

                termios tio;
                tcgetattr(fd, &tio);

                // Set hardware flow control settings
                if (hwflow_) {
                    tio.c_iflag &= ~(IXOFF | IXON);
                    tio.c_cflag |= CRTSCTS;
                } else {
                    tio.c_iflag &= ~(IXOFF | IXON);
                    tio.c_cflag &= ~CRTSCTS;
                }

                // Set serial port to "raw" mode to prevent EOF exit.
                cfmakeraw(&tio);

                // Commit settings
                tcsetattr(fd, TCSANOW, &tio);
            }
#endif

#if defined(__linux__)
            // Enable low latency mode on Linux
            {
                int fd = port_.native_handle();

                struct serial_struct ser_info;
                ioctl(fd, TIOCGSERIAL, &ser_info);

                ser_info.flags |= ASYNC_LOW_LATENCY;

                ioctl(fd, TIOCSSERIAL, &ser_info);
            }
#endif
        }
        catch (boost::system::system_error &err) {
            throw std::runtime_error("serial" + std::string(err.what()));
        }

    }
    int an_packet_encode_and_send(an_packet_t* packet) {
        an_packet_encode(packet);
        // need cast to prevent autosizing of packet
        return boost::asio::write(port_, boost::asio::buffer((const void*)an_packet_pointer(packet), an_packet_size(packet)));
    }
    // Process incoming data on serial link
    //
    // @brief Reads the serial buffer and dispatches the received payload to the
    // relevant message handling callback function.
    void processSerial(const boost::system::error_code &error,
                   std::size_t bytes_transferred) {
        EASY_FUNCTION(profiler::colors::Magenta);
        if (error == boost::system::errc::operation_canceled || error == boost::asio::error::eof) {
            EASY_BLOCK("EOF error");
            port_.async_read_some(boost::asio::buffer(an_decoder_pointer(&an_decoder_), an_decoder_size(&an_decoder_)),
                                  std::bind(&OrientusNode::processSerial, this, std::placeholders::_1, std::placeholders::_2));
            EASY_END_BLOCK;
            return;
        } else if (error) {
            throw std::runtime_error("processSerial: " + error.message());
        }

        processIncomingByte(bytes_transferred);

        // READ THE NEXT PACKET
        port_.async_read_some(boost::asio::buffer(an_decoder_pointer(&an_decoder_), an_decoder_size(&an_decoder_)),
                              std::bind(&OrientusNode::processSerial, this, std::placeholders::_1, std::placeholders::_2));

        return;
    }

    bool processIncomingByte(std::size_t bytes_read) {
        an_packet_t *an_packet = NULL;
        EASY_FUNCTION(profiler::colors::Red);

        an_decoder_increment(&an_decoder_, bytes_read);

        /* decode all the packets in the buffer */
        EASY_BLOCK("an_packet_decode");
        an_packet = an_packet_decode(&an_decoder_);
        EASY_END_BLOCK;
        while (an_packet != NULL) {
            EASY_BLOCK("an_packet parsing while loop");
            if (an_packet->id == packet_id_acknowledge) {
                acknowledge_packet_t acknowledge_packet;
                if (decode_acknowledge_packet(&acknowledge_packet, an_packet) != 0) {
                    ROS_WARN("Acknowledge packet decode failure");
                } else if (acknowledge_packet.acknowledge_result) {
                    ROS_WARN("Acknowledge Failure: %d", acknowledge_packet.acknowledge_result);
                }
            } else if (an_packet->id == packet_id_status) {
                if (decode_status_packet(&status_packet_, an_packet) != 0) {
                    ROS_WARN("Status packet decode failure");
                } else {
                    status_received_ = true;
                }
            } else if (an_packet->id == packet_id_running_time) {
                if (decode_running_time_packet(&running_time_packet_, an_packet) != 0) {
                    ROS_WARN("Running time packet decode failure");
                } else {
                    running_time_received_ = true;
                }
            } else if (an_packet->id == packet_id_device_information) {
                if (decode_device_information_packet(&device_information_packet_, an_packet) != 0) {
                    ROS_WARN("Device information decode failure");
                } else {
                    device_information_received_ = true;
                }
            } else if (an_packet->id == packet_id_quaternion_orientation_standard_deviation) {
                if (decode_quaternion_orientation_standard_deviation_packet(&quaternion_std_packet_, an_packet) !=
                    0) {
                    ROS_WARN("Quaternion orientation standard deviation packet decode failure");
                } else {
                    quaternion_orientation_std_received_ = true;
                }
            } else if (an_packet->id == packet_id_quaternion_orientation) {
                if (decode_quaternion_orientation_packet(&quaternion_packet_, an_packet) != 0) {
                    ROS_WARN("Quaternion packet decode failure");
                } else {
                    quaternion_orientation_received_ = true;
                }
            } else if (an_packet->id == packet_id_acceleration) {
                if (decode_acceleration_packet(&acceleration_packet_, an_packet) != 0) {
                    ROS_WARN("Acceleration packet decode failure");
                } else {
                    acceleration_received_ = true;
                }
            } else if (an_packet->id == packet_id_angular_velocity) {
                if (decode_angular_velocity_packet(&angular_velocity_packet_, an_packet) != 0) {
                    ROS_WARN("Angular velocity packet decode failure");
                } else {
                    angular_velocity_received_ = true;
                }
            } else if (an_packet->id == packet_id_raw_sensors) {
                if (decode_raw_sensors_packet(&raw_sensors_packet_, an_packet) != 0) {
                    ROS_WARN("Raw sensors packet decode failure");
                } else {
                    raw_sensors_received_ = true;
                }
            } else {
                //ROS_WARN("Unknown packet id: %d", an_packet->id);
            }

            an_packet_free(&an_packet);

            EASY_BLOCK("an_packet_decode");
            an_packet = an_packet_decode(&an_decoder_);
            EASY_END_BLOCK;
            EASY_END_BLOCK;
        }
    }

  bool receive_next_packet(){
    an_packet_t *an_packet;
    int bytes_received;

    try {
        if ((bytes_received = port_.read_some(
                boost::asio::buffer(an_decoder_pointer(&an_decoder_), an_decoder_size(&an_decoder_)))) > 0) {
            /* increment the decode buffer length by the number of bytes received */
            an_decoder_increment(&an_decoder_, bytes_received);

            /* decode all the packets in the buffer */
            if ((an_packet = an_packet_decode(&an_decoder_)) != NULL) {
                if (an_packet->id == packet_id_acknowledge) {
                    acknowledge_packet_t acknowledge_packet;
                    if (decode_acknowledge_packet(&acknowledge_packet, an_packet) != 0) {
                        ROS_WARN("Acknowledge packet decode failure");
                    } else if (acknowledge_packet.acknowledge_result) {
                        ROS_WARN("Acknowledge Failure: %d", acknowledge_packet.acknowledge_result);
                    }
                } else if (an_packet->id == packet_id_status) {
                    if (decode_status_packet(&status_packet_, an_packet) != 0) {
                        ROS_WARN("Status packet decode failure");
                    } else {
                        status_received_ = true;
                    }
                } else if (an_packet->id == packet_id_running_time) {
                    if (decode_running_time_packet(&running_time_packet_, an_packet) != 0) {
                        ROS_WARN("Running time packet decode failure");
                    } else {
                        running_time_received_ = true;
                    }
                } else if (an_packet->id == packet_id_device_information) {
                    if (decode_device_information_packet(&device_information_packet_, an_packet) != 0) {
                        ROS_WARN("Device information decode failure");
                    } else {
                        device_information_received_ = true;
                    }
                } else if (an_packet->id == packet_id_quaternion_orientation_standard_deviation) {
                    if (decode_quaternion_orientation_standard_deviation_packet(&quaternion_std_packet_, an_packet) !=
                        0) {
                        ROS_WARN("Quaternion orientation standard deviation packet decode failure");
                    } else {
                        quaternion_orientation_std_received_ = true;
                    }
                } else if (an_packet->id == packet_id_quaternion_orientation) {
                    if (decode_quaternion_orientation_packet(&quaternion_packet_, an_packet) != 0) {
                        ROS_WARN("Quaternion packet decode failure");
                    } else {
                        quaternion_orientation_received_ = true;
                    }
                } else if (an_packet->id == packet_id_acceleration) {
                    if (decode_acceleration_packet(&acceleration_packet_, an_packet) != 0) {
                        ROS_WARN("Acceleration packet decode failure");
                    } else {
                        acceleration_received_ = true;
                    }
                } else if (an_packet->id == packet_id_angular_velocity) {
                    if (decode_angular_velocity_packet(&angular_velocity_packet_, an_packet) != 0) {
                        ROS_WARN("Angular velocity packet decode failure");
                    } else {
                        angular_velocity_received_ = true;
                    }
                } else if (an_packet->id == packet_id_raw_sensors) {
                    if (decode_raw_sensors_packet(&raw_sensors_packet_, an_packet) != 0) {
                        ROS_WARN("Raw sensors packet decode failure");
                    } else {
                        raw_sensors_received_ = true;
                    }
                } else {
                    //ROS_WARN("Unknown packet id: %d", an_packet->id);
                }

                an_packet_free(&an_packet);
            }
        }
    }
    catch (std::exception& e)
    {
        //std::cerr << e.what() << std::endl;
    }
  }

  void deviceStatus(diagnostic_updater::DiagnosticStatusWrapper &status) {
    if(status_packet_.system_status.b.system_failure)
      status.summary(diagnostic_updater::DiagnosticStatusWrapper::ERROR, "System Failure");
    else if(status_packet_.system_status.b.serial_port_overflow_alarm)
      status.summary(diagnostic_updater::DiagnosticStatusWrapper::WARN, "Serial Port Overflow");
    else
      status.summary(diagnostic_updater::DiagnosticStatusWrapper::OK, "IMU is OK");

    status.add("Device", port_name_);
    status.add("TF frame", frame_id_);
    double running_time = (1.0e-6) * running_time_packet_.microseconds + running_time_packet_.seconds;
    status.add("Running time", running_time);
    status.add("Device ID", device_information_packet_.device_id);

    std::ostringstream software_version_stream;
    software_version_stream << std::fixed << std::setprecision(3);
    software_version_stream << device_information_packet_.software_version/1000.0;
    status.add("Software Version", software_version_stream.str());

    std::ostringstream hardware_revision_stream;
    hardware_revision_stream << std::fixed << std::setprecision(3);
    hardware_revision_stream << device_information_packet_.hardware_revision/1000.0;
    status.add("Hardware Revision", hardware_revision_stream.str());

    std::stringstream serial_number_stream;
    serial_number_stream << std::hex << std::setfill('0') << std::setw(8);
    serial_number_stream << device_information_packet_.serial_number[0];
    serial_number_stream << device_information_packet_.serial_number[1];
    serial_number_stream << device_information_packet_.serial_number[2];
    status.add("Serial Number", serial_number_stream.str());
  }
  void accelerometerStatus(diagnostic_updater::DiagnosticStatusWrapper &status) {
    if(status_packet_.system_status.b.accelerometer_sensor_failure)
      status.summary(diagnostic_updater::DiagnosticStatusWrapper::ERROR, "Accelerometer Sensor Failure");
    else if(status_packet_.system_status.b.accelerometer_over_range)
      status.summary(diagnostic_updater::DiagnosticStatusWrapper::WARN, "Accelerometer Sensor Over Range");
    else
      status.summary(diagnostic_updater::DiagnosticStatusWrapper::OK, "Accelerometer OK");
  }
  void gyroscopeStatus(diagnostic_updater::DiagnosticStatusWrapper &status) {
    if(status_packet_.system_status.b.gyroscope_sensor_failure)
      status.summary(diagnostic_updater::DiagnosticStatusWrapper::ERROR, "Gyroscope Sensor Failure");
    else if(status_packet_.system_status.b.gyroscope_over_range)
      status.summary(diagnostic_updater::DiagnosticStatusWrapper::WARN, "Gyroscope Sensor Over Range");
    else
      status.summary(diagnostic_updater::DiagnosticStatusWrapper::OK, "Gyroscope OK");
  }
  void magnetometerStatus(diagnostic_updater::DiagnosticStatusWrapper &status) {
    if(status_packet_.system_status.b.magnetometer_sensor_failure)
      status.summary(diagnostic_updater::DiagnosticStatusWrapper::ERROR, "Magnetometer Sensor Failure");
    else if(status_packet_.system_status.b.magnetometer_over_range)
      status.summary(diagnostic_updater::DiagnosticStatusWrapper::WARN, "Magnetometer Sensor Over Range");
    else
      status.summary(diagnostic_updater::DiagnosticStatusWrapper::OK, "Magnetometer OK");
  }
  void temperatureStatus(diagnostic_updater::DiagnosticStatusWrapper &status) {
    if(status_packet_.system_status.b.minimum_temperature_alarm)
      status.summary(diagnostic_updater::DiagnosticStatusWrapper::WARN, "Minimum Temperature Alarm");
    else if(status_packet_.system_status.b.maximum_temperature_alarm)
      status.summary(diagnostic_updater::DiagnosticStatusWrapper::WARN, "Maximum Temperature Alarm");
    else
      status.summary(diagnostic_updater::DiagnosticStatusWrapper::OK, "Temperature OK");
  }
  void voltageStatus(diagnostic_updater::DiagnosticStatusWrapper &status) {
    if(status_packet_.system_status.b.low_voltage_alarm)
      status.summary(diagnostic_updater::DiagnosticStatusWrapper::WARN, "Low Voltage");
    else if(status_packet_.system_status.b.high_voltage_alarm)
      status.summary(diagnostic_updater::DiagnosticStatusWrapper::WARN, "High Voltage");
    else
      status.summary(diagnostic_updater::DiagnosticStatusWrapper::OK, "Voltage OK");
  }
  void filterStatus(diagnostic_updater::DiagnosticStatusWrapper &status) {
    if(status_packet_.filter_status.b.orientation_filter_initialised)
      status.summary(diagnostic_updater::DiagnosticStatusWrapper::OK, "Filter OK");
    else
      status.summary(diagnostic_updater::DiagnosticStatusWrapper::ERROR, "Filter Not Initialised");

    status.add("Heading Initialised", (bool)status_packet_.filter_status.b.heading_initialised);
    status.add("Magnetometers Enabled", (bool)status_packet_.filter_status.b.dual_antenna_heading_active);
    status.add("Velocity Heading Enabled", (bool)status_packet_.filter_status.b.velocity_heading_enabled);
    status.add("External Position Active", (bool)status_packet_.filter_status.b.external_position_active);
    status.add("External Velocity Active", (bool)status_packet_.filter_status.b.external_velocity_active);
    status.add("External Heading Active", (bool)status_packet_.filter_status.b.external_heading_active);
  }

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
  void publish_temperature_msg() {
    sensor_msgs::Temperature temp_msg;

    temp_msg.header.stamp = ros::Time::now();
    temp_msg.header.frame_id = frame_id_;

    temp_msg.temperature = raw_sensors_packet_.imu_temperature;

    temperature_pub_.publish(temp_msg);
  }

};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "orientus_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  profiler::startListen();

  try {
    OrientusNode node(nh, pnh);
    node.spin();
  } catch(std::exception& e){
    ROS_FATAL_STREAM("Exception thrown: " << e.what());
  }

  return 0;

}

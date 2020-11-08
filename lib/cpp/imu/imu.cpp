#include "imu.h"

#include <iostream>
#include <iomanip>
#include <cmath>
#include <limits>
#include <stdio.h>
#include <easy/profiler.h>

#include "geodetic_conv.hpp"
#include "LatLong-UTM.h"

IMU::IMU(std::string port_name, uint32_t baud_rate)
    : port_name_{port_name}
    , baud_rate_{baud_rate}
    , hwflow_{false}
{
    g_geodetic_converter = std::make_unique<geodetic_converter::GeodeticConverter>();
    g_geodetic_converter->initialiseReference(1.28967, 103.85007, 23); // Singapore
}

IMU::~IMU()
{
    if (connected_) {
        Disconnect();
    }

    if (anpp_log_file) {
        fclose(anpp_log_file);
    }
}

void IMU::Connect()
{
    if (connected_) return;

    // From https://github.com/mavlink/mavros/blob/master/libmavconn/src/serial.cpp
    try {
        port_.open(port_name_);

        // Set baudrate and 8N1 mode
        port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
        port_.set_option(boost::asio::serial_port_base::character_size(8));
        port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));

#if BOOST_ASIO_VERSION >= 101200 || !defined(__linux__)
        // Flow control setting in older versions of Boost.ASIO is broken, use workaround (below) for now.
        serial_dev.set_option(SPB::flow_control( (hwflow) ? SPB::flow_control::hardware : SPB::flow_control::none));
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
        //throw std::runtime_error("serial" + std::string(err.what()));
        std::cout << "[IMU] Error: " << std::string(err.what()) << std::endl;
        return;
    }
    
    connected_ = true;

    Reset();
    Configure();
    an_decoder_initialise(&an_decoder_);

    port_.async_read_some(boost::asio::buffer(an_decoder_pointer(&an_decoder_), an_decoder_size(&an_decoder_)),
                          std::bind(&IMU::SerialCallback, this, std::placeholders::_1, std::placeholders::_2));
    /*boost::asio::async_read(
            //port_, boost::asio::buffer(read_buffer),
            port_, boost::asio::buffer(an_decoder_pointer(&an_decoder_), std::min(an_decoder_size(&an_decoder_), size_t(10))),
          std::bind(&OrientusNode::SerialCallback, this, std::placeholders::_1, std::placeholders::_2));
    */
    // Start the I/O service in its own thread.
    ioservice_thread = std::thread([&] { ioservice.run(); });

    // Wait for start
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Reset/Synchronize timestamps
    SynchronizeTime();
}

void IMU::Disconnect()
{
    if (!connected_) return;

    ioservice.stop();

    if (ioservice_thread.joinable()) {
        ioservice_thread.join();
    }

    try {
        port_.close();
    }
    catch (boost::system::system_error &err) {
        //throw std::runtime_error("serial" + std::string(err.what()));
        std::cout << "[IMU] Error: " << std::string(err.what()) << std::endl;
        return;
    }

    connected_ = false;

    if (position_publisher_thread.joinable()) {
        position_publisher_thread.join();
    }
}

void IMU::Configure(OutputType output, uint16_t RawDataRate, uint16_t EstimateDataRate, bool enableExternalHeadingInput, bool enableExternalPositionInput, bool includeStandardDeviations)
{
    an_packet_t *an_packet;
    if (!connected_) return;

    // Request device information
    an_packet = encode_request_packet(packet_id_device_information);
    SendPackage(an_packet);
    an_packet_free(&an_packet);

    // Setup packet rate timer
    float tick_frequency = 1000000.f / (float) tick_period_us_;
    auto freq2ticks = [tick_frequency](uint16_t freq) { return std::floor(tick_frequency / freq); };
    packet_timer_period_packet_t packet_timer_period_packet;
    packet_timer_period_packet.permanent = 0;
    packet_timer_period_packet.packet_timer_period = tick_period_us_;
    an_packet = encode_packet_timer_period_packet(&packet_timer_period_packet);
    SendPackage(an_packet);
    an_packet_free(&an_packet);

    // Configure sensor ranges
    sensor_ranges_packet_t sensor_ranges_packet;
    sensor_ranges_packet.permanent = 0;
    sensor_ranges_packet.accelerometers_range = accelerometer_range_4g;
    sensor_ranges_packet.gyroscopes_range = gyroscope_range_2000dps;
    sensor_ranges_packet.magnetometers_range = magnetometer_range_2g;
    an_packet = encode_sensor_ranges_packet(&sensor_ranges_packet);
    SendPackage(an_packet);

    // Configure filter settings
    filter_options_packet_t filter_options_packet;
    filter_options_packet.permanent = 0;
    filter_options_packet.vehicle_type = 13; // race car   //vehicle_type_unlimited; // vehicle_type_car
    filter_options_packet.internal_gnss_enabled = false;
    filter_options_packet.magnetometers_enabled = !enableExternalHeadingInput;
    filter_options_packet.atmospheric_altitude_enabled = false;
    filter_options_packet.velocity_heading_enabled = false;
    filter_options_packet.reversing_detection_enabled = false;
    filter_options_packet.motion_analysis_enabled = false;
    filter_options_packet.automatic_magnetic_calibration_enabled = !enableExternalHeadingInput;
    an_packet = encode_filter_options_packet(&filter_options_packet);
    SendPackage(an_packet);

    externalHeadingInputEnabled_ = enableExternalHeadingInput;
    enableExternalPositionInput_ = enableExternalPositionInput;

    // Configure packet rates
    uint8_t p = 0;
    packet_periods_packet_t packet_periods_packet;
    packet_periods_packet.permanent = 0;
    packet_periods_packet.clear_existing_packets = 1;

    packet_periods_packet.packet_periods[p].packet_id = packet_id_running_time;
    packet_periods_packet.packet_periods[p++].period = freq2ticks(std::max(RawDataRate, EstimateDataRate));

    if (output == Estimates || output == All) {
        packet_periods_packet.packet_periods[p].packet_id = packet_id_system_state;
        packet_periods_packet.packet_periods[p++].period = freq2ticks(EstimateDataRate);
    }
    if (output == RawSensors || output == RawAndIndividualEstimates || output == All) {
        packet_periods_packet.packet_periods[p].packet_id = packet_id_raw_sensors;
        packet_periods_packet.packet_periods[p++].period = freq2ticks(RawDataRate);
    }
    if (output == IndividualEstimates || output == RawAndIndividualEstimates || output == All) {
        //packet_periods_packet.packet_periods[p].packet_id = packet_id_body_acceleration;
        //packet_periods_packet.packet_periods[p++].period = freq2ticks(EstimateDataRate);
        //packet_periods_packet.packet_periods[p].packet_id = packet_id_acceleration;
        //packet_periods_packet.packet_periods[p++].period = freq2ticks(EstimateDataRate);

        if (enableExternalPositionInput_) {
            packet_periods_packet.packet_periods[p].packet_id = packet_id_geodetic_position;
            packet_periods_packet.packet_periods[p++].period = freq2ticks(EstimateDataRate);

            //packet_periods_packet.packet_periods[p].packet_id = packet_id_utm_position;
            //packet_periods_packet.packet_periods[p++].period = freq2ticks(EstimateDataRate);

            packet_periods_packet.packet_periods[p].packet_id = packet_id_body_velocity;
            packet_periods_packet.packet_periods[p++].period = freq2ticks(EstimateDataRate);
        }

        packet_periods_packet.packet_periods[p].packet_id = packet_id_quaternion_orientation;
        packet_periods_packet.packet_periods[p++].period = freq2ticks(EstimateDataRate);

        packet_periods_packet.packet_periods[p].packet_id = packet_id_angular_velocity;
        packet_periods_packet.packet_periods[p++].period = freq2ticks(EstimateDataRate);
        //packet_periods_packet.packet_periods[p].packet_id = packet_id_angular_acceleration;
        //packet_periods_packet.packet_periods[p++].period = freq2ticks(OutputDataRate);

        if (includeStandardDeviations) {
            packet_periods_packet.packet_periods[p].packet_id = packet_id_quaternion_orientation_standard_deviation;
            packet_periods_packet.packet_periods[p++].period = freq2ticks(EstimateDataRate);
            //packet_periods_packet.packet_periods[p].packet_id = packet_id_euler_orientation_standard_deviation;
            //packet_periods_packet.packet_periods[p++].period = freq2ticks(EstimateDataRate);
            packet_periods_packet.packet_periods[p].packet_id = packet_id_velocity_standard_deviation;
            packet_periods_packet.packet_periods[p++].period = freq2ticks(EstimateDataRate);
        }
    }

    packet_periods_packet.packet_periods[p].packet_id = packet_id_status;
    packet_periods_packet.packet_periods[p++].period = freq2ticks(1); // 1 Hz

    packet_periods_packet.packet_periods[p].packet_id = 0;
    an_packet = encode_packet_periods_packet(&packet_periods_packet);
    SendPackage(an_packet);
    an_packet_free(&an_packet);
}

void IMU::SynchronizeTime()
{
    an_packet_t *an_packet;
    if (!connected_) return;

#if 0
    // Setup packet rate timer
    float tick_frequency = 1000000.f / (float) tick_period_us_;
    packet_timer_period_packet_t packet_timer_period_packet;
    packet_timer_period_packet.permanent = 0;
    packet_timer_period_packet.packet_timer_period = tick_period_us_;
    an_packet = encode_packet_timer_period_packet(&packet_timer_period_packet);
    SendPackage(an_packet);
    an_packet_free(&an_packet);

    // Configure packet rates
    packet_periods_packet_t packet_periods_packet;
    packet_periods_packet.permanent = 0;
    packet_periods_packet.clear_existing_packets = 1;
    packet_periods_packet.packet_periods[0].packet_id = packet_id_running_time;
    packet_periods_packet.packet_periods[0].period = 1;
    packet_periods_packet.packet_periods[1].packet_id = 0;
    an_packet = encode_packet_periods_packet(&packet_periods_packet);
    SendPackage(an_packet);
    an_packet_free(&an_packet);
#endif

    timeSyncMutex.lock();
    timeSyncEnabled = true;
    initTimestamp = 0;
    initUnixTimeSeconds = 0;
    timeSyncMutex.unlock();
    std::unique_lock<std::mutex> lk(timeSyncMutex);
    timeSyncCV.wait(lk, [this]{return (timeSyncEnabled == false);}); // wait for time synchronization to finish
}

void IMU::LoadLog(std::string log_path)
{
    FILE * log_file = fopen(log_path.c_str(), "rb");
    if (!log_file) return;

    an_decoder_initialise(&an_decoder_);

    timeSyncEnabled = true;
    initTimestamp = 0;
    initUnixTimeSeconds = 0;

    int64_t local_time_init = 0;

    // This loop should be made into a thread instead
    size_t bytes_read;
    while ((bytes_read = fread(an_decoder_pointer(&an_decoder_), sizeof(uint8_t), std::min(an_decoder_size(&an_decoder_), (unsigned long)10), log_file)) > 0)
    {
        ProcessIncomingBytes(bytes_read);
        if (!timeSyncEnabled) {
            if (local_time_init == 0) {
                local_time_init = utime();
            }

            int64_t expected_local_time = local_time_init + 1000000*latestTimestamp;

            while (utime() < expected_local_time) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }

    fclose(log_file);

    std::cout << "[IMU] Finished playing back the log" << std::endl;
}

void IMU::RecordANPPLog(std::string log_folder)
{
    time_t rawtime = time(NULL);
    struct tm * timeinfo = localtime(&rawtime);

    char filename[32];
    sprintf(filename, "SpatialLog_%02d-%02d-%02d_%02d-%02d-%02d.anpp", timeinfo->tm_year-100, timeinfo->tm_mon+1, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);

    if (log_folder.length() > 0 && log_folder.at(log_folder.length()-1) != '/') {
        log_folder.append("/");
    }

    std::string log_path = log_folder + std::string(filename);

    anpp_logged_bytes = 0;
    anpp_log_file = fopen(log_path.c_str(), "wb");
}

void IMU::StopANPPLog()
{
    if (!anpp_log_file) return;
    fclose(anpp_log_file);
    anpp_log_file = 0;
}

int IMU::SendPackage(an_packet_t* packet)
{
    if (!connected_) return -1;
    an_packet_encode(packet);
    // need cast to prevent autosizing of packet
    return boost::asio::write(port_, boost::asio::buffer((const void*)an_packet_pointer(packet), an_packet_size(packet)));
}

// Process incoming data on serial link
void IMU::SerialCallback(const boost::system::error_code &error, std::size_t bytes_transferred)
{
    EASY_FUNCTION(profiler::colors::Magenta);
    if (error == boost::system::errc::operation_canceled || error == boost::asio::error::eof) {
        EASY_BLOCK("EOF error");
        port_.async_read_some(boost::asio::buffer(an_decoder_pointer(&an_decoder_), an_decoder_size(&an_decoder_)),
                              std::bind(&IMU::SerialCallback, this, std::placeholders::_1, std::placeholders::_2));
        EASY_END_BLOCK;
        return;
    } else if (error) {
        //throw std::runtime_error("serial asio: " + error.message());
        std::cout << "[IMU] Serial Callback error: " << error.message() << std::endl;
    }

    ProcessIncomingBytes(bytes_transferred);

    // READ THE NEXT PACKET
    port_.async_read_some(boost::asio::buffer(an_decoder_pointer(&an_decoder_), an_decoder_size(&an_decoder_)),
                          std::bind(&IMU::SerialCallback, this, std::placeholders::_1, std::placeholders::_2));

    return;
}

/// From https://stackoverflow.com/questions/22581315/how-to-discard-data-as-it-is-sent-with-boostasio/22598329#22598329
/// @brief Flush a serial port's buffers.
///
/// @param serial_port Port to flush.
/// @param what Determines the buffers to flush.
/// @param error Set to indicate what error occurred, if any.
boost::system::error_code IMU::flush_serial_port(
        boost::asio::serial_port& serial_port,
        flush_type what)
{
    if (0 == ::tcflush(serial_port.lowest_layer().native_handle(), what))
    {
        return boost::system::error_code();
    }
    else
    {
        return boost::system::error_code(errno,
                                          boost::asio::error::get_system_category());
    }
}

void IMU::ProcessIncomingBytes(size_t bytes_read)
{
    an_packet_t *an_packet = NULL;
    EASY_FUNCTION(profiler::colors::Red);

    if (anpp_log_file) {
        // Recording/logging enabled
        fwrite(an_decoder_pointer(&an_decoder_), sizeof(uint8_t), bytes_read, anpp_log_file);
        anpp_logged_bytes += bytes_read;

        if (anpp_logged_bytes >= 100)
        {
            fflush(anpp_log_file);
            anpp_logged_bytes = 0;
        }
    }

    an_decoder_increment(&an_decoder_, bytes_read);

    /* decode all the packets in the buffer */
    EASY_BLOCK("an_packet_decode");
    an_packet = an_packet_decode(&an_decoder_);
    EASY_END_BLOCK;
    while (an_packet != NULL) {
        EASY_BLOCK("an_packet parsing while loop");
        ProcessPackage(an_packet);
        an_packet_free(&an_packet);

        EASY_BLOCK("an_packet_decode");
        an_packet = an_packet_decode(&an_decoder_);
        EASY_END_BLOCK;
        EASY_END_BLOCK;
    }
}

void IMU::ReceiveNextPackage()
{
    // Function only used in poll mode  (currently not enabled)
    an_packet_t *an_packet;
    int bytes_received;

    try {
        if ((bytes_received = port_.read_some(
                boost::asio::buffer(an_decoder_pointer(&an_decoder_), an_decoder_size(&an_decoder_)))) > 0) {
            /* increment the decode buffer length by the number of bytes received */
            an_decoder_increment(&an_decoder_, bytes_received);

            /* decode all the packets in the buffer */
            if ((an_packet = an_packet_decode(&an_decoder_)) != NULL) {
                ProcessPackage(an_packet);
                an_packet_free(&an_packet);
            }
        }
    }
    catch (std::exception& e)
    {
        //std::cerr << e.what() << std::endl;
    }
}

void IMU::ProcessPackage(an_packet_t * an_packet)
{
    if (an_packet->id == packet_id_acknowledge) {
        acknowledge_packet_t acknowledge_packet;
        if (decode_acknowledge_packet(&acknowledge_packet, an_packet) != 0) {
            printf("[IMU] Acknowledge packet decode failure\n");
        } else if (acknowledge_packet.acknowledge_result) {
            printf("[IMU] Acknowledge Failure for package %d with error code: %d\n", acknowledge_packet.packet_id, acknowledge_packet.acknowledge_result);
        }

    } else if (an_packet->id == packet_id_system_state) {
        if (decode_system_state_packet(&system_state_packet_, an_packet) != 0) {
            printf("[IMU] System state packet decode failure\n");
        } else {
            // Do something with the decoded package here
            if (latestEstimate.timestamp != 0 && latestEstimate.timestamp == latestTimestamp) {
                printf("[IMU] Error: Received new state estimate package without updated timestamp\n");
            }

            //// ToDo Thomas: The timestamp of this Estimate package is not synchronized like the others
            if (initUnixTimeSeconds == 0) {
                initUnixTimeSeconds = system_state_packet_.unix_time_seconds;
            }
            latestEstimate.timestamp = float(system_state_packet_.unix_time_seconds - initUnixTimeSeconds) + (float(system_state_packet_.microseconds) / 1000000);
            latestEstimate.RPY[0] = system_state_packet_.orientation[0];
            latestEstimate.RPY[1] = system_state_packet_.orientation[1];
            latestEstimate.RPY[2] = system_state_packet_.orientation[2];

            latestEstimate.quaternion = GetRotation(latestEstimate.RPY); // Compute quaternion from RPY

            latestEstimate.velocity[0] = system_state_packet_.velocity[0];
            latestEstimate.velocity[1] = system_state_packet_.velocity[1];
            latestEstimate.velocity[2] = system_state_packet_.velocity[2];

            latestEstimate.acceleration[0] = system_state_packet_.body_acceleration[0];
            latestEstimate.acceleration[1] = system_state_packet_.body_acceleration[1];
            latestEstimate.acceleration[2] = system_state_packet_.body_acceleration[2];

            latestEstimate.angular_velocity[0] = system_state_packet_.angular_velocity[0];
            latestEstimate.angular_velocity[1] = system_state_packet_.angular_velocity[1];
            latestEstimate.angular_velocity[2] = system_state_packet_.angular_velocity[2];

            latestEstimate.angular_acceleration[0] = 0;
            latestEstimate.angular_acceleration[1] = 0;
            latestEstimate.angular_acceleration[2] = 0;

            double E, N, U;
            g_geodetic_converter->geodetic2Enu(rad2deg(system_state_packet_.latitude), rad2deg(system_state_packet_.longitude), system_state_packet_.height,
                    &E, &N, &U);
            latestEstimate.position_NED[0] = N; //system_state_packet_.latitude;
            latestEstimate.position_NED[1] = E; //system_state_packet_.longitude;
            latestEstimate.position_NED[2] = -U; //system_state_packet_.height;

            if (callbacks.estimate.size() > 0) {
                for (auto &cb : callbacks.estimate)
                    cb(latestEstimate);
            }

            latestPositionFusion.timestamp = latestEstimate.timestamp;
            latestPositionFusion.position_estimate[0] = rad2deg(system_state_packet_.latitude); //system_state_packet_.latitude;
            latestPositionFusion.position_estimate[1] = rad2deg(system_state_packet_.longitude); //system_state_packet_.longitude;
            latestPositionFusion.position_estimate[2] = system_state_packet_.height;

            if (callbacks.position_fusion.size() > 0) {
                for (auto &cb : callbacks.position_fusion)
                    cb(latestPositionFusion);
            }
        }

    } else if (an_packet->id == packet_id_raw_sensors) {
        if (decode_raw_sensors_packet(&raw_sensors_packet_, an_packet) != 0) {
            printf("[IMU] Raw sensors packet decode failure\n");
        } else {
            // Do something with the decoded package here
            if ((latestRawSensors.timestamp != 0 && latestRawSensors.timestamp == latestTimestamp)) {
                printf("[IMU] Error: Received new sensor package without updated timestamp\n");
            }

            //std::cout << "[IMU] packet_id_raw_sensors: " << utime() << " [" << latestTimestamp << "]" << std::endl;

            latestRawSensors.timestamp = latestTimestamp;
            latestRawSensors.accelerometer[0] = raw_sensors_packet_.accelerometers[0];
            latestRawSensors.accelerometer[1] = raw_sensors_packet_.accelerometers[1];
            latestRawSensors.accelerometer[2] = raw_sensors_packet_.accelerometers[2];

            latestRawSensors.gyroscope[0] = raw_sensors_packet_.gyroscopes[0];
            latestRawSensors.gyroscope[1] = raw_sensors_packet_.gyroscopes[1];
            latestRawSensors.gyroscope[2] = raw_sensors_packet_.gyroscopes[2];

            latestRawSensors.magnetometer[0] = raw_sensors_packet_.magnetometers[0];
            latestRawSensors.magnetometer[1] = raw_sensors_packet_.magnetometers[1];
            latestRawSensors.magnetometer[2] = raw_sensors_packet_.magnetometers[2];

            if (callbacks.raw_sensors.size() > 0) {
                for (auto &cb : callbacks.raw_sensors)
                    cb(latestRawSensors);
            }
            if (callbacks.accel.size() > 0) {
                for (auto &cb : callbacks.accel)
                    cb({latestRawSensors.timestamp, latestRawSensors.accelerometer});
            }
            if (callbacks.gyro.size() > 0) {
                for (auto &cb : callbacks.gyro)
                    cb({latestRawSensors.timestamp, latestRawSensors.gyroscope});
            }
            if (callbacks.mag.size() > 0) {
                for (auto &cb : callbacks.mag)
                    cb({latestRawSensors.timestamp, latestRawSensors.magnetometer});
            }
        }

    } else if (an_packet->id == packet_id_quaternion_orientation) {
        if (decode_quaternion_orientation_packet(&quaternion_packet_, an_packet) != 0) {
            printf("[IMU] Quaternion packet decode failure\n");
        } else {
            // Do something with the decoded package here
            if (latestOrientation.timestamp != 0 && latestOrientation.timestamp == latestTimestamp) {
                printf("[IMU] Error: Received new quaternion estimate package without updated timestamp\n");
            }

            latestOrientation.timestamp = latestTimestamp;
            latestOrientation.quaternion[0] = quaternion_packet_.orientation[1]; // x
            latestOrientation.quaternion[1] = quaternion_packet_.orientation[2]; // y
            latestOrientation.quaternion[2] = quaternion_packet_.orientation[3]; // z
            latestOrientation.quaternion[3] = quaternion_packet_.orientation[0]; // w

            latestOrientation.RPY = GetPoseEulerRPY(latestOrientation.quaternion, false);

            if (callbacks.orientation.size() > 0) {
                for (auto &cb : callbacks.orientation)
                    cb(latestOrientation);
            }
        }

    } else if (an_packet->id == packet_id_body_velocity) {
        if (decode_body_velocity_packet(&body_velocity_packet_, an_packet) != 0) {
            printf("[IMU] Velocity packet decode failure\n");
        } else {
            // Do something with the decoded package here
            if ((latestVelocity.timestamp != 0 && latestVelocity.timestamp == latestTimestamp)) {
                printf("[IMU] Error: Received new velocity estimate package without updated timestamp\n");
            }

            latestVelocity.timestamp = latestTimestamp;
            latestVelocity.estimate[0] = body_velocity_packet_.velocity[0];
            latestVelocity.estimate[1] = body_velocity_packet_.velocity[1];
            latestVelocity.estimate[2] = body_velocity_packet_.velocity[2];

            if (callbacks.velocity.size() > 0) {
                for (auto &cb : callbacks.velocity)
                    cb(latestVelocity);
            }
        }

    } else if (an_packet->id == packet_id_acceleration) {
        if (decode_acceleration_packet(&acceleration_packet_, an_packet) != 0) {
            printf("[IMU] Acceleration packet decode failure\n");
        } else {
            // Do something with the decoded package here
            if ((latestAcceleration.timestamp != 0 && latestAcceleration.timestamp == latestTimestamp)) {
                printf("[IMU] Error: Received new acceleration estimate package without updated timestamp\n");
            }

            latestAcceleration.timestamp = latestTimestamp;
            latestAcceleration.estimate[0] = acceleration_packet_.acceleration[0];
            latestAcceleration.estimate[1] = acceleration_packet_.acceleration[1];
            latestAcceleration.estimate[2] = acceleration_packet_.acceleration[2];

            if (callbacks.acceleration.size() > 0) {
                for (auto &cb : callbacks.acceleration)
                    cb(latestAcceleration);
            }
        }

    } else if (an_packet->id == packet_id_body_acceleration) {
        if (decode_body_acceleration_packet(&body_acceleration_packet_, an_packet) != 0) {
            printf("[IMU] Acceleration packet decode failure\n");
        } else {
            // Do something with the decoded package here
            if ((latestAcceleration.timestamp != 0 && latestAcceleration.timestamp == latestTimestamp)) {
                printf("[IMU] Error: Received new acceleration estimate package without updated timestamp\n");
            }

            latestAcceleration.timestamp = latestTimestamp;
            latestAcceleration.estimate[0] = body_acceleration_packet_.acceleration[0];
            latestAcceleration.estimate[1] = body_acceleration_packet_.acceleration[1];
            latestAcceleration.estimate[2] = body_acceleration_packet_.acceleration[2];

            if (callbacks.acceleration.size() > 0) {
                for (auto &cb : callbacks.acceleration)
                    cb(latestAcceleration);
            }
        }

    } else if (an_packet->id == packet_id_angular_velocity) {
        if (decode_angular_velocity_packet(&angular_velocity_packet_, an_packet) != 0) {
            printf("[IMU] Angular velocity packet decode failure\n");
        } else {
            // Do something with the decoded package here
            if ((latestAngularVelocity.timestamp != 0 && latestAngularVelocity.timestamp == latestTimestamp)) {
                printf("[IMU] Error: Received new angular velocity estimate package without updated timestamp\n");
            }

            //std::cout << "[IMU] packet_id_angular_velocity: " << utime() << " [" << latestTimestamp << "]" << std::endl;

            latestAngularVelocity.timestamp = latestTimestamp;
            latestAngularVelocity.estimate[0] = angular_velocity_packet_.angular_velocity[0];
            latestAngularVelocity.estimate[1] = angular_velocity_packet_.angular_velocity[1];
            latestAngularVelocity.estimate[2] = angular_velocity_packet_.angular_velocity[2];

            if (callbacks.angular_velocity.size() > 0) {
                for (auto &cb : callbacks.angular_velocity)
                    cb(latestAngularVelocity);
            }
        }

    } else if (an_packet->id == packet_id_angular_acceleration) {
        if (decode_angular_acceleration_packet(&angular_acceleration_packet_, an_packet) != 0) {
            printf("[IMU] Angular acceleration packet decode failure\n");
        } else {
            // Do something with the decoded package here
            if ((latestAngularAcceleration.timestamp != 0 && latestAngularAcceleration.timestamp == latestTimestamp)) {
                printf("[IMU] Error: Received new velocity estimate package without updated timestamp\n");
            }

            latestAngularAcceleration.timestamp = latestTimestamp;
            latestAngularAcceleration.estimate[0] = angular_acceleration_packet_.angular_acceleration[0];
            latestAngularAcceleration.estimate[1] = angular_acceleration_packet_.angular_acceleration[1];
            latestAngularAcceleration.estimate[2] = angular_acceleration_packet_.angular_acceleration[2];

            if (callbacks.angular_acceleration.size() > 0) {
                for (auto &cb : callbacks.angular_acceleration)
                    cb(latestAngularAcceleration);
            }
        }

    } else if (an_packet->id == packet_id_geodetic_position) {
        if (decode_geodetic_position_packet(&geodetic_position_packet_, an_packet) != 0) {
            printf("[IMU] Geodetic Position packet decode failure\n");
        } else {
            // Do something with the decoded package here
            if ((latestGeodeticPosition.timestamp != 0 && latestGeodeticPosition.timestamp == latestTimestamp)) {
                printf("[IMU] Error: Received new geodetic position package without updated timestamp\n");
            }

            latestGeodeticPosition.timestamp = latestTimestamp;
            latestGeodeticPosition.estimate[0] = geodetic_position_packet_.position[0];
            latestGeodeticPosition.estimate[1] = geodetic_position_packet_.position[1];
            latestGeodeticPosition.estimate[2] = geodetic_position_packet_.position[2];

            double E, N, U;
            g_geodetic_converter->geodetic2Enu(rad2deg(geodetic_position_packet_.position[0]), rad2deg(geodetic_position_packet_.position[1]), geodetic_position_packet_.position[2],
                                               &E, &N, &U);

            latestPositionNED.timestamp = latestTimestamp;
            latestPositionNED.estimate[0] = N; //system_state_packet_.latitude;
            latestPositionNED.estimate[1] = E; //system_state_packet_.longitude;
            latestPositionNED.estimate[2] = -U; //system_state_packet_.height;

            if (callbacks.positionNED.size() > 0) {
                for (auto &cb : callbacks.positionNED)
                    cb(latestPositionNED);
            }
        }

    } else if (an_packet->id == packet_id_utm_position) {
        if (decode_utm_position_packet(&utm_position_packet_, an_packet) != 0) {
            printf("[IMU] UTM Position packet decode failure\n");
        } else {
            // Do something with the decoded package here
            if ((latestPositionUTM.timestamp != 0 && latestPositionUTM.timestamp == latestTimestamp)) {
                printf("[IMU] Error: Received new UTM position package without updated timestamp\n");
            }

            latestPositionUTM.timestamp = latestTimestamp;
            latestPositionUTM.estimate[0] = utm_position_packet_.position[0];
            latestPositionUTM.estimate[1] = utm_position_packet_.position[1];
            latestPositionUTM.estimate[2] = utm_position_packet_.position[2];

            if (callbacks.positionUTM.size() > 0) {
                for (auto &cb : callbacks.positionUTM)
                    cb(latestPositionUTM);
            }
        }

    } else if (an_packet->id == packet_id_quaternion_orientation_standard_deviation) {
        if (decode_quaternion_orientation_standard_deviation_packet(&quaternion_std_packet_, an_packet) !=
            0) {
            printf("[IMU] Quaternion orientation standard deviation packet decode failure\n");
        } else {
            // Do something with the decoded package here
            if ((latestQuaternionStdDev.timestamp != 0 && latestQuaternionStdDev.timestamp == latestTimestamp)) {
                printf("[IMU] Error: Received new Quaternion orientation standard deviation package without updated timestamp\n");
            }

            latestQuaternionStdDev.timestamp = latestTimestamp;
            latestQuaternionStdDev.stddev[0] = quaternion_std_packet_.standard_deviation[1]; // x
            latestQuaternionStdDev.stddev[1] = quaternion_std_packet_.standard_deviation[2]; // y
            latestQuaternionStdDev.stddev[2] = quaternion_std_packet_.standard_deviation[3]; // z
            latestQuaternionStdDev.stddev[3] = quaternion_std_packet_.standard_deviation[0]; // w

            if (callbacks.quaternion_stddev.size() > 0) {
                for (auto &cb : callbacks.quaternion_stddev)
                    cb(latestQuaternionStdDev);
            }
        }

    } else if (an_packet->id == packet_id_euler_orientation_standard_deviation) {
        if (decode_euler_orientation_standard_deviation_packet(&euler_orientation_std_packet_t, an_packet) !=
            0) {
            printf("[IMU] Euler angles standard deviation packet decode failure\n");
        } else {
            // Do something with the decoded package here
        }

    } else if (an_packet->id == packet_id_velocity_standard_deviation) {
        if (decode_velocity_standard_deviation_packet(&velocity_std_packet_, an_packet) !=
            0) {
            printf("[IMU] Velocity standard deviation packet decode failure\n");
        } else {
            // Do something with the decoded package here
        }

    } else if (an_packet->id == packet_id_status) {
        if (decode_status_packet(&status_packet_, an_packet) != 0) {
            printf("[IMU] Status packet decode failure\n");
        } else {
            // Do something with the decoded package here
            if(status_packet_.system_status.b.system_failure) {
                printf("[IMU] System Failure\n");
            }
            if(status_packet_.system_status.b.serial_port_overflow_alarm) {
                printf("[IMU] Serial Port Overflow\n");
            }
            if(status_packet_.system_status.b.accelerometer_sensor_failure) {
                printf("[IMU] Accelerometer Sensor Failure\n");
            }
            if(status_packet_.system_status.b.gyroscope_sensor_failure) {
                printf("[IMU] Gyroscope Sensor Failure\n");
            }
            if (status_packet_.system_status.b.magnetometer_sensor_failure) {
                printf("[IMU] Magnetometer Sensor Failure\n");
            }
        }

    } else if (an_packet->id == packet_id_running_time) {
        if (decode_running_time_packet(&running_time_packet_, an_packet) != 0) {
            printf("[IMU] Running time packet decode failure\n");
        } else {
            // Do something with the decoded package here
            latestTimestamp = (float)running_time_packet_.seconds + (float(running_time_packet_.microseconds) / 1000000) - initTimestamp;

            //std::cout << "[IMU] packet_id_running_time: " << utime() << " [" << latestTimestamp << "]" << std::endl;

            timeSyncMutex.lock();
            if (timeSyncEnabled) {
                timeSyncEnabled = false;
                initTimestamp = latestTimestamp;
                latestTimestamp = 0;
                timeSyncMutex.unlock();
                timeSyncCV.notify_one();
            } else {
                timeSyncMutex.unlock();
            }
        }

    } else if (an_packet->id == packet_id_device_information) {
        if (decode_device_information_packet(&device_information_packet_, an_packet) != 0) {
            printf("[IMU] Device information decode failure\n");
        } else {
            // Do something with the decoded package here
        }

    } else if (an_packet->id == packet_id_unix_time) {
        // Do something with the decoded package here

    } else {
        printf("[IMU] Unknown packet id: %d\n", an_packet->id);
    }
}

void IMU::RegisterCallback_RawSensors(const std::function<void(CombinedSensorsMeasurement)> &callback)
{
    callbacks.raw_sensors.emplace_back(callback);
}
void IMU::RegisterCallback_Accelerometer(const std::function<void(SensorMeasurement)> &callback)
{
    callbacks.accel.emplace_back(callback);
}
void IMU::RegisterCallback_Gyroscope(const std::function<void(SensorMeasurement)> &callback)
{
    callbacks.gyro.emplace_back(callback);
}
void IMU::RegisterCallback_Magnetometer(const std::function<void(SensorMeasurement)> &callback)
{
    callbacks.mag.emplace_back(callback);
}
void IMU::RegisterCallback_Estimate(const std::function<void(Estimate)> &callback)
{
    callbacks.estimate.emplace_back(callback);
}
void IMU::RegisterCallback_Orientation(const std::function<void(Orientation)> &callback)
{
    callbacks.orientation.emplace_back(callback);
}
void IMU::RegisterCallback_QuaternionStdDev(const std::function<void(QuaternionStdDev)> &callback)
{
    callbacks.quaternion_stddev.emplace_back(callback);
}
void IMU::RegisterCallback_Velocity(const std::function<void(StateEstimate)> &callback)
{
    callbacks.velocity.emplace_back(callback);
}
void IMU::RegisterCallback_Acceleration(const std::function<void(StateEstimate)> &callback)
{
    callbacks.acceleration.emplace_back(callback);
}
void IMU::RegisterCallback_AngularVelocity(const std::function<void(StateEstimate)> &callback)
{
    callbacks.angular_velocity.emplace_back(callback);
}
void IMU::RegisterCallback_AngularAcceleration(const std::function<void(StateEstimate)> &callback)
{
    callbacks.angular_acceleration.emplace_back(callback);
}
void IMU::RegisterCallback_PositionNED(const std::function<void(StateEstimate)> &callback)
{
    callbacks.positionNED.emplace_back(callback);
}
void IMU::RegisterCallback_PositionUTM(const std::function<void(StateEstimate)> &callback)
{
    callbacks.positionUTM.emplace_back(callback);
}
void IMU::RegisterCallback_PositionFusion(const std::function<void(PositionFusion)> &callback)
{
    callbacks.position_fusion.emplace_back(callback);
}
void IMU::ClearCallbacks()
{
    callbacks.accel.clear();
    callbacks.gyro.clear();
    callbacks.mag.clear();
    callbacks.estimate.clear();
    callbacks.orientation.clear();
}

bool IMU::GetStatus()
{
    bool status = true;

    // Request device information
    an_packet_t *an_packet;
    an_packet = encode_request_packet(packet_id_device_information);
    SendPackage(an_packet);
    an_packet_free(&an_packet);
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // wait for the new package to arrive and be processed
    // The above waiting should be made smarter (with a semaphore-based waiting for the actual new message to arrive, since the above fixed-time waiting is no guarantee)

    if(status_packet_.system_status.b.system_failure) {
        printf("System Failure\n");
        status = false;
    } else if(status_packet_.system_status.b.serial_port_overflow_alarm) {
        printf("Serial Port Overflow\n");
        status = false;
    } else {
        printf("IMU is OK\n");
    }

    std::cout << "Device: " << port_name_ << std::endl;
    double running_time = (1.0e-6) * running_time_packet_.microseconds + running_time_packet_.seconds;
    std::cout << "Running time: " << running_time << std::endl;
    std::cout << "Device ID: " << device_information_packet_.device_id << std::endl;

    std::ostringstream software_version_stream;
    software_version_stream << std::fixed << std::setprecision(3);
    software_version_stream << device_information_packet_.software_version/1000.0;
    std::cout << "Software Version: " << software_version_stream.str() << std::endl;

    std::ostringstream hardware_revision_stream;
    hardware_revision_stream << std::fixed << std::setprecision(3);
    hardware_revision_stream << device_information_packet_.hardware_revision/1000.0;
    std::cout << "Hardware Revision: " << hardware_revision_stream.str() << std::endl;

    std::stringstream serial_number_stream;
    serial_number_stream << std::hex << std::setfill('0') << std::setw(8);
    serial_number_stream << device_information_packet_.serial_number[0];
    serial_number_stream << device_information_packet_.serial_number[1];
    serial_number_stream << device_information_packet_.serial_number[2];
    std::cout << "Serial Number: " << serial_number_stream.str() << std::endl;

    status &= accelerometerStatus();
    status &= gyroscopeStatus();
    status &= magnetometerStatus();
    status &= temperatureStatus();
    status &= voltageStatus();
    status &= filterStatus();

    std::cout << std::endl;

    return status;
}
bool IMU::accelerometerStatus() {
    if(status_packet_.system_status.b.accelerometer_sensor_failure) {
        printf("Accelerometer Sensor Failure\n");
        return false;
    } else if(status_packet_.system_status.b.accelerometer_over_range) {
        printf("Accelerometer Sensor Over Range\n");
        return false;
    } else {
        printf("Accelerometer OK\n");
        return true;
    }
}
bool IMU::gyroscopeStatus() {
    if(status_packet_.system_status.b.gyroscope_sensor_failure) {
        printf("Gyroscope Sensor Failure\n");
        return false;
    } else if(status_packet_.system_status.b.gyroscope_over_range) {
        printf("Gyroscope Sensor Over Range\n");
        return false;
    } else {
        printf("Gyroscope OK\n");
        return true;
    }
}
bool IMU::magnetometerStatus() {
    if (status_packet_.system_status.b.magnetometer_sensor_failure) {
        printf("Magnetometer Sensor Failure\n");
        return false;
    } else if (status_packet_.system_status.b.magnetometer_over_range) {
        printf("Magnetometer Sensor Over Range\n");
        return false;
    } else {
        printf("Magnetometer OK\n");
        return true;
    }
}
bool IMU::temperatureStatus() {
    if(status_packet_.system_status.b.minimum_temperature_alarm) {
        printf("Minimum Temperature Alarm\n");
        return false;
    } else if(status_packet_.system_status.b.maximum_temperature_alarm) {
        printf("Maximum Temperature Alarm\n");
        return false;
    } else {
        printf("Temperature OK\n");
        return true;
    }
}
bool IMU::voltageStatus() {
    if(status_packet_.system_status.b.low_voltage_alarm) {
        printf("Low Voltage\n");
        return false;
    } else if(status_packet_.system_status.b.high_voltage_alarm) {
        printf("High Voltage\n");
        return false;
    } else {
        printf("Voltage OK\n");
        return true;
    }
}
bool IMU::filterStatus() {
    bool status = true;
    if(status_packet_.filter_status.b.orientation_filter_initialised) {
        printf("Filter OK\n");
    } else {
        printf("Filter Not Initialised\n");
        status = false;
    }

    std::cout << "Heading Initialised: " << (bool)status_packet_.filter_status.b.heading_initialised << std::endl;
    std::cout << "Magnetometers Enabled: " << (bool)status_packet_.filter_status.b.dual_antenna_heading_active << std::endl;
    std::cout << "Velocity Heading Enabled: " << (bool)status_packet_.filter_status.b.velocity_heading_enabled << std::endl;
    std::cout << "External Position Active: " << (bool)status_packet_.filter_status.b.external_position_active << std::endl;
    std::cout << "External Velocity Active: " << (bool)status_packet_.filter_status.b.external_velocity_active << std::endl;
    std::cout << "External Heading Active: " << (bool)status_packet_.filter_status.b.external_heading_active << std::endl;

    return status;
}

void IMU::Reset()
{
    an_packet_t * an_packet = encode_reset_packet();
    SendPackage(an_packet);
    an_packet_free(&an_packet);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    flush_serial_port(port_, flush_both);
}


void IMU::SetHeading(float heading_rad, float standard_deviation_rad)
{
    if (!externalHeadingInputEnabled_) return;

    external_heading_packet_t external_heading_packet;
    external_heading_packet.heading = heading_rad;
    external_heading_packet.standard_deviation = standard_deviation_rad;

    an_packet_t * an_packet = encode_external_heading_packet(&external_heading_packet);
    SendPackage(an_packet);
    an_packet_free(&an_packet);
}

void IMU::SetPositionNED(float x_N, float y_E, float z_D, float standard_deviation_m)
{
    if (!enableExternalPositionInput_) return;

    double lat, lon, alt;
    g_geodetic_converter->enu2Geodetic(y_E, x_N, -z_D, &lat, &lon, &alt);
    //geo::UTMtoLL(y + 149477.98, x + 368704.28, 48, lat, lon); alt = z;

    external_position_packet_t external_position_packet;
    external_position_packet.position[0] = deg2rad(lat); // latitude are degrees north or south of equator
    external_position_packet.position[1] = deg2rad(lon); // longitude are degrees east or west of the prime meridian (Greenwich)
    external_position_packet.position[2] = alt;

    external_position_packet.standard_deviation[0] = standard_deviation_m;
    external_position_packet.standard_deviation[1] = standard_deviation_m;
    external_position_packet.standard_deviation[2] = standard_deviation_m;

    an_packet_t * an_packet = encode_external_position_packet(&external_position_packet);
    SendPackage(an_packet);
    an_packet_free(&an_packet);

    latestPositionFusion.position_setpoint[0] = lat;
    latestPositionFusion.position_setpoint[1] = lon;
    latestPositionFusion.position_setpoint[2] = alt;
}

void IMU::StartStaticPositionPublisher(float x_N, float y_E, float z_D, float standard_deviation_m)
{
    // Start position publisher thread
    position_publisher_thread = std::thread([this, x_N, y_E, z_D, standard_deviation_m] {
        while (connected_) {
            SetPositionNED(x_N, y_E, z_D, standard_deviation_m);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    });
}

IMU::Estimate IMU::GetEstimate()
{
    return latestEstimate;
}

IMU::Orientation IMU::GetOrientation()
{
    return latestOrientation;
}

IMU::Quaternion IMU::GetQuaternion()
{
    if (latestEstimate.timestamp > latestOrientation.timestamp)
        return latestEstimate.quaternion;
    else
        return latestOrientation.quaternion;
}

IMU::QuaternionStdDev IMU::GetQuaternionStdDev()
{
    return latestQuaternionStdDev;
}

IMU::CombinedSensorsMeasurement IMU::GetRawSensors()
{
    return latestRawSensors;
}

IMU::SensorMeasurement IMU::GetAccelerometer()
{
    return {latestRawSensors.timestamp, latestRawSensors.accelerometer};
}

IMU::SensorMeasurement IMU::GetGyroscope()
{
    return {latestRawSensors.timestamp, latestRawSensors.gyroscope};
}

IMU::SensorMeasurement IMU::GetMagnetometer()
{
    return {latestRawSensors.timestamp, latestRawSensors.magnetometer};
}

IMU::StateEstimate IMU::GetVelocity()
{
    return latestVelocity;
}

IMU::StateEstimate IMU::GetAcceleration()
{
    return latestAcceleration;
}

IMU::StateEstimate IMU::GetAngularVelocity()
{
    return latestAngularVelocity;
}

IMU::StateEstimate IMU::GetAngularAcceleration()
{
    return latestAngularAcceleration;
}

double IMU::GetLatestTimestamp()
{
    return latestTimestamp;
}

Eigen::Vector3f IMU::GetPoseEulerRPY(bool degrees)
{
    return GetPoseEulerRPY(GetRotation(latestEstimate));
}

Eigen::Vector3f IMU::GetPoseEulerRPY(Quaternion orientation, bool degrees)
{
    Eigen::Vector3f euler_ZYX = GetEulerAnglesZYX(GetRotation(orientation)); // The output format is [yaw, pitch, roll]
    if (degrees)
        return Eigen::Vector3f(rad2deg(euler_ZYX(2)), rad2deg(euler_ZYX(1)), rad2deg(euler_ZYX(0))); // Swap it to make the output [roll, pitch, yaw]
    else
        return Eigen::Vector3f(euler_ZYX(2), euler_ZYX(1), euler_ZYX(0)); // Swap it to make the output [roll, pitch, yaw]
}

Eigen::Vector3f IMU::GetPoseEulerRPY(Eigen::Matrix<float, 3, 3> rotation, bool degrees)
{
    Eigen::Vector3f euler_ZYX = GetEulerAnglesZYX(rotation); // The output format is [yaw, pitch, roll]
    if (degrees)
        return Eigen::Vector3f(rad2deg(euler_ZYX(2)), rad2deg(euler_ZYX(1)), rad2deg(euler_ZYX(0))); // Swap it to make the output [roll, pitch, yaw]
    else
        return Eigen::Vector3f(euler_ZYX(2), euler_ZYX(1), euler_ZYX(0)); // Swap it to make the output [roll, pitch, yaw]
}

Eigen::Vector3f IMU::GetPoseEulerRPY(Eigen::Affine3f pose, bool degrees)
{
    Eigen::Vector3f euler_ZYX = GetEulerAnglesZYX(pose.matrix()); // The output format is [yaw, pitch, roll]
    if (degrees)
        return Eigen::Vector3f(rad2deg(euler_ZYX(2)), rad2deg(euler_ZYX(1)), rad2deg(euler_ZYX(0))); // Swap it to make the output [roll, pitch, yaw]
    else
        return Eigen::Vector3f(euler_ZYX(2), euler_ZYX(1), euler_ZYX(0)); // Swap it to make the output [roll, pitch, yaw]
}


// Calculates rotation matrix based on Quaternion estimate
Eigen::Matrix<float, 3, 3> IMU::GetRotation(Quaternion& q)
{
    Eigen::Matrix<float, 3, 3, Eigen::ColMajor> mat;
    // Convert the quaternion into a rotation matrix, stored in column-major format
    mat.data()[0] = 1 - 2 * q.y()*q.y() - 2 * q.z()*q.z();   mat.data()[3] = 2 * q.x()*q.y() - 2 * q.z()*q.w();     mat.data()[6] = 2 * q.x()*q.z() + 2 * q.y()*q.w();
    mat.data()[1] = 2 * q.x()*q.y() + 2 * q.z()*q.w();       mat.data()[4] = 1 - 2 * q.x()*q.x() - 2 * q.z()*q.z(); mat.data()[7] = 2 * q.y()*q.z() - 2 * q.x()*q.w();
    mat.data()[2] = 2 * q.x()*q.z() - 2 * q.y()*q.w();       mat.data()[5] = 2 * q.y()*q.z() + 2 * q.x()*q.w();     mat.data()[8] = 1 - 2 * q.x()*q.x() - 2 * q.y()*q.y();
    return mat;
}

Eigen::Matrix<float, 3, 3> IMU::GetRotation(Estimate& est)
{
    return GetRotation(est.quaternion);
}

IMU::Quaternion IMU::GetRotation(Vector& RPY)
{
    // Convert roll, pitch, yaw from radians to quaternion format
    float phi = RPY[0] / 2.0f;
    float theta = RPY[1] / 2.0f;
    float psi = RPY[2] / 2.0f;
    float sin_phi = sinf(phi);
    float cos_phi = cosf(phi);
    float sin_theta = sinf(theta);
    float cos_theta = cosf(theta);
    float sin_psi = sinf(psi);
    float cos_psi = cosf(psi);

    return Quaternion(
         -cos_phi * sin_theta * sin_psi + sin_phi * cos_theta * cos_psi, // x
         cos_phi * sin_theta * cos_psi + sin_phi * cos_theta * sin_psi,  // y
         cos_phi * cos_theta * sin_psi - sin_phi * sin_theta * cos_psi,  // z
         cos_phi * cos_theta * cos_psi + sin_phi * sin_theta * sin_psi  // w
    );
}

Eigen::Vector3f IMU::GetEulerAnglesZYX(Eigen::Matrix<float, 3, 3> rotm)
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

Eigen::Vector3f IMU::GetEulerAnglesZYX(Eigen::Matrix<float, 4, 4> transform)
{
    return GetEulerAnglesZYX(static_cast<Eigen::Matrix<float, 3, 3>>(transform.block<3,3>(0,0).matrix()));
}

IMU::Quaternion IMU::quaternion_exp(Vector v)
{
    float x = v.x()/2.f, y = v.y()/2.f, z = v.z()/2.f, th2, th = sqrtf(th2 = x*x + y*y + z*z);
    float c = cosf(th), s = th2 < sqrtf(120*FLT_EPSILON) ? 1-th2/6 : sinf(th)/th;
    return Quaternion( s*x, s*y, s*z, c );
}

IMU::Quaternion IMU::quaternion_multiply(Quaternion a, Quaternion b)
{
    return Quaternion(
            a.x() * b.w() + a.w() * b.x() - a.z() * b.y() + a.y() * b.z(),
            a.y() * b.w() + a.z() * b.x() + a.w() * b.y() - a.x() * b.z(),
            a.z() * b.w() - a.y() * b.x() + a.x() * b.y() + a.w() * b.z(),
            a.w() * b.w() - a.x() * b.x() - a.y() * b.y() - a.z() * b.z()
    );
}

void IMU::quaternion_integration(boost::math::quaternion<float> &q, float gyroscopes[3], const float dt)
{
    float omega_norm = norm(gyroscopes, 3) + 1.0e-12;

    float sinOmega = sin(0.5f*omega_norm*dt);
    boost::math::quaternion<float> q_delta( std::cos(0.5f*omega_norm*dt),
                                            sinOmega*gyroscopes[0]/omega_norm,
                                            sinOmega*gyroscopes[1]/omega_norm,
                                            sinOmega*gyroscopes[2]/omega_norm
    );

    q *= q_delta;// q = q * q_delta;
}

double IMU::norm( const float x[], std::size_t sz )
{
    // http://en.cppreference.com/w/cpp/algorithm/inner_product
    return std::sqrt( std::inner_product( x, x+sz, x, 0.0 ) ) ;
}

IMU::Quaternion IMU::PredictOrientation(Estimate &est, float dt_s)
{
    /*Pose P = pose;
    P.translation.x = dt_s * (dt_s/2 * pose.acceleration.x + pose.velocity.x) + pose.translation.x;
    P.translation.y = dt_s * (dt_s/2 * pose.acceleration.y + pose.velocity.y) + pose.translation.y;
    P.translation.z = dt_s * (dt_s/2 * pose.acceleration.z + pose.velocity.z) + pose.translation.z;*/
    Vector W(
            dt_s * (dt_s/2 * est.angular_acceleration.x() + est.angular_velocity.x()), // x
            dt_s * (dt_s/2 * est.angular_acceleration.y() + est.angular_velocity.y()), // y
            dt_s * (dt_s/2 * est.angular_acceleration.z() + est.angular_velocity.z())  // z
    );
    return quaternion_multiply(quaternion_exp(W), est.quaternion);
}

int64_t IMU::utime()
{
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}
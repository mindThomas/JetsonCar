#ifndef JETSONCAR_LIBRARIES_DECAWAVE_DWM1001_H
#define JETSONCAR_LIBRARIES_DECAWAVE_DWM1001_H

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include <string>
#include <vector>
#include <numeric>
#include <cmath>

#include <thread>
#include <array>
#include <algorithm>
#include <mutex>
#include <condition_variable>

#include <boost/asio.hpp>
#include <boost/circular_buffer.hpp>

#if defined(__linux__)
#include <linux/serial.h>
#endif

#include <Eigen/Core>
#include <Eigen/Geometry>

#define deg2rad(angleDegrees) ((angleDegrees) * M_PI / 180.0)
#define rad2deg(angleRadians) ((angleRadians) * 180.0 / M_PI)

class DWM1001
{
public:
    typedef struct {
        float anchor_position[3] = {0}; // calibrated anchor position
        float range = 0;              // range measurement
    } TagToAnchorMeasurement;

    typedef struct TagToAnchorMeasurements {
        uint64_t timestamp = 0; // microseconds
        std::map<uint32_t, TagToAnchorMeasurement> measurements;
        struct {
            bool valid = false;
            float position[3] = {0}; // estimated tag position
            float quality = 0;
        } estimate;
    };

public:
    DWM1001(std::string port_name = "/dev/ttyACM0", uint32_t baud_rate = 115200);
    ~DWM1001();

    void Connect();
    void Disconnect();

    void Configure();
    void Reset();

    void EnterStreamMode();
    void ExitStreamMode();

    void RegisterMeasurementCallback(const std::function<void(TagToAnchorMeasurements)> &callback);
    void ClearCallbacks();

public:
    #include "dwm_api.h"

private:
    std::string port_name_;
    uint32_t baud_rate_;

    // Buffer for receiving data on the serial link.
    std::array<uint8_t, DWM1001_TLV_MAX_SIZE> tmp_buffer;
    boost::circular_buffer<uint8_t> serial_read_buffer;
    std::mutex buffer_mutex;
    std::condition_variable buffer_cv;

    // Serial port
    boost::asio::io_service ioservice;
    boost::asio::serial_port port_ =
            boost::asio::serial_port(ioservice);
    std::thread ioservice_thread;
    std::thread position_publisher_thread;
    const bool hwflow_{false};
    bool connected_{false};

    // Stream mode
    bool ascii_mode_{false};
    std::vector<std::function<void(TagToAnchorMeasurements)>> callbacks_;

private:
    /// @brief Different ways a serial port may be flushed.
    enum flush_type
    {
        flush_receive = TCIFLUSH,
        flush_send = TCOFLUSH,
        flush_both = TCIOFLUSH
    };

private:
    int SendPackage(uint8_t * buffer, size_t tx_len);
    int SendPackage(uint8_t * buffer, uint8_t * tx_len_ptr);
    int WaitForRx(uint8_t* data, uint16_t* length, int16_t exp_length = -1);

    void SerialCallback(const boost::system::error_code &error, std::size_t bytes_received);

    int WaitForCharacter(uint8_t chr);
    void PrintBufferContents(bool ascii = false);
    void PrintBufferUntil(uint8_t chr);

    std::string GetLineFromBuffer();
    std::vector<std::string> GetDelimitedLineFromBuffer();

    void ProcessAsciiStreamLine();

    boost::system::error_code flush_serial_port( boost::asio::serial_port& serial_port, flush_type what);

    template <typename SyncReadStream, typename MutableBufferSequence>
    bool readWithTimeout(SyncReadStream& s, const MutableBufferSequence& buffers, const boost::asio::deadline_timer::duration_type& expiry_time);

    int64_t utime();

};

#endif

#include "dwm1001.h"

#include <iostream>
#include <iomanip>
#include <cmath>
#include <limits>
#include <stdio.h>

#include <boost/asio/read.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/optional.hpp>
#include <boost/algorithm/string.hpp> // boost::split

DWM1001::DWM1001(std::string port_name, uint32_t baud_rate)
    : port_name_{port_name}
    , baud_rate_{baud_rate}
    , serial_read_buffer(3*DWM1001_TLV_MAX_SIZE)
    , hwflow_{false}
{
}

DWM1001::~DWM1001()
{
    if (connected_) {
        Disconnect();
    }
}

void DWM1001::Connect()
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
        //throw std::runtime_error("serial" + std::string(err.what()));
        std::cout << "[DWM1001] Error: " << std::string(err.what()) << std::endl;
        return;
    }

    flush_serial_port(port_, flush_both);

    connected_ = true;

    port_.async_read_some(boost::asio::buffer(tmp_buffer.begin(), std::min(tmp_buffer.size(), serial_read_buffer.capacity()-serial_read_buffer.size())),
                          std::bind(&DWM1001::SerialCallback, this, std::placeholders::_1, std::placeholders::_2));
    /*boost::asio::async_read(
            //port_, boost::asio::buffer(read_buffer),
            port_, boost::asio::buffer(an_decoder_pointer(&an_decoder_), std::min(an_decoder_size(&an_decoder_), size_t(10))),
          std::bind(&OrientusNode::SerialCallback, this, std::placeholders::_1, std::placeholders::_2));
    */
    // Start the I/O service in its own thread.
    ioservice_thread = std::thread([&] { ioservice.run(); });

    // Wait for start
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    Reset();
    flush_serial_port(port_, flush_both);
}

void DWM1001::Disconnect()
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
        std::cout << "[DWM1001] Error: " << std::string(err.what()) << std::endl;
        return;
    }

    connected_ = false;
}

void DWM1001::Configure()
{
}

int DWM1001::SendPackage(uint8_t * buffer, size_t tx_len)
{
    if (!connected_) return -1;
    if (boost::asio::write(port_, boost::asio::buffer(buffer, tx_len)))
        return RV_OK;
    else
        return RV_ERR;
}

int DWM1001::SendPackage(uint8_t * buffer, uint8_t * tx_len_ptr)
{
    if (!connected_) return -1;
    flush_serial_port(port_, flush_both);
    if (boost::asio::write(port_, boost::asio::buffer(buffer, *tx_len_ptr)))
        return RV_OK;
    else
        return RV_ERR;
}

int DWM1001::WaitForRx(uint8_t* data, uint16_t* length, int16_t exp_length)
{
    buffer_mutex.unlock();
    std::unique_lock<std::mutex> lk(buffer_mutex);

    if (exp_length > 0) {
        if (buffer_cv.wait_for(lk, std::chrono::milliseconds(100),
                               [this, exp_length] {
                                   //std::cout << "Buffer size = " << serial_read_buffer.size() << " | Expected size = " << exp_length << std::endl;
                                   return serial_read_buffer.size() >= exp_length;
                               })) {
            //return RV_OK;
        }/* else {
        //return RV_ERR; // timeout
    }*/
    } else {
        buffer_cv.wait_for(lk, std::chrono::milliseconds(200),
                           [this] {
                               //std::cout << "Buffer size = " << serial_read_buffer.size() << " | Expected size = " << exp_length << std::endl;
                               return serial_read_buffer.size() >= DWM1001_TLV_MAX_SIZE;
                           });
    }

    *length = 0;
    while (serial_read_buffer.size() && (*length) < DWM1001_TLV_MAX_SIZE) {
        (*length)++;
        *data++ = serial_read_buffer.front();
        serial_read_buffer.pop_front();
    }

    return RV_OK;

    /*if (readWithTimeout(port_, boost::asio::buffer(data, exp_length), boost::posix_time::seconds(10)))
        return RV_OK;
    else
        return RV_ERR;*/
}

int DWM1001::WaitForCharacter(uint8_t chr)
{
    if (!ascii_mode_) return RV_ERR;

    buffer_mutex.unlock();
    std::unique_lock<std::mutex> lk(buffer_mutex);

    if (buffer_cv.wait_for(lk, std::chrono::milliseconds(500),
                           [this, chr] {
                               return std::any_of(serial_read_buffer.begin(), serial_read_buffer.end(), [chr](auto&& byte){ return byte == chr; });
                           }))
    {
        return RV_OK;
    } else {
        return RV_ERR;
    }
}

template <typename SyncReadStream, typename MutableBufferSequence>
bool DWM1001::readWithTimeout(SyncReadStream& s, const MutableBufferSequence& buffers, const boost::asio::deadline_timer::duration_type& expiry_time)
{
    boost::optional<boost::system::error_code> timer_result;
    boost::asio::deadline_timer timer(s.get_io_service());
    timer.expires_from_now(expiry_time);
    timer.async_wait([&timer_result] (const boost::system::error_code& error) { timer_result.reset(error); });

    boost::optional<boost::system::error_code> read_result;
    boost::asio::async_read(s, buffers, [&read_result] (const boost::system::error_code& error, size_t) { read_result.reset(error); });

    s.get_io_service().reset();
    while (s.get_io_service().run_one())
    {
        if (read_result)
            timer.cancel();
        else if (timer_result)
            s.cancel();
    }
    s.get_io_service().reset();

    if (*read_result)
        return false; // error

    return true; // success
}

// Process incoming data on serial link
void DWM1001::SerialCallback(const boost::system::error_code &error, std::size_t bytes_received)
{
    if (error == boost::system::errc::operation_canceled || error == boost::asio::error::eof) {
        port_.async_read_some(boost::asio::buffer(tmp_buffer.begin(), std::min(tmp_buffer.size(), serial_read_buffer.capacity()-serial_read_buffer.size())),
                              std::bind(&DWM1001::SerialCallback, this, std::placeholders::_1, std::placeholders::_2));
        return;
    } else if (error) {
        //throw std::runtime_error("serial asio: " + error.message());
        std::cout << "[DWM1001] Serial Callback error: " << error.message() << std::endl;
        return;
    }

    // Buffer full, clear it!
    if (ascii_mode_ && bytes_received >= (serial_read_buffer.capacity() - serial_read_buffer.size())) {
        serial_read_buffer.clear();
    }

    if (bytes_received > 0) {
        buffer_mutex.lock();
        std::copy(tmp_buffer.begin(), tmp_buffer.begin() + bytes_received, std::back_inserter(serial_read_buffer));
        buffer_mutex.unlock();
        buffer_cv.notify_one();

        if (ascii_mode_ &&
            std::any_of(tmp_buffer.begin(), tmp_buffer.begin() + bytes_received, [](auto&& byte){ return byte == '\n'; })) {
            ProcessAsciiStreamLine();
        }
    }

    // READ THE NEXT PACKET
    port_.async_read_some(boost::asio::buffer(tmp_buffer.begin(), std::min(tmp_buffer.size(), serial_read_buffer.capacity()-serial_read_buffer.size())),
                          std::bind(&DWM1001::SerialCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void DWM1001::ProcessAsciiStreamLine()
{
    auto line = GetLineFromBuffer();

    TagToAnchorMeasurements m;
    m.timestamp = utime();

    std::vector<std::string> split;
    boost::split(split, line, boost::is_any_of(","));
    if (split.size() > 2) {
        if (split[0].compare("DIST") == 0) {
            int num_anchors = std::stoi(split[1]);
            for (int i = 0; i < num_anchors; i++) {
                if (split.size() < 2+6*i+5+1) break;

                char * p;
                uint32_t ID = strtoul(split[2+6*i+1].c_str(), &p, 16); // hex string to long
                if (*p != 0) {
                    continue;
                }

                float x = std::stof(split[2+6*i+2]);
                float y = std::stof(split[2+6*i+3]);
                float z = std::stof(split[2+6*i+4]);
                float range = std::stof(split[2+6*i+5]);

                m.measurements[ID] = TagToAnchorMeasurement{{x,y,z}, range};
            }
        }
        if (split.size() >= 7) {
            if (split[split.size()-5].compare("POS") == 0) {
                m.estimate.valid = true;
                m.estimate.position[0] = std::stof(split[split.size()-4]);
                m.estimate.position[1] = std::stof(split[split.size()-3]);
                m.estimate.position[2] = std::stof(split[split.size()-2]);
                m.estimate.quality = std::stof(split[split.size()-1]);
            }
        }
    }

    if (m.measurements.size()) {
        for (auto &cb : callbacks_) {
            cb(m);
        }
    }
}

void DWM1001::PrintBufferContents(bool ascii)
{
    if (ascii) {
        std::for_each(serial_read_buffer.begin(), serial_read_buffer.end(), [](auto byte) {
            printf("%c", byte);
        });
    } else {
        std::for_each(serial_read_buffer.begin(), serial_read_buffer.end(), [](auto byte)
        {
            printf("%02X ", byte);
        });
    }
    printf("\n");
    serial_read_buffer.clear();
}

void DWM1001::PrintBufferUntil(uint8_t chr)
{
    uint8_t byte;
    std::cout << "Buffer content size: " << serial_read_buffer.size() << std::endl;
    while (serial_read_buffer.size() > 0 && (byte = serial_read_buffer.front()) != chr) {
        printf("%c", byte);
        serial_read_buffer.pop_front();
    }
    if (serial_read_buffer.size() > 0)
        serial_read_buffer.pop_front();
    printf("\n");
}

std::string DWM1001::GetLineFromBuffer()
{
    if (!ascii_mode_) return "";

    const uint8_t chr = '\n';
    uint8_t byte;
    std::string out;

    while (serial_read_buffer.size() > 0 && (byte = serial_read_buffer.front()) != chr) {
        out += byte;
        serial_read_buffer.pop_front();
    }

    if (serial_read_buffer.size() > 0)
        serial_read_buffer.pop_front();

    return out;
}

void DWM1001::RegisterMeasurementCallback(const std::function<void(TagToAnchorMeasurements)> &callback)
{
    callbacks_.emplace_back(callback);
}
void DWM1001::ClearCallbacks()
{
    callbacks_.clear();
}

void DWM1001::Reset()
{
    if (!connected_) return;

    {
        uint8_t buf[] = {'\r', '\r'};
        SendPackage(buf, sizeof(buf));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    {
        uint8_t buf[] = "reset\r";
        SendPackage(buf, sizeof(buf));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    ascii_mode_ = false;
}

void DWM1001::EnterStreamMode()
{
    if (!connected_) return;

    if (!ascii_mode_)
    {
        uint8_t buf[] = {'\r', '\r'};
        SendPackage(buf, sizeof(buf));

        ascii_mode_ = true;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    {
        uint8_t buf[] = "lec\r";
        SendPackage(buf, sizeof(buf));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    flush_serial_port(port_, flush_both);
}

void DWM1001::ExitStreamMode()
{
    if (!connected_ || !ascii_mode_) return;

    {
        uint8_t buf[] = "reset\r";
        SendPackage(buf, sizeof(buf));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    ascii_mode_ = false;
}


/// From https://stackoverflow.com/questions/22581315/how-to-discard-data-as-it-is-sent-with-boostasio/22598329#22598329
/// @brief Flush a serial port's buffers.
///
/// @param serial_port Port to flush.
/// @param what Determines the buffers to flush.
/// @param error Set to indicate what error occurred, if any.
boost::system::error_code DWM1001::flush_serial_port(
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

    serial_read_buffer.clear();
}

int64_t DWM1001::utime()
{
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}
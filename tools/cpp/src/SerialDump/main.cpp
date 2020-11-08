#include <iostream>
#include <thread> // for ioservice thread
#include <string.h> // for memchr

#include <boost/asio.hpp>
#include <boost/asio.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/optional.hpp>

#if defined(__linux__)
#include <linux/serial.h>
#endif

#include <misc.hpp>

// OBS! You can change between async_read and async_read_some depending on whether to block 
// and wait for the buffer to be filled up or run the callback with whatever data 
// that is available at the time of reading
// See https://stackoverflow.com/a/28509826


// Serial port
boost::asio::io_service ioservice;
boost::asio::serial_port port = boost::asio::serial_port(ioservice);
std::thread ioservice_thread;
const bool hwflow{false};
std::array<uint8_t, 15> read_buffer;

void ProcessIncomingBytes(uint8_t * buffer, size_t bytes_received);

void ReadCallback(const boost::system::error_code &error, std::size_t bytes_transferred)
{    
    if (error == boost::system::errc::operation_canceled || error == boost::asio::error::eof) {
        // EOF error
        //port.async_read_some(boost::asio::buffer(read_buffer),
        //                      std::bind(&ReadCallback, std::placeholders::_1, std::placeholders::_2));        
        boost::asio::async_read(port, boost::asio::buffer(read_buffer),
                                std::bind(&ReadCallback, std::placeholders::_1, std::placeholders::_2));        
        return;
    } else if (error) {
        //throw std::runtime_error("serial asio: " + error.message());
        std::cout << "Serial Callback error: " << error.message() << std::endl;
    }

    ProcessIncomingBytes(&read_buffer[0], bytes_transferred);

    // READ THE NEXT PACKET
    //port.async_read_some(boost::asio::buffer(read_buffer),
    //                      std::bind(&ReadCallback, std::placeholders::_1, std::placeholders::_2));
    boost::asio::async_read(port, boost::asio::buffer(read_buffer),
                            std::bind(&ReadCallback, std::placeholders::_1, std::placeholders::_2));

    return;
}

void WriteCallback(const boost::system::error_code &error, size_t bytes_transferred) {    
}

void Connect(std::string port_name, uint32_t baud_rate)
{
    // From https://github.com/mavlink/mavros/blob/master/libmavconn/src/serial.cpp
    try {
        port.open(port_name);

        // Set baudrate and 8N1 mode
        port.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
        port.set_option(boost::asio::serial_port_base::character_size(8));
        port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));

#if BOOST_ASIO_VERSION >= 101200 || !defined(__linux__)
        // Flow control setting in older versions of Boost.ASIO is broken, use workaround (below) for now.
        serial_dev.set_option(SPB::flow_control( (hwflow) ? SPB::flow_control::hardware : SPB::flow_control::none));
#elif BOOST_ASIO_VERSION < 101200 && defined(__linux__)
        // Workaround to set some options for the port manually. This is done in
        // Boost.ASIO, but until v1.12.0 (Boost 1.66) there was a bug which doesn't enable relevant
        // code. Fixed by commit: https://github.com/boostorg/asio/commit/619cea4356
        {
            int fd = port.native_handle();

            termios tio;
            tcgetattr(fd, &tio);

            // Set hardware flow control settings
            if (hwflow) {
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
            int fd = port.native_handle();

            struct serial_struct ser_info;
            ioctl(fd, TIOCGSERIAL, &ser_info);

            ser_info.flags |= ASYNC_LOW_LATENCY;

            ioctl(fd, TIOCSSERIAL, &ser_info);
        }
#endif
    }
    catch (boost::system::system_error &err) {
        //throw std::runtime_error("serial" + std::string(err.what()));
        std::cout << "Error: " << std::string(err.what()) << std::endl;
        exit(-1);        
    }

    //port.async_read_some(boost::asio::buffer(read_buffer),
    //                     std::bind(&ReadCallback, std::placeholders::_1, std::placeholders::_2));
    boost::asio::async_read(port, boost::asio::buffer(read_buffer),
                        std::bind(&ReadCallback, std::placeholders::_1, std::placeholders::_2));
    // Start the I/O service in its own thread.
    ioservice_thread = std::thread([&] { ioservice.run(); });

    // Wait for start
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void Disconnect()
{
    ioservice.stop();

    if (ioservice_thread.joinable()) {
        ioservice_thread.join();
    }

    try {
        port.close();
    }
    catch (boost::system::system_error &err) {
        //throw std::runtime_error("serial" + std::string(err.what()));
        std::cout << "Error: " << std::string(err.what()) << std::endl;
        exit(-1);
    }
}


template <typename SyncReadStream, typename MutableBufferSequence>
void readWithTimeout(SyncReadStream& s, const MutableBufferSequence& buffers, const boost::asio::deadline_timer::duration_type& expiry_time)
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
        throw boost::system::system_error(*read_result);
}

boost::system::error_code Flush() {
    // From http://mnb.ociweb.com/mnb/MiddlewareNewsBrief-201303.html
#if 0
    boost::system::error_code ec;
#if !defined(BOOST_WINDOWS) && !defined(__CYGWIN__)
    const bool isFlushed =! ::tcflush(controller_port.native(), TCIOFLUSH);
    if (!isFlushed)
        ec = boost::system::error_code(errno,
                                       boost::asio::error::get_system_category());
#else
    const bool isFlushed = ::PurgeComm(port.native(),
    PURGE_RXABORT | PURGE_RXCLEAR | PURGE_TXABORT | PURGE_TXCLEAR);
if (!isFlushed)
    ec = boost::system::error_code(::GetLastError(),
        boost::asio::error::get_system_category());
#endif
    return ec;
#else
    bool continueReading = true;
    while (continueReading) {
        try
        {
            readWithTimeout(port, boost::asio::buffer(read_buffer), boost::posix_time::milliseconds(1));
        }
        catch (boost::system::system_error& e)
        {
            continueReading = false;
        }
    }
#endif
}

bool isOpen() {    
    return port.is_open();
}

void send(std::vector<uint8_t> buffer) {
    // Consider adding a mutex
    boost::asio::async_write(
            port, boost::asio::buffer(buffer),
            std::bind(&WriteCallback, std::placeholders::_1,
                      std::placeholders::_2));
}

// Should exit signals
EventSignal exit_signal;
void exitHandler(int s) {
    std::cout << "Exiting..." << std::endl;
    //should_exit = true;
    exit_signal.trigger();
}

int main(int argc, char* argv[]) 
{ 
    signal(SIGINT, exitHandler);

    Connect("/dev/ttyUSB0", 19200);
    exit_signal.waitForEvent();
    Disconnect();    
} 

void ProcessIncomingBytes(uint8_t * buffer, size_t bytes_received)
{    
    uint8_t * start = (uint8_t*)memchr((void*)buffer, 0xFE, bytes_received);
    if (start > 0) { 
        if ((start - buffer) + 9 < bytes_received) {
            if (start[8] == 0xAA) {
                for (unsigned int i = 0; i < 9; i++) {
                    printf("0x%x\t", *start++);
                }
                printf("\n");
            }
        }
    }
}
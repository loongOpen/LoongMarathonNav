/*
 * File:   qx_async_serial.cpp
 */

#include "qx_async_serial.h"

#include <string>
#include <algorithm>
#include <iostream>
#include <boost/bind.hpp>
#include <sys/ioctl.h>
#include <linux/serial.h>

//using namespace std;
using namespace boost;

//
//Class AsyncSerial
//

#ifndef __APPLE__

/****
 * 该类AsyncSerialImpl用于实现异步串口通信，主要功能包括：
 * 管理串口的打开状态和错误标志。
 * 提供线程进行读写操作。
 * 使用互斥锁保护对队列和缓冲区的访问。
 * 维护写入队列、写入缓冲区及读缓冲区。
 * 支持读取完成后的回调函数。 该类继承自boost::noncopyable以禁止拷贝构造和赋值操作。
 * */
class AsyncSerialImpl : private boost::noncopyable
{
public:
    AsyncSerialImpl() : io(), port(io), backgroundThread(), open(false),
                        error(false) {}

    boost::asio::io_service io;      ///< Io service object
    boost::asio::serial_port port;   ///< Serial port object
    boost::thread backgroundThread;  ///< Thread that runs read/write operations
    bool open;                       ///< True if port open
    bool error;                      ///< Error flag
    mutable boost::mutex errorMutex; ///< Mutex for access to error

    /// Data are queued here before they go in writeBuffer
    std::vector<char> writeQueue;
    boost::shared_array<char> writeBuffer;        ///< Data being written
    size_t writeBufferSize;                       ///< Size of writeBuffer
    boost::mutex writeQueueMutex;                 ///< Mutex for access to writeQueue
    char readBuffer[AsyncSerial::readBufferSize]; ///< data being read
    char readBuffer1[AsyncSerial::readBufferSize];
    char readBuffer2[AsyncSerial::readBufferSize];
    /// Read complete callback
    boost::function<void(const char *, size_t)> callback;
};

AsyncSerial::AsyncSerial() : pimpl(new AsyncSerialImpl)
{
}

AsyncSerial::AsyncSerial(const std::string &devname, unsigned int baud_rate,
                         asio::serial_port_base::parity opt_parity,
                         asio::serial_port_base::character_size opt_csize,
                         asio::serial_port_base::flow_control opt_flow,
                         asio::serial_port_base::stop_bits opt_stop)
        : pimpl(new AsyncSerialImpl)
{
    open(devname, baud_rate, opt_parity, opt_csize, opt_flow, opt_stop);
}

void AsyncSerial::open(const std::string &devname, unsigned int baud_rate,
                       asio::serial_port_base::parity opt_parity,
                       asio::serial_port_base::character_size opt_csize,
                       asio::serial_port_base::flow_control opt_flow,
                       asio::serial_port_base::stop_bits opt_stop)
{
    if (isOpen())
        close();

    setErrorStatus(true); //If an exception is thrown, error_ remains true
    /// 设置并打开串口
    pimpl->port.open(devname);
    pimpl->port.set_option(asio::serial_port_base::baud_rate(baud_rate));
    pimpl->port.set_option(opt_parity);
    pimpl->port.set_option(opt_csize);
    pimpl->port.set_option(opt_flow);
    pimpl->port.set_option(opt_stop);

    if(! pimpl->port.is_open()) {
        std::cout << "serial port " << devname << " failed to open." << std::endl;
        return;
    }else {
        std::cout << "serial port " << devname << " opened." << std::endl;
    }
//    boost::asio::basic_serial_port<boost::asio::serial_port_service>::native_type native = pimpl->port.native(); // serial_port_ is the boost's serial port class.
    /// 获取串口设备的原生句柄
    boost::asio::serial_port::native_handle_type native = pimpl->port.native_handle();
    struct serial_struct serial_tempf;
    ioctl(native, TIOCGSERIAL, &serial_tempf);
    serial_tempf.flags |= ASYNC_LOW_LATENCY; // (0x2000)
    ioctl(native, TIOCSSERIAL, &serial_tempf);
    //This gives some work to the io_service before it is started
    pimpl->io.post(boost::bind(&AsyncSerial::doRead, this));
    thread t(boost::bind(&asio::io_service::run, &pimpl->io));
    pimpl->backgroundThread.swap(t);
    setErrorStatus(false); //If we get here, no error
    pimpl->open = true;    //Port is now open
}

bool AsyncSerial::isOpen() const
{
//    if(pimpl == NULL) {
//        std::cout << "pimpl == NULL" << std::endl;
//        return false;
//    }
    return pimpl->open;
}

bool AsyncSerial::errorStatus() const
{
    lock_guard<mutex> l(pimpl->errorMutex);
    return pimpl->error;
}

void AsyncSerial::close()
{
    std::cout << "AsyncSerial::close()" << std::endl;
    if (!isOpen()) {
        std::cout << "!isOpen()" << std::endl;
        return;
    }

    pimpl->open = false;
    pimpl->io.post(boost::bind(&AsyncSerial::doClose, this));
    pimpl->backgroundThread.join();
    pimpl->io.reset();
    if (errorStatus())
    {
        throw(boost::system::system_error(boost::system::error_code(),
                                          "Error while closing the device"));
    }
    std::cout << "close serial port" << std::endl;
}

void AsyncSerial::write(const char *data, size_t size)
{
    {
        lock_guard<mutex> l(pimpl->writeQueueMutex);
        pimpl->writeQueue.insert(pimpl->writeQueue.end(), data, data + size);
    }
    pimpl->io.post(boost::bind(&AsyncSerial::doWrite, this));
}

void AsyncSerial::write(const std::vector<char> &data)
{
    {
        lock_guard<mutex> l(pimpl->writeQueueMutex);
        pimpl->writeQueue.insert(pimpl->writeQueue.end(), data.begin(),
                                 data.end());
    }
    pimpl->io.post(boost::bind(&AsyncSerial::doWrite, this));
}

void AsyncSerial::writeString(const std::string &s)
{
    {
        lock_guard<mutex> l(pimpl->writeQueueMutex);
        pimpl->writeQueue.insert(pimpl->writeQueue.end(), s.begin(), s.end());
    }
    pimpl->io.post(boost::bind(&AsyncSerial::doWrite, this));
}

AsyncSerial::~AsyncSerial()
{
    if (isOpen())
    {
        try
        {
            close();
        }
        catch (...)
        {
            //Don't throw from a destructor
        }
    }
}

void AsyncSerial::doRead()
{
    pimpl->port.async_read_some(asio::buffer(pimpl->readBuffer, readBufferSize),
                                boost::bind(&AsyncSerial::readEnd,
                                            this,
                                            asio::placeholders::error,
                                            asio::placeholders::bytes_transferred));

    // asio::async_read(pimpl->port,asio::buffer(pimpl->readBuffer,readBufferSize),asio::transfer_exactly(54),
    //         boost::bind(&AsyncSerial::readEnd,
    //         this,
    //         asio::placeholders::error,
    //         asio::placeholders::bytes_transferred));
}

void AsyncSerial::readEnd(const boost::system::error_code &error,
                          size_t bytes_transferred)
{
    if (error)
    {
        std::cout << "Error code: " << error.value() << ", Message: " << error.message() << std::endl;
#ifdef __APPLE__
        if (error.value() == 45)
        {
            //Bug on OS X, it might be necessary to repeat the setup
            //http://osdir.com/ml/lib.boost.asio.user/2008-08/msg00004.html
            doRead();
            return;
        }
#endif //__APPLE__
        //error can be true even because the serial port was closed.
        //In this case it is not a real error, so ignore
        if (isOpen())
        {
            doClose();
            setErrorStatus(true);
        }
    }
    else
    {
        if (pimpl->callback)
        {
            pimpl->callback(pimpl->readBuffer, bytes_transferred);
        }
        doRead();
    }
}

void AsyncSerial::doWrite()
{
    //If a write operation is already in progress, do nothing
    if (pimpl->writeBuffer == 0)
    {
        lock_guard<mutex> l(pimpl->writeQueueMutex);
        pimpl->writeBufferSize = pimpl->writeQueue.size();
        pimpl->writeBuffer.reset(new char[pimpl->writeQueue.size()]);
        copy(pimpl->writeQueue.begin(), pimpl->writeQueue.end(),
             pimpl->writeBuffer.get());
        pimpl->writeQueue.clear();
        async_write(pimpl->port, asio::buffer(pimpl->writeBuffer.get(), pimpl->writeBufferSize),
                    boost::bind(&AsyncSerial::writeEnd, this, asio::placeholders::error));
    }
}

void AsyncSerial::writeEnd(const boost::system::error_code &error)
{
    if (!error)
    {
        lock_guard<mutex> l(pimpl->writeQueueMutex);
        if (pimpl->writeQueue.empty())
        {
            pimpl->writeBuffer.reset();
            pimpl->writeBufferSize = 0;

            return;
        }
        pimpl->writeBufferSize = pimpl->writeQueue.size();
        pimpl->writeBuffer.reset(new char[pimpl->writeQueue.size()]);
        copy(pimpl->writeQueue.begin(), pimpl->writeQueue.end(),
             pimpl->writeBuffer.get());
        pimpl->writeQueue.clear();
        async_write(pimpl->port, asio::buffer(pimpl->writeBuffer.get(), pimpl->writeBufferSize),
                    boost::bind(&AsyncSerial::writeEnd, this, asio::placeholders::error));
    }
    else
    {
        setErrorStatus(true);
        doClose();
    }
}

void AsyncSerial::doClose()
{
    boost::system::error_code ec;
    pimpl->port.cancel(ec);
    if (ec)
        setErrorStatus(true);
    pimpl->port.close(ec);
    if (ec)
        setErrorStatus(true);
}

void AsyncSerial::setErrorStatus(bool e)
{
    lock_guard<mutex> l(pimpl->errorMutex);
    pimpl->error = e;
}

void AsyncSerial::setReadCallback(const boost::function<void(const char *, size_t)> &callback)
{
    pimpl->callback = callback;
}

void AsyncSerial::clearReadCallback()
{
    pimpl->callback.clear();
}

#else //__APPLE__

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

class AsyncSerialImpl : private boost::noncopyable
{
  public:
    AsyncSerialImpl() : backgroundThread(), open(false), error(false) {}

    boost::thread backgroundThread;  ///< Thread that runs read operations
    bool open;                       ///< True if port open
    bool error;                      ///< Error flag
    mutable boost::mutex errorMutex; ///< Mutex for access to error

    int fd; ///< File descriptor for serial port

    char readBuffer[AsyncSerial::readBufferSize]; ///< data being read

    /// Read complete callback
    boost::function<void(const char *, size_t)> callback;
};

AsyncSerial::AsyncSerial() : pimpl(new AsyncSerialImpl)
{
}

AsyncSerial::AsyncSerial(const std::string &devname, unsigned int baud_rate,
                         asio::serial_port_base::parity opt_parity,
                         asio::serial_port_base::character_size opt_csize,
                         asio::serial_port_base::flow_control opt_flow,
                         asio::serial_port_base::stop_bits opt_stop)
    : pimpl(new AsyncSerialImpl)
{
    open(devname, baud_rate, opt_parity, opt_csize, opt_flow, opt_stop);
}

void AsyncSerial::open(const std::string &devname, unsigned int baud_rate,
                       asio::serial_port_base::parity opt_parity,
                       asio::serial_port_base::character_size opt_csize,
                       asio::serial_port_base::flow_control opt_flow,
                       asio::serial_port_base::stop_bits opt_stop)
{
    if (isOpen())
        close();

    setErrorStatus(true); //If an exception is thrown, error remains true

    struct termios new_attributes;
    speed_t speed;
    int status;

    // Open port
    pimpl->fd = ::open(devname.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (pimpl->fd < 0)
        throw(boost::system::system_error(
            boost::system::error_code(), "Failed to open port"));

    // Set Port parameters.
    status = tcgetattr(pimpl->fd, &new_attributes);
    if (status < 0 || !isatty(pimpl->fd))
    {
        ::close(pimpl->fd);
        throw(boost::system::system_error(
            boost::system::error_code(), "Device is not a tty"));
    }
    new_attributes.c_iflag = IGNBRK;
    new_attributes.c_oflag = 0;
    new_attributes.c_lflag = 0;
    new_attributes.c_cflag = (CS8 | CREAD | CLOCAL); //8 data bit,Enable receiver,Ignore modem
    /* In non canonical mode (Ctrl-C and other disabled, no echo,...) VMIN and VTIME work this way:
    if the function read() has'nt read at least VMIN chars it waits until has read at least VMIN
    chars (even if VTIME timeout expires); once it has read at least vmin chars, if subsequent
    chars do not arrive before VTIME expires, it returns error; if a char arrives, it resets the
    timeout, so the internal timer will again start from zero (for the nex char,if any)*/
    new_attributes.c_cc[VMIN] = 1;  // Minimum number of characters to read before returning error
    new_attributes.c_cc[VTIME] = 1; // Set timeouts in tenths of second

    // Set baud rate
    switch (baud_rate)
    {
    case 50:
        speed = B50;
        break;
    case 75:
        speed = B75;
        break;
    case 110:
        speed = B110;
        break;
    case 134:
        speed = B134;
        break;
    case 150:
        speed = B150;
        break;
    case 200:
        speed = B200;
        break;
    case 300:
        speed = B300;
        break;
    case 600:
        speed = B600;
        break;
    case 1200:
        speed = B1200;
        break;
    case 1800:
        speed = B1800;
        break;
    case 2400:
        speed = B2400;
        break;
    case 4800:
        speed = B4800;
        break;
    case 9600:
        speed = B9600;
        break;
    case 19200:
        speed = B19200;
        break;
    case 38400:
        speed = B38400;
        break;
    case 57600:
        speed = B57600;
        break;
    case 115200:
        speed = B115200;
        break;
    case 230400:
        speed = B230400;
        break;
    default:
    {
        ::close(pimpl->fd);
        throw(boost::system::system_error(
            boost::system::error_code(), "Unsupported baud rate"));
    }
    }

    cfsetospeed(&new_attributes, speed);
    cfsetispeed(&new_attributes, speed);

    //Make changes effective
    status = tcsetattr(pimpl->fd, TCSANOW, &new_attributes);
    if (status < 0)
    {
        ::close(pimpl->fd);
        throw(boost::system::system_error(
            boost::system::error_code(), "Can't set port attributes"));
    }

    //These 3 lines clear the O_NONBLOCK flag
    status = fcntl(pimpl->fd, F_GETFL, 0);
    if (status != -1)
        fcntl(pimpl->fd, F_SETFL, status & ~O_NONBLOCK);

    setErrorStatus(false); //If we get here, no error
    pimpl->open = true;    //Port is now open

    thread t(bind(&AsyncSerial::doRead, this));
    pimpl->backgroundThread.swap(t);
}

bool AsyncSerial::isOpen() const
{
    return pimpl->open;
}

bool AsyncSerial::errorStatus() const
{
    lock_guard<mutex> l(pimpl->errorMutex);
    return pimpl->error;
}

void AsyncSerial::close()
{
    if (!isOpen())
        return;

    pimpl->open = false;

    ::close(pimpl->fd); //The thread waiting on I/O should return

    pimpl->backgroundThread.join();
    if (errorStatus())
    {
        throw(boost::system::system_error(boost::system::error_code(),
                                          "Error while closing the device"));
    }
}

void AsyncSerial::write(const char *data, size_t size)
{
    if (::write(pimpl->fd, data, size) != size)
        setErrorStatus(true);
}

void AsyncSerial::write(const std::vector<char> &data)
{
    if (::write(pimpl->fd, &data[0], data.size()) != data.size())
        setErrorStatus(true);
}

void AsyncSerial::writeString(const std::string &s)
{
    if (::write(pimpl->fd, &s[0], s.size()) != s.size())
        setErrorStatus(true);
}

AsyncSerial::~AsyncSerial()
{
    if (isOpen())
    {
        try
        {
            close();
        }
        catch (...)
        {
            //Don't throw from a destructor
        }
    }
}

void AsyncSerial::doRead()
{
    //Read loop in spawned thread
    for (;;)
    {
        int received = ::read(pimpl->fd, pimpl->readBuffer, readBufferSize);
        if (received < 0)
        {
            if (isOpen() == false)
                return; //Thread interrupted because port closed
            else
            {
                setErrorStatus(true);
                continue;
            }
        }
        if (pimpl->callback)
            pimpl->callback(pimpl->readBuffer, received);
    }
}

void AsyncSerial::readEnd(const boost::system::error_code &error,
                          size_t bytes_transferred)
{
    //Not used
}

void AsyncSerial::doWrite()
{
    //Not used
}

void AsyncSerial::writeEnd(const boost::system::error_code &error)
{
    //Not used
}

void AsyncSerial::doClose()
{
    //Not used
}

void AsyncSerial::setErrorStatus(bool e)
{
    lock_guard<mutex> l(pimpl->errorMutex);
    pimpl->error = e;
}

void AsyncSerial::setReadCallback(const function<void(const char *, size_t)> &callback)
{
    pimpl->callback = callback;
}

void AsyncSerial::clearReadCallback()
{
    pimpl->callback.clear();
}

#endif //__APPLE__

//
//Class CallbackAsyncSerial
//

CallbackAsyncSerial::CallbackAsyncSerial() : AsyncSerial()
{
}

CallbackAsyncSerial::CallbackAsyncSerial(const std::string &devname,
                                         unsigned int baud_rate,
                                         asio::serial_port_base::parity opt_parity,
                                         asio::serial_port_base::character_size opt_csize,
                                         asio::serial_port_base::flow_control opt_flow,
                                         asio::serial_port_base::stop_bits opt_stop)
        : AsyncSerial(devname, baud_rate, opt_parity, opt_csize, opt_flow, opt_stop)
{
}

void CallbackAsyncSerial::setCallback(const boost::function<void(const char *, size_t)> &callback)
{
//    std::cout << "CallbackAsyncSerial::setCallback" << std::endl;
    setReadCallback(callback);
}

void CallbackAsyncSerial::clearCallback()
{
    clearReadCallback();
}

CallbackAsyncSerial::~CallbackAsyncSerial()
{
    clearReadCallback();
}

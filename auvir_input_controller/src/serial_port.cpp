#include "serial_port.hpp"


CSerialPort::CSerialPort()
{

}

std::string CSerialPort::read_response()
{
    static boost::asio::io_service ios;
    boost::asio::serial_port sp(ios, "/dev/ttyUSB0");
    sp.set_option(boost::asio::serial_port::baud_rate(115200));
    // You can set other options using similar syntax
    char tmp[128];
    auto length = sp.read_some(boost::asio::buffer(tmp));
    // process the info received
    sp.close();
    return std::string(tmp);
}

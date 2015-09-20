#include "serial_port.hpp"

int main(void)
{
    std::cout << "this is input controller project" << std::endl;

    try
    {
        SimpleSerial serial("/dev/ttyUSB0",115200);
        std::cout << serial.readLine()<< std::endl;
    }

    catch(boost::system::system_error& e)
    {
        std::cout << "Error: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "Press enter...." << std::endl;
    std::getchar();
}

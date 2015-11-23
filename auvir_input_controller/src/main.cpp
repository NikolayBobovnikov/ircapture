#include "serial_port.hpp"

int main(void)
{
    std::cout << "this is input controller project" << std::endl;

    try
    {
        //std::string port_name = "/dev/ttyUSB0";
        std::string port_name = "COM4";

        SimpleSerial serial(port_name,115200);
        std::cout << "Opened " + port_name << std::endl;
        std::cout << "MPU6050_data_t: " + std::to_string(sizeof(MPU6050_data_t)) << std::endl;

        while(1)
        {
            std::cout << serial.get_mpu6050_data_str() << std::endl;
            //std::cout << serial.readLine() << std::endl;
        }
    }

    catch(boost::system::system_error& e)
    {
        std::cout << "Exception occured, exiting..."  << std::endl;
        return 1;
    }

    std::cout << "Press enter...." << std::endl;
    std::getchar();
}


#include "serial_port.hpp"

int main(void)
{
    std::cout << "this is input controller project" << std::endl;

    try
    {
        SimpleSerial serial("/dev/ttyUSB0",115200);
        std::cout << "Opened /dev/ttyUSB0" << std::endl;
        std::cout << "MPU6050_data_t: " + std::to_string(sizeof(MPU6050_data_t)) << std::endl;

        while(1)
        {
            std::cout << serial.get_mpu6050_data_str() << std::endl;
        }
    }

    catch(boost::system::system_error& e)
    {
        std::cout << "Error: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "Press enter...." << std::endl;
    std::getchar();
}


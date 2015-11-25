#include <iostream>
#include <chrono>
#include <thread>
#include "timeout_serial.hpp"

typedef struct
{
    int16_t Accelerometer_X;
    int16_t Accelerometer_Y;
    int16_t Accelerometer_Z;
    int16_t Temperature;
    int16_t Gyroscope_X;
    int16_t Gyroscope_Y;
    int16_t Gyroscope_Z;
} MPU6050_MotionData_t;

enum UART_Commands {
    UART_COMMAND_NOT_RECEIVED = 0,
    UART_REQUEST_SEND,
    UART_REQUEST_STOP,
    UART_REQUEST_SEND_BYTE,
    UART_REQUEST_SEND_MPU6050_TEST_DATA,
    UART_REQUEST_SEND_MPU6050_DATA,
    UART_REQUEST_SEND_MPU6050_PACKET
};

std::string construct_mpu6050_data_str(MPU6050_MotionData_t * mpu6050_motion_data)
{
    std::string result_str = "";
    result_str += std::string   ("  A_X ") +
            std::string   ("     A_Y ") +
            std::string   ("     A_Z ") +
            std::string   ("     Tstr ") +
            std::string   ("         G_X ") +
            std::string   ("     G_Y ") +
            std::string   ("     G_Z ") + "\n";
    // info
    result_str += "  "    + std::to_string(mpu6050_motion_data->Accelerometer_X  ) +
            "     " + std::to_string(mpu6050_motion_data->Accelerometer_Y  ) +
            "     " + std::to_string(mpu6050_motion_data->Accelerometer_Z  ) +
            "     " +std::to_string((mpu6050_motion_data->Temperature/340 + 36.53)) +
            "     " + std::to_string(mpu6050_motion_data->Gyroscope_X      ) +
            "     " + std::to_string(mpu6050_motion_data->Gyroscope_Y      ) +
            "     " + std::to_string(mpu6050_motion_data->Gyroscope_Z      ) + "\n";

    return result_str;
}

int main(void)
{    using namespace std;
     using namespace boost;
     try {

        //std::string port_name = "/dev/ttyUSB0";
        std::string port_name = "COM3";
        TimeoutSerial serial(port_name,9600);
        serial.setTimeout(posix_time::seconds(10));
        uint8_t command;
        MPU6050_MotionData_t data;
        char buffer_data[sizeof(MPU6050_MotionData_t)];

        while(true)
        {
            //serial.flush();
            command = UART_REQUEST_SEND_MPU6050_DATA;
            serial.write((char*)&command, 10);

            switch(command)
            {
                case UART_REQUEST_SEND_MPU6050_DATA:
                {
                    serial.read(buffer_data,sizeof(buffer_data));
                    memcpy(&data, buffer_data, sizeof(data));
                    std::cout << construct_mpu6050_data_str(&data);
                    break;
                }
                case UART_REQUEST_SEND_MPU6050_TEST_DATA:
                {
                    serial.read(buffer_data,sizeof(buffer_data));
                    memcpy(&data, buffer_data, sizeof(data));
                    std::cout << construct_mpu6050_data_str(&data);
                    break;
                }
                default:
                    break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        serial.close();

     } catch(boost::system::system_error& e)
    {
        std::string err_msg = std::string(e.what());
        cout<<"Error: " << err_msg <<endl;
        return 1;
    }
}


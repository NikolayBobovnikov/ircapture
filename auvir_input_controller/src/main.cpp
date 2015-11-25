#include <iostream>
#include <chrono>
#include <thread>
#include "timeout_serial.hpp"

#define TRY_CATCH_EXIT(function)\
try\
{\
    function;\
}\
catch(...)\
{\
    std::cout << "Error: couldn't open serial port. Press any key to exit" << std::endl;\
}\

//TODO
void try_catch_exit_function(std::function<void()> f)
{
    try
    {
        f();
    }
    catch(...)
    {
        std::cout << "Error: couldn't open serial port. Press any key to exit" << std::endl;
        getchar();
        return;
    }

}

typedef struct
{
    int16_t Accelerometer_X;
    int16_t Accelerometer_Y;
    int16_t Accelerometer_Z;
    int16_t Temperature;
    int16_t Gyroscope_X;
    int16_t Gyroscope_Y;
    int16_t Gyroscope_Z;
    uint32_t delta_time;
} MPU6050_MotionData_t;

enum UART_Commands {
    UART_COMMAND_NOT_RECEIVED = 0,
    UART_REQUEST_SEND,
    UART_REQUEST_STOP,
    UART_REQUEST_SEND_BYTE,
    UART_REQUEST_SEND_MPU6050_TEST_DATA,
    UART_REQUEST_SEND_MPU6050_DATA,
    UART_REQUEST_SEND_MPU6050_PACKET,
    UART_RESET_HRDWR
};

void construct_mpu6050_data_str(MPU6050_MotionData_t * mpu6050_motion_data)
{
    std::string result_str = "";
    // info
    result_str +=  "A_X: " + std::to_string(mpu6050_motion_data->Accelerometer_X  ) + "\n" +
                   "A_Y: " + std::to_string(mpu6050_motion_data->Accelerometer_Y  ) + "\n" +
                   "A_Z: " + std::to_string(mpu6050_motion_data->Accelerometer_Z  ) + "\n" +
                   "T:   " + std::to_string((mpu6050_motion_data->Temperature/340 + 36.53)) + "\n" +
                   "G_X: " + std::to_string(mpu6050_motion_data->Gyroscope_X      ) + "\n" +
                   "G_Y: " + std::to_string(mpu6050_motion_data->Gyroscope_Y      ) + "\n" +
                   "G_Z: " + std::to_string(mpu6050_motion_data->Gyroscope_Z      ) + "\n" +
                   "dt:  " + std::to_string(mpu6050_motion_data->delta_time      );

    system("cls");
    std::cout << result_str << std::endl;
}

int main(void)
{    using namespace std;
     using namespace boost;
     try {

        //std::string port_name = "/dev/ttyUSB0";
        std::string port_name = "COM6";
        int baud_rate = 115200;
        boost::shared_ptr<TimeoutSerial> serial(new TimeoutSerial());

        std::cout << "Initial opening of the serial port " << std::endl;
        TRY_CATCH_EXIT(serial->open(port_name,baud_rate));

        //std::function<void()> f = std::bind(&TimeoutSerial::open, serial, port_name,baud_rate);

        serial->setTimeout(posix_time::seconds(1));
        uint8_t command;
        MPU6050_MotionData_t data;
        char buffer_data[sizeof(MPU6050_MotionData_t)];

        // start clock
        auto time_start = std::chrono::high_resolution_clock::now();

        while(serial->isOpen())
        {
            //serial.flush();
            command = UART_REQUEST_SEND_MPU6050_DATA;
            size_t command_size = sizeof(command);

            serial->write((char*)&command, command_size);

            switch(command)
            {
                case UART_REQUEST_SEND_MPU6050_DATA:
                {
                    try
                    {
                        serial->read(buffer_data,sizeof(buffer_data));
                    }
                    catch(...)
                    {
                        std::cout << "Time elapsed: "
                                  << std::chrono::duration_cast<std::chrono::hours>(time_start - std::chrono::high_resolution_clock::now()).count() <<"h "
                                  << std::chrono::duration_cast<std::chrono::minutes>(time_start - std::chrono::high_resolution_clock::now()).count() <<"m "
                                  << std::chrono::duration_cast<std::chrono::seconds>(time_start - std::chrono::high_resolution_clock::now()).count() <<"s "
                                  << std::endl;
                        time_start = std::chrono::high_resolution_clock::now();

                        std::cout << "Error during reading from serial. Debug: start sending bytes. " << std::endl;

                        // send command to reset
                        command = UART_RESET_HRDWR;
                        serial->write((char*)&command, command_size);
                        // restart serial connection
                        serial->close();
                        serial.reset();
                        serial = boost::shared_ptr<TimeoutSerial>(new TimeoutSerial());
                        TRY_CATCH_EXIT(serial->open(port_name,baud_rate));

                        // start from the beginning of the loop
                        break;
                        //std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    }


                    memcpy(&data, buffer_data, sizeof(data));
                    construct_mpu6050_data_str(&data);
                    break;
                }
                case UART_REQUEST_SEND_MPU6050_TEST_DATA:
                {
                    serial->read(buffer_data,sizeof(buffer_data));
                    memcpy(&data, buffer_data, sizeof(data));
                    construct_mpu6050_data_str(&data);
                    break;
                }
                default:
                    break;
            }
            //std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        auto time_finish = std::chrono::high_resolution_clock::now();
        std::cout << "Time elapsed: "
                  << std::chrono::duration_cast<std::chrono::hours>(time_start - time_finish).count() <<"h "
                  << std::chrono::duration_cast<std::chrono::minutes>(time_start - time_finish).count() <<"m "
                  << std::chrono::duration_cast<std::chrono::seconds>(time_start - time_finish).count() <<"s "
                  << std::endl;

        serial->close();

     } catch(boost::system::system_error& e)
    {
        //std::string err_msg = std::string(e.what());
        //std::cout<<"Error: " << err_msg <<endl;
        std::cout<<"Error: exception occured" << std::endl;
        getchar();
        return 1;
    }
}


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

//TODO: learn this stuff
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

//TODO: move common structures to outer header file which will be used by both client and server sides
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

    float angle_x;
    float angle_y;
    float angle_z;
} MPU6050_MotionData_t;

typedef struct
{
    int mean_ax;
    int mean_ay;
    int mean_az;
    int mean_gx;
    int mean_gy;
    int mean_gz;
    int var_ax;
    int var_ay;
    int var_az;
    int var_gx;
    int var_gy;
    int var_gz;
    // TODO: offsets
    int16_t offset_ax;
    int16_t offset_ay;
    int16_t offset_az;
    int16_t offset_gx;
    int16_t offset_gy;
    int16_t offset_gz;
} MPU6050_CalibrationData_t;

enum UART_Commands {
    UART_COMMAND_NOT_RECEIVED = 0,
    UART_REQUEST_SEND,
    UART_REQUEST_STOP,
    UART_REQUEST_SEND_BYTE,
    UART_REQUEST_SEND_MPU6050_TEST_DATA,
    UART_REQUEST_SEND_MPU6050_DATA,
    UART_REQUEST_CALIB_DATA,

    UART_TEST_CONNECTION,			// request
    UART_CONNECTION_FAILURE,
    UART_CONENCTION_OK,

    UART_MPU6050_TEST_CONNECTION,	// request
    UART_MPU6050_CONENCTION_FAILURE,
    UART_MPU6050_CONENCTION_OK,
    UART_NULL_RESPONSE,

    UART_MPU6050_RESET,              // request
    UART_MPU6050_RESET_OK,
    UART_MPU6050_RESET_FAILURE
};

void clearscreen()
{
#ifdef __linux
        std::system("clear");
#elif  WIN32
        std::system("cls");
#endif
}
void construct_mpu6050_data_str(MPU6050_MotionData_t * mpu6050_motion_data)
{
    clearscreen();
    std::string result_str = "";
    // info
    result_str +=  "A_X: " + std::to_string(mpu6050_motion_data->Accelerometer_X  ) + "\n" +
                   "A_Y: " + std::to_string(mpu6050_motion_data->Accelerometer_Y  ) + "\n" +
                   "A_Z: " + std::to_string(mpu6050_motion_data->Accelerometer_Z  ) + "\n" +
                   "T:   " + std::to_string((mpu6050_motion_data->Temperature/340 + 36.53)) + "\n" +
                   "G_X: " + std::to_string(mpu6050_motion_data->Gyroscope_X      ) + "\n" +
                   "G_Y: " + std::to_string(mpu6050_motion_data->Gyroscope_Y      ) + "\n" +
                   "G_Z: " + std::to_string(mpu6050_motion_data->Gyroscope_Z      ) + "\n\n" +
                   "roll:  " + std::to_string(mpu6050_motion_data->angle_x      )+ "\n" +
                   "pitch:  " + std::to_string(mpu6050_motion_data->angle_y      )+ "\n" +
                   "yaw:  " + std::to_string(mpu6050_motion_data->angle_z    )      + "\n\n" +
                    "dt:  " + std::to_string(mpu6050_motion_data->delta_time      );

    std::cout << result_str << std::endl;
    //std::this_thread::sleep_for(std::chrono::milliseconds(300));
}
void construct_mpu6050_calib_data_str(MPU6050_CalibrationData_t * mpu6050_calib_data)
{
    clearscreen();
    std::string result_str = "";
    // info
    result_str += "mean_ax  : " + std::to_string(mpu6050_calib_data->mean_ax  ) + "\n" +
                  "mean_ay  : " + std::to_string(mpu6050_calib_data->mean_ay  ) + "\n" +
                  "mean_az  : " + std::to_string(mpu6050_calib_data->mean_az  ) + "\n" +
                  "mean_gx  : " + std::to_string(mpu6050_calib_data->mean_gx  ) + "\n" +
                  "mean_gy  : " + std::to_string(mpu6050_calib_data->mean_gy  ) + "\n" +
                  "mean_gz  : " + std::to_string(mpu6050_calib_data->mean_gz  ) + "\n" +
                  "var_ax : " + std::to_string(mpu6050_calib_data->var_ax ) + "\n" +
                  "var_ay : " + std::to_string(mpu6050_calib_data->var_ay ) + "\n" +
                  "var_az : " + std::to_string(mpu6050_calib_data->var_az ) + "\n" +
                  "var_gx : " + std::to_string(mpu6050_calib_data->var_gx ) + "\n" +
                  "var_gy : " + std::to_string(mpu6050_calib_data->var_gy ) + "\n" +
                  "var_gz : " + std::to_string(mpu6050_calib_data->var_gz ) + "\n" +
                  "offset_ax: " + std::to_string(mpu6050_calib_data->offset_ax) + "\n" +
                  "offset_ay: " + std::to_string(mpu6050_calib_data->offset_ay) + "\n" +
                  "offset_az: " + std::to_string(mpu6050_calib_data->offset_az) + "\n" +
                  "offset_gx: " + std::to_string(mpu6050_calib_data->offset_gx) + "\n" +
                  "offset_gy: " + std::to_string(mpu6050_calib_data->offset_gy) + "\n" +
                  "offset_gz: " + std::to_string(mpu6050_calib_data->offset_gz) + "\n";
    std::cout << result_str << std::endl;
}

int main(void)
{
    try {

        std::string port_name;
#ifdef __linux
        port_name = "/dev/ttyUSB0";
#elif  WIN32
        port_name = "COM7";
#endif
        int baud_rate = 115200;
        boost::shared_ptr<TimeoutSerial> serial(new TimeoutSerial());
        serial->setTimeout(boost::posix_time::seconds(10));

        std::cout << "sizeof(int) = " << sizeof(int) << std::endl;
        std::cout << "sizeof(uint8) = " << sizeof(uint8_t) << std::endl;
        std::cout << "sizeof(uint16) = " << sizeof(uint16_t) << std::endl;
        std::cout << "sizeof(uint32) = " << sizeof(uint32_t) << std::endl;
        std::cout << "Initial opening of the serial port " << std::endl;

        TRY_CATCH_EXIT(serial->open(port_name,baud_rate));

        if(!serial->isOpen())
        {
            std::cout << "Couldn't open serial port " + port_name << std::endl;
            return -1;
        }
        std::cout << "Opened serial port " << std::endl;

        //std::function<void()> f = std::bind(&TimeoutSerial::open, serial, port_name,baud_rate);

        uint8_t command;
        char response;
        MPU6050_MotionData_t data;
        MPU6050_CalibrationData_t calib_data;
        // start clock
        auto time_start = std::chrono::high_resolution_clock::now();

        // communication
        while(serial->isOpen())
        {
            command = UART_REQUEST_SEND_MPU6050_DATA;//UART_REQUEST_CALIB_DATA;
            response = UART_NULL_RESPONSE;
            size_t command_size = sizeof(command);

            serial->write((char*)&command, command_size);

            if((UART_REQUEST_SEND_MPU6050_DATA == command) || (UART_REQUEST_SEND_MPU6050_TEST_DATA == command))
            {
                try
                {
                    serial->read((char*)&data, sizeof(data));
                }
                catch(...)
                {
                    std::cout << "Time elapsed: "
                              << std::chrono::duration_cast<std::chrono::hours>(time_start - std::chrono::high_resolution_clock::now()).count() <<"h "
                              << std::chrono::duration_cast<std::chrono::minutes>(time_start - std::chrono::high_resolution_clock::now()).count() <<"m "
                              << std::chrono::duration_cast<std::chrono::seconds>(time_start - std::chrono::high_resolution_clock::now()).count() <<"s "
                              << std::endl;
                    time_start = std::chrono::high_resolution_clock::now();
                    std::cout << "Error during reading from serial. " << std::endl;
                    // TODO: send command to reset sensor?
                    break;
                }
                construct_mpu6050_data_str(&data);
            }
            else if(UART_REQUEST_CALIB_DATA == command)
            {
                serial->setTimeout(boost::posix_time::seconds(100));

                try
                {
                    serial->read((char *)&calib_data,sizeof(calib_data));
                }
                catch(...)
                {
                    std::cout << "Time elapsed: "
                              << std::chrono::duration_cast<std::chrono::hours>(time_start - std::chrono::high_resolution_clock::now()).count() <<"h "
                              << std::chrono::duration_cast<std::chrono::minutes>(time_start - std::chrono::high_resolution_clock::now()).count() <<"m "
                              << std::chrono::duration_cast<std::chrono::seconds>(time_start - std::chrono::high_resolution_clock::now()).count() <<"s "
                              << std::endl;
                    time_start = std::chrono::high_resolution_clock::now();

                    std::cout << "Error during reading from serial. " << std::endl;
                }

                construct_mpu6050_calib_data_str(&calib_data);
                serial->setTimeout(boost::posix_time::seconds(10));
            }
            else if (UART_TEST_CONNECTION == command)
            {
                // initial value
                char responce_byte = UART_CONNECTION_FAILURE;
                serial->setTimeout(boost::posix_time::seconds(10));
                try
                {
                    serial->read(&responce_byte,sizeof(responce_byte));
                }
                catch(...)
                {
                    std::cout << "Time elapsed: "
                              << std::chrono::duration_cast<std::chrono::hours>(time_start - std::chrono::high_resolution_clock::now()).count() <<"h "
                              << std::chrono::duration_cast<std::chrono::minutes>(time_start - std::chrono::high_resolution_clock::now()).count() <<"m "
                              << std::chrono::duration_cast<std::chrono::seconds>(time_start - std::chrono::high_resolution_clock::now()).count() <<"s "
                              << std::endl;
                    time_start = std::chrono::high_resolution_clock::now();
                    std::cout << "Error during reading from serial. " << std::endl;
                }
                if(UART_CONENCTION_OK == responce_byte)
                {
                    clearscreen();
                    std::cout << "UART connection is OK" << std::endl;
                }
                serial->setTimeout(boost::posix_time::seconds(10));
            }
            else if (UART_MPU6050_TEST_CONNECTION == command)
            {
                // initial value
                serial->setTimeout(boost::posix_time::seconds(10));
                try
                {
                    serial->read(&response,sizeof(response));
                }
                catch(...)
                {
                    std::cout << "Time elapsed: "
                              << std::chrono::duration_cast<std::chrono::hours>(time_start - std::chrono::high_resolution_clock::now()).count() <<"h "
                              << std::chrono::duration_cast<std::chrono::minutes>(time_start - std::chrono::high_resolution_clock::now()).count() <<"m "
                              << std::chrono::duration_cast<std::chrono::seconds>(time_start - std::chrono::high_resolution_clock::now()).count() <<"s "
                              << std::endl;
                    time_start = std::chrono::high_resolution_clock::now();
                    std::cout << "Error during reading from serial. " << std::endl;
                }
                // process result
                if(UART_MPU6050_CONENCTION_OK == response)
                {
                    clearscreen();
                    std::cout << "MPU6050 is connected" << std::endl;
                }
                else
                {
                    clearscreen();
                    std::cout << "MPU6050 is NOT connected" << std::endl;
                }
                serial->setTimeout(boost::posix_time::seconds(10));
            }

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


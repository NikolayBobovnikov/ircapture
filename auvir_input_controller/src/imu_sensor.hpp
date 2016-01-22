#include <stdexcept>
#include <iostream>
#include <chrono>
#include <thread>
#include <boost/utility.hpp>
#include <boost/asio.hpp>
#include <boost/enable_shared_from_this.hpp>
#include "timeout_serial.hpp"


#define TRY_CATCH_EXIT(function)\
{\
    try\
    {\
        function;\
    }\
    catch(...)\
    {\
        std::cout << "Error: couldn't open serial port. Press any key to exit" << std::endl;\
    }\
}\

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
    UART_MPU6050_RESET_FAILURE,

    UART_ECHO
};

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp> 
#include <boost/bind.hpp>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <string>

typedef struct {
    int16_t Accelerometer_X;
    int16_t Accelerometer_Y;
    int16_t Accelerometer_Z;
    int16_t Gyroscope_X;
    int16_t Gyroscope_Y;
    int16_t Gyroscope_Z;
    int16_t Temperature;
} MPU6050_data_t;

typedef struct
{
    int16_t Accelerometer_X;
    int16_t Accelerometer_Y;
    int16_t Accelerometer_Z;
    int16_t Gyroscope_X;
    int16_t Gyroscope_Y;
    int16_t Gyroscope_Z;
    int16_t Temperature;
    char strbuf[16];
} MPU6050_MotionData_t;

typedef struct
{
    uint8_t START;

    uint8_t data_uint8_t;
    uint16_t data_uint16_t;
    int int_number;
    float float_number;
    char string[128];

    uint8_t END;
} UART_TestSerialization_t;


enum UART_Packet_Condition {UART_PACKET_START = 0xFF, UART_PACKET_END = 0};
typedef struct
{
    const uint8_t START;
    const uint8_t data_size_begin;

    MPU6050_MotionData_t data;

    const uint8_t data_size_end;
    const uint8_t END;
} UART_Packet_t;


class CSerialPort
{
public:
    CSerialPort();
    std::string read_response();
};

class SimpleSerial
{
public:
    /**
     * Constructor.
     * \param port device name, example "/dev/ttyUSB0" or "COM4"
     * \param baud_rate communication speed, example 9600 or 115200
     * \throws boost::system::system_error if cannot open the
     * serial device
     */
    SimpleSerial(std::string port, unsigned int baud_rate)
    : io(), serial(io,port)
    {
        serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));

        //mpu6050_data_buf = {0};
        std::fill( data_buffer, data_buffer + sizeof( data_buffer ), 0 );
        //std::fill( &mpu6050_data_struct, &mpu6050_data_struct + sizeof( mpu6050_data_struct ), 0 );
        //std::fill( &mpu6050_motion_data, &mpu6050_motion_data + sizeof( mpu6050_motion_data ), 0 );
    }

    /**
     * Write a string to the serial device.
     * \param s string to write
     * \throws boost::system::system_error on failure
     */
    void writeString(std::string s)
    {
        boost::asio::write(serial,boost::asio::buffer(s.c_str(),s.size()));
    }

    /**
     * Blocks until a line is received from the serial device.
     * Eventual '\n' or '\r\n' characters at the end of the string are removed.
     * \return a string containing the received line
     * \throws boost::system::system_error on failure
     */
    std::string readLine();

    void read_mpu6050_data();
    bool read_mpu6050_packet_ok();
    std::string get_mpu6050_data_str();
    std::string test_serialization();

private:
    boost::asio::io_service io;
    boost::asio::serial_port serial;
    uint8_t data_buffer[128];
    MPU6050_data_t mpu6050_data_struct;
    MPU6050_MotionData_t mpu6050_motion_data;
};

std::string mpu6050_temperature_str(uint16_t temp_reg_value);

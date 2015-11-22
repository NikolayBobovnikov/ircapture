#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp> 
#include <boost/bind.hpp>
#include <cstdlib>
#include <iostream>
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
        std::fill( mpu6050_data_buf, mpu6050_data_buf + sizeof( mpu6050_data_buf ), 0 );
        mpu6050_data_struct = {0};
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
    std::string get_mpu6050_temperature();

private:
    boost::asio::io_service io;
    boost::asio::serial_port serial;
    uint8_t mpu6050_data_buf[15];
    MPU6050_data_t mpu6050_data_struct;
};

std::string mpu6050_temperature_str(uint16_t temp_reg_value);

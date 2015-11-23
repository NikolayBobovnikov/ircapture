#include "serial_port.hpp"
#include "stdio.h"
#include <iomanip>


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

std::string SimpleSerial::readLine()
{
    //Reading data char by char, code is optimized for simplicity, not speed
    using namespace boost;
    char c;
    std::string result;
    for(;;)
    {
        asio::read(serial,asio::buffer(&c,1));
        switch(c)
        {
            case '\r':
                break;
            case '\n':
                return result;
            case '\0':
                return result;
            default:
                result+=c;
        }
    }
}

void SimpleSerial::read_mpu6050_data()
{
}

bool SimpleSerial::read_mpu6050_packet_ok()
{
    using namespace boost;
    uint8_t motion_data_size = sizeof(MPU6050_Data_Reg_t);

    // 1. Wait for START
    int max_tries = 5;
    bool started = false;
    while(!started)
    {
        if(max_tries-- == 0)
        {
            return false;
        }

        uint8_t b_start = 2;
        boost::asio::read(serial, boost::asio::buffer(&b_start, 1));
        if(b_start == UART_PACKET_START) // Check that this is start byte
        {
            uint8_t b_size_begin = 0;
            boost::asio::read(serial, boost::asio::buffer(&b_size_begin, 1)); //check that the next goes size of data
            if(b_size_begin == motion_data_size)
            {
                started = true;
            }
        }
    }
    // 2. Read data
    size_t bytes_received = boost::asio::read(serial, boost::asio::buffer(&mpu6050_motion_data, motion_data_size));

    if(bytes_received != motion_data_size)
    {
        return false;
    }

    // 3. Check the end of the packet
    uint8_t b_size_end = 0;
    boost::asio::read(serial, boost::asio::buffer(&b_size_end, 1));
    if(b_size_end != motion_data_size)
    {
        return false;
    }
    uint8_t b_end = 1;
    boost::asio::read(serial, boost::asio::buffer(&b_end, 1));
    if(b_end != UART_PACKET_END)
    {
        return false;
    }

    return true;
}

std::string SimpleSerial::get_mpu6050_data_str()
{
    std::string result_str = "";
    if(read_mpu6050_packet_ok())
    {
        //read_mpu6050_data();
//        result_str += "Accelerometer_X: " + std::to_string(mpu6050_motion_data.Accelerometer_X  ) + "\n";
//        result_str += "Accelerometer_Y: " + std::to_string(mpu6050_motion_data.Accelerometer_Y  ) + "\n";
//        result_str += "Accelerometer_Z: " + std::to_string(mpu6050_motion_data.Accelerometer_Z  ) + "\n";
//        result_str += "Temperature:     " + std::to_string(mpu6050_motion_data.Temperature      ) + "\n";
//        result_str += "Gyroscope_X:     " + std::to_string(mpu6050_motion_data.Gyroscope_X      ) + "\n";
//        result_str += "Gyroscope_Y:     " + std::to_string(mpu6050_motion_data.Gyroscope_Y      ) + "\n";
//        result_str += "Gyroscope_Z:     " + std::to_string(mpu6050_motion_data.Gyroscope_Z      ) + "\n";
//        result_str += "STRING:          " + std::string   (mpu6050_motion_data.strbuf           ) + "\n";

        // header
        result_str += std::string   ("     A_X ") +
                      std::string   ("     A_Y ") +
                      std::string   ("     A_Z ") +
                      std::string   ("     Tstr ") +
                      std::string   ("         G_X ") +
                      std::string   ("     G_Y ") +
                      std::string   ("     G_Z ") + "\n";
        // info
        std::setw(2);
        std::setprecision(2);
        result_str += "  "    + std::to_string(mpu6050_motion_data.Accelerometer_X  ) +
                      "     " + std::to_string(mpu6050_motion_data.Accelerometer_Y  ) +
                      "     " + std::to_string(mpu6050_motion_data.Accelerometer_Z  ) +
                      "     " + std::to_string((mpu6050_motion_data.Temperature/340 + 36.53)) +
                      "     " + std::to_string(mpu6050_motion_data.Gyroscope_X      ) +
                      "     " + std::to_string(mpu6050_motion_data.Gyroscope_Y      ) +
                      "     " + std::to_string(mpu6050_motion_data.Gyroscope_Z      ) + "\n";
    }


    //float temp_value = (mpu6050_data_struct.Temperature)/340 + 36.53;
    //return std::to_string(temp_value);
    return result_str;
}

std::string SimpleSerial::test_serialization()
{
    // construct
    const char * test_str = "HelloSerialization!\0";
    UART_TestSerialization_t test_packet;
    test_packet.data_uint8_t = 8;
    test_packet.data_uint16_t = 16;
    test_packet.float_number = 10.10;
    test_packet.int_number = 9;
    int size = sizeof(test_str);
    strncpy(test_packet.string, test_str, strnlen(test_str, 128));

    char t[128] = {0};
    strncpy(t, test_str, strnlen(test_str,128));

    // place to a buffer
    uint8_t buffer[sizeof(UART_TestSerialization_t)];
    memcpy(buffer, &test_packet, sizeof(test_packet));

    // reconstruct from a buffer
    UART_TestSerialization_t test_packet_reconstructed;
    memcpy(&test_packet_reconstructed, buffer, sizeof(UART_TestSerialization_t));

    std::string result_str = std::string(test_packet_reconstructed.string);
    return result_str;
}

std::string mpu6050_temperature_str(uint16_t temp_reg_value)
{
    char buf[128]={0};
    float temperature = (temp_reg_value)/340 + 36.53;
    int integer_part = (int) temperature;           // Get the integer part
    int fractional_part = (int)((temperature - integer_part) * 1000);   // Get fractional part
    //TODO: snprintf: identifier not fount on windows; //snprintf(buf, sizeof(buf), "%d.%03d", integer_part, fractional_part,'\0');

    std::string result = std::string(buf);
    return result;
}

SerialPort::SerialPort(boost::asio::io_service &io_service, unsigned int baud, const string &peripheral)
    :io_service(io_service), port(io_service, peripheral), timeout_timer(io_service)
{
    if(!port.is_open())
    {

        throw std::runtime_error("The serial port is already open.");
        return;
    }

    port.set_option(boost::asio::serial_port_base::baud_rate(baud));
    cout << "Serial Port " << peripheral << " opened." << endl;
}

void SerialPort::close()
{
    io_service.post(boost::bind(&SerialPort::do_close, this, boost::system::error_code()));
}

void SerialPort::do_read()
{
    cout << "..Do_read called." << endl;
    boost::asio::async_read(port,boost::asio::buffer(buffer_read, sizeof(buffer_read)), boost::asio::transfer_at_least(sizeof(buffer_read)), boost::bind(&SerialPort::read_callback, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    timeout_timer.expires_from_now(boost::posix_time::milliseconds(SERIAL_TIMEOUT_MS));
    timeout_timer.async_wait(boost::bind(&SerialPort::wait_callback, this, boost::asio::placeholders::error));

}

void SerialPort::read_callback(const boost::system::error_code &error, size_t bytes_transferred)
{
    //cout << "..Read_callback called." << endl;
    if(error)
    {
        cerr << "Read_callback called with an error." << endl;
        do_close(error);
        return;
    }
    if(!error && bytes_transferred == 0)
    {
        cerr << ".. ..Unexpected behaviour: no error but 0 byte transferred. \n" << endl;
        //timeout_timer.cancel(); // THIS LINE SHOULD BE INSTEAD OF THE NEXT ONE
        do_read(); // NASTY SOLUTION TO GET IT WORKING
        return;
    }

    //cout << ".. ..Bytes transferred: " << bytes_transferred << " bytes." << endl;
    //cout << ".. ..Message: " << buffer_read[0] << buffer_read[1] << buffer_read[2] << buffer_read[3] << buffer_read[4] << buffer_read[5] << endl;
    memcpy(&packet, buffer_read, sizeof(buffer_read));
    std::string result_str="";
    result_str += std::string   ("     A_X ") +
                  std::string   ("     A_Y ") +
                  std::string   ("     A_Z ") +
                  std::string   ("     Tstr ") +
                  std::string   ("         G_X ") +
                  std::string   ("     G_Y ") +
                  std::string   ("     G_Z ") + "\n";
    // info
    std::setprecision(2);
    result_str += "  "    + std::to_string(packet.data.Accelerometer_X  ) +
                  "     " + std::to_string(packet.data.Accelerometer_Y  ) +
                  "     " + std::to_string(packet.data.Accelerometer_Z  ) +
                  "  "    + std::to_string(packet.data.Temperature      ) +
                  "     " + std::to_string((packet.data.Temperature/340 + 36.53)) +
                  "     " + std::to_string(packet.data.Gyroscope_X      ) +
                  "     " + std::to_string(packet.data.Gyroscope_Y      ) +
                  "     " + std::to_string(packet.data.Gyroscope_Z      ) + "\n";

    std::cout << result_str << std::endl;

    timeout_timer.cancel();
    do_read();
}

void SerialPort::wait_callback(const boost::system::error_code &error)
{
    cout << "..Wait_callback called." << endl;
    if (error)
    {
        cout << ".. ..Timeout cancelled." << endl;
        if(error != boost::asio::error::operation_aborted) // if timer fired
            do_close(error); // close the serial port only if error is not "operation_aborted"
        return;
    }

    cout << ".. ..Timeout fired." << endl;
    port.cancel();  // Close all asynchronous operation with serial port. will cause read_callback to fire with an error
                    // Does io_service keep running ?
    cout << ".. ..All operations on serial port cancelled.\n" <<endl;
    return;
}

void SerialPort::do_close(const boost::system::error_code &error)
{
    cout << "..Do_close called" << endl;
    if(error)
    {
        cerr << "Error detected: " << error.message() << endl;
    }
    if(error == boost::asio::error::operation_aborted)
    {
        return;
    }
    timeout_timer.cancel();
    port.close();
    cout << "Serial Port closed" << endl;
    return;
}

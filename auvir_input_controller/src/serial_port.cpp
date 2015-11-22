#include "serial_port.hpp"


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
    using namespace boost;
    uint8_t b;

    // read buffer
    bool finished_reading = false;
    int data_buf_ind = 0;
    while(!finished_reading && data_buf_ind < sizeof(mpu6050_data_buf))
    {
        boost::asio::read(serial, boost::asio::buffer(&b, 1));
        if(b == 0)
        {
            finished_reading = true;
        }
        else
        {
            mpu6050_data_buf[data_buf_ind++] = b;
        }
    }
    // convert to structure
    // 1 variant
//    uint16_t reg_data[7] = {0};
//    for(int i = 0; i < sizeof(reg_data); i++)
//    {
//        int mpu6050_data_buf_index = i * 2;
//        reg_data[i] = (((int16_t) mpu6050_data_buf[mpu6050_data_buf_index]) << 8) | mpu6050_data_buf[mpu6050_data_buf_index + 1];
//    }
//    mpu6050_data_struct.Accelerometer_X = reg_data[0];
//    mpu6050_data_struct.Accelerometer_Y = reg_data[1];
//    mpu6050_data_struct.Accelerometer_Z = reg_data[2];
//    mpu6050_data_struct.Temperature =     reg_data[3];
//    mpu6050_data_struct.Gyroscope_X =     reg_data[4];
//    mpu6050_data_struct.Gyroscope_Y =     reg_data[5];
//    mpu6050_data_struct.Gyroscope_Z =     reg_data[6];
      //2 variant:
    uint8_t buf[sizeof(mpu6050_data_struct)];
    memcpy(&mpu6050_data_struct, buf, sizeof(mpu6050_data_struct));
}

std::string SimpleSerial::get_mpu6050_temperature()
{
    read_mpu6050_data();
    float temp_value = (mpu6050_data_struct.Temperature)/340 + 36.53;
    return std::to_string(temp_value);
}

std::string mpu6050_temperature_str(uint16_t temp_reg_value)
{
    char buf[128]={0};
    float temperature = (temp_reg_value)/340 + 36.53;
    int integer_part = (int) temperature;           // Get the integer part
    int fractional_part = (int)((temperature - integer_part) * 1000);   // Get fractional part
    snprintf(buf, sizeof(buf), "%d.%03d", integer_part, fractional_part,'\0');

    std::string result = std::string(buf);
}

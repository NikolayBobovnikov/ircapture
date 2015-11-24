#include "serial_port.hpp"
#include "timeout_serial.hpp"

std::string construct_mpu6050_data_str(MPU6050_Data_Reg_t * mpu6050_motion_data)
{
    std::string result_str = "";
    result_str += std::string   ("     A_X ") +
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


#ifdef READ_CHARS_ONLY
int main(void)
{    std::cout << "this is input controller project" << std::endl;
     try
     {
        //std::string port_name = "/dev/ttyUSB0";
        std::string port_name = "COM3";

        SimpleSerial serial(port_name,115200);
        std::cout << "Opened " + port_name << std::endl;
        while(1)
        {
            std::cout << serial.readLine();
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
#endif

#ifndef USE_TIMEOUT_SERIAL_READ_1_BYTE
int main(void)
{    using namespace std;
     using namespace boost;
     try {

        TimeoutSerial serial("COM3",115200);
        serial.setTimeout(posix_time::seconds(10));

        //Text test
        //serial.writeString("Hello world\n");
        //std::cout<<serial.readStringUntil("\r\n")<<std::endl;

        //Binary test
        uint8_t command;
        MPU6050_Data_Reg_t data;
        char buffer_data[sizeof(MPU6050_Data_Reg_t)];
        UART_Packet_t packet;
        char buffer_packet[sizeof(UART_Packet_t)];


        while(true)
        {
            command = UART_REQUEST_SEND_MPU6050_DATA;
            serial.write((char*)&command, 10);

            switch(command)
            {
                case UART_REQUEST_SEND_BYTE:
                {
                    uint8_t byte;
                    serial.read((char*)&byte,1);

                    std::cout << std::to_string(byte) << "; ";
                    break;
                }
                case UART_REQUEST_SEND_MPU6050_TEST_DATA:
                {
                    serial.read(buffer_data,sizeof(buffer_data));
                    memcpy(&data, buffer_data, sizeof(data));
                    std::string result_str;
                    result_str += std::string   ("     A_X ") +
                            std::string   ("     A_Y ") +
                            std::string   ("     A_Z ") +
                            std::string   ("     Tstr ") +
                            std::string   ("         G_X ") +
                            std::string   ("     G_Y ") +
                            std::string   ("     G_Z ") + "\n";
                    // info
                    result_str += "  "    + std::to_string(data.Accelerometer_X  ) +
                            "     " + std::to_string(data.Accelerometer_Y  ) +
                            "     " + std::to_string(data.Accelerometer_Z  ) +
                            "     " + std::to_string(data.Temperature      ) +
                            "     " + std::to_string(data.Gyroscope_X      ) +
                            "     " + std::to_string(data.Gyroscope_Y      ) +
                            "     " + std::to_string(data.Gyroscope_Z      ) + "\n";
                    std::cout << result_str;
                    break;
                }
                case UART_REQUEST_SEND_MPU6050_DATA:
                {
                    serial.read(buffer_data,sizeof(buffer_data));
                    memcpy(&data, buffer_data, sizeof(data));

                    std::string result_str;
                    result_str += std::string   ("     A_X ") +
                            std::string   ("     A_Y ") +
                            std::string   ("     A_Z ") +
                            std::string   ("     Tstr ") +
                            std::string   ("         G_X ") +
                            std::string   ("     G_Y ") +
                            std::string   ("     G_Z ") + "\n";
                    // info
                    result_str += "  "    + std::to_string(data.Accelerometer_X  ) +
                            "     " + std::to_string(data.Accelerometer_Y  ) +
                            "     " + std::to_string(data.Accelerometer_Z  ) +
                            "  "   + std::to_string((data.Temperature/340 + 36.53)) +
                            "     " + std::to_string(data.Gyroscope_X      ) +
                            "     " + std::to_string(data.Gyroscope_Y      ) +
                            "     " + std::to_string(data.Gyroscope_Z      ) + "\n";
                    std::cout << result_str;
                    break;
                }
                case UART_REQUEST_SEND_MPU6050_PACKET:
                {
                    serial.read(buffer_packet,sizeof(buffer_packet));
                    memcpy(&packet, buffer_packet, sizeof(packet));

                    std::string result_str;
                    result_str += std::string   ("     A_X ") +
                            std::string   ("     A_Y ") +
                            std::string   ("     A_Z ") +
                            std::string   ("     Tstr ") +
                            std::string   ("         G_X ") +
                            std::string   ("     G_Y ") +
                            std::string   ("     G_Z ") + "\n";
                    // info
                    result_str += "  "    + std::to_string(packet.data.Accelerometer_X  ) +
                            "     " + std::to_string(packet.data.Accelerometer_Y  ) +
                            "     " + std::to_string(packet.data.Accelerometer_Z  ) +
                            "  " +   std::to_string((packet.data.Temperature/340 + 36.53)) +
                            "     " + std::to_string(packet.data.Gyroscope_X      ) +
                            "     " + std::to_string(packet.data.Gyroscope_Y      ) +
                            "     " + std::to_string(packet.data.Gyroscope_Z      ) + "\n";
                    std::cout << result_str;
                    break;
                }
            default:
                break;
            }

        }

        serial.close();

     } catch(boost::system::system_error& e)
    {
        cout<<"Error: "<<e.what()<<endl;
        return 1;
    }
}
#endif

#ifdef USE_TIMEOUT_SERIAL_READ_MPU6050_DATA_PACKET
int main(void)
{
    using namespace std;
    using namespace boost;
    try {

        TimeoutSerial serial("COM3",115200);
        serial.setTimeout(posix_time::seconds(10));

        UART_Packet_t packet;
        MPU6050_Data_Reg_t data;
        char byte;
        char data_buffer[sizeof(MPU6050_Data_Reg_t)];
        bool packet_received_ok = false;
        while(serial.isOpen())
        {
            // read header
            serial.read(&byte,1);
            if(byte == UART_PACKET_START)
            {
                serial.read(&byte,1);
                if(byte == sizeof(MPU6050_Data_Reg_t))
                {
                    // read data
                    serial.read(data_buffer,sizeof(data_buffer));
                    memcpy(&data, data_buffer, sizeof(data));
                    std::cout << construct_mpu6050_data_str(&(data)) << std::endl;

                    // read remaining
                    serial.read(&byte,1);
                    if(byte == sizeof(MPU6050_Data_Reg_t))
                    {
                        serial.read(&byte,1);
                        if(byte == UART_PACKET_END)
                        {
                            packet_received_ok = true;
                        }
                    }
                    else
                    {
                        serial.flush();
                    }
                }
                else
                {
                    serial.flush();
                }
            }
            else
            {
                serial.flush();
            }
        }

        serial.close();

    } catch(boost::system::system_error& e)
    {
        cout<<"Error: "<<e.what()<<endl;
        return 1;
    }
}
#endif

#ifdef USE_TIMEOUT_SERIAL_READ_MPU6050_DATA
int main(void)
{    using namespace std;
     using namespace boost;
     try {

        TimeoutSerial serial("COM3",115200);
        serial.setTimeout(posix_time::seconds(1));

        UART_Packet_t packet;
        MPU6050_Data_Reg_t data;
        char databuf[sizeof(MPU6050_Data_Reg_t)];
        while(serial.isOpen())
        {
            // read header
            serial.read(databuf,sizeof(databuf));
            memcpy(&data, databuf, sizeof(data));
            std::cout << construct_mpu6050_data_str(&(data)) << std::endl;
        }

        serial.close();

     } catch(boost::system::system_error& e)
    {
        cout<<"Error: "<<e.what()<<endl;
        return 1;
    }
}
#endif









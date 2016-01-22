#include "ir_sensor.hpp"

void clearscreen();

boost::shared_ptr<DataFrame_t> create_data_frame(uint8_t id, uint8_t angle)
{
    boost::shared_ptr<DataFrame_t> data_ptr(new DataFrame_t());
    data_ptr->_1_beamer_id = id;
    data_ptr->_2_angle_code = angle;
    data_ptr->_3_angle_code_rev = ~angle;

    return data_ptr;
}

int ir_sensor_main()
{
    uint8_t command;
    char response;
    DataFrame_t data;

    size_t command_size = sizeof(command);
    size_t data_size = sizeof(data);

    try {

        std::string port_name;
#ifdef __linux
        port_name = "/dev/ttyUSB0";
        //port_name = "/dev/ttyACM0";
#elif  WIN32
        port_name = "COM7";
#endif
        int baud_rate = 115200;
        boost::shared_ptr<TimeoutSerial> serial(new TimeoutSerial());
        serial->setTimeout(boost::posix_time::seconds(1000));

        TRY_CATCH_EXIT(serial->open(port_name,baud_rate));
        if(!serial->isOpen())
        {
            std::cout << "Couldn't open serial port " + port_name << std::endl;
            return -1;
        }
        std::cout << "Opened serial port " << std::endl;

        // start clock
        auto time_start = std::chrono::high_resolution_clock::now();

        // communication
        while(serial->isOpen())
        {
            // send command (should be done ragardless of the command type)
            command = UART_DEBUG_DATA_TRANSMIT;//UART_ECHO;
            serial->write((char*)&command, command_size);
            //break;// for debugging, TODO: remove

            // after sending command, do command specific actions
            switch(command)
            {
                case UART_DEBUG_DATA_TRANSMIT:
                {
                    // 1. generate random data
                    // TODO
                    boost::shared_ptr<DataFrame_t> data_frame = create_data_frame(0b11111111,0b11111111);
                    // 2. send it to transmitter
                    serial->write((char*)data_frame.get(), data_size);

                    // 3. get data from receiver

                    // 4. compare original data with received one

                    break;
                }
                default:
                {
                    // todo: error handling
                    break;
                }
            }
        }

        // measure time
        auto time_finish = std::chrono::high_resolution_clock::now();
        std::cout << "Time elapsed: "
                  << std::chrono::duration_cast<std::chrono::hours>(time_start - time_finish).count() <<"h "
                  << std::chrono::duration_cast<std::chrono::minutes>(time_start - time_finish).count() <<"m "
                  << std::chrono::duration_cast<std::chrono::seconds>(time_start - time_finish).count() <<"s "
                  << std::endl;

        serial->close();

    }
    catch(boost::system::system_error& e)
    {
        //std::string err_msg = std::string(e.what());
        //std::cout<<"Error: " << err_msg <<endl;
        std::cout<<"Error: exception occured" << std::endl;
        getchar();
        return 1;
    }
}

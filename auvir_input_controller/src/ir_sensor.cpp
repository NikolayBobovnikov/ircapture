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
    uint8_t response;
    DataFrame_t data;
    USART_msg_t usart_msg;

    size_t command_size = sizeof(command);
    size_t data_size = sizeof(data);
    size_t uart_msg_size = sizeof(usart_msg);

    size_t total_number_of_trials = 100;
    size_t current_trial_number = 0;
    size_t number_of_errors = 0;

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
            //serial->write((char*)&command, command_size);

            //break;// for debugging, TODO: remove

            // after sending command, do command specific actions
            switch(command)
            {
                case UART_DEBUG_DATA_TRANSMIT:
                {
                    // 1. generate random data
                    // TODO
                    //tx_data_frame._1_beamer_id = 0b10101010;
                    //tx_data_frame._2_angle_code = 0b10101010;
                    uint8_t beam_id = 0b10101010;
                    uint8_t angle = 0b10101010;
                    boost::shared_ptr<DataFrame_t> data_frame = create_data_frame(beam_id,angle);

                    // 2. send it to transmitter
                    //serial->write((char*)data_frame.get(), data_size);
                    usart_msg._ir_hub_id = 1;
                    usart_msg._ir_sensor_id = 2;
                    usart_msg.data._1_beamer_id = data_frame->_1_beamer_id;
                    usart_msg.data._2_angle_code = data_frame->_2_angle_code;
                    usart_msg.data._3_angle_code_rev = data_frame->_3_angle_code_rev;
                    serial->write((char*)&usart_msg, data_size);

                    current_trial_number++;

                    // 3. get responce (what has been received by IR sensor)
                    //serial->read((char*)&usart_msg, sizeof(usart_msg));
                    /*
                    std::cout << "hub id: " << std::to_string(usart_msg._ir_hub_id) << " ; " <<
                                 "sensor id: " << std::to_string(usart_msg._ir_sensor_id) << " ; " <<
                                 "beamer id: " << std::to_string(usart_msg.data._1_beamer_id) << " ; " <<
                                 "angle: " << std::to_string(usart_msg.data._2_angle_code) << " ; " <<
                              std::endl;
                              */

                    // 4. compare original data with received one
                    if( beam_id != usart_msg.data._1_beamer_id || angle != usart_msg.data._2_angle_code){
                        number_of_errors++;
                    }

                    // 5. report
                    if(current_trial_number == total_number_of_trials){
                        size_t persentage_of_errors = number_of_errors * 100 / total_number_of_trials;
                        std::cout << "% of errors: " << std::to_string(persentage_of_errors) << std::endl;

                        current_trial_number = 0;
                        number_of_errors = 0;
                    }
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

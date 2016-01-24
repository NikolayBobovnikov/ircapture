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

enum UART_Commands {
    UART_COMMAND_NOT_RECEIVED = 0,
    UART_DEBUG_DATA_TRANSMIT,
    UART_DEBUG_DATA_TRANSMIT_OK,
    UART_ECHO
};

typedef struct
{
    uint8_t _1_beamer_id;
    uint8_t _2_angle_code;
    uint8_t _3_angle_code_rev;
} DataFrame_t;

typedef struct
{
    uint8_t _ir_hub_id;
    uint8_t _ir_sensor_id;
    DataFrame_t data;
} USART_msg_t;

boost::shared_ptr<DataFrame_t> create_data_frame(uint8_t id, uint8_t angle);
int ir_sensor_main();




#include <iostream>
#include <chrono>
#include <thread>

//#include "imu_sensor.hpp" TODO: combine with IR
#include "ir_sensor.hpp"

//TODO: learn this stuff (passing functions as arguments)
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
void clearscreen()
{
#ifdef __linux
        std::system("clear");
#elif  WIN32
        std::system("cls");
#endif
}


int main(void)
{
    while(true)
    {
        ir_sensor_main();
    }
    return 1;
}


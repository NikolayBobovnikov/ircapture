#include <iostream>
#include <chrono>
#include <thread>

#include "imu_sensor.hpp"

int imu_sensor_main();
int ir_sensor_main();

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
    ir_sensor_main();
    return 1;
}


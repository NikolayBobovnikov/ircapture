#include "serial_port.hpp"

int simple_read()
{
    std::cout << "this is input controller project" << std::endl;
    try
    {
        std::string port_name = "/dev/ttyUSB0";
        //std::string port_name = "COM4";

        SimpleSerial serial(port_name,115200);
        std::cout << "Opened " + port_name << std::endl;
        std::cout << "MPU6050_data_t: " + std::to_string(sizeof(MPU6050_data_t)) << std::endl;

        while(1)
        {
            std::cout << serial.get_mpu6050_data_str() << std::endl;
            //std::cout << serial.readLine() << std::endl;
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

int main(void)
{
#ifdef THIS_IS_SKIPPED
    simple_read();
#endif
    try
    {
        boost::asio::io_service io_service;
        auto_ptr<boost::asio::io_service::work> work(new boost::asio::io_service::work(io_service));
            // to prevent io_service to stop if no work left

        SerialPort serial_port(io_service, 115200, "/dev/ttyUSB0");
        boost::thread thread_motor(boost::bind(&boost::asio::io_service::run, &io_service));

        serial_port.do_read();
        cin.get();

        serial_port.close(); // close the serial port
        work.reset(); // wait for handlers to finish normally. io_service stops when no more work to do.
        thread_motor.join(); // delete the current thread

    }
    catch (std::exception& e)
    {
        std::cerr << "Exception caught: " << e.what() << '\n';
    }

    cout <<"Press any key to exit..."<<endl;
    cin.get();
    return 0;
}


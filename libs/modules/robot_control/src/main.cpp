#include <iostream>
#include <soem/ethercat.h>

auto main() -> int
{
    std::cout << "Hello, World!" << std::endl;

    // Initialise the EtherCAT master
    if (ec_init("enp0s31f6") > 0)
    {
        std::cout << "EtherCAT master initialised successfully" << std::endl;
    }
    else
    {
        std::cout << "Failed to initialise EtherCAT master" << std::endl;
    }
    return 0;
}
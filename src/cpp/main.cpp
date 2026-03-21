#include "main.h"

PicoContainer pico_container{};

int main()
{
    if (!Constants::TEST_MODE)
    {
        multicore_launch_core1([]() { pico_container.core2_loop(); });

        pico_container.main_loop();
    }
    else
    {          
        pico_container.test();
    }

    return 0;
}
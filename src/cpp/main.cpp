#include "main.h"

// For more examples of I2C use see https://github.com/raspberrypi/pico-examples/tree/master/i2c

PicoContainer pico_container{};

int main()
{
    stdio_init_all(); // Initialize all standard IO (for USB communication)

    // multicore not available for this target currently; run main loop on core0
    // If multicore is supported later, restore multicore_launch_core1 usage.
    multicore_launch_core1([]() { pico_container.core2_loop(); });

    while (true)
    {
        pico_container.main_loop();
    }


    return 0;
}
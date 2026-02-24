#include "main.h"

// For more examples of I2C use see https://github.com/raspberrypi/pico-examples/tree/master/i2c

PicoContainer pico_container{};

int main()
{
    stdio_init_all(); // Initialize all standard IO (for USB communication)

    multicore_launch_core1([]() { pico_container.core2_loop(); });
    
    pico_container.main_loop();


    return 0;
}
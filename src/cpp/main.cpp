#include "main.h"

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9

// For more examples of I2C use see https://github.com/raspberrypi/pico-examples/tree/master/i2c

int main()
{
    PicoContainer pico_container{};

    while (true)
    {
        pico_container.main_loop();
    }

    return 0;
}

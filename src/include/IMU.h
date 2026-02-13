#pragma once

#pragma region Includes
#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "Constants.h"
#pragma endregion

class IMU
{
    public:
        explicit IMU(uint8_t i2c_address);
        void     read_accelerometer(float *x, float *y, float *z);
        void     read_gyroscope(float *x, float *y, float *z);
    private:
        uint8_t address;
};
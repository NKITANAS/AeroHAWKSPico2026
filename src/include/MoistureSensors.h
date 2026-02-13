#pragma once

#pragma region Includes
#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "Constants.h"
#pragma endregion

class SoilMoistureSensor
{
    public:
        explicit SoilMoistureSensor(int pin);
        float    read_moisture();
    private:

};
#pragma once

#pragma region Includes
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"

#include "Constants.h"
#pragma endregion

class SoilMoistureSensor
{
    public:
        explicit SoilMoistureSensor(int pin, int adc_input);
        void  init();
        float read_moisture(); // Returns moisture as a percentage (0.0 = dry, 100.0 = wet)
    private:
        int m_adc_channel;
        int m_pin_number;
};
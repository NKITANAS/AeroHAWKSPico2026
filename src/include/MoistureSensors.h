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
        uint16_t read_moisture();
    private:
        int m_adc_channel;
        int m_pin_number;
};
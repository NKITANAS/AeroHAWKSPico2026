#include "MoistureSensors.h"

#pragma region Constructor
/// @brief Intialises the soil moisture sensor on the specified pin. The pin will be set as analog input.
/// @param pin The GPIO pin number to which the soil moisture sensor is connected.
/// @param adc_input The ADC channel number corresponding to the GPIO pin (0-3 for GPIO26-29).
SoilMoistureSensor::SoilMoistureSensor(int pin, int adc_input)
{
    // Store the pin number and ADC channel for later use
    m_pin_number = pin;
    m_adc_channel = adc_input;
}
#pragma endregion

#pragma region Init
/// @brief Initializes the soil moisture sensor.
void SoilMoistureSensor::init()
{
    // Set the pin as an input
    adc_gpio_init(m_pin_number); 
}

#pragma region Read Moisture
/// @brief Reads the moisture level from the soil moisture sensor.
/// @return Moisture as a percentage: 0.0 (dry) to 100.0 (wet).
float SoilMoistureSensor::read_moisture()
{
    adc_select_input(m_adc_channel);
    return (adc_read() / 4095.0f) * 100.0f;
}
#pragma endregion
#pragma once

#pragma region Includes
#include <cmath>
#pragma endregion

namespace Constants
{
    // Conversion factor for ADC readings to voltage (assuming a 12-bit ADC and 3.3V reference)
    constexpr auto CONVERSION_FACTOR             = 3.3f / (1 << 12); 

    // IMU
    constexpr auto IMU_I2C_ADDRESS               = 0x68;       // I2C address for the MPU6050 IMU sensor
    constexpr auto IMU_SDA_PIN                   = 4;          // GPIO pin number for I2C SDA
    constexpr auto IMU_SCL_PIN                   = 5;          // GPIO pin number for I2C SCL

    // Moisture Sensors
    constexpr auto MOISTURE_SENSOR_1_PIN         = 26;         // GPIO pin number for soil moisture sensor 1 (ADC channel 0)
    constexpr auto MOISTURE_SENSOR_1_ADC_CHANNEL = 0;          // ADC channel for soil moisture sensor 1 (GPIO26)
    constexpr auto MOISTURE_SENSOR_2_PIN         = 27;         // GPIO pin number for soil moisture sensor 2 (ADC channel 1)
    constexpr auto MOISTURE_SENSOR_2_ADC_CHANNEL = 1;          // ADC channel for soil moisture sensor 2 (GPIO27)

    // Altimeter
    constexpr auto ALTIMITER_I2C_ADDRESS         = 0x77;       // I2C address for the BMP280 altimeter sensor
    constexpr auto ALTIMITER_SDA_PIN             = 8;          // GPIO pin number for I2C SDA
    constexpr auto ALTIMITER_SCL_PIN             = 9;          // GPIO pin number for I2C SCL
    constexpr auto GAS_CONSTANT                  = 8.3144598f; // Universal gas constant in J/(mol*K)
    constexpr auto GRAVITY                       = 9.81f;      // Acceleration due to gravity in m/s^2

    // Stepper Motor
    constexpr auto STEPPER_DIR_PIN               = 10;         // GPIO pin number for stepper motor direction control
    constexpr auto STEPPER_STEP_PIN              = 11;         // GPIO pin number for stepper motor step control
    constexpr auto STEPPER_ENABLE_PIN            = 12;         // GPIO pin number for stepper motor enable control (active LOW)
    constexpr auto STEPPER_SLEEP_PIN             = 13;         // GPIO pin number for stepper motor sleep control (active HIGH)

    // Stepper Locations
    constexpr auto STEPPER_WINDOW1               = 0;          // Stepper position for window 1
    constexpr auto STEPPER_WINDOW2               = 50;         // Stepper position for window 2
    constexpr auto STEPPER_WINDOW3               = 50;         // Stepper position for window 3
    constexpr auto STEPPER_WINDOW4               = 0;          // Stepper position for window 4


    // Linear Actuator
    constexpr auto ACTUATOR_1_PIN_1              = 19;         // GPIO pin number for linear actuator 1 control pin 1
    constexpr auto ACTUATOR_1_PIN_2              = 20;         // GPIO pin number for linear actuator 1 control pin 2
    constexpr auto ACTUATOR_2_PIN_1              = 21;         // GPIO pin number for linear actuator 2 control pin 1
    constexpr auto ACTUATOR_2_PIN_2              = 22;         // GPIO pin number for linear actuator 2 control pin 2

}
#pragma once

#pragma region Includes
#include <cmath>
#pragma endregion

namespace Constants
{
    // TEST MODE: Run test() rather than main_loop() to test individual components without flight logic
    constexpr auto TEST_MODE                      = false;      // Set to true to run the test() function instead of the main flight loop, for testing individual components without flight logic
    constexpr auto DEMONSTRATION_MODE             = false;      // Defunct, no rocket fair
    
    constexpr auto LORA_HAT_MODE                  = true;       // Enable if using LORA directly with the pico
    constexpr auto HARDWARE_SAVE                  = true;       // Enable to save data to flash memory (for post-flight analysis, not real-time)
    
    // Conversion factor for ADC readings to voltage (assuming a 12-bit ADC and 3.3V reference)
    constexpr auto CONVERSION_FACTOR             = 3.3f / (1 << 12); 

    // IMU
    constexpr auto IMU_I2C_ADDRESS               = 0x68;       // I2C address for the MPU6050 IMU sensor
    constexpr auto IMU_SDA_PIN                   = 14;         // GPIO pin number for I2C SDA
    constexpr auto IMU_SCL_PIN                   = 15;         // GPIO pin number for I2C SCL
    constexpr auto IMU_CONFIGURED_BEFORE_ACCEL   = true;       // Apparently the pull ups are fighting each other, causing corrupt data. They dont need to be in 2 places

    // Moisture Sensors
    constexpr auto MOISTURE_SENSOR_1_PIN         = 26;         // GPIO pin number for soil moisture sensor 1 (ADC channel 0)
    constexpr auto MOISTURE_SENSOR_1_ADC_CHANNEL = 0;          // ADC channel for soil moisture sensor 1 (GPIO26)
    constexpr auto MOISTURE_SENSOR_2_PIN         = 27;         // GPIO pin number for soil moisture sensor 2 (ADC channel 1)
    constexpr auto MOISTURE_SENSOR_2_ADC_CHANNEL = 1;          // ADC channel for soil moisture sensor 2 (GPIO27)

    // Altimeter
    constexpr auto ALTIMITER_I2C_ADDRESS         = 0x77;       // I2C address for the BMP280 altimeter sensor
    constexpr auto ALTIMITER_SDA_PIN             = 12;         // GPIO pin number for I2C SDA
    constexpr auto ALTIMITER_SCL_PIN             = 13;         // GPIO pin number for I2C SCL
    constexpr auto GAS_CONSTANT                  = 8.3144598f; // Universal gas constant in J/(mol*K)
    constexpr auto GRAVITY                       = 9.81f;      // Acceleration due to gravity in m/s^2

    // Angle Constants
    constexpr auto MIN_ANGLE_SIDE_1              = 43;         // Start of servo reach window (degrees)
    constexpr auto MAX_ANGLE_SIDE_1              = 98;         // End of servo reach window (43 + 55 degrees of travel)
    constexpr auto ANGLE_SHIFT                   = 50;         // Degrees to shift the x and y axis by when calculating angle

    // Stepper Locations
    constexpr auto STEPPER_WINDOW1               = 0;          // Stepper position for window 1
    constexpr auto STEPPER_WINDOW2               = 50;         // Stepper position for window 2
    constexpr auto STEPPER_WINDOW3               = 50;         // Stepper position for window 3
    constexpr auto STEPPER_WINDOW4               = 0;          // Stepper position for window 4


    // Linear Actuator
    constexpr auto ACTUATOR_1_PIN_1              = 18;         // GPIO pin number for linear actuator 1 control pin 1
    constexpr auto ACTUATOR_1_PIN_2              = 19;         // GPIO pin number for linear actuator 1 control pin 2
    constexpr auto ACTUATOR_2_PIN_1              = 20;         // GPIO pin number for linear actuator 2 control pin 1
    constexpr auto ACTUATOR_2_PIN_2              = 21;         // GPIO pin number for linear actuator 2 control pin 2

    // Flash
    constexpr auto FLASH_EXTRACT_PIN             = 22;         // GPIO pin number for flash memory extraction (if using hardware save)
    constexpr auto FLASH_OFFSET                  = 0x380000;

    // Servo Motor(if used)
    constexpr auto SERVO_PWM_PIN                 = 2;         // GPIO pin number for servo motor PWM control

    constexpr auto USE_UART                      = true;      // Set to true to enable UART communication (for debugging or additional sensors)
    constexpr auto TRANSMIT_ONLY                 = false;     // Set to true to skip all flight logic and only transmit collected sensor data every loop iteration

}
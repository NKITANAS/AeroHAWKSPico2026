#pragma once

#pragma region Includes
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "Constants.h"
#pragma endregion

class Altimeter
{
    public:
        explicit Altimeter(uint8_t i2c_address, int sda_pin, int scl_pin, i2c_inst_t *i2c_port = i2c0);
        void     init();
        void     read_altitude(float *altitude, float temperature);
    private:
        // I2C addresses for altimeter sensor
        uint8_t     m_address;               // I2C address of the altimeter sensor
        uint8_t     m_altitude_start = 0xF7; // Starting register for altitude data in BMP280
        uint8_t     m_altitude_end   = 0xF9; // Ending register for altitude data in BMP280
        // GPIO pins for I2C communication
        int         m_sda_pin;               // GPIO pin number for I2C SDA
        int         m_scl_pin;               // GPIO pin number for I2C SCL

        i2c_inst_t *i2c_port;                // I2C port to use for communication

        // Altitude calculation
        float m_initial_pressure; // Initial pressure at sea level for altitude calculation
        float m_current_pressure; // Current pressure reading from the sensor

        void  read_pressure(float *pressure);
};
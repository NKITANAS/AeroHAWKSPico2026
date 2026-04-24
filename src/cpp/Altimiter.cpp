#include "Altimiter.h"
#include <cstdio>

#pragma region Constructor
/// @brief Initializes the altimeter with the specified I2C address.
/// @param i2c_address The I2C address of the altimeter sensor.
/// @param sda_pin The GPIO pin number for I2C SDA.
/// @param scl_pin The GPIO pin number for I2C SCL.
/// @param i2c_port The I2C port to use (default is i2c0, as i2c1 will be slave to the pi).
Altimeter::Altimeter(uint8_t i2c_address, int sda_pin, int scl_pin, i2c_inst_t *i2c_port)
{
    m_address = i2c_address;
    m_sda_pin = sda_pin;
    m_scl_pin = scl_pin;
    this->i2c_port = i2c_port;

}
#pragma endregion

#pragma region Init
/// @brief Initializes the altimeter sensor.
void Altimeter::init()
{
    i2c_init(i2c_port, 400*1000); // Initialize I2C at 400 kHz
    gpio_set_function(m_sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(m_scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(m_sda_pin);
    gpio_pull_up(m_scl_pin);
}

#pragma region Read Altitude
/// @brief Reads the altitude data from the altimeter sensor.
/// @param altitude Pointer to store the altitude value in meters.
/// @param temperature The current temperature in Kelvin (used for altitude calculation).
void Altimeter::read_altitude(float *altitude, float temperature)
{
    read_pressure(&m_current_pressure);
    if (m_initial_pressure == 0.0f)
        m_initial_pressure = m_current_pressure; // Set initial pressure on first valid read
    if (m_initial_pressure == 0.0f || m_current_pressure == 0.0f) {
        *altitude = 0.0f;
        return;
    }
    float temp_kelvin = temperature + 273.15f; // Convert Celsius to Kelvin
    *altitude = (Constants::GAS_CONSTANT * temp_kelvin) / (Constants::GRAVITY) * std::log(m_initial_pressure / m_current_pressure);
}

#pragma region Read Pressure
/// @brief Reads the pressure data from the altimeter sensor.
/// @param pressure Pointer to store the pressure value in hPa.
/// @return The pressure value in hPa.
void Altimeter::read_pressure(float *pressure)
{
    uint8_t pressure_data[3];
    uint8_t reg = m_altitude_start; // Starting register for pressure data
    constexpr uint I2C_TIMEOUT_US = 10000; // 10ms timeout - aborts on bus lockup
    if (i2c_write_timeout_us(i2c_port, m_address, &reg, 1, true,  I2C_TIMEOUT_US) < 0) return;
    if (i2c_read_timeout_us (i2c_port, m_address, pressure_data, 3, false, I2C_TIMEOUT_US) < 0) return;
    // Combine the 3 bytes into a single 20-bit value (BMP280 pressure data is 20 bits across 3 registers)
    uint32_t raw_pressure = ((uint32_t)pressure_data[0] << 12) | ((uint32_t)pressure_data[1] << 4) | (pressure_data[2] >> 4);
    // Convert raw pressure to hPa (BMP280 datasheet specifies that the raw value needs to be divided by 256 to get pressure in hPa)
    *pressure = raw_pressure / 256.0f;
}
#pragma endregion
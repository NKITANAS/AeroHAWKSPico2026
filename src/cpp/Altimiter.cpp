#include "Altimiter.h"

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
    m_initial_pressure ? : m_initial_pressure = m_current_pressure; // Set initial pressure if not already set
    *altitude = (Constants::GAS_CONSTANT * temperature) / (Constants::GRAVITY) * std::log(m_initial_pressure / m_current_pressure);
}

#pragma region Read Pressure
/// @brief Reads the pressure data from the altimeter sensor.
/// @param pressure Pointer to store the pressure value in hPa.
/// @return The pressure value in hPa.
void Altimeter::read_pressure(float *pressure)
{
    uint8_t pressure_data[3]; // 3 bytes for pressure data (20-bit value)
    i2c_read_blocking(i2c_port, m_address, pressure_data, 3, false);
    // Convert the raw data to hPa (assuming a 20-bit value for pressure)
    *pressure = ((pressure_data[0] << 16) | (pressure_data[1] << 8) | pressure_data[2]) / 256.0f;
}
#pragma endregion
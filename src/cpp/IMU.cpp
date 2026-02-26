#include "IMU.h"

#pragma region Constructor
/// @brief Initializes the IMU with the specified I2C address.
/// @param i2c_address The I2C address of the IMU sensor.
/// @param sda_pin The GPIO pin number for I2C SDA.
/// @param scl_pin The GPIO pin number for I2C SCL.
/// @param i2c_port The I2C port to use (default is i2c0, as i2c1 will be slave to the pi).
IMU::IMU(uint8_t i2c_address, int sda_pin, int scl_pin, i2c_inst_t *i2c_port)
{
    m_address = i2c_address;
    m_sda_pin = sda_pin;
    m_scl_pin = scl_pin;
    this->i2c_port = i2c_port;
}
#pragma endregion

#pragma region Init
/// @brief Initializes the IMU sensor by waking it up from sleep mode.
void IMU::init()
{
    // init pins
    gpio_set_function(m_sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(m_scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(m_sda_pin);
    gpio_pull_up(m_scl_pin);
    // Wake up the MPU6050 by writing 0 to the power management register (0x6B)
    uint8_t wake_command[2] = {0x6B, 0x00};
    i2c_write_blocking(i2c_port, m_address, wake_command, 2, false);
    // Set accelerometer full-scale range to +-16g (AFS_SEL = 0b11 -> 0x18 to register 0x1C)
    uint8_t accel_config[2] = {0x1C, 0x18};
    i2c_write_blocking(i2c_port, m_address, accel_config, 2, false);
}

#pragma region Read MPUerometer
/// @brief Reads the accelerometer data from the IMU sensor.
/// @param x Pointer to store the X-axis acceleration value.
/// @param y Pointer to store the Y-axis acceleration value.
/// @param z Pointer to store the Z-axis acceleration value.
void IMU::read_accelerometer(float *x, float *y, float *z)
{
    uint8_t accel_data[6]; // 6 bytes for X, Y, Z (2 bytes each)
    i2c_read_blocking(i2c_port, m_address, accel_data, 6, false);
    // Convert the raw data to m/s^2 (assuming a sensitivity of 16384 LSB/g for the MPU6050)
    *x = (int16_t)((accel_data[0] << 8) | accel_data[1]) / MPUConstants::ACCEL_SENSITIVITY * MPUConstants::GRAVITY;
    *y = (int16_t)((accel_data[2] << 8) | accel_data[3]) / MPUConstants::ACCEL_SENSITIVITY * MPUConstants::GRAVITY;
    *z = (int16_t)((accel_data[4] << 8) | accel_data[5]) / MPUConstants::ACCEL_SENSITIVITY * MPUConstants::GRAVITY;
}
#pragma endregion

#pragma region Read Velocity
/// @brief Integrates accelerometer data over time to compute velocity in m/s.
/// @param vx Pointer to store the X-axis velocity (m/s).
/// @param vy Pointer to store the Y-axis velocity (m/s).
/// @param vz Pointer to store the Z-axis velocity (m/s).
void IMU::read_velocity(float *vx, float *vy, float *vz)
{
    float ax, ay, az;
    read_accelerometer(&ax, &ay, &az);

    uint64_t now = time_us_64();
    if (m_last_time_us != 0)
    {
        float dt = (now - m_last_time_us) * 1e-6f; // microseconds to seconds
        m_vx += ax * dt;
        m_vy += ay * dt;
        m_vz += az * dt;
    }
    m_last_time_us = now;

    *vx = m_vx;
    *vy = m_vy;
    *vz = m_vz;
}

/// @brief Resets the integrated velocity state to zero.
void IMU::reset_velocity()
{
    m_vx = 0.0f;
    m_vy = 0.0f;
    m_vz = 0.0f;
    m_last_time_us = 0;
}
#pragma endregion

#pragma region Read Gyroscope
/// @brief Reads the gyroscope data from the IMU sensor.
/// @param x Pointer to store the X-axis angular velocity value.
/// @param y Pointer to store the Y-axis angular velocity value.
/// @param z Pointer to store the Z-axis angular velocity value.
void IMU::read_gyroscope(float *x, float *y, float *z)
{
    uint8_t gyro_data[6]; // 6 bytes for X, Y, Z (2 bytes each)
    i2c_read_blocking(i2c_port, m_address, gyro_data, 6, false);
    // Convert the raw data to degrees per second (assuming a sensitivity of 131 LSB/deg/s for the MPU6050)
    *x = (int16_t)((gyro_data[0] << 8) | gyro_data[1]) / 131.0f;
    *y = (int16_t)((gyro_data[2] << 8) | gyro_data[3]) / 131.0f;
    *z = (int16_t)((gyro_data[4] << 8) | gyro_data[5]) / 131.0f;
}
#pragma endregion

#pragma region Read Temperature
/// @brief Reads the temperature data from the IMU sensor.
/// @param temp Pointer to store the temperature value in degrees Celsius.
void IMU::read_temperature(float *temp)
{
    uint8_t temp_data[2]; // 2 bytes for temperature
    i2c_read_blocking(i2c_port, m_address, temp_data, 2, false);
    // Convert the raw data to degrees Celsius (using the formula from the MPU6050 datasheet)
    int16_t raw_temp = (temp_data[0] << 8) | temp_data[1];
    *temp = raw_temp / 340.0f + 36.53f; // Convert to °C
}
#pragma endregion


#pragma once

#pragma region Includes
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "Constants.h"
#pragma endregion

#pragma region MPU Constants
namespace MPUConstants
{
    // REFER TO DATASHEET TO MAKE SENSITIVITY CHANGES: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
    constexpr auto ACCEL_SENSITIVITY = 2048.0f;  // LSB/g for the MPU6050 at +-16g range
    constexpr auto GRAVITY           = 9.81f;    // Acceleration due to gravity in m/s^2
    constexpr auto GYRO_SENSITIVITY  = 131.0f;   // LSB/deg/s for the MPU6050 at +-250deg/s range
}


class IMU
{
    public:
        explicit IMU(uint8_t i2c_address, int sda_pin, int scl_pin, i2c_inst_t *i2c_port = i2c0);
        void     init();
        void     read_accelerometer(float *x, float *y, float *z);
        void     read_velocity(float *vx, float *vy, float *vz);
        void     reset_velocity();
        void     read_gyroscope(float *x, float *y, float *z);
        void     read_temperature(float *temp);
    private:
        // Velocity state (integrated from accelerometer, m/s)
        float       m_vx = 0.0f;
        float       m_vy = 0.0f;
        float       m_vz = 0.0f;
        uint64_t    m_last_time_us = 0;
        // I2C address of the IMU sensor
        uint8_t     m_address;
        // Register addresses for the MPU6050 IMU sensor
        uint8_t     m_accel_start = 0x3B;   // Starting register for accelerometer data in MPU6050
        uint8_t     m_accel_end   = 0x40;   // Ending register for accelerometer data in MPU6050
        uint8_t     m_gyro_start  = 0x43;   // Starting register for gyroscope data in MPU6050
        uint8_t     m_gyro_end    = 0x48;   // Ending register for gyroscope data in MPU6050
        uint8_t     m_temp_start  = 0x41;   // Starting register for temperature data in MPU6050
        uint8_t     m_temp_end    = 0x42;   // Ending register for temperature data in MPU6050
        // GPIO pins for I2C communication
        int         m_sda_pin; // GPIO pin number for I2C SDA
        int         m_scl_pin; // GPIO pin number for I2C SCL
        // I2C port to use for communication
        i2c_inst_t *i2c_port;

};
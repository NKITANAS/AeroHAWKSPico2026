#pragma once

#pragma region Includes
#include <cstdio>
#include <cstring>
#include <optional>

#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "pico/multicore.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"

#include "Constants.h"

#include "IMU.h"
#include "MoistureSensors.h"
#include "Altimiter.h"
#pragma endregion

class PicoContainer
{
    public:
        explicit PicoContainer();
        void     main_loop();
        void     core2_loop();

        volatile float accel_x, accel_y, accel_z; // Accelerometer data in m/s^2
        volatile float gyro_x, gyro_y, gyro_z;    // Gyroscope data in degrees/s
        volatile float temperature;               // Temperature data in degrees Celsius
        volatile float altitude;                  // Altitude data in meters
        volatile float speed_x, speed_y, speed_z; // Velocity derived from accel data
        volatile float orint_x, orint_y, orint_z; // Orientation derived from gyro data
        volatile uint16_t moisture_1, moisture_2;              // Moisture level from sensor 1
    private:
        IMU                m_imu{Constants::IMU_I2C_ADDRESS, Constants::IMU_SDA_PIN, Constants::IMU_SCL_PIN};
        SoilMoistureSensor m_moisture_sensor_1{Constants::MOISTURE_SENSOR_1_PIN, Constants::MOISTURE_SENSOR_1_ADC_CHANNEL};
        SoilMoistureSensor m_moisture_sensor_2{Constants::MOISTURE_SENSOR_2_PIN, Constants::MOISTURE_SENSOR_2_ADC_CHANNEL};
        Altimeter          m_altimeter{Constants::ALTIMITER_I2C_ADDRESS, Constants::ALTIMITER_SDA_PIN, Constants::ALTIMITER_SCL_PIN};

        char m_buffer[64]; // Buffer for USB
        int m_buffer_pos = 0; // Current position in buffer for non-blocking reads

        absolute_time_t    m_old_time;
        absolute_time_t    m_current_time;
};
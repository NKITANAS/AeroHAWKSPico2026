#pragma once

#pragma region Includes
#include <cstdio>
#include <cstring>
#include <optional>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

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

        float accel_x, accel_y, accel_z; // Accelerometer data in m/s^2
        float gyro_x, gyro_y, gyro_z;    // Gyroscope data in degrees/s
        float temperature;               // Temperature data in degrees Celsius
        float altitude;                  // Altitude data in meters
    private:
        std::optional<IMU>                m_imu               = std::nullopt;
        std::optional<SoilMoistureSensor> m_moisture_sensor_1 = std::nullopt;
        std::optional<SoilMoistureSensor> m_moisture_sensor_2 = std::nullopt;
        std::optional<Altimeter>          m_altimeter         = std::nullopt;

        char               m_buffer[64]; // Buffer for USB
};
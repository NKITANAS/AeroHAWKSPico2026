#pragma once

#pragma region Includes
#include <cstdio>
#include <cstring>
#include <optional>

#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "pico/multicore.h"
#include "pico/critical_section.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"

#include "Constants.h"

#include "IMU.h"
#include "MoistureSensors.h"
#include "Altimiter.h"
#include "Stepper.h"
#include "LinearActuator.h"
#include "KalmanFilter.h"
#pragma endregion

enum State
{
    IDLE,
    ASCENT,
    LANDED
};

class PicoContainer
{
    public:
        explicit PicoContainer();
        void     main_loop();
        void     core2_loop();

        volatile float    accel_x, accel_y, accel_z; // Accelerometer data in m/s^2
        volatile float    gyro_x, gyro_y, gyro_z;    // Gyroscope data in degrees/s
        volatile float    temperature;               // Temperature data in degrees Celsius
        volatile float    altitude;                  // Altitude data in meters
        volatile float    speed_x, speed_y, speed_z; // Velocity derived from accel data
        volatile float    orint_x, orint_y, orint_z; // Orientation derived from gyro data
        volatile float    filtered_altitude;         // Kalman-filtered altitude (m)
        volatile float    filtered_velocity;         // Kalman-filtered vertical velocity (m/s)
        volatile uint16_t moisture_1, moisture_2;    // Moisture level from the sensors

        volatile State current_state = IDLE; // Current state of the flight, initialized to IDLE
        
    private:
        IMU                m_imu{Constants::IMU_I2C_ADDRESS, Constants::IMU_SDA_PIN, Constants::IMU_SCL_PIN};
        SoilMoistureSensor m_moisture_sensor_1{Constants::MOISTURE_SENSOR_1_PIN, Constants::MOISTURE_SENSOR_1_ADC_CHANNEL};
        SoilMoistureSensor m_moisture_sensor_2{Constants::MOISTURE_SENSOR_2_PIN, Constants::MOISTURE_SENSOR_2_ADC_CHANNEL};
        Altimeter          m_altimeter{Constants::ALTIMITER_I2C_ADDRESS, Constants::ALTIMITER_SDA_PIN, Constants::ALTIMITER_SCL_PIN};
        Stepper            m_stepper{Constants::STEPPER_DIR_PIN, Constants::STEPPER_STEP_PIN, Constants::STEPPER_ENABLE_PIN, Constants::STEPPER_SLEEP_PIN};
        LinearActuator     m_actuator_1{Constants::ACTUATOR_1_PIN_1, Constants::ACTUATOR_1_PIN_2};
        LinearActuator     m_actuator_2{Constants::ACTUATOR_2_PIN_1, Constants::ACTUATOR_2_PIN_2};
        KalmanFilter                m_kalman_filter; // Kalman filter for altitude/velocity estimation

        char m_buffer[64]; // Buffer for USB
        int  m_buffer_pos = 0; // Current position in buffer for non-blocking reads

        absolute_time_t    m_old_time;
        absolute_time_t    m_current_time;

        critical_section_t m_data_lock; // Critical section for thread-safe access to shared data

        float accel_x_temp, accel_y_temp, accel_z_temp; // Temporary variables for accelerometer data processing
        float gyro_x_temp, gyro_y_temp, gyro_z_temp;    // Temporary variables for gyroscope data processing
        float temperature_temp;                          // Temporary variable for temperature data processing

};
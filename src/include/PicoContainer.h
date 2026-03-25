#pragma once

#pragma region Includes
#include <cstdio>
#include <cstring>
#include <string>
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
#include "Servo.h"
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
        void     test(); // Function to test individual components without flight logic (if TEST_MODE is true)

    private:
        State       current_state = IDLE;           // Current state of the flight, initialized to IDLE
        bool        full_kalman = false;            // Flag to indicate whether to use the full Kalman filter (with barometer) or just accelerometer integration (for testing)
        float       accel_x, accel_y, accel_z;      // Accelerometer data in m/s^2
        float       gyro_x, gyro_y, gyro_z;         // Gyroscope data in degrees/s
        float       temperature;                    // Temperature data in degrees Celsius
        float       altitude;                       // Altitude data in meters
        float       speed_x, speed_y, speed_z;      // Velocity derived from accel data
        float       orint_x, orint_y, orint_z;      // Orientation derived from gyro data
        float       filtered_altitude;              // Kalman-filtered altitude (m)
        float       filtered_velocity;              // Kalman-filtered vertical velocity (m/s)
        uint16_t    moisture_1, moisture_2;         // Moisture level from the sensors
        bool        start_signal_received;          // Flag to indicate if a start signal has been received from the Raspberry Pi
        bool        land_signal_received;           // Flag to indicate if a land signal has been received from the Raspberry Pi
        bool        land_manual_interrupt_recieved; // Flag to indicate if a manual interrupt signal has been received from the Raspberry Pi to stop landing sequence
                 std::string serial_input;                   // Buffer for serial input from Raspberry Pi (if using UART)


        IMU                m_imu{Constants::IMU_I2C_ADDRESS, Constants::IMU_SDA_PIN, Constants::IMU_SCL_PIN};
        SoilMoistureSensor m_moisture_sensor_1{Constants::MOISTURE_SENSOR_1_PIN, Constants::MOISTURE_SENSOR_1_ADC_CHANNEL};
        SoilMoistureSensor m_moisture_sensor_2{Constants::MOISTURE_SENSOR_2_PIN, Constants::MOISTURE_SENSOR_2_ADC_CHANNEL};
        Altimeter          m_altimeter{Constants::ALTIMITER_I2C_ADDRESS, Constants::ALTIMITER_SDA_PIN, Constants::ALTIMITER_SCL_PIN};
        Stepper            m_stepper{Constants::STEPPER_DIR_PIN, Constants::STEPPER_STEP_PIN, Constants::STEPPER_ENABLE_PIN, Constants::STEPPER_SLEEP_PIN};
        LinearActuator     m_actuator_1{Constants::ACTUATOR_1_PIN_1, Constants::ACTUATOR_1_PIN_2};
        LinearActuator     m_actuator_2{Constants::ACTUATOR_2_PIN_1, Constants::ACTUATOR_2_PIN_2};
        Servo              m_servo{Constants::SERVO_PWM_PIN};
        KalmanFilter       m_kalman_filter; // Kalman filter for altitude/velocity estimation

        char m_buffer[64]; // Buffer for USB
        int  m_buffer_pos = 0; // Current position in buffer for non-blocking reads

        absolute_time_t    m_old_time;
        absolute_time_t    m_current_time;
        absolute_time_t    m_ascent_start_time;

        critical_section_t m_data_lock; // Critical section for thread-safe access to shared data

        float accel_x_temp, accel_y_temp, accel_z_temp; // Temporary variables for accelerometer data processing
        float gyro_x_temp, gyro_y_temp, gyro_z_temp;    // Temporary variables for gyroscope data processing
        float temperature_temp;                         // Temporary variable for temperature data processing
        float altitude_temp;                            // Temporary variable for altitude

        std::string get_serial_input();                // Helper function to format sensor data for serial transmission
        

};
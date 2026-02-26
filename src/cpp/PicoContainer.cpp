#include "PicoContainer.h"

#pragma region Constructor
/// @brief Initializes the PicoContainer, which manages the IMU and soil moisture sensor, and handles communication with the Raspberry Pi.
PicoContainer::PicoContainer() 
{
    // init things
    stdio_init_all(); // Initialize all standard IO (for USB communication)
    adc_init();       // Initialize the ADC for soil moisture sensors
    // Initialize the I2C communication for the IMU sensor
    i2c_init(i2c0, 400*1000);
    // Initialize subsystems
    m_imu.init();
    m_moisture_sensor_1.init();
    m_moisture_sensor_2.init();
    m_altimeter.init();

    // Set initial speed and orientation
    speed_x = 0;
    speed_y = 0;
    speed_z = 0;

    orint_x = 0;
    orint_y = 0;
    orint_z = 0;

    filtered_altitude = 0;
    filtered_velocity = 0;

    // Initialize Kalman filter for vertical velocity estimation
    // accel_variance: accelerometer noise (higher = trust accel less)
    // altitude_variance: barometer noise (higher = trust baro less)
    m_kalman_filter.init(0.0f, 1.0f, 2.0f);

    // Find current time
    m_current_time = get_absolute_time();

    critical_section_init(&m_data_lock); // Initialize critical section for thread safety
}
#pragma endregion

#pragma region Main Loop
/// @brief The main loop of the PicoContainer, which continuously reads data from the IMU and soil moisture sensors, and sends it to the Raspberry Pi over I2C.
void PicoContainer::main_loop()
{
    absolute_time_t kalman_last_time = get_absolute_time();
    while (true)
    {
        // Compute dt for Kalman filter using local timestamps (not shared Core 1 timestamps)
        absolute_time_t kalman_now = get_absolute_time();
        float dt = absolute_time_diff_us(kalman_last_time, kalman_now) * 1e-6f;
        kalman_last_time = kalman_now;

        // Run Kalman filter to fuse accelerometer + barometric altitude
        // accel_z is gravity-compensated: subtract gravity (sensor reads ~+9.81 when stationary)
        critical_section_enter_blocking(&m_data_lock);       // Enter critical section to safely read shared data
        float vertical_accel = accel_z - Constants::GRAVITY;
        critical_section_exit(&m_data_lock);                 // Exit critical section

        m_kalman_filter.update(vertical_accel, dt);
        filtered_altitude = m_kalman_filter.get_altitude();
        filtered_velocity = m_kalman_filter.get_velocity();
        
        if (current_state == State::IDLE)
        {
            if (filtered_velocity > 0.5) // Simple condition to detect launch (adjust threshold as needed)
            {
                current_state = State::ASCENT; // Transition to ASCENT state
            }
        }
        else if (current_state == State::ASCENT)
        {


            printf("ALT: %.2f m, VEL: %.2f m/s\n", filtered_altitude, filtered_velocity); // Debug output

            if (filtered_velocity < 0.5 && filtered_velocity > -0.5 && altitude < 10) // Simple condition to detect landing (adjust thresholds as needed)
            {
                sleep_ms(5000);
                current_state = State::LANDED; // Transition to LANDED state
            }
        }
        else if (current_state == State::LANDED)
        {
            // Once landed, determine the payload's orientation using the accelerometer
            // We know that after landing, it will lay flat on it's side. The accelerometer gives a value of 9.8 when still, and they will be only on x and y axis.
            // This can easily be used to determine the optimal window to the ground for the moisture senosor.
            /*
                |  /
                | /
            ____|/___
                |
                |
            */
           if (accel_x > 0 && accel_y > 0)
           {
                // Up Window
                printf("Determined Up Window\n");
                m_stepper.step_forward(Constants::STEPPER_WINDOW1); // Move stepper to window 1 position
                m_actuator_1.extend(); // Extend actuator
                m_moisture_sensor_1.read_moisture(); // Read moisture from sensor 1
                printf("Moisture: %d\n", moisture_1);
           }
           else if (accel_x < 0 && accel_y > 0)
           {
                // Down window
                printf("Determined Down Window\n");
                m_stepper.step_forward(Constants::STEPPER_WINDOW2); // Move stepper to window 2 position
                m_actuator_2.extend(); // Extend actuator 2 to open window 2
                m_moisture_sensor_2.read_moisture(); // Read moisture from sensor 2
                printf("Moisture: %d\n", moisture_2);
           }
           else if (accel_x < 0 && accel_y < 0)
           {
                // Left Window
                printf("Determined Left Window\n");
                m_stepper.step_forward(Constants::STEPPER_WINDOW3); // Move stepper to window 3 position
                m_actuator_1.extend(); // Extend actuator 1 to open window 3
                m_moisture_sensor_1.read_moisture(); // Read moisture from sensor 1
                printf("Moisture: %d\n", moisture_1);
           }
           else if (accel_x > 0 && accel_y < 0)
           {
                // Right Window
                printf("Determined Right Window\n");
                m_stepper.step_forward(Constants::STEPPER_WINDOW4); // Move stepper to window 4 position
                m_actuator_2.extend(); // Extend actuator 2 to open window 4
                m_moisture_sensor_2.read_moisture(); // Read moisture from sensor 2
                printf("Moisture: %d\n", moisture_2);
           }
            
            
        }
        sleep_us(500); // small delay to reduce CPU usage
    }
}
#pragma endregionL stat

#pragma region Core 2 Loop
/// @brief The loop that runs on the second core of the Pico, which will be used to process numbers when implemented
void PicoContainer::core2_loop()
{
    while (true)
    {
        // Read data from the IMU sensor
        m_imu.read_accelerometer(const_cast<float*>(&accel_x_temp), const_cast<float*>(&accel_y_temp), const_cast<float*>(&accel_z_temp));
        m_imu.read_gyroscope(const_cast<float*>(&gyro_x_temp), const_cast<float*>(&gyro_y_temp), const_cast<float*>(&gyro_z_temp));
        m_imu.read_temperature(const_cast<float*>(&temperature_temp));

        // Find the current time and update the old time
        m_old_time     = m_current_time;
        m_current_time = get_absolute_time();

        // Calculate dt in seconds
        float dt = absolute_time_diff_us(m_old_time, m_current_time) * 1e-6f;

        /*
        // Use time to derive speed (simple integration, kept for X/Y axes)
        speed_x += accel_x * dt;
        speed_y += accel_y * dt;
        speed_z += accel_z * dt;

        // For orientation, integrate the gyro data (basic approach, may drift)
        orint_x += gyro_x * dt;
        orint_y += gyro_y * dt;
        orint_z += gyro_z * dt;
        */

        // Write to the shared variables
        critical_section_enter_blocking(&m_data_lock); // Enter critical section to safely update shared data
        accel_x = accel_x_temp;
        accel_y = accel_y_temp;
        accel_z = accel_z_temp;
        gyro_x = gyro_x_temp;
        gyro_y = gyro_y_temp;
        gyro_z = gyro_z_temp;
        temperature = temperature_temp;
        critical_section_exit(&m_data_lock); // Exit critical section
        sleep_us(500); // small delay to reduce CPU usage
    }
}
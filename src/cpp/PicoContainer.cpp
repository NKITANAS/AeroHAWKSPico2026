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
}
#pragma endregion

#pragma region Main Loop
/// @brief The main loop of the PicoContainer, which continuously reads data from the IMU and soil moisture sensors, and sends it to the Raspberry Pi over I2C.
void PicoContainer::main_loop()
{
    while (true)
    {
        // Run Kalman filter to fuse accelerometer + barometric altitude
        // accel_z is gravity-compensated: subtract gravity (sensor reads ~+9.81 when stationary)
        float vertical_accel = accel_z - Constants::GRAVITY;
        float dt = absolute_time_diff_us(m_old_time, m_current_time) * 1e-6f;
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

            if (filtered_velocity < 0.5 && filtered_altitude < 10) // Simple condition to detect landing (adjust thresholds as needed)
            {
                sleep_ms(5000);
                current_state = State::LANDED; // Transition to APOGEE state
            }
        }
        else if (current_state == State::LANDED)
        {
            // Once landed, we can continue reading sensors and sending data, or we can enter a low-power mode if desired
            
        }
    }
}
#pragma endregion

#pragma region Core 2 Loop
/// @brief The loop that runs on the second core of the Pico, which will be used to process numbers when implemented
void PicoContainer::core2_loop()
{
    while (true)
    {
        // Read data from the IMU sensor
        m_imu.read_accelerometer(const_cast<float*>(&accel_x), const_cast<float*>(&accel_y), const_cast<float*>(&accel_z));
        m_imu.read_gyroscope(const_cast<float*>(&gyro_x), const_cast<float*>(&gyro_y), const_cast<float*>(&gyro_z));
        m_imu.read_temperature(const_cast<float*>(&temperature));

        // Find the current time and update the old time
        m_old_time     = m_current_time;
        m_current_time = get_absolute_time();

        // Calculate dt in seconds
        float dt = absolute_time_diff_us(m_old_time, m_current_time) * 1e-6f;

        // Use time to derive speed (simple integration, kept for X/Y axes)
        speed_x += accel_x * dt;
        speed_y += accel_y * dt;
        speed_z += accel_z * dt;

        // For orientation, integrate the gyro data (basic approach, may drift)
        orint_x += gyro_x * dt;
        orint_y += gyro_y * dt;
        orint_z += gyro_z * dt;

        // Read data from the soil moisture sensors
        moisture_1 = m_moisture_sensor_1.read_moisture();
        moisture_2 = m_moisture_sensor_2.read_moisture();
        // Read altitude data from the altimeter
        m_altimeter.read_altitude(const_cast<float*>(&altitude), temperature); 
        sleep_ms(1); // small delay to reduce CPU usage
    }
}
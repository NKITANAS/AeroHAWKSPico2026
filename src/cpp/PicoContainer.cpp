#include "PicoContainer.h"

#pragma region Constructor
/// @brief Initializes the PicoContainer, which manages the IMU and soil moisture sensor, and handles communication with the Raspberry Pi.
PicoContainer::PicoContainer() 
{
    // init things
    stdio_init_all(); // Initialize all standard IO (for USB communication)
    if (Constants::USE_UART)
    {
        // Initialize UART for debugging or additional sensors
        stdio_uart_init_full(uart0, 115200, 0, 1); // uart, baud, tx_pin, rx_pin
    }
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

    start_signal_received = false;

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

        if constexpr (Constants::TRANSMIT_ONLY)
        {
            // Transmit-only mode: skip all flight-stage logic and just broadcast sensor data
            printf("ACCEL: %.2f %.2f %.2f | GYRO: %.2f %.2f %.2f | TEMP: %.2f | ALT: %.2f m | VEL: %.2f m/s\n",
                accel_x, accel_y, accel_z,
                gyro_x,  gyro_y,  gyro_z,
                temperature,
                filtered_altitude, filtered_velocity);
            sleep_us(500);
            continue;
        }

        if (current_state == State::IDLE)
        {
            if (filtered_velocity > 0.5 && start_signal_received) // Simple condition to detect launch (adjust threshold as needed)
            {
                m_ascent_start_time = get_absolute_time(); // Record the time of ascent start
                current_state = State::ASCENT; // Transition to ASCENT state
            }
        }
        else if (current_state == State::ASCENT)
        {

            if (absolute_time_diff_us(m_ascent_start_time, get_absolute_time()) < 30000000) // After 30 seconds of ascent, stop trnasmitting to be able to recieve the landing signal
            {
                printf("ALT: %.2f m, VEL: %.2f m/s\n", filtered_altitude, filtered_velocity); // Debug output
            }
            if (filtered_velocity < 0.5 && filtered_velocity > -0.5 && altitude < 10 && !land_manual_interrupt_recieved) // Simple condition to detect landing (adjust thresholds as needed)
            {
                sleep_ms(5000);
                current_state = State::LANDED; // Transition to LANDED state
            }
        }
        else if (current_state == State::LANDED)
        {
            // Moscow!

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
            float angle      = atan2(accel_y, accel_x) * 360 / M_PI; // Calculate angle needed to rotate to the optimal position
            bool  actuator_1 = false; // Flag to indicate whether to use actuator 1 or 2 for the servo rotation
            // Account for unreachable angles - two ranges: -15-15, 165-195 (adjust as needed based on actual sensor readings and desired orientation). 
            // Other angles are reachable and the servo should be rotated to them
            if (angle > 15 && angle < 165)
            {
                m_servo.angle_servo(angle); // Rotate servo to the calculated angle to orient the payload for optimal moisture sensing, actuator 1
                actuator_1 = true;
            }
            else if (angle > 195 && angle < 345)
            {
                m_servo.angle_servo(angle - 180); // Rotate servo to the calculated angle to orient the payload for optimal moisture sensing, actuator 2
                actuator_1 = false;
            }
            else if (angle > 165 && angle < 180)
            {
                m_servo.angle_servo(160); // Rotate servo to 160 degrees to orient the payload for optimal moisture sensing, actuator 1
                actuator_1 = true;
            }
            else if (angle > 180 && angle < 195)
            {
                m_servo.angle_servo(20); // Rotate servo to 20 degrees to orient the payload for optimal moisture sensing, actuator 2
                actuator_1 = false;
            }
            else if (angle > 345)
            {
                m_servo.angle_servo(160); // Rotate servo to 160 degrees to orient the payload for optimal moisture sensing, actuator 2
                actuator_1 = false;
            }
            else if (angle < 15)
            {
                m_servo.angle_servo(20); // Rotate servo to 20 degrees to orient the payload for optimal moisture sensing, actuator 1
                actuator_1 = true;
            }
            sleep_ms(2000); // Make sure the servo moved and all is fine

            if (actuator_1)
            {
                m_actuator_1.extend(); // Extend the linear actuator to expose the moisture sensor to the ground, actuator 1
                calculate_moisture_1 = true; // Set flag to start calculating moisture for sensor 1 in the core 2 loop
            }
            else
            {
                m_actuator_2.extend(); // Extend the linear actuator to expose the moisture sensor to the ground, actuator 2
                calculate_moisture_2 = true; // Set flag to start calculating moisture for sensor 2 in the core 2 loop
            }
            sleep_ms(2000); // Wait
            current_state = State::TRANSMISSION; // Transition to TRANSMISSION state
        }
        if (current_state == State::TRANSMISSION && !moisture_transmitted)
        {
            constexpr int REQUIRED_SAMPLES = 50;
            if ((int)moisture_readings.size() >= REQUIRED_SAMPLES)
            {
                float sum = 0;
                for (float r : moisture_readings) sum += r;
                float average = sum / moisture_readings.size();
                int sensor_id = calculate_moisture_1 ? 1 : 2;
                printf("Average moisture sensor %d: %.1f%%\n", sensor_id, average);
                moisture_transmitted = true;
            }
        }
        sleep_us(500); // small delay to reduce CPU usage
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
        m_imu.read_accelerometer(const_cast<float*>(&accel_x_temp), const_cast<float*>(&accel_y_temp), const_cast<float*>(&accel_z_temp));
        m_imu.read_gyroscope(const_cast<float*>(&gyro_x_temp), const_cast<float*>(&gyro_y_temp), const_cast<float*>(&gyro_z_temp));
        m_imu.read_temperature(const_cast<float*>(&temperature_temp));

        // Find the current time and update the old time
        m_old_time     = m_current_time;
        m_current_time = get_absolute_time();

        // Calculate dt in seconds
        float dt = absolute_time_diff_us(m_old_time, m_current_time) * 1e-6f;

        std::string msg = get_serial_input();

        // Message Processing
        if      (msg == "LAND")
        {
            // If we receive a "LAND" command from the Raspberry Pi, transition to the LANDED state immediately
            current_state = State::LANDED;
        }
        else if (msg == "STOPLAND")
        {
            // If we receive a "STOPLAND" command from the Raspberry Pi, transition back to the ASCENT state immediately
            current_state = State::ASCENT; 
            land_manual_interrupt_recieved = true; // Set the flag to indicate that a manual interrupt has been received to stop the landing sequence
        }
        else if (msg == "START")
        {
            // If we receive a "START" command from the Raspberry Pi, transition to the ASCENT state immediately
            current_state = State::ASCENT;
        }
        else if (msg == "PING")
        {
            printf("Saw your message!\n");
        }

        if (calculate_moisture_1)
        {
            moisture_1_temp = m_moisture_sensor_1.read_moisture(); // Read moisture level from sensor 1
        }
        if (calculate_moisture_2)
        {
            moisture_2_temp = m_moisture_sensor_2.read_moisture(); // Read moisture level from sensor 2
        }

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
        if (calculate_moisture_1)
        {
            moisture_readings.push_back(moisture_1_temp);
        }
        if (calculate_moisture_2)
        {
            moisture_readings.push_back(moisture_2_temp);
        }
        accel_x      = accel_x_temp;
        accel_y      = accel_y_temp;
        accel_z      = accel_z_temp;
        gyro_x       = gyro_x_temp;
        gyro_y       = gyro_y_temp;
        gyro_z       = gyro_z_temp;
        temperature  = temperature_temp;
        serial_input = msg;
        critical_section_exit(&m_data_lock); // Exit critical section
        sleep_us(500); // small delay to reduce CPU usage
    }
}
#pragma endregion

#pragma region Serial Input
/// @brief Function that listens for messages from the rpi over UART
/// @return Message recieved(if any)(consider using a pointer)
std::string PicoContainer::get_serial_input()
{
    // Non-blocking read: drain all available characters into the buffer
    int ch;
    while ((ch = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT)
    {
        if (ch == '\n' || ch == '\r')
        {
            if (m_buffer_pos > 0)
            {
                // Null-terminate and build string, then reset buffer
                m_buffer[m_buffer_pos] = '\0';
                std::string message(m_buffer, m_buffer_pos);
                m_buffer_pos = 0;
                return message;
            }
            // Skip empty lines
            continue;
        }

        // Guard against buffer overflow
        if (m_buffer_pos < static_cast<int>(sizeof(m_buffer)) - 1)
        {
            m_buffer[m_buffer_pos++] = static_cast<char>(ch);
        }
    }

    // No complete line yet
    return std::string{};
}
#pragma endregion

#pragma region Tests
/// @brief A function to test individual components without flight logic (if TEST_MODE is true)
void PicoContainer::test()
{
    m_servo.angle_servo(180); // Test servo by setting it to 180 degrees
    printf("Set servo to 180 degrees\n");
    sleep_ms(1000);
    m_servo.angle_servo(90);  // Test servo by setting it to 90 degrees
    printf("Set servo to 90 degrees\n");
    sleep_ms(1000);
    m_servo.angle_servo(45);  // Test servo by setting it to 45 degrees
    printf("Set servo to 45 degrees\n");
    sleep_ms(1000);
    m_servo.angle_servo(0);  // Test servo by setting it back to 0 degrees
    printf("Set servo back to 0 degrees\n");
    sleep_ms(1000);
    while (true)
    {
        m_imu.read_accelerometer(const_cast<float*>(&accel_x), const_cast<float*>(&accel_y), const_cast<float*>(&accel_z));
        m_imu.read_gyroscope(const_cast<float*>(&gyro_x), const_cast<float*>(&gyro_y), const_cast<float*>(&gyro_z));
        m_imu.read_temperature(const_cast<float*>(&temperature));
        printf("ACCEL: %.2f %.2f %.2f | GYRO: %.2f %.2f %.2f | TEMP: %.2f\n",
            accel_x, accel_y, accel_z,
            gyro_x,  gyro_y,  gyro_z,
            temperature);
        sleep_ms(1000);
    }
}
#pragma endregion
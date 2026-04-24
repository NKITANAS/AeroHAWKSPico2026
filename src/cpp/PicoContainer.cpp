#include "PicoContainer.h"

#pragma region Constructor
/// @brief Initializes the PicoContainer, which manages the IMU and soil moisture sensor, and handles communication with the Raspberry Pi.
PicoContainer::PicoContainer() 
{
    // init things
    stdio_init_all(); // Initialize all standard IO (for USB communication)
    adc_init();       // Initialize the ADC for soil moisture sensors
    // Initialize I2C1 (GPIO 6/7 and 14/15 both map to i2c1)
    i2c_init(i2c1, 400*1000);
    // Initialize subsystems
    m_imu.init();
    m_altimeter.init();
    m_moisture_sensor_1.init();
    m_moisture_sensor_2.init();

    // Set initial speed and orientation
    speed_x = 0;
    speed_y = 0;
    speed_z = 0; 

    orint_x = 0;
    orint_y = 0;
    orint_z = 0;

    filtered_altitude = 0;
    filtered_velocity = 0;

    start_signal_received = true;

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
    // Recovery Code
    gpio_init(Constants::FLASH_EXTRACT_PIN); // Initialize flash extract pin
    gpio_set_dir(Constants::FLASH_EXTRACT_PIN, GPIO_IN); // Set flash extract pin as input
    gpio_pull_up(Constants::FLASH_EXTRACT_PIN); // Pull-up on flash extract pin to prevent accidental grounding
    if (gpio_get(Constants::FLASH_EXTRACT_PIN) == 0) // If the flash extract pin is grounded, enter recovery mode to allow flash extraction
    {
        printf("Timestamp,SensorValue,SensorPtr\n"); // CSV header
        const uint8_t *flash_ptr = (const uint8_t*)(XIP_BASE + Constants::FLASH_OFFSET); // Pointer to the start of our data in flash memory
        for (uint32_t i = 0; i < MAX_FLASH_RECORDS; i++) // Loop through flash records until we hit an unused entry.
        {
            const FlashData* entry = reinterpret_cast<const FlashData*>(flash_ptr + i * sizeof(FlashData));
            if (entry->timestamp == 0xFFFFFFFF) // If we reach an empty entry (timestamp of 0xFFFFFFFF indicates unused flash), stop printing
                break;
            printf("%u,%.2f,%u\n", entry->timestamp, entry->sensor_value, entry->sensor_ptr);
        }
    }
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

        // Barometer is not ventilated until ~60s into flight - use predict-only until then
        constexpr int64_t BARO_VENTILATION_TIME_US = 60000000; // 60 seconds
        bool baro_ready = current_state == State::ASCENT &&
                          absolute_time_diff_us(m_ascent_start_time, get_absolute_time()) > BARO_VENTILATION_TIME_US;
        if (baro_ready)
            m_kalman_filter.update(vertical_accel, altitude, dt); // Full baro + accel fusion
        else
            m_kalman_filter.update(vertical_accel, dt);           // Accel-only until baro is ventilated
        filtered_altitude = m_kalman_filter.get_altitude();
        filtered_velocity = m_kalman_filter.get_velocity();

        if constexpr (Constants::TRANSMIT_ONLY)
        {
            // Transmit-only mode: skip all flight-stage logic and just broadcast sensor data
            critical_section_enter_blocking(&m_data_lock);
            float tx_ax = accel_x, tx_ay = accel_y, tx_az = accel_z;
            float tx_gx = gyro_x,  tx_gy = gyro_y,  tx_gz = gyro_z;
            float tx_temp = temperature;
            critical_section_exit(&m_data_lock);
            printf("ACCEL: %.2f %.2f %.2f | GYRO: %.2f %.2f %.2f | TEMP: %.2f | ALT: %.2f m | VEL: %.2f m/s\n",
                tx_ax, tx_ay, tx_az,
                tx_gx, tx_gy, tx_gz,
                tx_temp,
                filtered_altitude, filtered_velocity);
            sleep_us(500);
            continue;
        }

        if (current_state == State::IDLE)
        {
            if (accel_z >= 25) // Simple condition to detect launch (adjust threshold as needed)
            {
                m_ascent_start_time = get_absolute_time(); // Record the time of ascent start
                current_state = State::ASCENT; // Transition to ASCENT state
            }
        }
        else if (current_state == State::ASCENT)
        {
            if (absolute_time_diff_us(m_ascent_start_time, get_absolute_time()) < 30000000) // After 30 seconds of ascent, stop transmitting to be able to receive the landing signal
            {
                printf("ALT: %.2f m, VEL: %.2f m/s\n", filtered_altitude, filtered_velocity); // Debug output
            }
            // Transition to LANDED after 3 minutes of flight
            constexpr int64_t FLIGHT_DURATION_US = 200000000; // 200 seconds (3 minutes)
            if (absolute_time_diff_us(m_ascent_start_time, get_absolute_time()) > FLIGHT_DURATION_US && !land_manual_interrupt_recieved)
            {
                current_state = State::LANDED;
            }
        }
        else if (current_state == State::LANDED)
        {
            m_servo.angle_servo(0); // Move servo to initial position before landing sequence
            run_landing_sequence();
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

#pragma region Landing Sequence
void PicoContainer::run_landing_sequence()
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
    float angle = atan2(accel_y, -accel_x) * 180.0f / M_PI - 45.0f; // Convert radians to degrees then apply mounting offset
    while (angle <   0.0f) angle += 360.0f; // Normalize to 0-360
    while (angle >= 360.0f) angle -= 360.0f;

    // Side 1 window: [WIN_LO, WIN_HI]. Side 2 is the mirror 180° opposite.
    constexpr float WIN_LO  = Constants::MIN_ANGLE_SIDE_1;        // 43
    constexpr float WIN_HI  = Constants::MAX_ANGLE_SIDE_1;        // 98
    constexpr float WIN2_LO = WIN_LO + 180.0f;                    // 223
    constexpr float WIN2_HI = WIN_HI + 180.0f;                    // 278

    // Shortest angular distance between two scalar angles
    auto angular_dist = [](float a, float b) -> float {
        float d = fmodf(fabsf(a - b), 360.0f);
        return d > 180.0f ? 360.0f - d : d;
    };

    // Shortest angular distance from angle a to window [lo, hi]
    auto dist_to_window = [&](float a, float lo, float hi) -> float {
        if (a >= lo && a <= hi) return 0.0f;
        return fminf(angular_dist(a, lo), angular_dist(a, hi));
    };

    // Pick the window the ground direction is closest to
    float dist1 = dist_to_window(angle, WIN_LO, WIN_HI);
    float dist2 = dist_to_window(angle, WIN2_LO, WIN2_HI);
    bool actuator_1 = (dist1 <= dist2);

    if (actuator_1)
    {
        // Clamp ground angle to side-1 window, send directly to servo
        float target = (angle >= WIN_LO && angle <= WIN_HI) ? angle
                     : (angular_dist(angle, WIN_LO) <= angular_dist(angle, WIN_HI)) ? WIN_LO : WIN_HI;
        m_servo.angle_servo(target);
    }
    else
    {
        // Clamp ground angle to side-2 window, subtract 180 to get servo command
        float target = (angle >= WIN2_LO && angle <= WIN2_HI) ? angle
                     : (angular_dist(angle, WIN2_LO) <= angular_dist(angle, WIN2_HI)) ? WIN2_LO : WIN2_HI;
        m_servo.angle_servo(target - 180.0f);
    }
    sleep_ms(2000); // Let servo settle on initial alignment

    // Sweep through the 55-degree window in 10-degree steps
    for (float step = 0.0f; step <= 55.0f; step += 10.0f)
    {
        // WIN2 side: servo command is identical for both sides
        m_servo.angle_servo(step);
        sleep_ms(1000); // Let servo reach position

        m_actuator_1.extend();
        m_actuator_2.extend();
        sleep_ms(2000); // Let actuators fully extend

        float m1 = m_moisture_sensor_1.read_moisture();
        float m2 = m_moisture_sensor_2.read_moisture();
        printf("Sweep %.0fdeg: sensor1=%.1f%% sensor2=%.1f%%\n", step, m1, m2);

        if constexpr (Constants::HARDWARE_SAVE)
        {
            FlashData fd1 = { .timestamp = to_ms_since_boot(get_absolute_time()), .sensor_value = m1, .sensor_ptr = 1 };
            save_to_flash(&fd1);
            FlashData fd2 = { .timestamp = to_ms_since_boot(get_absolute_time()), .sensor_value = m2, .sensor_ptr = 2 };
            save_to_flash(&fd2);
        }

        m_actuator_1.retract();
        m_actuator_2.retract();
        sleep_ms(2000); // Let actuators retract before next position
    }

    current_state = State::TRANSMISSION;
}
#pragma endregion

#pragma region Core 2 Loop
/// @brief The loop that runs on the second core of the Pico, which will be used to process numbers when implemented
void PicoContainer::core2_loop()
{
    absolute_time_t last_flash_time = get_absolute_time(); // Throttle flash writes to every 100ms
    while (true)
    {
        // Read all IMU data in a single coherent 14-byte burst to avoid stale/mixed axis values
        bool imu_ok = m_imu.read_all(&accel_x_temp, &accel_y_temp, &accel_z_temp,
                                     &gyro_x_temp,  &gyro_y_temp,  &gyro_z_temp,
                                     &temperature_temp);
        if (!imu_ok)
        {
            // I2C peripheral is stuck after a failed transaction - deinit/reinit to recover the bus
            i2c_deinit(i2c1);
            i2c_init(i2c1, 400*1000);
            m_imu.init();       // Re-wake the MPU6050 and restore its config registers
            m_altimeter.init(); // Restore altimeter GPIO and bus config
            continue;           // Skip writing stale temps to shared vars this iteration
        }
        m_altimeter.read_altitude(&altitude_temp, temperature_temp); // Read barometric altitude
        if (current_state == State::LANDED)
        {
            calculate_moisture_1 = true;
            calculate_moisture_2 = true;
        }

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

        const bool flash_due = absolute_time_diff_us(last_flash_time, get_absolute_time()) >= 100000; // 100ms
        if (calculate_moisture_1)
        {
            moisture_1_temp = m_moisture_sensor_1.read_moisture(); // Read moisture level from sensor 1
            if constexpr (Constants::HARDWARE_SAVE)
            {
                if (flash_due)
                {
                FlashData flash_data = {
                    .timestamp = to_ms_since_boot(get_absolute_time()),
                    .sensor_value = moisture_1_temp,
                    .sensor_ptr = 1
                };
                save_to_flash(&flash_data);
                last_flash_time = get_absolute_time();
                }
            }
        }
        if (calculate_moisture_2)
        {
            moisture_2_temp = m_moisture_sensor_2.read_moisture(); // Read moisture level from sensor 2
            if constexpr (Constants::HARDWARE_SAVE)
            {
                if (flash_due)
                {
                FlashData flash_data = {
                    .timestamp = to_ms_since_boot(get_absolute_time()),
                    .sensor_value = moisture_2_temp,
                    .sensor_ptr = 2
                };
                save_to_flash(&flash_data);
                last_flash_time = get_absolute_time();
                }
            }
        }

        // Write to the shared variables
        critical_section_enter_blocking(&m_data_lock); // Enter critical section to safely update shared data
        moisture_readings.push_back(moisture_1_temp);
        moisture_readings.push_back(moisture_2_temp);
        accel_x      = accel_x_temp;
        accel_y      = accel_y_temp;
        accel_z      = accel_z_temp;
        gyro_x       = gyro_x_temp;
        gyro_y       = gyro_y_temp;
        gyro_z       = gyro_z_temp;
        temperature  = temperature_temp;
        altitude     = altitude_temp;
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

#pragma region Flash Save
/// @brief A function to save data to flash memory (if HARDWARE_SAVE is true)
void PicoContainer::save_to_flash(FlashData *data)
{
    if (data == nullptr)
    {
        return;
    }

    if (!m_flash_initialized)
    {
        uint32_t ints = save_and_disable_interrupts();
        flash_range_erase(Constants::FLASH_OFFSET, FLASH_SECTOR_SIZE);
        restore_interrupts(ints);
        m_flash_initialized = true;
        m_flash_write_index = 0;
    }

    if (m_flash_write_index >= MAX_FLASH_RECORDS)
    {
        return;
    }

    const uint32_t byte_offset = m_flash_write_index * sizeof(FlashData);
    const uint32_t page_base_offset = Constants::FLASH_OFFSET + ((byte_offset / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE);
    const uint32_t in_page_offset = byte_offset % FLASH_PAGE_SIZE;

    uint8_t page_buffer[FLASH_PAGE_SIZE];
    const uint8_t *flash_page_ptr = reinterpret_cast<const uint8_t *>(XIP_BASE + page_base_offset);
    memcpy(page_buffer, flash_page_ptr, FLASH_PAGE_SIZE);
    memcpy(page_buffer + in_page_offset, data, sizeof(FlashData));

    uint32_t ints = save_and_disable_interrupts();
    flash_range_program(page_base_offset, page_buffer, FLASH_PAGE_SIZE);
    restore_interrupts(ints);

    ++m_flash_write_index;
}
#pragma endregion

#pragma region Tests
/// @brief Executes an end-to-end landing sequence test in a finite script.
void PicoContainer::test()
{
    m_actuator_1.retract();
    m_actuator_2.retract();
}
#pragma endregion
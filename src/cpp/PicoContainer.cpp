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

    // Find current time
    m_current_time = get_absolute_time();
}
#pragma endregion

#pragma region Main Loop
/// @brief The main loop of the PicoContainer, which continuously reads data from the IMU and soil moisture sensors, and sends it to the Raspberry Pi over I2C.
void PicoContainer::main_loop()
{
    // Non-blocking read from USB stdio: accumulate until newline
    int ch;
    bool got_command = false;
    while ((ch = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        if (ch == '\r') continue;
        if (ch == '\n' || m_buffer_pos >= (int)sizeof(m_buffer) - 1) {
            m_buffer[m_buffer_pos] = '\0';
            got_command = true;
            m_buffer_pos = 0;
            break;
        } else {
            m_buffer[m_buffer_pos++] = (char)ch;
        }
    }

    if (got_command) 
    {
        if (strncmp(m_buffer, "GET_DATA", 8) == 0) 
        {
            printf("ACCEL: %.2f, %.2f, %.2f\n", accel_x, accel_y, accel_z);
            printf("GYRO: %.2f, %.2f, %.2f\n", gyro_x, gyro_y, gyro_z);
            printf("TEMP: %.2f\n", temperature);
            printf("ALTITUDE: %.2f\n", altitude);
            printf("MOISTURE_1: %u\n", moisture_1);
            printf("MOISTURE_2: %u\n", moisture_2);
        }
        else if (strncmp(m_buffer, "GET_ACCEL", 9) == 0)
        {
            printf("ACCEL: %.2f, %.2f, %.2f\n", accel_x, accel_y, accel_z);
        }
        else if (strncmp(m_buffer, "GET_GYRO", 8) == 0)
        {
            printf("GYRO: %.2f, %.2f, %.2f\n", gyro_x, gyro_y, gyro_z);
        }
        else if (strncmp(m_buffer, "GET_TEMP", 8) == 0)
        {
            printf("TEMP: %.2f\n", temperature);
        }
        else if (strncmp(m_buffer, "GET_ALTITUDE", 12) == 0)
        {
            printf("ALTITUDE: %.2f\n", altitude);
        }
        else if (strncmp(m_buffer, "GET_MOISTURE_1", 14) == 0)
        {
            printf("MOISTURE_1: %u\n", moisture_1);
        }
        else if (strncmp(m_buffer, "GET_MOISTURE_2", 14) == 0)
        {
            printf("MOISTURE_2: %u\n", moisture_2);
        }
        else if (strncmp(m_buffer, "GET_SPEED", 9) == 0)
        {
            printf("SPEED: %.2f, %.2f, %.2f\n", speed_x, speed_y, speed_z);
        }
        else if (strncmp(m_buffer, "GET_ORIENTATION", 9) == 0)
        {
            printf("ORIENTATION: %.2f, %.2f, %.2f\n", orint_x, orint_y, orint_z);
        }
        else if (strncmp(m_buffer, "PING", 4) == 0) 
        {
            printf("PONG\n");
        }
        else 
        {
            printf("UNKNOWN_COMMAND\n");
        }
    }

            sleep_ms(10);  // small delay to reduce CPU usage
}
#pragma endregion

#pragma region Core 2 Loop
/// @brief The loop that runs on the second core of the Pico, which will be used to process numbers when implemented
void PicoContainer::core2_loop()
{
    while (true)
    {
        // Read data from the IMU sensor
        m_imu.read_accelerometer(&accel_x, &accel_y, &accel_z);
        m_imu.read_gyroscope(&gyro_x, &gyro_y, &gyro_z);
        m_imu.read_temperature(&temperature);

        // Find the current time and update the old time
        m_old_time     = m_current_time;
        m_current_time = get_absolute_time();

        // Use time to derive speed and orientation
        speed_x += accel_x * (absolute_time_diff_us(m_current_time, m_old_time) * 1e6f);
        speed_y += accel_y * (absolute_time_diff_us(m_current_time, m_old_time) * 1e6f);
        speed_z += accel_z * (absolute_time_diff_us(m_current_time, m_old_time) * 1e6f);

        // For orientation, we can simply integrate the gyro data (this is a very basic approach and may drift over time)
        orint_x += gyro_x * (absolute_time_diff_us(m_current_time, m_old_time) * 1e6f);
        orint_y += gyro_y * (absolute_time_diff_us(m_current_time, m_old_time) * 1e6f);
        orint_z += gyro_z * (absolute_time_diff_us(m_current_time, m_old_time) * 1e6f);

        // Read data from the soil moisture sensors
        moisture_1 = m_moisture_sensor_1.read_moisture();
        moisture_2 = m_moisture_sensor_2.read_moisture();
        // Read altitude data from the altimeter
        m_altimeter.read_altitude(&altitude, temperature); 
        sleep_ms(2); // small delay to reduce CPU usage
    }
}
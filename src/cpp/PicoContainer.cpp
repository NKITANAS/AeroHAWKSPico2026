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
    m_imu.value() = IMU(Constants::IMU_I2C_ADDRESS, Constants::IMU_SDA_PIN, Constants::IMU_SCL_PIN, i2c0);
    m_moisture_sensor_1.value() = SoilMoistureSensor(Constants::MOISTURE_SENSOR_1_PIN, Constants::MOISTURE_SENSOR_1_ADC_CHANNEL);
    m_moisture_sensor_2.value() = SoilMoistureSensor(Constants::MOISTURE_SENSOR_2_PIN, Constants::MOISTURE_SENSOR_2_ADC_CHANNEL);
    m_altimeter.value()         = Altimeter(Constants::ALTIMITER_I2C_ADDRESS, Constants::ALTIMITER_SDA_PIN, Constants::ALTIMITER_SCL_PIN);

    // Wait for USB
    while (!stdio_usb_connected()) 
    {
        sleep_ms(100);
    }
}
#pragma endregion

#pragma region Main Loop
/// @brief The main loop of the PicoContainer, which continuously reads data from the IMU and soil moisture sensors, and sends it to the Raspberry Pi over I2C.
void PicoContainer::main_loop()
{
    // Read data from the IMU sensor
    m_imu.value().read_accelerometer(&accel_x, &accel_y, &accel_z);
    m_imu.value().read_gyroscope(&gyro_x, &gyro_y, &gyro_z);
    m_imu.value().read_temperature(&temperature);
    // Read data from the soil moisture sensors
    uint16_t moisture_1 = m_moisture_sensor_1.value().read_moisture();
    uint16_t moisture_2 = m_moisture_sensor_2.value().read_moisture();
    // Read altitude data from the altimeter
    m_altimeter.value().read_altitude(&altitude, temperature); 

    // Send data to the RPi via USB
    if (fgets(m_buffer, sizeof(m_buffer), stdin) != NULL) 
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
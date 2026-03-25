#include "LinearActuator.h"

#pragma region Constructor
/// @brief Initializes the LinearActuator with the specified control pins, sets them as outputs, and ensures the actuator is off initially.
/// @param pin_1 IN 1
/// @param pin_2 IN 2
LinearActuator::LinearActuator(uint pin_1, uint pin_2) : m_pin_1(pin_1), m_pin_2(pin_2)
{
    gpio_init(m_pin_1);
    gpio_init(m_pin_2);
    gpio_set_dir(m_pin_1, GPIO_OUT);
    gpio_set_dir(m_pin_2, GPIO_OUT);
    // Ensure the actuator is off initially
    gpio_put(m_pin_1, 0); 
    gpio_put(m_pin_2, 0);
}
#pragma endregion

#pragma region Extend
/// @brief Extends the linear actuator by activating pin_1 and deactivating pin_2, running for a specified duration.
void LinearActuator::extend()
{
    gpio_put(m_pin_2, 0); // Ensure pin 2 is off first
    gpio_put(m_pin_1, 1); // Activate pin 1 to extend
    sleep_ms(2000);       // Run motor for 2 seconds
    gpio_put(m_pin_1, 0); // Turn off pin 1 after extending
}
#pragma endregion

#pragma region Retract
/// @brief Retracts the linear actuator by activating pin_2 and deactivating pin_1, running for a specified duration.
void LinearActuator::retract()
{
    gpio_put(m_pin_1, 0); // Ensure pin 1 is off first
    gpio_put(m_pin_2, 1); // Activate pin 2 to retract
    sleep_ms(2000);       // Run motor for 2 seconds
    gpio_put(m_pin_2, 0); // Turn off pin 2 after retracting
}
#pragma endregion
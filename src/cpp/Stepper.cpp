#include "Stepper.h"

#pragma region Constructor
/// @brief Initializes the Stepper motor with the specified control pins, sets them as outputs, and ensures the motor is disabled and in sleep mode initially.
/// @param dir_pin Direction control pin
/// @param step_pin Step control pin
/// @param enable_pin Enable control pin
/// @param sleep_pin Sleep control pin
Stepper::Stepper(uint dir_pin, uint step_pin, uint enable_pin, uint sleep_pin) : m_dir_pin(dir_pin), m_step_pin(step_pin), m_enable_pin(enable_pin), m_sleep_pin(sleep_pin)
{
    gpio_init(m_dir_pin);
    gpio_init(m_step_pin);
    gpio_init(m_enable_pin);
    gpio_init(m_sleep_pin);
    gpio_set_dir(m_dir_pin, GPIO_OUT);
    gpio_set_dir(m_step_pin, GPIO_OUT);
    gpio_set_dir(m_enable_pin, GPIO_OUT);
    gpio_set_dir(m_sleep_pin, GPIO_OUT);
    // Wake up the motor and make it ready to recieve commands
    gpio_put(m_sleep_pin, 1); // Wake up the motor
    gpio_put(m_enable_pin, 0); // Enable the motor (active LOW)
}
#pragma endregion

#pragma region Step Forward
/// @brief Steps the motor forward by the specified number of steps, setting the direction pin accordingly, and toggling the step pin to generate the required pulses for stepping.
/// @param steps Number of steps to move forward
void Stepper::step_forward(int steps)
{
    gpio_put(m_dir_pin, 1); // Set direction to forward
    for (int i = 0; i < steps; ++i)
    {
        gpio_put(m_step_pin, 1); // Step HIGH
        sleep_ms(10);            // Short delay for step pulse timing (adjust as needed)
        gpio_put(m_step_pin, 0); // Step LOW 
        sleep_ms(10);            // Short delay before the next step (adjust as needed)
        m_step_counter++;        // Increment step counter
    }
}
#pragma endregion

#pragma region Step Backward
/// @brief Steps the motor backward by the specified number of steps, setting the direction pin accordingly, and toggling the step pin to generate the required pulses for stepping.
/// @param steps Number of steps to move backward
void Stepper::step_backward(int steps)
{
    gpio_put(m_dir_pin, 0); // Set direction to backward
    for (int i = 0; i < steps; ++i)
    {
        gpio_put(m_step_pin, 1); // Step HIGH
        sleep_ms(10);            // Short delay for step pulse timing (adjust as needed)
        gpio_put(m_step_pin, 0); // Step LOW
        sleep_ms(10);            // Short delay before the next step (adjust as needed)
        m_step_counter--;         // Decrement step counter
    }
}
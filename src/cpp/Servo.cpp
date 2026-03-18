#include "Servo.h"

Servo::Servo(uint pwm_pin) : m_pwm_pin(pwm_pin)
{
    // Initialize the PWM for the servo control pin
    gpio_set_function(m_pwm_pin, GPIO_FUNC_PWM); // Set the GPIO pin to PWM function
    m_slice_num = pwm_gpio_to_slice_num(m_pwm_pin); // Get the PWM slice number for the pin
    pwm_set_wrap(m_slice_num, 20000); // Set the PWM wrap value for a 20ms period (50Hz)
    pwm_set_chan_level(m_slice_num, pwm_gpio_to_channel(m_pwm_pin), 0); // Start with 0% duty cycle
    pwm_set_enabled(m_slice_num, true); // Enable the PWM slice
}

void Servo::angle_servo(uint angle)
{
    // Map the angle (0-180) to the corresponding PWM duty cycle for the servo
    // Assuming a standard servo with a pulse width of 1ms (0 degrees) to 2ms (180 degrees) at a frequency of 50Hz
    uint32_t duty_cycle = (angle * 1000 / 180) + 1000; // Map angle to pulse width in microseconds
    pwm_set_gpio_level(m_pwm_pin, duty_cycle); // Set the PWM duty cycle to control the servo position
}
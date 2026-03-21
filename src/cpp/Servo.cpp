#include "Servo.h"

Servo::Servo(uint pwm_pin) : m_pwm_pin(pwm_pin)
{
    // Initialize the PWM for the servo control pin
    gpio_set_function(m_pwm_pin, GPIO_FUNC_PWM); // Set the GPIO pin to PWM function
    m_slice_num = pwm_gpio_to_slice_num(m_pwm_pin); // Get the PWM slice number for the pin
    // Dynamically compute divider: sys_clk / 1 MHz → 1 µs per count, wrap at 20000 = 20 ms (50 Hz)
    float clk_div = (float)clock_get_hz(clk_sys) / 1000000.0f;
    pwm_set_clkdiv(m_slice_num, clk_div);
    pwm_set_wrap(m_slice_num, 20000); // 20000 × 1 µs = 20 ms period (50 Hz)
    pwm_set_chan_level(m_slice_num, pwm_gpio_to_channel(m_pwm_pin), 1500); // Start at neutral (90°)
    pwm_set_enabled(m_slice_num, true); // Enable the PWM slice
}

void Servo::angle_servo(uint angle)
{
    if (angle > 180) angle = 180; // clamp
    // Map 0-180° to 500-2500µs (full servo travel range)
    uint32_t pulse_us = 300 + (angle * 2000 / 180);
    pwm_set_gpio_level(m_pwm_pin, pulse_us);
}
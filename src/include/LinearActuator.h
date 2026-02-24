#pragma once

#pragma region Includes
#include <pico/stdlib.h>
#pragma endregion

/*
    The Linear Actuators are controlled via an H-Bridge.
    To extend the actuator:
        - Set pin_1 HIGH and pin_2 LOW
        - Wait for the actuator to fully extend (simulated with sleep_ms)
        - Set pin_1 LOW to stop the actuator
    To retract the actuator:
        - Set pin_1 LOW and pin_2 HIGH
        - Wait for the actuator to fully retract (simulated with sleep_ms)
        - Set pin_2 LOW to stop the actuator
*/

class LinearActuator
{
    public:
        explicit LinearActuator(uint pin_1, uint pin_2);
        void extend();
        void retract();
    private:
        uint m_pin_1, m_pin_2;
        int m_step_counter = 0; // Counter to track the current step position
};
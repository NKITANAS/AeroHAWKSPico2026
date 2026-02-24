#pragma once

#pragma region Includes
#include <pico/stdlib.h>
#pragma endregion

class Stepper
{
    public:
        explicit Stepper(uint dir_pin, uint step_pin, uint enable_pin, uint sleep_pin);
        void step_forward(int steps);
        void step_backward(int steps);
    private:
        uint m_dir_pin, m_step_pin, m_enable_pin, m_sleep_pin;
        int m_step_counter = 0; // Counter to track the current step position
        
};
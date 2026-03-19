#include "Constants.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

class Servo
{
    public:
        explicit Servo(uint pwm_pin);
        void     angle_servo(uint angle);
    private:
        uint m_pwm_pin, m_slice_num;

};
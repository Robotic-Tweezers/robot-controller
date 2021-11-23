#ifndef _MOTOR_HPP_
#define _MOTOR_HPP_

#include <Arduino.h>
#include <stdint.h>

namespace motor
{
    /**
     * @brief Test class for setting up PlatformIO dependencies
     */
    class Motor
    {
        private:
        
        uint8_t index;
        uint8_t pin_number;

        public:

        Motor(uint8_t index, uint8_t pin_number);

        void pwmStep(uint8_t duty_cycle);
    };
}

#endif // _MOTOR_HPP_
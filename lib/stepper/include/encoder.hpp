#ifndef _ENCODER_HPP_
#define _ENCODER_HPP_

#include <Arduino.h>
#include <stdint.h>

namespace RobotTweezers
{
    class Encoder
    {
        private:

        uint8_t index;
        uint8_t pin_a;
        uint8_t pin_b;
        long window_count;
        const unsigned int total_windows;
        volatile bool state_a;
        volatile bool state_b;

        void initialState(void);

        public:

        Encoder(void);

        Encoder(uint8_t index, uint8_t pin_a, uint8_t pin_b, unsigned int total_windows);

        void configureInputPins(uint8_t pin_a, uint8_t pin_b);

        void pinInterruptA(void);

        void pinInterruptB(void);

        float position(void);

        float velocity(float interval);

        uint8_t getPinA(void);

        uint8_t getPinB(void);
    };
}

#endif // _ENCODER_HPP_
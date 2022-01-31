#ifndef _STEPPER_HPP_
#define _STEPPER_HPP_

#include <stdint.h>
#include <FreeRTOS_TEENSY4.h>
#include <TMCStepper.h>

// Useful links
// https://github.com/bigtreetech/BIGTREETECH-Stepper-Motor-Driver/blob/master/TMC2209/V1.1/manual/TMC2209-V1.1-manual.pdf
// https://github.com/teemuatlut/TMCStepper
// https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC2209_Datasheet_V103.pdf

#define STEPS 200
#define MICROSTEP_MAX(RESOLUTION) STEPS *(RESOLUTION)
#define REGISTER_STEP_COUNTER(INDEX)                                                                  \
    ({                                                                                                \
        if (steppers[(INDEX)])                                                                        \
        {                                                                                             \
            auto ISR = [](void) -> void                                                               \
            {                                                                                         \
                steppers[(INDEX)]->microstep_count += steppers[(INDEX)]->direction ? 1 : -1;          \
                if (abs(steppers[(INDEX)]->microstep_count) >= STEPS * steppers[(INDEX)]->microstep)  \
                {                                                                                     \
                    steppers[(INDEX)]->microstep_count = 0;                                           \
                }                                                                                     \
            };                                                                                        \
            pinMode(steppers[(INDEX)]->step_counter_pin, INPUT_PULLUP);                               \
            attachInterrupt(digitalPinToInterrupt(steppers[(INDEX)]->step_counter_pin), ISR, RISING); \
        }                                                                                             \
    })

namespace RobotTweezers
{
    /**
     * @brief A step/direction + uart control interface using the TMC stepper library
     *
     */
    class Stepper
    {
    private:
        uint8_t step_pin;
        uint8_t direction_pin;
        TMC2209Stepper uart;
        static uint8_t enable;

        /**
         * @brief Construct a new Stepper object, not initialized
         *
         */
        Stepper();

        Stepper(HardwareSerial *serial, uint8_t address, uint8_t step_pin, uint8_t direction_pin);

        void SetDirection(bool direction);

        void SetPWMFrequency(float frequency);

    public:
        uint8_t step_counter_pin;
        uint16_t microstep;
        bool direction;
        volatile uint64_t microstep_count;

        bool Initialize(void);

        uint8_t Address(void);

        void SetVelocity(float velocity);

        float GetPosition(void);

        static Stepper *StepperFactory(HardwareSerial *serial, uint8_t address, uint8_t step_pin, uint8_t direction);

        static void SetEnablePin(uint8_t enable);

        static void Enable(void);

        static void Disable(void);
    };
}

#endif // _STEPPER_HPP_

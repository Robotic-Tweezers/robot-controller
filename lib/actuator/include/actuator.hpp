#ifndef _ACTUATOR_HPP_
#define _ACTUATOR_HPP_

#include <stdint.h>
#include <FreeRTOS_TEENSY4.h>
#include <ArduinoEigen.h>
#include <TMCStepper.h>
#include <actuator_settings.hpp>

// Useful links
// https://github.com/bigtreetech/BIGTREETECH-Stepper-Motor-Driver/blob/master/TMC2209/V1.1/manual/TMC2209-V1.1-manual.pdf
// https://github.com/teemuatlut/TMCStepper
// https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC2209_Datasheet_V103.pdf

#define STEPS 200
#define MINIMUM_VELOCITY 0.001
#define MICROSTEP_MAX(RESOLUTION) STEPS *(RESOLUTION)
#define REGISTER_STEP_COUNTER(INDEX)                                                               \
    ({                                                                                             \
        auto ISR = [](void) -> void                                                                \
        {                                                                                          \
            actuators[(INDEX)]->microstep_count += actuators[(INDEX)]->direction ? 1 : -1;         \
        };                                                                                         \
        pinMode(actuators[(INDEX)]->step_counter_pin, INPUT_PULLUP);                               \
        attachInterrupt(digitalPinToInterrupt(actuators[(INDEX)]->step_counter_pin), ISR, RISING); \
    })

namespace RobotTweezers
{
    /**
     * @brief A step/direction + uart control interface using the TMC actuator library
     *
     */
    class Actuator
    {
    private:
        uint8_t step_pin;
        uint8_t direction_pin;
        TMC2209Stepper uart;
        struct
        {
            float min;
            float max;
        } motion_limits;
        float gear_ratio;
        static uint8_t enable;

        /**
         * @brief Construct a new Actuator object, not initialized
         *
         */
        Actuator(void);

        Actuator(HardwareSerial *serial, ActuatorSettings &settings);

        Actuator(HardwareSerial *serial, uint8_t address, uint8_t step_pin, uint8_t direction_pin);

        void SetDirection(bool direction);

        void SetPWMFrequency(float frequency);

    public:
        uint8_t step_counter_pin;
        uint16_t microstep;
        bool direction;
        volatile int64_t microstep_count;

        bool Initialize(void);

        uint8_t Address(void);

        void SetVelocity(float velocity);

        float GetPosition(void);

        void SetMotionLimits(float min, float max);

        void SetGearRatio(float gear_ratio);

        static Actuator *ActuatorFactory(HardwareSerial *serial, ActuatorSettings &settings);

        static Actuator *ActuatorFactory(HardwareSerial *serial, uint8_t address, uint8_t step_pin, uint8_t direction);

        static void SetEnablePin(uint8_t enable);

        static void Enable(void);

        static void Disable(void);

        static void Delete(Actuator *actuators[], uint8_t size);

        static void SetVelocity(Actuator *actuators[], const Eigen::Vector3f &velocity, uint8_t size);

        static Eigen::Vector3f GetPosition(Actuator *actuators[], uint8_t size);
    };
}

#endif // _ACTUATOR_HPP_

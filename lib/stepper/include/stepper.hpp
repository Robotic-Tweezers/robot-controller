#ifndef _STEPPER_HPP_
#define _STEPPER_HPP_

#include <stdint.h>
#include <FreeRTOS_TEENSY4.h>
#include <TMCStepper.h>

// Useful links
// https://github.com/bigtreetech/BIGTREETECH-Stepper-Motor-Driver/blob/master/TMC2209/V1.1/manual/TMC2209-V1.1-manual.pdf
// https://github.com/teemuatlut/TMCStepper
// https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC2209_Datasheet_V103.pdf

namespace RobotTweezers
{
    /**
     * @brief A step/direction + uart control interface using the TMC stepper library
     * 
     */
    class Stepper
    {
        private:

        uint8_t step;
        uint8_t direction;
        uint32_t period;
        uint32_t position;
        static uint8_t enable;

        public:

        TMC2209Stepper* uart;

        /**
         * @brief Construct a new Stepper object, not initialized
         * 
         */
        Stepper();

        Stepper(HardwareSerial* serial, uint8_t address, uint8_t step, uint8_t direction);

        bool initialize(void);

        uint8_t address(void);

        static void stepMotor(void *arg);

        static void setEnablePin(uint8_t enable);

        static void enableSteppers(void);

        static void disableSteppers(void);
    };
}

#endif // _STEPPER_HPP_

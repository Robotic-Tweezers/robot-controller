#ifndef _STEPPER_HPP_
#define _STEPPER_HPP_

#include <Arduino.h>
#include <stdint.h>
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


        public:

        TMC2209Stepper uart;

        /**
         * @brief Construct a new Stepper object, not initialized
         * 
         */
        Stepper();

        Stepper(HardwareSerial* serial, uint8_t address, uint8_t step, uint8_t direction);

        bool initialize(void);

        uint8_t address(void);
    };
}

#endif // _STEPPER_HPP_
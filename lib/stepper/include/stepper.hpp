#ifndef _STEPPER_HPP_
#define _STEPPER_HPP_

#include <Arduino.h>
#include <stdint.h>
#include <stepper_uart.hpp>

// Useful links
// https://github.com/bigtreetech/BIGTREETECH-Stepper-Motor-Driver/blob/master/TMC2209/V1.1/manual/TMC2209-V1.1-manual.pdf
// https://github.com/teemuatlut/TMCStepper
// https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC2209_Datasheet_V103.pdf

namespace RobotTweezers
{
    class Stepper
    {
        private:
        
        uint8_t step;
        uint8_t direction;
        uint8_t enable;
        StepperUart uart;

        public:

        /****************************************
         * Step/Direction + UART control
        *****************************************/
        /**
         * @brief Construct a new Stepper object, not initialized
         * 
         */
        Stepper();

        Stepper(Stream* serial, uint8_t slave_address, uint8_t step, uint8_t direction, uint8_t enable);

        void testStepperComm(void);
    };
}

#endif // _STEPPER_HPP_
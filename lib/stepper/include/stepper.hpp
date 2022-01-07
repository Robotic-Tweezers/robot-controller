#ifndef _STEPPER_HPP_
#define _STEPPER_HPP_

#include <Arduino.h>
#include <stdint.h>

// Useful links
// https://github.com/bigtreetech/BIGTREETECH-Stepper-Motor-Driver/blob/master/TMC2209/V1.1/manual/TMC2209-V1.1-manual.pdf
// https://github.com/teemuatlut/TMCStepper
// https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC2209_Datasheet_V103.pdf

namespace RobotTweezers
{
    class Stepper
    {
        private:
        
        uint8_t slave_address;
        uint8_t step;
        uint8_t direction;
        uint8_t enable;
        Stream* serial;

        uint8_t calculateCRC(const uint8_t *datagram, uint8_t datagramLength);
        void flushReadBuffer(void);
        bool read(uint8_t address, uint8_t* data);
        void write(uint8_t address);

        public:

        /****************************************
         * Step/Direction control
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
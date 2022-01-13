#ifndef _STEPPER_HPP_
#define _STEPPER_HPP_

#include <Arduino.h>
#include <stdint.h>
#include <TMCStepper.h>

// Useful links
// https://github.com/bigtreetech/BIGTREETECH-Stepper-Motor-Driver/blob/master/TMC2209/V1.1/manual/TMC2209-V1.1-manual.pdf
// https://github.com/teemuatlut/TMCStepper
// https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC2209_Datasheet_V103.pdf

// General Configuration Registers
#define GLOBAL_CONFIG_REG 0x00
#define GLOBAL_STATUS_REG 0x01
#define INTERFACE_TRANS_COUNT_REG 0x02
#define SLAVE_CONFIG_REG 0x03
#define OTP_PROGRAM_REG 0x04
#define OTP_READ_REG 0x05
#define IO_INPUT_STATE_REG 0x06
#define FACTORY_CONFIG_REG 0x07

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

        TMC2209Stepper* uart;

        /**
         * @brief Construct a new Stepper object, not initialized
         * 
         */
        Stepper();

        Stepper(HardwareSerial* serial, uint8_t address, uint8_t step, uint8_t direction);

        ~Stepper();

        bool initialize(void);

        uint8_t address(void);
    };
}

#endif // _STEPPER_HPP_

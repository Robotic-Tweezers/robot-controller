#ifndef _STEPPER_HPP_
#define _STEPPER_HPP_

#include <Arduino.h>
#include <stdint.h>
#include <unordered_map>
#include <stepper_uart.hpp>

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
    class Stepper
    {
        private:
        
        uint8_t step;
        uint8_t direction;
        uint8_t enable;
        StepperUart uart;
        std::unordered_map<uint8_t, uint32_t> register_data;

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

        /****************************************
         * Global configuration register setting
         *****************************************/
        void setGlobalConfig(uint16_t flags);
        uint16_t getGlobalConfig(void);
        void setGlobalStatus(uint8_t flags);
        uint8_t getGlobalStatus_all(void);
    };
}

#endif // _STEPPER_HPP_

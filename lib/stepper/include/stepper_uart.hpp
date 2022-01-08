#ifndef _STEPPER_UART_HPP_
#define _STEPPER_UART_HPP_

#include <Arduino.h>
#include <stdint.h>

namespace RobotTweezers
{
    class StepperUart 
    {
        private:

        uint8_t slave_address;
        uint32_t baud;
        uint32_t read_delay;
        Stream* serial;

        /**
         * @brief Function to calculate the TMC2209 checksum
         * 
         * https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC2209_Datasheet_V103.pdf
         * 
         * @param datagram          The UART transfer buffer
         * @param datagramLength    Length of buffer
         * @return uint8_t          Checksum byte
         */
        uint8_t calculateCRC(const uint8_t datagram[], uint8_t datagramLength);

        /**
         * @brief Clear read buffer after writing to the TMC2209 (single wire interface)
         * 
         */
        void flushReadBuffer(void);

        /**
         * @brief Send a read request signal to a TMC2209
         * 
         * @param address       The register address 
         */
        void readRequest(uint8_t address);

        public:

        /**
         * @brief Construct a new Stepper Uart object
         * 
         */
        StepperUart();

        /**
         * @brief Construct a new Stepper Uart object
         * 
         * @param serial        The Arduino Stream object used for UART transfers
         * @param slave_address The TMC2209 UART address 
         */
        StepperUart(Stream* serial, uint8_t slave_address);

        /**
         * @brief Read a TMC2209 register
         * 
         * @param address       The register address
         * @param data          Data array returned from driver
         * @return true         Read was successful (CRC matches)
         * @return false        Read failed
         */
        bool read(uint8_t address, uint8_t data[]);

        /**
         * @brief Write to a TMC2209 register
         * 
         * @param address       The register address
         * @param data          Data array returned from driver
         */
        void write(uint8_t address, uint8_t data[]);
    };
}

#endif // _STEPPER_UART_HPP_
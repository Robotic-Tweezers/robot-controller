#ifndef _STEPPER_UART_HPP_
#define _STEPPER_UART_HPP_

#include <Arduino.h>
#include <stdint.h>

namespace RobotTweezers
{
    typedef union uart_write_s
    {
        uint8_t bytes[8];
        struct
        {
            uint8_t sync;
            uint8_t slave_address;
            uint8_t register_address;
            uint8_t data[4];
            uint8_t crc;
        } datagram;
    } uart_write_s;

    typedef union uart_request_s
    {
        uint8_t bytes[4];
        struct 
        {
            uint8_t sync;
            uint8_t slave_address;
            uint8_t register_address;
            uint8_t crc;
        } datagram;
    } uart_request_s;

    typedef union uart_read_s
    {
        uint8_t bytes[8];
        struct
        {
            uint8_t sync;
            uint8_t slave_address;
            uint8_t register_address;
            uint8_t data[4];
            uint8_t crc;
        } datagram;
    } uart_read_s;

    class StepperUart 
    {
        private:

        uint8_t slave_address;
        Stream* serial;

        uint8_t calculateCRC(const uint8_t datagram[], uint8_t datagramLength);
        void flushReadBuffer(void);
        void readRequest(uint8_t address);

        public: 

        StepperUart();
        StepperUart(Stream* serial, uint8_t slave_address);
        bool read(uint8_t address, uint8_t data[]);
        void write(uint8_t address, uint8_t data[]);
    };
}

#endif // _STEPPER_UART_HPP_
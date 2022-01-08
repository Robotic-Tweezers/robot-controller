#include <stepper_uart.hpp>

#define UART_READ (uint8_t)0x00
#define UART_WRITE (uint8_t)0x80
#define SYNC (uint8_t)0x05

RobotTweezers::StepperUart::StepperUart()
    : serial(nullptr), slave_address(0xFF) {}

RobotTweezers::StepperUart::StepperUart(Stream *serial, uint8_t slave_address)
    : serial(serial), slave_address(slave_address) {}

uint8_t RobotTweezers::StepperUart::calculateCRC(const uint8_t datagram[], uint8_t datagramLength)
{
    uint8_t currentByte;
    uint8_t crc = 0;
    // Execute for all bytes of a message
    for (int i = 0; i < (datagramLength - 1); i++)
    {
        currentByte = datagram[i]; // Retrieve a byte to be sent from Array
        for (int j = 0; j < 8; j++)
        {
            // update CRC based result of XOR operation
            crc = (crc >> 7) ^ (currentByte & 0x01) ? (crc << 1) ^ 0x07 : crc << 1;
            currentByte >>= 1;
        }
    }

    return crc;
}

void RobotTweezers::StepperUart::flushReadBuffer(void)
{
    while (serial->available() > 0)
    {
        serial->read();
    }
}

void RobotTweezers::StepperUart::readRequest(uint8_t address)
{
    uart_request_s request;

    request.datagram.sync = SYNC;
    request.datagram.slave_address = slave_address;
    request.datagram.register_address = static_cast<uint8_t>(address | UART_READ);
    request.datagram.crc = calculateCRC(request.bytes, 4);

    serial->write(request.bytes, 4);
    serial->flush();
    flushReadBuffer();
}

bool RobotTweezers::StepperUart::read(uint8_t address, uint8_t data[])
{
    uart_read_s response;
    uint8_t i = 0;
    if (serial == nullptr)
    {
        return false;
    }

    serial->flush();
    readRequest(address);
    delay(2);
    while (serial->available() > 0)
    {
        serial->readBytes(response.bytes + (i++), 1);
    }

    memcpy(data, response.datagram.data, 4 * sizeof(uint8_t));
    return calculateCRC(response.bytes, 8) == response.datagram.crc;
}

void RobotTweezers::StepperUart::write(uint8_t address, uint8_t data[])
{
    uart_write_s write;
    if (serial == nullptr)
    {
        return;
    }

    write.datagram.sync = SYNC;
    write.datagram.slave_address = slave_address;
    write.datagram.register_address = static_cast<uint8_t>(address | UART_WRITE);
    memcpy(write.datagram.data, data, 4 * sizeof(uint8_t));
    write.datagram.crc = calculateCRC(write.bytes, 8);

    serial->write(write.bytes, 8);
    serial->flush();
    flushReadBuffer();
}

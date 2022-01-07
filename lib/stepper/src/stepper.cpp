#include <stepper.hpp>

#define SERIAL_READ     (uint8_t)0x00
#define SERIAL_WRITE    (uint8_t)0x80

RobotTweezers::Stepper::Stepper() : serial(nullptr) {}

RobotTweezers::Stepper::Stepper(Stream* serial, uint8_t slave_address, uint8_t step, uint8_t direction, uint8_t enable)
{
    this->serial = serial;
    this->slave_address = slave_address;
    this->step = step;
    this->direction = direction;
    this->enable = enable;
}

uint8_t RobotTweezers::Stepper::calculateCRC(const uint8_t *datagram, uint8_t datagramLength)
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

void RobotTweezers::Stepper::flushReadBuffer(void)
{
    while (serial->available() > 0) serial->read();
}

bool RobotTweezers::Stepper::read(uint8_t address, uint8_t* data)
{
    uint8_t request[4] = {0x5, slave_address, address | SERIAL_READ, 0x00};
    uint8_t response[8];
    uint8_t bytes = 0;

    serial->flush();
    request[3] = calculateCRC(request, 4);
    serial->write(request, 4);
    serial->flush();
    flushReadBuffer();
    delay(2);
    while (serial->available() > 0)
    {
        serial->readBytes(response + (bytes++), 1);
    }

    memcpy(data, response + 3, 4 * sizeof(uint8_t));    
    serial->flush();
    return calculateCRC(response, 8) == response[7];
}

void RobotTweezers::Stepper::testStepperComm(void)
{
    uint8_t data[4];
    if (read(0x06, data))
    {
        Serial.print(data[0], HEX);
        Serial.print(data[1], HEX);
        Serial.print(data[2], HEX);
        Serial.println(data[3], HEX);
    }
        
    /*
    Serial1.flush();
    uint8_t read_request[4] = {0x5, 0x00, 0x06, 0x00};
    uint8_t response[12];
    int bytes = 0;
    calculateCRC(read_request, 4);
    Serial.println("Sent:");
    for (auto byte : read_request)
    {
        Serial.print(byte, HEX);
        Serial.print(" ");
    }
    Serial1.write(read_request, 4);
    Serial1.flush();
    while (Serial1.available() > 0) Serial1.read(); // Flush
    delay(2);
    while (Serial1.available() > 0)
    {
        Serial1.readBytes(response + (bytes++), 1);
    }
    
    Serial.println("\nReceived:");
    for (int i = 0; i < bytes; i++)
    {
        Serial.print(response[i], HEX);
        Serial.print(" ");
    }

    Serial.println();
    Serial1.flush();
    */
}
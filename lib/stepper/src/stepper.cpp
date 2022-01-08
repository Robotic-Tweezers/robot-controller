#include <stepper.hpp>

#define SERIAL_READ     (uint8_t)0x00
#define SERIAL_WRITE    (uint8_t)0x80

RobotTweezers::Stepper::Stepper() {}

RobotTweezers::Stepper::Stepper(Stream* serial, uint8_t slave_address, uint8_t step, uint8_t direction, uint8_t enable)
{
    uart = StepperUart(serial, slave_address);
    this->step = step;
    this->direction = direction;
    this->enable = enable;
}

void RobotTweezers::Stepper::testStepperComm(void)
{
    static uint8_t i = 0;
    uint8_t data[4] = {0x00, 0x00, 0x00, i++ % 2 == 1};
    Serial.println("Writing:");
    Serial.print(data[0], HEX);
    Serial.print(data[1], HEX);
    Serial.print(data[2], HEX);
    Serial.println(data[3], HEX);
    uart.write(0x00, data);
    if (uart.read(0x00, data))
    {
        Serial.println("Reading:");
        Serial.print(data[0], HEX);
        Serial.print(data[1], HEX);
        Serial.print(data[2], HEX);
        Serial.println(data[3], HEX);
    }
}

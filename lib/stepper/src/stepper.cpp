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

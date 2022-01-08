#include <stepper.hpp>

RobotTweezers::Stepper::Stepper() : uart(nullptr, 0, 0b00) {}

RobotTweezers::Stepper::Stepper(HardwareSerial* serial, uint8_t address, uint8_t step, uint8_t direction) : uart(TMC2209Stepper(serial, 0.11f, address))
{
}

bool RobotTweezers::Stepper::initialize()
{
    return false;
}

uint8_t RobotTweezers::Stepper::address()
{
    return uart.ms2() << 1 | uart.ms1();
}

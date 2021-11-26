#include <stepper.hpp>

void robot_tweezers::Stepper::configureOutputPins(uint8_t step, uint8_t direction, uint8_t enable, uint8_t microstep1, uint8_t microstep2)
{
    this->step = step;
    this->direction = direction;
    this->enable = enable;
    this->microstep1 = microstep1;
    this->microstep2 = microstep2;
    
    pinMode(step, OUTPUT);
    pinMode(direction, OUTPUT);
    pinMode(enable, OUTPUT);
    pinMode(microstep1, OUTPUT);
    pinMode(microstep2, OUTPUT);
}

robot_tweezers::Stepper::Stepper() { }

robot_tweezers::Stepper::Stepper(uint8_t step, uint8_t direction)
{
    step_state = false;
    direction_state = false;
    resolution = MICROSTEP8;
    configureOutputPins(step, direction, 255, 255, 255);
    digitalWrite(step, LOW);
    digitalWrite(direction, LOW);
}

robot_tweezers::Stepper::Stepper(uint8_t step, uint8_t direction, uint8_t enable)
{
    step_state = false;
    direction_state = false;
    resolution = MICROSTEP8;
    configureOutputPins(step, direction, enable, 255, 255);
    digitalWrite(step, LOW);
    digitalWrite(direction, LOW);
    digitalWrite(enable, LOW);
}

robot_tweezers::Stepper::Stepper(uint8_t step, uint8_t direction, uint8_t enable, uint8_t microstep1, uint8_t microstep2)
{
    step_state = false;
    direction_state = false;
    resolution = MICROSTEP8;
    configureOutputPins(step, direction, enable, microstep1, microstep2);
    digitalWrite(step, LOW);
    digitalWrite(direction, LOW);
    digitalWrite(enable, LOW);
    digitalWrite(microstep1, LOW);
    digitalWrite(microstep2, LOW);
}

void robot_tweezers::Stepper::stepMotor(void)
{
    step_state = !step_state;
    digitalWrite(step, step_state);
}

void robot_tweezers::Stepper::setResolution(resolution_e resolution)
{
    this->resolution = resolution;
    digitalWrite(microstep2, (bool)((uint8_t)resolution >> 1));
    digitalWrite(microstep2, (bool)resolution);
}

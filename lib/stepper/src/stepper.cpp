#include <stepper.hpp>

void RobotTweezers::Stepper::setResolution(resolution_e resolution)
{
    digitalWrite(microstep1, (int)resolution == 32 || (int)resolution == 16);
    digitalWrite(microstep2, (int)resolution == 64 || (int)resolution == 16);
}

void RobotTweezers::Stepper::setDirection(bool positive)
{
    digitalWrite(this->direction, positive);
}

RobotTweezers::Stepper::Stepper() { }

RobotTweezers::Stepper::Stepper(uint8_t index, uint8_t step, uint8_t direction, uint8_t encoder_a, uint8_t encoder_b)
: index(index), encoder(index, encoder_a, encoder_b, 100)
{
    configureOutputPins(step, direction, 255, 255, 255);
    digitalWrite(step, LOW);
    digitalWrite(direction, HIGH);
}

RobotTweezers::Stepper::Stepper(uint8_t index, uint8_t step, uint8_t direction, uint8_t enable, uint8_t encoder_a, uint8_t encoder_b)
: index(index), encoder(index, encoder_a, encoder_b, 100)
{
    configureOutputPins(step, direction, enable, 255, 255);
    digitalWrite(step, LOW);
    digitalWrite(direction, HIGH);
    digitalWrite(enable, LOW);
}

RobotTweezers::Stepper::Stepper
    (uint8_t index, uint8_t step, uint8_t direction, uint8_t enable, uint8_t microstep1, uint8_t microstep2, 
    uint8_t encoder_a, uint8_t encoder_b) : index(index), encoder(index, encoder_a, encoder_b, 100)
{
    configureOutputPins(step, direction, enable, microstep1, microstep2);
    digitalWrite(step, LOW);
    digitalWrite(direction, HIGH);
    digitalWrite(enable, LOW);
    digitalWrite(microstep1, LOW);
    digitalWrite(microstep2, LOW);
}

void RobotTweezers::Stepper::configureOutputPins(uint8_t step, uint8_t direction, uint8_t enable, uint8_t microstep1, uint8_t microstep2)
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

void RobotTweezers::Stepper::configureEncoder(uint8_t encoder_a, uint8_t encoder_b)
{
    encoder.configureInputPins(encoder_a, encoder_b);
}

void RobotTweezers::Stepper::enableStepper(void)
{
    digitalWrite(enable, LOW);
}

void RobotTweezers::Stepper::disableStepper(void)
{
    digitalWrite(enable, HIGH);
}

void RobotTweezers::Stepper::setVelocity(float velocity)
{
    float frequency;
    float velocity_abs = abs(velocity);
    // Prioritize high resolution
    if (velocity_abs < MAX_VELOCITY(MICROSTEP64))
    {
        setResolution(MICROSTEP64);
        frequency = FREQUENCY(MICROSTEP64, velocity_abs);
    }
    else if (velocity_abs < MAX_VELOCITY(MICROSTEP32))
    {
        setResolution(MICROSTEP32);
        frequency = FREQUENCY(MICROSTEP32, velocity_abs);
    }
    else if (velocity_abs < MAX_VELOCITY(MICROSTEP16))
    {
        setResolution(MICROSTEP16);
        frequency = FREQUENCY(MICROSTEP16, velocity_abs);
    }
    else if (velocity_abs < MAX_VELOCITY(MICROSTEP8))
    {
        setResolution(MICROSTEP8);
        frequency = FREQUENCY(MICROSTEP8, velocity_abs);
    }
    else
    {
        setResolution(MICROSTEP8);
        frequency = MAX_FREQUENCY;
    }

    setDirection(velocity > 0);
    analogWriteFrequency(step, frequency);
    analogWrite(step, 128);
}
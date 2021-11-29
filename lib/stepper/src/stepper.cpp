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

RobotTweezers::Stepper::Stepper() { }

RobotTweezers::Stepper::Stepper(uint8_t step, uint8_t direction, uint8_t encoder_a, uint8_t encoder_b)
 : encoder(encoder_a, encoder_b, 100)
{
    configureOutputPins(step, direction, 255, 255, 255);
    digitalWrite(step, LOW);
    digitalWrite(direction, HIGH);
}

RobotTweezers::Stepper::Stepper(uint8_t step, uint8_t direction, uint8_t enable, uint8_t encoder_a, uint8_t encoder_b)
 : encoder(encoder_a, encoder_b, 100)
{
    configureOutputPins(step, direction, enable, 255, 255);
    digitalWrite(step, LOW);
    digitalWrite(direction, HIGH);
    digitalWrite(enable, LOW);
}

RobotTweezers::Stepper::Stepper
    (uint8_t step, uint8_t direction, uint8_t enable, uint8_t microstep1, uint8_t microstep2, 
    uint8_t encoder_a, uint8_t encoder_b) : encoder(encoder_a, encoder_b, 100)
{
    configureOutputPins(step, direction, enable, microstep1, microstep2);
    digitalWrite(step, LOW);
    digitalWrite(direction, HIGH);
    digitalWrite(enable, LOW);
    digitalWrite(microstep1, LOW);
    digitalWrite(microstep2, LOW);
}

void RobotTweezers::Stepper::setVelocity(float velocity)
{
    if (velocity == 0.00)
    {
        // Hold position
        digitalWrite(step, HIGH);
        return;
    }

    float frequency;
    // Prioritize high resolution
    if (abs(velocity) < MAX_VELOCITY(MICROSTEP64))
    {
        setResolution(MICROSTEP64);
        frequency = FREQUENCY(MICROSTEP64, abs(velocity));
    }
    else if (abs(velocity) < MAX_VELOCITY(MICROSTEP32))
    {
        setResolution(MICROSTEP32);
        frequency = FREQUENCY(MICROSTEP32, abs(velocity));
    }
    else if (abs(velocity) < MAX_VELOCITY(MICROSTEP16))
    {
        setResolution(MICROSTEP16);
        frequency = FREQUENCY(MICROSTEP16, abs(velocity));
    }
    else if (abs(velocity) < MAX_VELOCITY(MICROSTEP8))
    {
        setResolution(MICROSTEP8);
        frequency = FREQUENCY(MICROSTEP8, abs(velocity));
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
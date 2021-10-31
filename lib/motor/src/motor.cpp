#include <motor.hpp>

motor::Motor::Motor(uint8_t index, uint8_t pin_number)
{
    this->index = index;
    this->pin_number = pin_number;
    pinMode(pin_number, OUTPUT);
}

void motor::Motor::pwmStep(uint8_t duty_cycle)
{
    static uint16_t counter = 0;
    counter++ < duty_cycle ? digitalWrite(this->pin_number, HIGH) : digitalWrite(this->pin_number, LOW);
    if (counter == 40)
    {
        counter = 0;
    }
}
#include <encoder.hpp>

void RobotTweezers::Encoder::initialState(void)
{
    state_a = digitalRead(pin_a);
    state_b = digitalRead(pin_b);
}

RobotTweezers::Encoder::Encoder(void) : pin_a(255), pin_b(255), window_count(0), total_windows(100)
{
    initialState();
}

RobotTweezers::Encoder::Encoder(uint8_t index, uint8_t pin_a, uint8_t pin_b, unsigned int total_windows)
 : index(index), pin_a(pin_a), pin_b(pin_b), window_count(0), total_windows(total_windows)
{
    initialState();
}

void RobotTweezers::Encoder::configureInputPins(uint8_t pin_a, uint8_t pin_b)
{
    this->pin_a = pin_a;
    this->pin_b = pin_b;
    initialState();
} 

void RobotTweezers::Encoder::pinInterruptA(void)
{
    state_a = digitalRead(pin_a);
    window_count += state_a == state_b ? 1 : -1;
}

void RobotTweezers::Encoder::pinInterruptB(void)
{
    state_b = digitalRead(pin_b);
    window_count += state_b != state_a ? 1 : -1;
}

float RobotTweezers::Encoder::position(void)
{
    return (float)window_count * PI / total_windows;
}

float RobotTweezers::Encoder::velocity(float interval)
{
    static float previous_position = 0.00;
    float position = this->position();
    float velocity = (position - previous_position) / interval;
    previous_position = position;
    return velocity;
}

uint8_t RobotTweezers::Encoder::getPinA(void)
{
    return pin_a;
}

uint8_t RobotTweezers::Encoder::getPinB(void)
{
    return pin_b;
}
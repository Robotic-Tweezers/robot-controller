#include <stepper.hpp>

#define RSENSE  0.11f

uint8_t RobotTweezers::Stepper::enable = 12;

RobotTweezers::Stepper::Stepper() : uart(nullptr) {}

RobotTweezers::Stepper::Stepper(HardwareSerial* serial, uint8_t address, uint8_t step, uint8_t direction)
    : step(step), direction(direction)
{
    uart = new TMC2209Stepper(serial, 0.11f, address);
    period = 2000;
    
    pinMode(step, OUTPUT);
    pinMode(direction, OUTPUT);
    initialize();
}

bool RobotTweezers::Stepper::initialize()
{
    //uint32_t gconf_data = 0x00C0;
    //uart->begin();
    if (uart->test_connection() != 0)
    {
        return false;
    }

    uart->SLAVECONF(0x0000);
    uart->VACTUAL(5000);
    //uart->GCONF(gconf_data);
    //uart->SLAVECONF(0x0000);
    //uart->toff(4);
    ///// @TODO Figure out what these do
    //uart->blank_time(24);
    //uart->rms_current(400);
    //// Set microstep resolution
    //uart->microsteps(64);
    ///// @TODO Figure out what these do
    //uart->TCOOLTHRS(0xFFFFF);
    //uart->semin(5);
    //uart->semax(2);
    //uart->sedn(0b01);
    //uart->SGTHRS(50);
    return true;
}

uint8_t RobotTweezers::Stepper::address()
{
    return uart->ms2() << 1 | uart->ms1();
}

void RobotTweezers::Stepper::stepMotor(void *arg)
{
    Stepper* stepper = static_cast<Stepper*>(arg);

    while (true)
    {
        // Set direction
        bool stepper_direction = true;
        digitalWrite(stepper->direction, stepper_direction);

        digitalWrite(stepper->step, HIGH);
        vTaskDelay(stepper->period * configTICK_RATE_HZ / 2000000UL);
        digitalWrite(stepper->step, LOW);
        vTaskDelay(stepper->period * configTICK_RATE_HZ / 2000000UL);
        stepper->position += stepper_direction ? 1 : -1;
    }
}

void RobotTweezers::Stepper::setEnablePin(uint8_t enable)
{
    Stepper::enable = enable;
    pinMode(Stepper::enable, OUTPUT);
}

void RobotTweezers::Stepper::enableSteppers(void)
{
    return digitalWrite(enable, LOW);
}

void RobotTweezers::Stepper::disableSteppers(void)
{
    return digitalWrite(enable, HIGH);
}
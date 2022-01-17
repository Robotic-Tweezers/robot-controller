#include <stepper.hpp>

#define RSENSE  0.11f

inline bool RobotTweezers::Stepper::SetDirection(void)
{
    digitalWrite(direction, desired_position > state);
    return desired_position > state;
}

uint8_t RobotTweezers::Stepper::enable = 12;

RobotTweezers::Stepper::Stepper(void) : uart(nullptr) {}

RobotTweezers::Stepper::Stepper(HardwareSerial* serial, uint8_t address, uint8_t step, uint8_t direction)
    : step(step), direction(direction)
{
    uart = new TMC2209Stepper(serial, 0.11f, address);
    period = 350;
    state = 0;
    desired_position = 5000;
    
    pinMode(step, OUTPUT);
    pinMode(direction, OUTPUT);
    Initialize();
}

RobotTweezers::Stepper::~Stepper(void)
{
    //delete uart;
} 

bool RobotTweezers::Stepper::Initialize(void)
{
    //uint32_t gconf_data = 0x00C0;
    //uart->begin();

    uart->SLAVECONF(0x0000);
    uart->microsteps(8);
    //uart->VACTUAL(5000);
    //uart->GCONF(gconf_data);
    //uart->SLAVECONF(0x0000);
    //uart->toff(4);
    ///// @TODO Figure out what these do
    //uart->blank_time(24);
    //uart->rms_current(400);
    //// Set microstep resolution
    ///// @TODO Figure out what these do
    //uart->TCOOLTHRS(0xFFFFF);
    //uart->semin(5);
    //uart->semax(2);
    //uart->sedn(0b01);
    //uart->SGTHRS(50);
    return true;
}

uint8_t RobotTweezers::Stepper::Address(void)
{
    return uart->ms2() << 1 | uart->ms1();
}

void RobotTweezers::Stepper::SetPosition(double position)
{
    uint16_t resolution = uart->microsteps();
    desired_position = position / (2 * PI) * resolution;
}

RobotTweezers::Stepper* RobotTweezers::Stepper::StepperFactory(HardwareSerial* serial, uint8_t address, uint8_t step, uint8_t direction)
{
    Stepper* stepper = new Stepper(serial, address, step, direction);
    if (stepper->uart->test_connection() != 0)
    {
        delete stepper;
        return nullptr;
    }

    return stepper;
}

void RobotTweezers::Stepper::StepMotor(void *arg)
{
    Stepper* stepper = static_cast<Stepper*>(arg);
    if (not stepper)
    {
        return;
    }

    while (true)
    {
        //bool positive_dir = stepper->SetDirection();
        //if (stepper->desired_position == stepper->state)
        //{
        //    continue;
        //}

        digitalWrite(stepper->step, HIGH);
        vTaskDelay(stepper->period * configTICK_RATE_HZ / 2000000UL);
        digitalWrite(stepper->step, LOW);
        vTaskDelay(stepper->period * configTICK_RATE_HZ / 2000000UL);
        //stepper->state += positive_dir ? 1 : -1;
    }
}

void RobotTweezers::Stepper::SetEnablePin(uint8_t enable)
{
    Stepper::enable = enable;
    pinMode(Stepper::enable, OUTPUT);
}

void RobotTweezers::Stepper::Enable(void)
{
    return digitalWrite(enable, LOW);
}

void RobotTweezers::Stepper::Disable(void)
{
    return digitalWrite(enable, HIGH);
}
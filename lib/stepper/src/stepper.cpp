#include <stepper.hpp>

#define RSENSE 0.11f

uint8_t RobotTweezers::Stepper::enable = 12;

RobotTweezers::Stepper::Stepper() : uart(nullptr, 0.11f, 0b00) {}

RobotTweezers::Stepper::Stepper(HardwareSerial *serial, uint8_t address, uint8_t step_pin, uint8_t direction_pin)
    : step_pin(step_pin), direction_pin(direction_pin), uart(serial, 0.11f, address)
{
    pinMode(step_pin, OUTPUT);
    pinMode(direction_pin, OUTPUT);
    SetDirection(false);
    Initialize();
}

void RobotTweezers::Stepper::SetDirection(bool direction)
{
    this->direction = direction;
    digitalWrite(direction_pin, direction);
}

void RobotTweezers::Stepper::SetPWMFrequency(float frequency)
{
    analogWriteFrequency(step_pin, frequency);
    analogWrite(step_pin, 128);
}

bool RobotTweezers::Stepper::Initialize(void)
{
    // uint32_t gconf_data = 0x00C0;
    microstep_count = 0;
    microstep = 1;

    SetVelocity(0);

    uart.begin();

    uart.SLAVECONF(0x0000);
    uart.microsteps(0);

    // uart.VACTUAL(5000);
    // uart.GCONF(gconf_data);
    // uart.SLAVECONF(0x0000);
    // uart.toff(4);
    ///// @TODO Figure out what these do
    // uart.blank_time(24);
    // uart.rms_current(400);
    //// Set microstep resolution
    ///// @TODO Figure out what these do
    // uart.TCOOLTHRS(0xFFFFF);
    // uart.semin(5);
    // uart.semax(2);
    // uart.sedn(0b01);
    // uart.SGTHRS(50);
    return true;
}

uint8_t RobotTweezers::Stepper::Address(void)
{
    return uart.ms2() << 1 | uart.ms1();
}

void RobotTweezers::Stepper::SetVelocity(float velocity)
{
    // Undefined behaviour when writing zero frequency
    if (abs(velocity) < 0.1)
    {
        digitalWrite(step_pin, LOW);
        return;
    }

    float frequency = (float)STEPS * microstep * abs(velocity) / (2 * PI);
    SetDirection(velocity > 0.00);
    SetPWMFrequency(frequency);
}

float RobotTweezers::Stepper::GetPosition(void)
{
    return microstep != 0 ? 2.00f * PI * microstep_count / (STEPS * microstep) : 0.00;
}

RobotTweezers::Stepper *RobotTweezers::Stepper::StepperFactory(HardwareSerial *serial, uint8_t address, uint8_t step, uint8_t direction)
{
    TMC2209Stepper test_stepper(serial, 0.11f, address);
    return test_stepper.test_connection() == 0 ? new Stepper(serial, address, step, direction) : nullptr;
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
#include <actuator.hpp>

/// @TODO Verify this
#define RSENSE 0.11f

uint8_t RobotTweezers::Actuator::enable = 12;

RobotTweezers::Actuator::Actuator() : uart(nullptr, RSENSE, 0b00) {}

RobotTweezers::Actuator::Actuator(HardwareSerial *serial, ActuatorSettings& settings)
    : uart(serial, RSENSE, settings.uart_address)
{
    step_pin = settings.step;
    direction_pin = settings.direction;
    motion_limits.min = settings.motion_limits.min;
    motion_limits.max = settings.motion_limits.max;
    gear_ratio = settings.gear_ratio;
    step_counter_pin = settings.step_counter;
    Initialize();
}

RobotTweezers::Actuator::Actuator(HardwareSerial *serial, uint8_t address, uint8_t step_pin, uint8_t direction_pin)
    : step_pin(step_pin), direction_pin(direction_pin), uart(serial, RSENSE, address)
{
    SetGearRatio(1);
    SetMotionLimits(-PI, PI);
    Initialize();
}

void RobotTweezers::Actuator::SetDirection(bool direction)
{
    this->direction = direction;
    digitalWrite(direction_pin, direction);
}

void RobotTweezers::Actuator::SetPWMFrequency(float frequency)
{
    analogWriteFrequency(step_pin, frequency);
    analogWrite(step_pin, 128);
}

bool RobotTweezers::Actuator::Initialize(void)
{
    // uint32_t gconf_data = 0x00C0;
    microstep_count = 0;
    microstep = 256;

    pinMode(step_pin, OUTPUT);
    pinMode(direction_pin, OUTPUT);
    SetVelocity(0.00);

    uart.begin();

    uart.SLAVECONF(0x0000);
    uart.microsteps(microstep);

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

uint8_t RobotTweezers::Actuator::Address(void)
{
    return uart.ms2() << 1 | uart.ms1();
}

void RobotTweezers::Actuator::SetVelocity(float velocity)
{
    // Undefined behaviour when writing zero frequency
    if (abs(velocity) < MINIMUM_VELOCITY)
    {
        digitalWrite(step_pin, LOW);
        return;
    }

    float frequency = (float)STEPS * microstep * abs(velocity) / (2 * PI);
    SetDirection(velocity > 0.00);
    SetPWMFrequency(frequency);
}

float RobotTweezers::Actuator::GetPosition(void)
{
    return microstep != 0 ? 2.00f * PI * microstep_count / (STEPS * microstep) : 0.00;
}

void RobotTweezers::Actuator::SetMotionLimits(float min, float max)
{
    motion_limits.min = min;
    motion_limits.max = max;
}

void RobotTweezers::Actuator::SetGearRatio(float gear_ratio)
{
    this->gear_ratio = gear_ratio;
}

RobotTweezers::Actuator *RobotTweezers::Actuator::ActuatorFactory(HardwareSerial *serial, ActuatorSettings& settings)
{
    TMC2209Stepper connection(serial, RSENSE, settings.uart_address);
    return connection.test_connection() == 0 ? new Actuator(serial, settings) : nullptr;
}

RobotTweezers::Actuator *RobotTweezers::Actuator::ActuatorFactory(HardwareSerial *serial, uint8_t address, uint8_t step, uint8_t direction)
{
    TMC2209Stepper connection(serial, RSENSE, address);
    return connection.test_connection() == 0 ? new Actuator(serial, address, step, direction) : nullptr;
}

void RobotTweezers::Actuator::SetEnablePin(uint8_t enable)
{
    Actuator::enable = enable;
    pinMode(Actuator::enable, OUTPUT);
}

void RobotTweezers::Actuator::Enable(void)
{
    return digitalWrite(enable, LOW);
}

void RobotTweezers::Actuator::Disable(void)
{
    return digitalWrite(enable, HIGH);
}
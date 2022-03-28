#include <actuator.hpp>
#include <cmath>

/// @TODO Verify this
#define RSENSE 0.11f

uint8_t RobotTweezers::Actuator::enable = 12;

RobotTweezers::Actuator::Actuator() : driver(), uart(nullptr, RSENSE, 0b00) {}

RobotTweezers::Actuator::Actuator(HardwareSerial *serial, ActuatorSettings &settings)
    : driver(AccelStepper::DRIVER, settings.step, settings.direction), uart(serial, RSENSE, settings.uart_address)
{
    stall_threshold = settings.stall_threshold;
    SetMotionLimits(settings.motion_limits.min, settings.motion_limits.max);
    SetGearRatio(settings.gear_ratio);
    Initialize();
}

void RobotTweezers::Actuator::SetMicrostepResolution(uint16_t microstep)
{
    this->microstep = microstep;
    uart.microsteps(microstep);
}

float RobotTweezers::Actuator::StepsToRadians(long steps, float gear_ratio, uint16_t microstep)
{
    return microstep != 0 ? 2.00f * PI * steps / (STEPS * microstep * gear_ratio) : 0.00;
}

long RobotTweezers::Actuator::RadiansToSteps(float radians, float gear_ratio, uint16_t microstep)
{
    return microstep != 0 ? radians * STEPS * microstep * gear_ratio / (2.00f * PI) : 0.00;
}

bool RobotTweezers::Actuator::Initialize(void)
{
    // Initialize UART
    uart.begin();
    uart.GCONF(0x1C0);
    uart.SGTHRS(stall_threshold);
    uart.senddelay(0);
    SetMicrostepResolution(8);

    // Initialize stepper control
    driver.setCurrentPosition(0);
    driver.setMaxSpeed(RadiansToSteps(8, gear_ratio, microstep));

    // Set initial velocity
    SetVelocity(0.00);
    return true;
}


uint8_t RobotTweezers::Actuator::Address(void)
{
    return uart.ms2() << 1 | uart.ms1();
}

void RobotTweezers::Actuator::SetVelocity(float velocity)
{
    long velocity_steps = RadiansToSteps(velocity, gear_ratio, microstep);
    driver.setSpeed(velocity_steps);
}

float RobotTweezers::Actuator::GetPosition(void)
{
    long steps = driver.currentPosition();
    return StepsToRadians(steps, gear_ratio, microstep);
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

bool RobotTweezers::Actuator::Home(float home_position)
{
    SetVelocity(1);
    do
    {
        driver.runSpeed();
        delayMicroseconds(100);
    }
    while ((uart.SG_RESULT() > stall_threshold) && (GetPosition() <= TWO_PI));

    if (GetPosition() <= TWO_PI)
    {
        driver.setCurrentPosition(RadiansToSteps(home_position, gear_ratio, microstep));
        return true;
    }

    return false;
}

RobotTweezers::Actuator *RobotTweezers::Actuator::ActuatorFactory(HardwareSerial *serial, ActuatorSettings &settings)
{
    TMC2209Stepper connection(serial, RSENSE, settings.uart_address);
    connection.begin();
    // If we cannot connect over UART, something is wrong
    return connection.test_connection() == 0 ? new Actuator(serial, settings) : nullptr;
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

void RobotTweezers::Actuator::Delete(RobotTweezers::Actuator *actuators[], uint8_t size)
{
    for (uint8_t i = 0; i < size; i++)
    {
        delete actuators[i];
    }
}

void RobotTweezers::Actuator::SetVelocity(RobotTweezers::Actuator *actuators[], const Eigen::Vector3f &velocity, uint8_t size)
{
    for (uint8_t i = 0; i < size; i++)
    {
        actuators[i]->SetVelocity(velocity(i));
    }
}

Eigen::Vector3f RobotTweezers::Actuator::GetPosition(RobotTweezers::Actuator *actuators[], uint8_t size)
{
    Eigen::Vector3f position;
    for (uint8_t i = 0; i < size; i++)
    {
        position(i) = actuators[i]->GetPosition();
    }

    return position;
}

void RobotTweezers::Actuator::Run(Actuator *actuators[], uint8_t size)
{
    for (uint8_t i = 0; i < size; i++)
    {
        actuators[i]->driver.runSpeed();
    }
}

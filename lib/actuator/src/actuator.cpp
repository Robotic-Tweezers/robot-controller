#include <actuator.hpp>
#include <cmath>

/// @TODO Verify this
#define RSENSE 0.11f

uint8_t RobotTweezers::Actuator::enable = 12;

RobotTweezers::Actuator::Actuator() : driver(), uart(nullptr, RSENSE, 0b00) {}

RobotTweezers::Actuator::Actuator(HardwareSerial *serial, ActuatorSettings &settings)
    : driver(AccelStepper::DRIVER, settings.step, settings.direction), uart(serial, RSENSE, settings.uart_address)
{
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
    // uint32_t gconf_data = 0x00C0;

    SetMicrostepResolution(64);

    driver.setCurrentPosition(0);
    driver.setMaxSpeed(RadiansToSteps(15, gear_ratio, microstep));
    // driver.setAcceleration(1000);

    SetVelocity(0.00);
    
    uart.begin();
    uart.SLAVECONF(0x0000);
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
    long velocity_steps = RadiansToSteps(velocity, gear_ratio, microstep);
    driver.setSpeed(velocity_steps);
    // Undefined behaviour when writing zero frequency
    if (std::abs(velocity) > MINIMUM_VELOCITY)
    {
        driver.runSpeed();
    }
}

float RobotTweezers::Actuator::GetPosition(void)
{
    return driver.currentPosition();
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

RobotTweezers::Actuator *RobotTweezers::Actuator::ActuatorFactory(HardwareSerial *serial, ActuatorSettings &settings)
{
    TMC2209Stepper connection(serial, RSENSE, settings.uart_address);
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
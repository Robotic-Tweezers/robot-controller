#include <unity.h>
#include <config.hpp>
#include <actuator.hpp>
#include <AccelStepper.h>

void unittest_StepperControl(void)
{
    AccelStepper stepper(AccelStepper::DRIVER, THETA0_STEP, THETA0_DIRECTION);
    TMC2209Stepper stepper_uart(&Serial1, 0.11F, THETA0_ADDRESS);
    uint16_t microstep_res = 8;

    stepper.setEnablePin(ENABLE_PIN);
    stepper.setPinsInverted(false, false, true);
    stepper.enableOutputs();

    Serial1.begin(ACTUATOR_BAUDRATE);
    TEST_ASSERT_EQUAL(0l, stepper.currentPosition());
    TEST_ASSERT_EQUAL(0, stepper_uart.test_connection());

    stepper_uart.microsteps(microstep_res);

    Serial.println("Moving stepper...");
    stepper.setMaxSpeed(3000);
    stepper.setAcceleration(20);
    //stepper.moveTo(RADIAN_TO_STEP(3.1415, microstep_res));
    
    while (stepper.distanceToGo() != 0l)
    {
        stepper.run();
    }
    
    TEST_ASSERT_EQUAL(true, true);
}
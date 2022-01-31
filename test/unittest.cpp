#include <Arduino.h>
#include <unity.h>
#include <stepper.hpp>

extern void unittest_skew3(void);
extern void unittest_xRotation(void);
extern void unittest_zRotation(void);
extern void unittest_Kahan_problem1(void);
extern void unittest_Kahan_problem2(void);
extern void unittest_Kahan_problem3(void);
extern void unittest_Kahan_problem4(void);
extern void unittest_setCoordinates(void);

#define stepper_serial Serial1
#define THETA0_ADDRESS 0b00
#define THETA1_ADDRESS 0b01
#define THETA2_ADDRESS 0b10

using namespace RobotTweezers;

void UnitTest_StepperPinConnection(void)
{
    // address, step, direction, step_count
    uint8_t device_pins[3][4] = {
        {THETA0_ADDRESS, 3, 2, 4}, 
        {THETA1_ADDRESS, 8, 6, 9},
        {THETA2_ADDRESS, 10, 7, 11} 
    };

    //Stepper::
    for (uint8_t address = 0b00; address < 0b11; address++)
    {
        Stepper* stepper = Stepper::StepperFactory(&stepper_serial, address, device_pins[address][1], device_pins[address][2]);
        pinMode(device_pins[address][3], INPUT_PULLUP);
        digitalWrite(device_pins[address][1], LOW);
        digitalWrite(device_pins[address][2], LOW);
        TEST_ASSERT_EQUAL(stepper->uart->test_connection(), 0);
        TEST_ASSERT_EQUAL(stepper->Address(), address);
        uint32_t io_state = stepper->uart->IOIN();
        bool enable = io_state & 1;
        bool step = (io_state >> 7) & 1;
        bool direction = (io_state >> 9) & 1;
        TEST_ASSERT_EQUAL(enable, false);
        TEST_ASSERT_EQUAL(step, false);
        TEST_ASSERT_EQUAL(direction, false);
        // delete stepper;
    }
}

void setup()
{
    // NOTE!!! Wait for >2 secs
    // if board doesn't support software reset via Serial.DTR/RTS
    delay(2000);
    UNITY_BEGIN();

    /*
    // utils.hpp
    RUN_TEST(unittest_skew3);
    RUN_TEST(unittest_xRotation);
    RUN_TEST(unittest_zRotation);
    RUN_TEST(unittest_Kahan_problem1);
    RUN_TEST(unittest_Kahan_problem2);

    // coordinates.hpp
    RUN_TEST(unittest_setCoordinates);
     */
    auto test = [](void) -> void
    {
        TEST_ASSERT_EQUAL(true, true);
    };
    RUN_TEST(test);
    RUN_TEST(UnitTest_StepperPinConnection);
}

void loop()
{
    UNITY_END();
}
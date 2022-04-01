#include <Arduino.h>
#include <unity.h>
#include <actuator.hpp>

#include "unittest_utils.hpp"

using namespace RobotTweezers;

// Coordinates tests
extern void unittest_SetFrameOrigin(void);
extern void unittest_SetCoordinates(void);
extern void unittest_SubtractOperator(void);
extern void unittest_EqualsOperator(void);
extern void unittest_MultiplyOperator(void);

// Kinematic tests
extern void Kinematic_setup(void);
extern void unittest_DenavitHartenbergTransform(void);
extern void unittest_UpdateDenavitHartenbergTable(void);
extern void unittest_DirectKinematics(void);
extern void unittest_InverseKinematics(void);
extern void unittest_Jacobian(void);

// Utilities tests
extern void unittest_skew3(void);
extern void unittest_xRotation(void);
extern void unittest_zRotation(void);
extern void unittest_Kahan_problem1(void);
extern void unittest_Kahan_problem2(void);
extern void unittest_Kahan_problem3(void);
extern void unittest_Kahan_problem4(void);

extern void unittest_StepperControl(void);


void setup()
{
    // NOTE!!! Wait for >2 secs
    // if board doesn't support software reset via Serial.DTR/RTS
    delay(2000);
    UNITY_BEGIN();
    Serial.println("Robot Tweezers test build");

    // Coordinates tests
    RUN_TEST(unittest_SetFrameOrigin);
    RUN_TEST(unittest_SetCoordinates);
    RUN_TEST(unittest_SubtractOperator);
    RUN_TEST(unittest_EqualsOperator);
    RUN_TEST(unittest_MultiplyOperator);

    // Kinematic tests
    RUN_TEST(Kinematic_setup);
    RUN_TEST(unittest_DenavitHartenbergTransform);
    RUN_TEST(unittest_UpdateDenavitHartenbergTable);
    RUN_TEST(unittest_DirectKinematics);
    RUN_TEST(unittest_InverseKinematics);
    RUN_TEST(unittest_Jacobian);

    // Utilities tests
    RUN_TEST(unittest_skew3);
    RUN_TEST(unittest_xRotation);
    RUN_TEST(unittest_zRotation);
    RUN_TEST(unittest_Kahan_problem1);
    RUN_TEST(unittest_Kahan_problem2);
    RUN_TEST(unittest_Kahan_problem3);
    RUN_TEST(unittest_Kahan_problem4);
}

void loop()
{
    UNITY_END();
}
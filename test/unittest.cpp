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

    // RUN_TEST(unittest_InverseKinematics);
    
}

void loop()
{
    UNITY_END();
}
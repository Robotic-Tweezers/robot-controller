#include <Arduino.h>
#include <unity.h>

extern void unittest_skew3(void);
extern void unittest_xRotation(void);
extern void unittest_zRotation(void);
extern void unittest_Kahan_problem1(void);
extern void unittest_Kahan_problem2(void);
extern void unittest_Kahan_problem3(void);
extern void unittest_Kahan_problem4(void);
extern void unittest_setCoordinates(void);

void setup()
{
    // NOTE!!! Wait for >2 secs
    // if board doesn't support software reset via Serial.DTR/RTS
    delay(2000);
    UNITY_BEGIN();

    // utils.hpp
    RUN_TEST(unittest_skew3);
    RUN_TEST(unittest_xRotation);
    RUN_TEST(unittest_zRotation);
    RUN_TEST(unittest_Kahan_problem1);
    RUN_TEST(unittest_Kahan_problem2);

    // coordinates.hpp
    RUN_TEST(unittest_setCoordinates);
}

void loop()
{
    UNITY_END();
}
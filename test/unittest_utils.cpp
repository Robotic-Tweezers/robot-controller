#include <Arduino.h>
#include <unity.h>

#include <utils.hpp>

void unittest_skew3(void)
{
    Eigen::Vector3f test(1, 2, 3);
    Eigen::Matrix3f expected;
    expected << 
        0, -3, 2,
        3, 0, -1,
        -2, 1,  0;

    TEST_ASSERT_TRUE(robot_tweezers::skew3(test) == expected);
}

void unittest_xRotation(void)
{
    Eigen::Matrix3f expected;
    expected << 
        1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    
    TEST_ASSERT_TRUE(robot_tweezers::xRotation(0) == expected);
    expected << 
        1, 0, 0,
        0, -1, 0,
        0, 0, -1;
    TEST_ASSERT_TRUE(robot_tweezers::xRotation(PI) == expected);
}

void unittest_zRotation(void)
{
    Eigen::Matrix3f expected;
    expected << 
        1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    TEST_ASSERT_TRUE(robot_tweezers::zRotation(0) == expected);
    expected << 
        -1, 0, 0,
        0, -1, 0,
        0, 0, 1;
    TEST_ASSERT_TRUE(robot_tweezers::zRotation(PI) == expected);
}

void unittest_Kahan_problem1(void)
{
    Eigen::Vector3f s(1, 0, 0);
    Eigen::Vector3f t(0, 1, 0);
    TEST_ASSERT_EQUAL(robot_tweezers::Kahan::problem1(s, t), PI);
}

void unittest_Kahan_problem2(void)
{
    Eigen::Vector3f s_unit(1, 0, 0);
    Eigen::Vector3f u(1, 1, 0);
    Eigen::Vector3f v(1, -1, 0);
    bool valid = true;
    TEST_ASSERT_EQUAL(robot_tweezers::Kahan::problem2(s_unit, u, v, &valid), PI);
}

void unittest_Kahan_problem3(void)
{
    // std::pair<float[2], float[2]> robot_tweezers::Kahan::problem3(s_unit, t_unit, u, v)
}

void unittest_Kahan_problem4(void)
{
    // float robot_tweezers::Kahan::problem4(a, b, c)
}

void setup()
{
    // NOTE!!! Wait for >2 secs
    // if board doesn't support software reset via Serial.DTR/RTS
    delay(2000);
    UNITY_BEGIN();

    RUN_TEST(unittest_skew3);
    RUN_TEST(unittest_xRotation);
    RUN_TEST(unittest_zRotation);
    RUN_TEST(unittest_Kahan_problem1);
    RUN_TEST(unittest_Kahan_problem2);
}

void loop()
{
    UNITY_END();
}
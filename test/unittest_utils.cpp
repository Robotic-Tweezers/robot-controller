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
    TEST_ASSERT_TRUE(RobotTweezers::skew3(test) == expected);
}

void unittest_xRotation(void)
{
    Eigen::Matrix3f expected;
    expected << 
        1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    TEST_ASSERT_TRUE(RobotTweezers::xRotation(0) == expected);
    expected << 
        1, 0, 0,
        0, -1, 0,
        0, 0, -1;
    TEST_ASSERT_TRUE(RobotTweezers::xRotation(PI) == expected);
}

void unittest_zRotation(void)
{
    Eigen::Matrix3f expected;
    expected << 
        1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    TEST_ASSERT_TRUE(RobotTweezers::zRotation(0) == expected);
    expected << 
        -1, 0, 0,
        0, -1, 0,
        0, 0, 1;
    TEST_ASSERT_TRUE(RobotTweezers::zRotation(PI) == expected);
}

void unittest_Kahan_problem1(void)
{
    Eigen::Vector3f s(1, 0, 0);
    Eigen::Vector3f t(0, 1, 0);
    TEST_ASSERT_EQUAL(RobotTweezers::Kahan::problem1(s, t), PI);
}

void unittest_Kahan_problem2(void)
{
    Eigen::Vector3f s_unit(1, 0, 0);
    Eigen::Vector3f u(1, 1, 0);
    Eigen::Vector3f v(1, -1, 0);
    bool valid = true;
    TEST_ASSERT_EQUAL(RobotTweezers::Kahan::problem2(s_unit, u, v, &valid), PI);
}

void unittest_Kahan_problem3(void)
{
    // std::pair<float[2], float[2]> RobotTweezers::Kahan::problem3(s_unit, t_unit, u, v)
}

void unittest_Kahan_problem4(void)
{
    // float RobotTweezers::Kahan::problem4(a, b, c)
}

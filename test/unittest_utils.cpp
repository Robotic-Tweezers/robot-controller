#include <Arduino.h>
#include <unity.h>

#include <utils.hpp>

using namespace RobotTweezers;
using namespace Eigen;

void unittest_skew3(void)
{
    Vector3f test(1, 2, 3);
    Matrix3f expected;
    expected << 
        0, -3, 2,
        3, 0, -1,
        -2, 1,  0;
    TEST_ASSERT_TRUE(Skew3(test) == expected);
}

void unittest_xRotation(void)
{
    Matrix3f expected;
    expected << 
        1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    TEST_ASSERT_TRUE(XRotation(0) == expected);
    expected << 
        1, 0, 0,
        0, -1, 0,
        0, 0, -1;
    TEST_ASSERT_TRUE(XRotation(PI) == expected);
}

void unittest_zRotation(void)
{
    Matrix3f expected;
    expected << 
        1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    TEST_ASSERT_TRUE(ZRotation(0) == expected);
    expected << 
        -1, 0, 0,
        0, -1, 0,
        0, 0, 1;
    TEST_ASSERT_TRUE(ZRotation(PI) == expected);
}

void unittest_Kahan_problem1(void)
{
    Vector3f s(1, 0, 0);
    Vector3f t(0, 1, 0);
    TEST_ASSERT_EQUAL(Kahan::Problem1(s, t), PI);
}

void unittest_Kahan_problem2(void)
{
    Vector3f s_unit(1, 0, 0);
    Vector3f u(1, 1, 0);
    Vector3f v(1, -1, 0);
    bool valid = true;
    TEST_ASSERT_EQUAL(Kahan::Problem2(s_unit, u, v, &valid), PI);
}

void unittest_Kahan_problem3(void)
{
    // std::pair<float[2], float[2]> Kahan::Problem3(s_unit, t_unit, u, v)
}

void unittest_Kahan_problem4(void)
{
    // float Kahan::Problem4(a, b, c)
}

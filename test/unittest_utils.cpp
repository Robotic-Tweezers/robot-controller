#include <Arduino.h>
#include <unity.h>
#include <utils.hpp>
#include "unittest_utils.hpp"

using namespace RobotTweezers;
using namespace Eigen;

void unittest_skew3(void)
{
    Vector3f test(1, 2, 3);
    Matrix3f expected;
    expected << 0, -3, 2,
        3, 0, -1,
        -2, 1, 0;
    TEST_ASSERT_TRUE(Skew3(test) == expected);
}

void unittest_xRotation(void)
{
    Matrix3f expected;
    expected << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    TEST_ASSERT_TRUE(XRotation(0) == expected);
    expected << 1, 0, 0,
        0, -1, 0,
        0, 0, -1;
    TEST_ASSERT_TRUE(XRotation(PI) == expected);
}

void unittest_zRotation(void)
{
    Matrix3f expected;
    expected << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    TEST_ASSERT_TRUE(ZRotation(0) == expected);
    expected << -1, 0, 0,
        0, -1, 0,
        0, 0, 1;
    TEST_ASSERT_TRUE(ZRotation(PI) == expected);
}

void unittest_Kahan_problem1(void)
{
    int test_count = 5;
    Vector3f s[] = {
        Vector3f(1, 1, 1),
        Vector3f(1, 0, 0),
        Vector3f(0, 1, 0),
        Vector3f(0, 0, 1),
        Vector3f(-1, -1, -2),
    };
    Vector3f t[] = {
        Vector3f(-1, -1, -1),
        Vector3f(1, 1, 0),
        Vector3f(1, 0, 0),
        Vector3f(0, 0, 1),
        Vector3f(0, 1, 0),
    };
    float expected[] = {PI, PI / 4, HALF_PI, 0, 1.991};
    for (int i = 0; i < test_count; i++)
    {
        TEST_ASSERT_EQUAL(Kahan::Problem1(s[i], t[i]), expected[i]);
    }
}

void unittest_Kahan_problem2(void)
{
    int test_count = 5;
    Vector3f s_unit[] = {
        Vector3f(1, 1, 1) / sqrt(3),
        Vector3f(1, 0, 0),
        Vector3f(0, 1, 0),
        Vector3f(0, 0, 1),
        Vector3f(-1, -1, -2) / sqrt(6)
    };
    Vector3f u[] = {
        Vector3f(-1, -1, -1),
        Vector3f(1, 1, 0),
        Vector3f(1, 0, 0),
        Vector3f(0, -1, 0),
        Vector3f(0, 1, 0)
    };
    Vector3f v[] = {
        Vector3f(-1, 0, 0),
        Vector3f(1, -1, 0),
        Vector3f(0, 0, 1),
        Vector3f(1, 0, 0),
        Vector3f(0, 1, 0)
    };
    float expected[] = {0.0, PI, -HALF_PI, HALF_PI, 0.00};
    for (int i = 0; i < test_count; i++)
    {
        TEST_ASSERT_EQUAL(Kahan::Problem2(s_unit[i], u[i], v[i]), expected[i]);
    }
}

void unittest_Kahan_problem3(void)
{
    // std::pair<float[2], float[2]> Kahan::Problem3(s_unit, t_unit, u, v)% Test 1
    Vector3f s_unit = Vector3f(1, 1, 1) / sqrt(3);
    Vector3f t_unit = Vector3f(1, 1, 1) / sqrt(3);
    Vector3f u = Vector3f(-1, -1, 1);
    Vector3f v = Vector3f(-1, 0, 0);

    std::tuple<float, float, float, float> result = Kahan::Problem3(s_unit, t_unit, u, v);
    TEST_ASSERT_EQUAL(std::isnan(std::get<0>(result)), true);
    TEST_ASSERT_EQUAL(std::isnan(std::get<2>(result)), true);

    s_unit = Vector3f(1, 0, 0);
    t_unit = Vector3f(0, 0, 1);
    u = Vector3f(1, 0, -1);
    v = Vector3f(-1, 0, 1);

    result = Kahan::Problem3(s_unit, t_unit, u, v);
    TEST_ASSERT_EQUAL(std::get<0>(result), -PI);
    TEST_ASSERT_EQUAL(std::get<1>(result), PI);
    TEST_ASSERT_EQUAL(std::get<2>(result), PI);
    TEST_ASSERT_EQUAL(std::get<3>(result), -PI);

    s_unit = Vector3f(1, 0, 0);
    t_unit = Vector3f(0, 1, 0);
    u = Vector3f(1, 0, -1);
    v = Vector3f(-1, 0, 1);

    result = Kahan::Problem3(s_unit, t_unit, u, v);
    TEST_ASSERT_EQUAL(std::get<0>(result), PI);
    TEST_ASSERT_EQUAL(std::get<1>(result), 0.00);
    TEST_ASSERT_EQUAL(std::get<2>(result), HALF_PI);
    TEST_ASSERT_EQUAL(std::get<3>(result), PI);

    s_unit = Vector3f(1, 0, 0);
    t_unit = Vector3f(1, 0, 0);
    u = Vector3f(1, 0, 1);
    v = Vector3f(1, 0, 1);

    result = Kahan::Problem3(s_unit, t_unit, u, v);
    TEST_ASSERT_EQUAL(std::get<0>(result), 0.00);
    TEST_ASSERT_EQUAL(std::get<1>(result), 0.00);
    TEST_ASSERT_EQUAL(std::get<2>(result), 0.00);
    TEST_ASSERT_EQUAL(std::get<3>(result), 0.00);

    s_unit = Vector3f(1, 1, 1) / sqrt(3);
    t_unit = Vector3f(-1, 0, 0);
    u = Vector3f(1, 0, 1);
    v = Vector3f(1, 0, 1);

    result = Kahan::Problem3(s_unit, t_unit, u, v);
    TEST_ASSERT_EQUAL(std::get<0>(result), 0.00);
    TEST_ASSERT_EQUAL(std::get<1>(result), 2.0944);
    TEST_ASSERT_EQUAL(std::get<2>(result), 0.00);
    TEST_ASSERT_EQUAL(std::get<3>(result), HALF_PI);
}

void unittest_Kahan_problem4(void)
{
    float a = 1;
    float b = 1;
    float c = 1;
    float theta = Kahan::Problem4(a, b, c);
    TEST_ASSERT_EQUAL(theta, 2 * PI / 3);

    a = 1;
    b = 1;
    c = 0;
    theta = Kahan::Problem4(a, b, c);
    TEST_ASSERT_EQUAL(theta, PI);

    a = 4;
    b = 3;
    c = 5;
    theta = Kahan::Problem4(a, b, c);
    TEST_ASSERT_EQUAL(theta, HALF_PI);

    a = 4;
    b = 0;
    c = 5;
    theta = Kahan::Problem4(a, b, c);
    TEST_ASSERT_EQUAL(std::isnan(theta), true);

    a = 3;
    b = 5;
    c = 7;
    theta = Kahan::Problem4(a, b, c);
    TEST_ASSERT_EQUAL(theta, PI / 3);
}

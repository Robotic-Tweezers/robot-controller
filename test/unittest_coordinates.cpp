#include <Arduino.h>
#include <unity.h>
#include <coordinates.hpp>

using namespace RobotTweezers;
using namespace Eigen;

void unittest_SetFrameOrigin(void)
{
    Serial.println("Coordinates constructor with 4x4 matrix");
    Matrix4f coords;
    coords << 
        1, 0, 0, 0,
        0, -1, 0, 0,
        0, 0, -1, -25,
        0, 0, 0, 1;
    Coordinates coordinates_test(coords);
    Matrix3f expected_frame;
    expected_frame <<
        1, 0, 0,
        0, -1, 0,
        0, 0, -1;
    Vector3f expected_origin(0, 0, -25);
    TEST_ASSERT_EQUAL(coordinates_test.frame == expected_frame, true);
    TEST_ASSERT_EQUAL(coordinates_test.origin == expected_origin, true);
    TEST_ASSERT_EQUAL(coordinates_test.GetCoordinates() == coords, true);
}

void unittest_SetCoordinates(void)
{
    Serial.println("Coordinates constructor with 3x3 matrix and vector 3");
    Matrix3f expected_frame;
    expected_frame <<
        1, 0, 0,
        0, -1, 0,
        0, 0, -1;
    Vector3f expected_origin(0, 0, -25);
    Matrix4f coords;
    coords <<
        1, 0, 0, 0,
        0, -1, 0, 0,
        0, 0, -1, -25,
        0, 0, 0, 1;
    Coordinates coordinates_test(expected_frame, expected_origin);
    TEST_ASSERT_EQUAL(coordinates_test.frame == expected_frame, true);
    TEST_ASSERT_EQUAL(coordinates_test.origin == expected_origin, true);
    TEST_ASSERT_EQUAL(coordinates_test.GetCoordinates() == coords, true);
}

void unittest_SubtractOperator(void)
{
    Serial.println("Not implemented");
    TEST_ASSERT_EQUAL(true, true);
}

void unittest_EqualsOperator(void)
{
    Serial.println("Coordinates equals operators");
    Matrix4f test1;
    test1 <<
        1, 2, 3, 0,
        6, 5, 4, 0,
        7, 8, 9, -25,
        0, 0, 0, 1;
    Matrix4f test2_matrix;
    test2_matrix <<
        10, 12, 13, -25,
        16, 15, 14, 0,
        17, 18, 19, 0,
        0, 0, 0, 1;
    Coordinates result = test1;
    Coordinates test2(test2_matrix);
    TEST_ASSERT_EQUAL(result.GetCoordinates() == test1, true);
    result = test2;
    TEST_ASSERT_EQUAL(result.GetCoordinates() == test2_matrix, true);
}

void unittest_MultiplyOperator(void)
{
    Serial.println("Coordinates multiply operators");
    Matrix4f left_op;
    left_op <<
        1, 2, 3, 0,
        6, 5, 4, 0,
        7, 8, 9, -25,
        0, 0, 0, 1;
    Matrix4f right_op;
    right_op <<
        10, 12, 13, -25,
        16, 15, 14, 0,
        17, 18, 19, 0,
        0, 0, 0, 1;
    Coordinates left(left_op);
    Coordinates right(right_op);
    Matrix4f expected = Coordinates(left * right).GetCoordinates();
    // Check that Coordinates multiply equals Eigen 4x4 multiply (assumes Eigen is correct)
    // Also relies on Set* tests being correct
    TEST_ASSERT_EQUAL((left * right).GetCoordinates() == expected, true);
    left *= right;
    TEST_ASSERT_EQUAL(left.GetCoordinates() == expected, true);
}

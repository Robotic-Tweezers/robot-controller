#include <Arduino.h>
#include <unity.h>

#include <coordinates.hpp>

using namespace RobotTweezers;

void unittest_setCoordinates(void)
{
    Eigen::Matrix4f coords;
    coords << 
        1, 0, 0, 0,
        0, -1, 0, 0,
        0, 0, -1, -25,
        0, 0, 0, 1;
    Coordinates coordinates_test(coords);
    Eigen::Matrix3f expected_frame;
    expected_frame << 
        1, 0, 0,
        0, -1, 0,
        0, 0, -1;
    Eigen::Vector3f expected_origin;
    expected_origin << 0, 0, -25;

    TEST_ASSERT_EQUAL(coordinates_test.frame == expected_frame, true);
    TEST_ASSERT_EQUAL(coordinates_test.origin == expected_origin, true);
    TEST_ASSERT_EQUAL(coordinates_test.GetCoordinates() == coords, true);
}

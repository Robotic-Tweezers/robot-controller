#include <Arduino.h>
#include <unity.h>

#include <kinematic.hpp>
#include <config.hpp>

using namespace RobotTweezers;
using namespace Eigen;

float length1 = 10, length2 = 15;
float dh_table[3][3] = {
    {0, length1, HALF_PI},
    {0, 0, -HALF_PI},
    {0, length2, 0},
};

void Kinematic_setup(void)
{
    Kinematic::SetDegreesOfFreedom(3);
    Kinematic::SetMaxGravityTorque(0.05);
    Kinematic::SetBaseFrame(XRotation(PI));
}

void unittest_DenavitHartenbergTransform(void)
{
    Serial.println("Denavit Hartenberg transform");
    Coordinates result = Kinematic::DenavitHartenbergTransform(PI, 0, 0);
    Matrix4f expected;
    expected <<
        -1, 0, 0, 0,
        0, -1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    TEST_ASSERT_EQUAL(result.GetCoordinates().isApprox(expected, 0.01), true);
    result = Kinematic::DenavitHartenbergTransform(0, 0, PI);
    expected <<
        1, 0, 0, 0,
        0, -1, 0, 0,
        0, 0, -1, 0,
        0, 0, 0, 1;
    TEST_ASSERT_EQUAL(result.GetCoordinates().isApprox(expected, 0.01), true);
    result = Kinematic::DenavitHartenbergTransform(0, 10, 0);
    expected <<
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 10,
        0, 0, 0, 1;
    TEST_ASSERT_EQUAL(result.GetCoordinates().isApprox(expected, 0.01), true);
    result = Kinematic::DenavitHartenbergTransform(HALF_PI, 10, 0);
    expected <<
        0, -1, 0, 0,
        1, 0, 0, 0,
        0, 0, 1, 10,
        0, 0, 0, 1;
    TEST_ASSERT_EQUAL(result.GetCoordinates().isApprox(expected, 0.01), true);
}

void unittest_UpdateDenavitHartenbergTable(void)
{
    Kinematic::UpdateDenavitHartenbergTable(dh_table, Vector3f(1, 2, 3));
    TEST_ASSERT_EQUAL(dh_table[0][0], 1);
    TEST_ASSERT_EQUAL(dh_table[1][0], 2);
    TEST_ASSERT_EQUAL(dh_table[2][0], 3);
}

void unittest_DirectKinematics(void)
{
    Vector3f joint_state(0, 0, 0);
    Kinematic::UpdateDenavitHartenbergTable(dh_table, joint_state);
    Coordinates end_effector = Kinematic::DirectKinematics(dh_table);
    Matrix4f expected;
    expected <<
        1, 0, 0, 0,
        0, -1, 0, 0,
        0, 0, -1, -25,
        0, 0, 0, 1;
    TEST_ASSERT_EQUAL(end_effector.GetCoordinates().isApprox(expected, 0.01), true);
    joint_state << HALF_PI, 0, 0;
    Kinematic::UpdateDenavitHartenbergTable(dh_table, joint_state);
    end_effector = Kinematic::DirectKinematics(dh_table);
    expected <<
        0, -1, 0, 0,
        1, 0, 0, 0,
        0, 0, 1, 25,
        0, 0, 0, 1;
    expected = Coordinates(XRotation(PI), Vector3f(0, 0, 0)).GetCoordinates() * expected;
    TEST_ASSERT_EQUAL(end_effector.GetCoordinates().isApprox(expected, 0.01), true);
    joint_state << 0, HALF_PI, 0;
    Kinematic::UpdateDenavitHartenbergTable(dh_table, joint_state);
    end_effector = Kinematic::DirectKinematics(dh_table);
    expected <<
        0, 0, -1, -15,
        0, 1, 0, 0,
        1, 0, 0, 10,
        0, 0, 0, 1;
    expected = Coordinates(XRotation(PI), Vector3f(0, 0, 0)).GetCoordinates() * expected;
    TEST_ASSERT_EQUAL(end_effector.GetCoordinates().isApprox(expected, 0.01), true);
    joint_state << 0, 0, HALF_PI;
    Kinematic::UpdateDenavitHartenbergTable(dh_table, joint_state);
    end_effector = Kinematic::DirectKinematics(dh_table);
    expected <<
        0, -1, 0, 0,
        1, 0, 0, 0,
        0, 0, 1, 25,
        0, 0, 0, 1;
    expected = Coordinates(XRotation(PI), Vector3f(0, 0, 0)).GetCoordinates() * expected;
    TEST_ASSERT_EQUAL(end_effector.GetCoordinates().isApprox(expected, 0.01), true);
}

void unittest_InverseKinematics(void)
{
    float dh_table[3][3] = {
        {0, LENGTH1, PI / 2},
        {HALF_PI, 0, -PI / 2},
        {0, LENGTH2, 0},
    };
    //std::pair<Eigen::Vector3f, Eigen::Vector3f> RobotTweezers::Kinematic::InverseKinematics(const float roll, const float pitch, const float yaw)
    for (int i = 0; i < 100; i++)
    {
        dh_table[0][0] = (float)i * TWO_PI / 100.00;
        Coordinates test = Kinematic::DirectKinematics(dh_table);
        auto solutions = Kinematic::InverseKinematics(test);
        Serial.print(solutions.first(0));
        Serial.print(" ");
        Serial.print(solutions.second(0));
        Serial.print(" ");
        Serial.print(solutions.first(1));
        Serial.print(" ");
        Serial.print(solutions.second(1));
        Serial.print(" ");
        Serial.print(solutions.first(2));
        Serial.print(" ");
        Serial.print(solutions.second(2));
        Serial.print(" ");
    }
}

void unittest_Jacobian(void)
{
    Vector3f joint_state(0, 0, 0);
    Kinematic::UpdateDenavitHartenbergTable(dh_table, joint_state);
    MatrixXf expected(6, 3);
    expected <<
        0, -15, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 1, 0,
        -1, 0, -1;
    TEST_ASSERT_EQUAL(Kinematic::Jacobian(dh_table).isApprox(expected, 0.01), true);
    joint_state << 0, HALF_PI, 0;
    Kinematic::UpdateDenavitHartenbergTable(dh_table, joint_state);
    expected <<
        0, 0, 0,
        15, 0, 0,
        0, 15, 0,
        0, 0, -1,
        0, 1, 0,
        -1, 0, 0;
    TEST_ASSERT_EQUAL(Kinematic::Jacobian(dh_table).isApprox(expected, 0.01), true);
}

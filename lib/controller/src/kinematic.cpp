#include <kinematic.hpp>

int RobotTweezers::Kinematic::degrees_of_freedom = 3;

float RobotTweezers::Kinematic::max_gravity_torque = 0.05;

Eigen::Matrix3f RobotTweezers::Kinematic::base_frame = RobotTweezers::XRotation(PI);

void RobotTweezers::Kinematic::SetDegreesOfFreedom(int degrees_of_freedom)
{
    RobotTweezers::Kinematic::degrees_of_freedom = degrees_of_freedom;
}

void RobotTweezers::Kinematic::SetMaxGravityTorque(float max_gravity_torque)
{
    RobotTweezers::Kinematic::max_gravity_torque = max_gravity_torque;
}

void RobotTweezers::Kinematic::SetBaseFrame(Eigen::Matrix3f base_frame)
{
    RobotTweezers::Kinematic::base_frame = base_frame;
}

RobotTweezers::Coordinates RobotTweezers::Kinematic::DenavitHartenbergTransform(float theta, float length, float alpha)
{
    return RobotTweezers::Coordinates(ZRotation(theta) * XRotation(alpha), Eigen::Vector3f(0, 0, length));
}

void RobotTweezers::Kinematic::UpdateDenavitHartenbergTable(float dh_table[][3], const Eigen::Vector3f& theta)
{
    for (int i = 0; i < degrees_of_freedom; i++)
    {
        dh_table[i][0] = theta(i);
    }
}

RobotTweezers::Coordinates RobotTweezers::Kinematic::DirectKinematics(const float dh_table[][3])
{
    Coordinates end_effector(base_frame, Eigen::Vector3f(0, 0, 0));

    for (int i = 0; i < degrees_of_freedom; i++)
    {
        end_effector *= DenavitHartenbergTransform(dh_table[i][0], dh_table[i][1], dh_table[i][2]);
    }

    return end_effector;
}

Eigen::Vector3f InverseKinematics(const RobotTweezers::Coordinates &end_effector)
{
    return Eigen::Vector3f();
}

Eigen::MatrixXf RobotTweezers::Kinematic::Jacobian(const float dh_table[][3])
{
    Eigen::MatrixXf jacobian_matrix(6, degrees_of_freedom);
    RobotTweezers::Coordinates joint_coords(base_frame, Eigen::Vector3f(0, 0, 0));
    RobotTweezers::Coordinates end_effector = DirectKinematics(dh_table);
    
    for (int i = 0; i < degrees_of_freedom; i++)
    {
        Eigen::Vector3f k_axis = joint_coords.frame.block<3, 1>(0, 2);
        Eigen::Vector3f distance_to_end = end_effector.origin - joint_coords.origin;
        jacobian_matrix.col(i) <<
            Skew3(k_axis) * (distance_to_end),
            k_axis;
        joint_coords *= DenavitHartenbergTransform(dh_table[i][0], dh_table[i][1], dh_table[i][2]);
    }

    return jacobian_matrix;
}

Eigen::MatrixXf RobotTweezers::Kinematic::Jacobian(const float dh_table[][3], const Coordinates& end_effector)
{
    Eigen::MatrixXf jacobian_matrix(6, degrees_of_freedom);
    RobotTweezers::Coordinates joint_coords(base_frame, Eigen::Vector3f(0, 0, 0));

    for (int i = 0; i < degrees_of_freedom; i++)
    {
        Eigen::Vector3f k_axis = joint_coords.frame.block<3, 1>(0, 2);
        Eigen::Vector3f distance_to_end = end_effector.origin - joint_coords.origin;
        jacobian_matrix.col(i) <<
            Skew3(k_axis) * (distance_to_end),
            k_axis;
        joint_coords *= DenavitHartenbergTransform(dh_table[i][0], dh_table[i][1], dh_table[i][2]);
    }

    return jacobian_matrix;
}

Eigen::Vector3f RobotTweezers::Kinematic::GravityTorque(const Eigen::Vector3f& theta)
{
    // Torque acting on theta 2 when theta 2 = pi / 2
    return Eigen::Vector3f(0, -max_gravity_torque * sinf(theta(1)), 0);
}

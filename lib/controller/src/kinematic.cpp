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

std::pair<Eigen::Vector3f, Eigen::Vector3f> RobotTweezers::Kinematic::InverseKinematics(const RobotTweezers::Coordinates &end_effector)
{
    std::pair<Eigen::Vector3f, Eigen::Vector3f> theta;
    std::pair<Eigen::Vector2f, Eigen::Vector2f> theta13;
    Eigen::Vector3f i_unit_base, k_unit_base;
    Eigen::Vector3f i_unit_end, k_unit_end;
    Eigen::Vector3f u, v;
    Eigen::Vector2f theta2;
    float psi = PI / 2;

    i_unit_base = base_frame.block<3, 1>(0, 0);
    i_unit_end = end_effector.frame.block<3, 1>(0, 0);
    k_unit_base = base_frame.block<3, 1>(0, 2);
    k_unit_end = end_effector.frame.block<3, 1>(0, 2);
    v = Rotation(psi, i_unit_end) * k_unit_end;
    u = Rotation(psi, i_unit_base) * k_unit_base;

    // If base and end effector k-axes are in the same direction, one solution available
    if ((Skew3(k_unit_base) * k_unit_end).isApprox(Eigen::Vector3f(0, 0, 0)))
    {
        if (!IsEqual(k_unit_base.dot(k_unit_end), 1.00F, 0.01))
        {
            theta.first(1) = PI;
        }

        theta.first(2) = 0;
        theta.first(0) = Kahan::Problem2(k_unit_base, u, v);
        // Copying the single solution into a second row to ensure the theta
        // matrix is always the same size.
        theta.second = theta.first;
        return theta;
    }

    // Will return two possible solutions for theta 1 and 3
    theta13 = Kahan::Problem3(k_unit_base, k_unit_end, u, v);

    for (int i = 0; i < 2; i++)
    {
        Eigen::Matrix3f frame2 = end_effector.frame * Rotation(theta13.second(i), Eigen::Vector3f(0, 0, 1));
        Eigen::Matrix3f frame1 = base_frame * Rotation(theta13.first(i), Eigen::Vector3f(0, 0, 1)) * Rotation(psi, Eigen::Vector3f(1, 0, 0));

        Eigen::Vector3f i_unit1 = frame1 * Eigen::Vector3f(1, 0, 0);
        Eigen::Vector3f k_unit1 = frame1 * Eigen::Vector3f(0, 0, 1);
        Eigen::Vector3f i_unit2 = frame2 * Eigen::Vector3f(1, 0, 0);

        theta2(i) = Kahan::Problem2(k_unit1, i_unit1, i_unit2);
    }
    
    theta.first = Eigen::Vector3f(theta13.first(0), theta2(0), theta13.second(0));
    theta.second = Eigen::Vector3f(theta13.first(1), theta2(1), theta13.second(1));
    return theta;
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

#include <kinematic.hpp>

Eigen::Matrix4f RobotTweezers::Kinematic::DenavitHartenbergTransform(float theta, float length, float alpha)
{
    Eigen::Matrix4f dh_transform;
    dh_transform <<
        ZRotation(theta) * XRotation(alpha), Eigen::Vector3f(0, 0, length),
        0, 0, 0, 1;
    return dh_transform;
}

void RobotTweezers::Kinematic::UpdateDHTable(const Eigen::Vector3f& theta)
{
    dh_table[0][0] = theta(0);
    dh_table[1][0] = theta(1);
    dh_table[2][0] = theta(2);
}

RobotTweezers::Kinematic::Kinematic(void)
{
    UpdateDHTable(Eigen::Vector3f(0, 0, 0));
}

RobotTweezers::Kinematic::Kinematic(const Eigen::Vector3f& theta)
{
    UpdateDHTable(theta);
}

Eigen::Matrix4f RobotTweezers::Kinematic::DirectKinematics(const Eigen::Vector3f& theta)
{
    Eigen::Matrix4f end_effector;
    origins[0] << 0, 0, 0;
    k_axes[0] << 0, 0, -1;
    end_effector << 
        XRotation(PI), Eigen::Vector3f(0, 0 ,0),
        0, 0, 0, 1;
    
    UpdateDHTable(theta);

    for (int i = 0; i < 3; i++)
    {
        end_effector *= DenavitHartenbergTransform(dh_table[i][0], dh_table[i][1], dh_table[i][2]);
        origins[i + 1] << end_effector(0, 3), end_effector(1, 3), end_effector(2, 3);
        k_axes[i + 1] << end_effector(0, 2), end_effector(1, 2), end_effector(2, 2);
    }

    return end_effector;
}

Eigen::MatrixXf RobotTweezers::Kinematic::Jacobian(void)
{
    Eigen::MatrixXf jacobian_matrix(6, 3);
    for (int i = 0; i < 3; i++)
    {
        Eigen::Vector3f distance_to_end_eff = origins[3] - origins[i];
        Eigen::Vector3f angular_contrib = Skew3(k_axes[i]) * distance_to_end_eff;
        jacobian_matrix.col(i) <<
            angular_contrib,
            k_axes[i];
    }
    // Address home position sigularity here
    if (ApproxEqual(jacobian_matrix(5, 0), jacobian_matrix(5, 2), 0.05))
    {
        jacobian_matrix(5, 0) = 0.00;
    }

    return jacobian_matrix;
}

Eigen::Vector3f RobotTweezers::Kinematic::GravityTorque(const Eigen::Vector3f& theta)
{
    // Torque acting on theta 2 when theta 2 = pi / 2
    const float max_gravity_torque = 0.05;
    return Eigen::Vector3f(0, -max_gravity_torque * sinf(theta[1]), 0);
}
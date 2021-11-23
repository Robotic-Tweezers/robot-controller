#include <kinematic.hpp>

using namespace Eigen;
using namespace robot_tweezers;

Matrix4f Kinematic::denavitHartenbergTransform(float theta, float length, float alpha)
{
    Matrix4f dh_transform;
    dh_transform <<
        zRotation(theta) * xRotation(alpha), Vector3f(0, 0, length),
        0, 0, 0, 1;
    return dh_transform;
}

void Kinematic::updateDHTable(float theta[])
{
    dh_table[0][0] = theta[0];
    dh_table[1][0] = theta[1];
    dh_table[2][0] = theta[2];
}

Kinematic::Kinematic(float theta[])
{
    updateDHTable(theta);
}

Coordinates Kinematic::directKinematics(float theta[])
{
    Matrix4f end_effector;
    origins[0] << 0, 0, 0;
    k_axes[0] << 0, 0, -1;
    end_effector << 
        xRotation(PI), Vector3f(0, 0 ,0),
        0, 0, 0, 1;
    
    updateDHTable(theta);

    for (int i = 0; i < 3; i++)
    {
        end_effector *= denavitHartenbergTransform(dh_table[i][0], dh_table[i][1], dh_table[i][2]);
        origins[i + 1] << end_effector(0, 3), end_effector(1, 3), end_effector(2, 3);
        k_axes[i + 1] << end_effector(0, 2), end_effector(1, 2), end_effector(2, 2);
    }

    return Coordinates(end_effector);
}

MatrixXf Kinematic::jacobian(float theta[])
{
    MatrixXf jacobian_matrix(6, 3);
    for (int i = 0; i < 3; i++)
    {
        /*
        Vector3f angular_contrib = k_axes[i].cross3(origins[3] - origins[i]);
        jacobian_matrix(0, i) = angular_contrib(0);
        jacobian_matrix(1, i) = angular_contrib(1);
        jacobian_matrix(2, i) = angular_contrib(2);
        jacobian_matrix(3, i) = k_axes[i](0);
        jacobian_matrix(4, i) = k_axes[i](1);
        jacobian_matrix(5, i) = k_axes[i](2);
        */
    }

    return jacobian_matrix;
}
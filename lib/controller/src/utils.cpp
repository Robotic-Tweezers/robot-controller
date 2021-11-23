#include <utils.hpp>
extern "C"
{
#include <math.h>
}

Eigen::Matrix3f robot_tweezers::skew3(Eigen::Vector3f s)
{
    Eigen::Matrix3f skew_sym;
    skew_sym << 
        0, -s(2), s(1),
        s(2), 0, -s(0),
        -s(1), s(0),  0;
    return skew_sym;
}

Eigen::Matrix3f robot_tweezers::xRotation(float alpha)
{
    Eigen::Matrix3f rotation;
    rotation << 
        1, 0, 0,
        0, cosf(alpha), -sinf(alpha),
        0, sinf(alpha), cosf(alpha);
    return rotation;
}

Eigen::Matrix3f robot_tweezers::zRotation(float theta)
{
    Eigen::Matrix3f rotation;
    rotation << 
        cosf(theta), -sinf(theta), 0,
        sinf(theta), cosf(theta), 0,
        0, 0, 1;
    return rotation;
}

float robot_tweezers::Kahan::problem1(Eigen::Vector3f s, Eigen::Vector3f t)
{
    Eigen::Vector3f u = s.normalized() - t.normalized();
    Eigen::Vector3f v = s.normalized() + t.normalized();
    return atanf(u.norm() / v.norm()) * 2.00F;
}

float robot_tweezers::Kahan::problem2(Eigen::Vector3f s_unit, Eigen::Vector3f u, Eigen::Vector3f v, bool* valid)
{
    u.normalize();
    v.normalize();

    if (s_unit.dot(u) != s_unit.dot(v))
    {
        *valid = false;
        return 0.00F;
    }

    return 0.00F;
}

std::pair<float[2], float[2]> robot_tweezers::Kahan::problem3(Eigen::Vector3f s_unit, Eigen::Vector3f t_unit, Eigen::Vector3f u, Eigen::Vector3f v)
{
    std::pair<float[2], float[2]> ret;
    return ret;
}

float robot_tweezers::Kahan::problem4(float a, float b, float c)
{
    return 0.00F;
}
#include <utils.hpp>
#include <math.h>

using namespace std;
using namespace Eigen;
using namespace robot_tweezers;

Matrix3f skew3(Vector3f s)
{
    Matrix3f skew_sym;
    skew_sym << 
        0, -s(2), s(1),
        s(2), 0, -s(0),
        -s(1), s(0),  0;
    return skew_sym;
}

Matrix3f xRotation(float alpha)
{
    Matrix3f rotation;
    rotation << 
        1, 0, 0,
        0, cosf(alpha), -sinf(alpha),
        0, sinf(alpha), cosf(alpha);
    return rotation;
}

Matrix3f zRotation(float theta)
{
    Matrix3f rotation;
    rotation << 
        cosf(theta), -sinf(theta), 0,
        sinf(theta), cosf(theta), 0,
        0, 0, 1;
    return rotation;
}

float Kahan::problem1(Vector3f s, Vector3f t)
{
    Vector3f u = s.normalized() - t.normalized();
    Vector3f v = s.normalized() + t.normalized();
    return atanf(u.norm() / v.norm()) * 2.00F;
}

float Kahan::problem2(Vector3f s_unit, Vector3f u, Vector3f v, bool* valid)
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

pair<float[2], float[2]> Kahan::problem3(Vector3f s_unit, Vector3f t_unit, Vector3f u, Vector3f v)
{
    pair<float[2], float[2]> ret;
    return ret;
}

float Kahan::problem4(float a, float b, float c)
{
    return 0.00F;
}
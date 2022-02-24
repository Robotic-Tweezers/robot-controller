#include <utils.hpp>
extern "C"
{
#include <math.h>
}

static float precision = 0.01;

Eigen::Matrix3f RobotTweezers::Skew3(const Eigen::Vector3f &s)
{
    Eigen::Matrix3f skew_sym;
    skew_sym << 0, -s(2), s(1),
        s(2), 0, -s(0),
        -s(1), s(0), 0;
    return skew_sym;
}

Eigen::Matrix3f RobotTweezers::XRotation(float alpha)
{
    Eigen::Matrix3f rotation;
    rotation << 1, 0, 0,
        0, cosf(alpha), -sinf(alpha),
        0, sinf(alpha), cosf(alpha);
    return rotation;
}

Eigen::Matrix3f RobotTweezers::ZRotation(float theta)
{
    Eigen::Matrix3f rotation;
    rotation << cosf(theta), -sinf(theta), 0,
        sinf(theta), cosf(theta), 0,
        0, 0, 1;
    return rotation;
}

Eigen::Matrix3f RobotTweezers::Rotation(float angle, Eigen::Vector3f axis)
{
    return Eigen::AngleAxisf(angle, axis).toRotationMatrix();
}

Eigen::Matrix6f RobotTweezers::VectorToDiagnol6(const float vector[])
{
    Eigen::Matrix6f diagnol;
    diagnol << vector[0], 0, 0, 0, 0, 0,
        0, vector[1], 0, 0, 0, 0,
        0, 0, vector[2], 0, 0, 0,
        0, 0, 0, vector[3], 0, 0,
        0, 0, 0, 0, vector[4], 0,
        0, 0, 0, 0, 0, vector[5];
    return diagnol;
}

Eigen::Matrix3f RobotTweezers::EulerXYZToRotation(const float x, const float y, const float z)
{
    Eigen::AngleAxisf x_rotation(x, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf y_rotation(y, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf z_rotation(z, Eigen::Vector3f::UnitZ());
    return (x_rotation * y_rotation * z_rotation).toRotationMatrix();
}

float RobotTweezers::Kahan::Problem1(const Eigen::Vector3f &s, const Eigen::Vector3f &t)
{
    s.normalized();
    t.normalized();
    return atanf((s - t).norm() / (s + t).norm()) * 2.00F;
}

float RobotTweezers::Kahan::Problem2(const Eigen::Vector3f &s_unit, Eigen::Vector3f &u, Eigen::Vector3f &v, bool *valid)
{
    Eigen::Vector3f q;
    Eigen::Vector3f r;
    float theta;

    u.normalize();
    v.normalize();

    if (std::abs(s_unit.dot(u) - s_unit.dot(v)) <= precision)
    {
        *valid = false;
        return 0.00F;
    }

    q = Skew3(s_unit) * (u - v);
    r = Skew3(s_unit) * (u + v);
    theta = 2 * atan(q.norm() / r.norm());

    return v.dot(Skew3(s_unit) * (u - v)) < 0 ? theta : -theta;
}

std::tuple<float, float, float, float> RobotTweezers::Kahan::Problem3(
    const Eigen::Vector3f &s_unit, const Eigen::Vector3f &t_unit, Eigen::Vector3f &u, Eigen::Vector3f &v)
{
    u.normalized();
    v.normalized();
    Eigen::Vector3f st_cross = Skew3(s_unit) * t_unit;
    float st_dot = s_unit.dot(t_unit);
    float us_dot = u.dot(s_unit);
    float vt_dot = v.dot(t_unit);
    bool valid;

    if (s_unit.isApprox(t_unit, precision) && u.isApprox(v, precision))
    {
        return std::tuple<float, float, float, float>(0, 0, 0, 0);
    }

    float alpha = (us_dot - st_dot * vt_dot) / st_cross.dot(st_cross);
    float beta = (vt_dot - st_dot * us_dot) / st_cross.dot(st_cross);

    Eigen::Vector3f z = alpha * s_unit + beta * t_unit;

    if (1 - pow(z.norm(), 2) < 0)
    {
        return std::tuple<float, float, float, float>(0, 0, 0, 0);
    }

    Eigen::Vector3f w_unit1 = z + (sqrt(1 - pow(z.norm(), 2)) * st_cross / st_cross.norm());
    Eigen::Vector3f w_unit2 = z - (sqrt(1 - pow(z.norm(), 2)) * st_cross / st_cross.norm());

    return std::tuple<float, float, float, float>(
        Kahan::Problem2(s_unit, u, w_unit1, &valid), Kahan::Problem2(s_unit, u, w_unit2, &valid),
        Kahan::Problem2(t_unit, v, w_unit1, &valid), Kahan::Problem2(t_unit, v, w_unit2, &valid));
}

float RobotTweezers::Kahan::Problem4(float a, float b, float c)
{
    if (a + b >= c && c >= std::abs(a - b))
    {
        return 2 * atan(sqrt(pow(a + b, 2) - pow(c, 2)) / sqrt(pow(c, 2) - pow(a - b, 2)));
    }
    else
    {
        return -1;
    }
}

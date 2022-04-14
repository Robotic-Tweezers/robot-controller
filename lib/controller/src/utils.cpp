#include <utils.hpp>
#include <limits.h>
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

Eigen::Matrix3f RobotTweezers::ToDiagnol3(const float x, const float y, const float z)
{
    Eigen::Matrix3f diagnol;
    diagnol << x, 0, 0,
        0, y, 0,
        0, 0, z;
    return diagnol;
}

Eigen::Matrix6f RobotTweezers::ToDiagnol6(const float x, const float y, const float z, const float r, const float p, const float w)
{
    Eigen::Matrix6f diagnol;
    diagnol << x, 0, 0, 0, 0, 0,
        0, y, 0, 0, 0, 0,
        0, 0, z, 0, 0, 0,
        0, 0, 0, r, 0, 0,
        0, 0, 0, 0, p, 0,
        0, 0, 0, 0, 0, w;
    return diagnol;
}

Eigen::Matrix3f RobotTweezers::EulerXYZToRotation(const float x, const float y, const float z)
{
    Eigen::AngleAxisf x_rotation(x, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf y_rotation(y, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf z_rotation(z, Eigen::Vector3f::UnitZ());
    
    return Eigen::Quaternionf(x_rotation * y_rotation * z_rotation).toRotationMatrix();
}

Eigen::Matrix3f RobotTweezers::EulerXZYToRotation(const float x, const float y, const float z)
{
    Eigen::AngleAxisf x_rotation(x, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf y_rotation(y, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf z_rotation(z, Eigen::Vector3f::UnitZ());
    
    return Eigen::Quaternionf(x_rotation * z_rotation * y_rotation).toRotationMatrix();
}

bool RobotTweezers::IsEqual(float a, float b, float precision)
{
    return std::abs(a - b) <= precision;
}

float RobotTweezers::Kahan::Problem1(const Eigen::Vector3f &s, const Eigen::Vector3f &t)
{
    Eigen::Vector3f s_unit = s.normalized();
    Eigen::Vector3f t_unit = t.normalized();
    return 2.00F * atanf((s_unit - t_unit).norm() / (s_unit + t_unit).norm());
}

float RobotTweezers::Kahan::Problem2(const Eigen::Vector3f &s_unit, Eigen::Vector3f &u, Eigen::Vector3f &v)
{
    Eigen::Vector3f u_unit;
    Eigen::Vector3f v_unit;
    Eigen::Vector3f q;
    Eigen::Vector3f r;
    float theta;

    u_unit = u.normalized();
    v_unit = v.normalized();
    
    if (IsEqual(s_unit.dot(u_unit), s_unit.dot(v_unit), precision) == false)
    {
        // Solution not possible;
        return NaN;
    }

    q = Skew3(s_unit) * (u_unit - v_unit);
    r = Skew3(s_unit) * (u_unit + v_unit);
    theta = 2 * atan(q.norm() / r.norm());
    return v_unit.dot(q) < 0 ? -theta : theta;
}

std::pair<Eigen::Vector2f, Eigen::Vector2f> RobotTweezers::Kahan::Problem3(
    const Eigen::Vector3f &s_unit, const Eigen::Vector3f &t_unit, Eigen::Vector3f &u, Eigen::Vector3f &v)
{
    Eigen::Vector3f u_unit, v_unit;
    Eigen::Vector3f w_unit1, w_unit2;
    Eigen::Vector3f st_cross;
    Eigen::Vector3f z;
    float st_dot, us_dot, vt_dot;
    float alpha, beta;
    float z_adj;

    u_unit = u.normalized();
    v_unit = v.normalized();
    st_cross = Skew3(s_unit) * t_unit;
    st_dot = s_unit.dot(t_unit);
    us_dot = u_unit.dot(s_unit);
    vt_dot = v_unit.dot(t_unit);

    if (s_unit.isApprox(t_unit, precision) && u_unit.isApprox(v_unit, precision))
    {
        return std::pair<Eigen::Vector2f, Eigen::Vector2f>(Eigen::Vector2f(0, 0), Eigen::Vector2f(0, 0));
    }

    alpha = (us_dot - st_dot * vt_dot) / st_cross.dot(st_cross);
    beta = (vt_dot - st_dot * us_dot) / st_cross.dot(st_cross);

    z = alpha * s_unit + beta * t_unit;
    z_adj = 1 - pow(z.norm(), 2);

    if (z_adj < 0)
    {
        return std::pair<Eigen::Vector2f, Eigen::Vector2f>(Eigen::Vector2f(NaN, NaN), Eigen::Vector2f(NaN, NaN));
    }

    w_unit1 = z + (sqrt(z_adj) * st_cross / st_cross.norm());
    w_unit2 = z - (sqrt(z_adj) * st_cross / st_cross.norm());

    return std::pair<Eigen::Vector2f, Eigen::Vector2f>(
        Eigen::Vector2f(Kahan::Problem2(s_unit, u_unit, w_unit1), Kahan::Problem2(s_unit, u_unit, w_unit2)),
        Eigen::Vector2f(Kahan::Problem2(t_unit, v_unit, w_unit1), Kahan::Problem2(t_unit, v_unit, w_unit2)));
}

float RobotTweezers::Kahan::Problem4(float a, float b, float c)
{
    float a_plus_b = a + b;
    float a_minus_b = a - b;
    return (a_plus_b >= c && c >= std::abs(a_minus_b)) 
        ? 2.00F * atan(sqrt(pow(a_plus_b, 2) - pow(c, 2)) / sqrt(pow(c, 2) - pow(a_minus_b, 2))) : NaN;
}

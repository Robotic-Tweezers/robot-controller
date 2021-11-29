#include <coordinates.hpp>

RobotTweezers::Coordinates::Coordinates(void) { }

RobotTweezers::Coordinates::Coordinates(const Eigen::Matrix3f& frame, Eigen::Vector3f& origin)
: frame(frame), origin(origin) { }

RobotTweezers::Coordinates::Coordinates(const Eigen::Matrix4f& coordinates)
{
    setCoordinates(coordinates);
}

void RobotTweezers::Coordinates::setCoordinates(const Eigen::Matrix4f& coordinates)
{
    for (int i = 0; i < 3; i++)
    {
        this->origin(i) = coordinates(i, 3);
        for (int j = 0; j < 3; j++)
        {
            this->frame(i, j) = coordinates(i, j);
        }
    }
}

Eigen::Matrix4f RobotTweezers::Coordinates::getCoordinates(void)
{
    Eigen::Matrix4f matrix;
    matrix << 
        frame, origin,
        0, 0, 0, 1;
    return matrix;
}

Eigen::Vector6f RobotTweezers::Coordinates::error(const Coordinates& a, const Coordinates& b)
{
    Eigen::Vector3f rotation_error(0, 0, 0);
    if (!a.frame.transpose().isApprox(b.frame, 0.001))
    {
        Eigen::AngleAxisf rotation_error_angle_axis(a.frame.transpose() * b.frame);
        rotation_error << rotation_error_angle_axis.angle() * rotation_error_angle_axis.axis();
    }
    
    Eigen::Vector6f error;
    error << 
        a.origin - b.origin,
        rotation_error;
    return error;
}
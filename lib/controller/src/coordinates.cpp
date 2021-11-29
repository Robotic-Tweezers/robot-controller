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

Eigen::Vector6f RobotTweezers::Coordinates::operator-(const Coordinates& op)
{
    Eigen::Vector3f rotation_error(0, 0, 0);
    if (!frame.transpose().isApprox(op.frame, 0.001))
    {
        Eigen::AngleAxisf rotation_error_angle_axis(frame.transpose() * op.frame);
        rotation_error << rotation_error_angle_axis.angle() * rotation_error_angle_axis.axis();
    }
    
    Eigen::Vector6f error;
    error << 
        origin - op.origin,
        rotation_error;
    return error;
}
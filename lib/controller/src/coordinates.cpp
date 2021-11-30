#include <coordinates.hpp>

RobotTweezers::Coordinates::Coordinates(void) { }

RobotTweezers::Coordinates::Coordinates(const Eigen::Matrix3f& frame, Eigen::Vector3f& origin)
: frame(frame), origin(origin) { }

RobotTweezers::Coordinates::Coordinates(const Eigen::Matrix4f& coordinates)
{
    *this = coordinates;
}

RobotTweezers::Coordinates& RobotTweezers::Coordinates::operator=(const RobotTweezers::Coordinates& op)
{
    frame = op.frame;
    origin = op.origin;
    return *this;
}

RobotTweezers::Coordinates& RobotTweezers::Coordinates::operator=(const Eigen::Matrix4f& op)
{
    frame = op.block<3, 3>(0, 0);
    origin = op.block<3, 1>(0, 3);
    return *this;
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
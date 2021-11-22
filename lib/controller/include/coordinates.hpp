#ifndef _COORDINATES_HPP_
#define _COORDINATES_HPP_

#include <ArduinoEigen.h>

namespace robot_tweezers
{
    class Coordinates
    {
        public:

        Eigen::Matrix3f frame;
        
        Eigen::Vector3f origin;

        Coordinates(Eigen::Matrix3f& frame, Eigen::Vector3f& origin);

        Coordinates(Eigen::Matrix4f& coordinates);

        void setMatrix(Eigen::Matrix4f& coordinates);

        Eigen::Matrix4f getMatrix();
    };
}

#endif // _COORDINATES_HPP_
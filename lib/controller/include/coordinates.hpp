#ifndef _COORDINATES_HPP_
#define _COORDINATES_HPP_

#include <ArduinoEigen.h>

namespace robot_tweezers
{
    class Coordinates
    {
        public:

        /**
         * @brief The coordinate frame, represents the coordinates orientation
         * 
         */
        Eigen::Matrix3f frame;
        
        /**
         * @brief The point indicating the position of the coordinate
         * 
         */
        Eigen::Vector3f origin;

        /**
         * @brief Construct a new Coordinates object
         * 
         */
        Coordinates();

        /**
         * @brief Construct a new Coordinates object
         * 
         * @param frame The coordinate frame
         * @param origin The coordinate origin
         */
        Coordinates(Eigen::Matrix3f& frame, Eigen::Vector3f& origin);

        /**
         * @brief Construct a new Coordinates object
         * 
         * @param coordinates The 4x4 matrix representing the coordinate frame and origin
         */
        Coordinates(Eigen::Matrix4f& coordinates);

        /**
         * @brief Sets
         * 
         * @param coordinates 
         */
        void setCoordinates(Eigen::Matrix4f& coordinates);

        Eigen::Matrix4f getCoordinates();
    };
}

#endif // _COORDINATES_HPP_
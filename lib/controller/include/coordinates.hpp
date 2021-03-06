#ifndef _COORDINATES_HPP_
#define _COORDINATES_HPP_

#include <ArduinoEigen.h>
#include <types.hpp>
#include <utils.hpp>

namespace RobotTweezers
{
    struct Coordinates
    {
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
        Coordinates(void);

        /**
         * @brief Construct a new Coordinates object
         * 
         * @param frame The coordinate frame
         * @param origin The coordinate origin
         */
        Coordinates(const Eigen::Matrix3f frame, Eigen::Vector3f origin);

        /**
         * @brief Construct a new Coordinates object
         * 
         * @param coordinates The 4x4 matrix representing the coordinate frame and origin
         */
        Coordinates(const Eigen::Matrix4f& coordinates);

        /**
         * @brief Sets the frame and origin to another coordinate system
         * 
         * @param op Original Coordinates 
         */
        Coordinates& operator=(const Coordinates& op);

        /**
         * @brief Sets the frame and origin represented in the 4x4 coordinate matrix
         * 
         * @param op A 4x4 matrix representing the frame and origin
         */
        Coordinates& operator=(const Eigen::Matrix4f& op);

        /**
         * @brief Get the Coordinates as a 4x4 matrix
         * 
         * @return Eigen::Matrix4f 
         */
        Eigen::Matrix4f GetCoordinates(void);

        /**
         * @brief Calculates the translational and rotational error as a 6 dimensional vector
         * 
         * @todo Verify this function, seeing issues with controller when using this to calculate error 
         * 
         * @param op Coordinates to subtract from object
         * @return Eigen::Vector6f 
         */
        Eigen::Vector6f operator-(const Coordinates& op);

        /**
         * @brief Multiplication operator for coordinates
         * 
         * @param op            Operand
         * @return Coordinates  Product of coordinates
         */
        Coordinates operator*(const Coordinates& op);
        
        /**
         * @brief Multiplication operator for coordinates
         * 
         * @param op            Operand
         * @return Coordinates  Product of coordinates
         */
        Coordinates& operator*=(const Coordinates& op);
    };
}

#endif // _COORDINATES_HPP_
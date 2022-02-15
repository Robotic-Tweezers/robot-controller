#ifndef _KINEMATIC_HPP_
#define _KINEMATIC_HPP_

#include <coordinates.hpp>
#include <utils.hpp>
#include <types.hpp>

namespace RobotTweezers
{
    class Kinematic
    {
    private:
        /**
         * @brief Holds the xyz origin of each joint variable and end effector
         *
         */
        Eigen::Vector3f origins[4];

        /**
         * @brief Holds the k axis unit vector for each joint variable and end effector
         *
         */
        Eigen::Vector3f k_axes[4];

        /**
         * @brief The DH transformation parameters for the wrist manipulator (a is omitted)
         *
         */
        float dh_table[3][3] = {
            {0, LENGTH1, PI / 2},
            {0, 0, -PI / 2},
            {0, LENGTH2, 0},
        };

        /**
         * @brief Denavit Hartenberg transform for spherical wrist (Assumes offset value is zero in all cases)
         *
         * @param theta     The z-rotation parameter
         * @param length    The z-translation parameter
         * @param alpha     The x-rotation parameter
         * @return Eigen::Matrix4f
         */
        Eigen::Matrix4f DenavitHartenbergTransform(float theta, float length, float alpha);

        /**
         * @brief Updates the wrist dh table with the current joint variable state
         *
         * @param theta
         */
        inline void UpdateDHTable(const Eigen::Vector3f &theta);

    public:
        /**
         * @brief Construct a new Kinematic object
         *
         */
        Kinematic(void);

        /**
         * @brief Construct a new Kinematic object
         *
         * @param theta Joint variable initial state
         */
        Kinematic(const Eigen::Vector3f &theta);

        /**
         * @brief Direct kinematics for a 3-degree of freedom spherical wrist
         *
         * @param theta Current joint variable state
         * @return Eigen::Matrix4f
         */
        Eigen::Matrix4f DirectKinematics(const Eigen::Vector3f &theta);

        /**
         * @brief Calculates the Jacobian matrix for a spherical wrist based on the current joint state
         *
         * @param theta Current joint variable state
         * @return Eigen::MatrixXf
         */
        Eigen::MatrixXf Jacobian(void);

        /**
         * @brief Approximated as a sinusoidal torque acting on theta 2
         *
         * @param theta Current joint variable state
         * @return Eigen::Vector3f
         */
        Eigen::Vector3f GravityTorque(const Eigen::Vector3f &theta);
    };
}

#endif // _KINEMATIC_HPP_
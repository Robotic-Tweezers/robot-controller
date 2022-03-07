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
        static int degrees_of_freedom;
        static float max_gravity_torque;
        static Eigen::Matrix3f base_frame;

    public:
        static void SetDegreesOfFreedom(int degrees_of_freedom);

        static void SetMaxGravityTorque(float max_gravity_torque);

        static void SetBaseFrame(Eigen::Matrix3f base_frame);

        /**
         * @brief Denavit Hartenberg transform for spherical wrist (Assumes x-offset value is zero in all cases)
         *
         * @param theta     The z-rotation parameter
         * @param length    The z-translation parameter
         * @param alpha     The x-rotation parameter
         * @return RobotTweezers::Coordinates
         */
        static RobotTweezers::Coordinates DenavitHartenbergTransform(float theta, float length, float alpha);

        /**
         * @brief Updates the wrist dh table with the current joint variable state
         *
         * @param theta
         */
        static void UpdateDenavitHartenbergTable(float dh_table[][3], const Eigen::Vector3f &theta);

        /**
         * @brief Direct kinematics for a 3-degree of freedom spherical wrist
         *
         * @param theta Current joint variable state
         * @return Coordinates
         */
        static Coordinates DirectKinematics(const float dh_table[][3]);

        static std::pair<Eigen::Vector3f, Eigen::Vector3f> InverseKinematics(const RobotTweezers::Coordinates &end_effector);

        /**
         * @brief 
         * 
         * @param dh_table 
         * @return Eigen::MatrixXf 
         */
        static Eigen::MatrixXf Jacobian(const float dh_table[][3]);

        /**
         * @brief Calculates the Jacobian matrix for a spherical wrist based on the current joint state
         *
         * @param theta Current joint variable state
         * @return Eigen::MatrixXf
         */
        static Eigen::MatrixXf Jacobian(const float dh_table[][3], const Coordinates& end_effector);

        /**
         * @brief Approximated as a sinusoidal torque acting on theta 2
         *
         * @param theta Current joint variable state
         * @return Eigen::Vector3f
         */
        static Eigen::Vector3f GravityTorque(const Eigen::Vector3f &theta);
    };
}

#endif // _KINEMATIC_HPP_
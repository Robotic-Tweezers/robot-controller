#ifndef _KINEMATIC_HPP_
#define _KINEMATIC_HPP_

#include <coordinates.hpp>
#include <utils.hpp>
#include <types.hpp>

namespace robot_tweezers
{
    class Kinematic
    {
        private:

        Eigen::Vector3f origins[4];
        Eigen::Vector3f k_axes[4];
        float dh_table[3][3] = {
            {0, LENGTH1, PI / 2},
            {0, 0, -PI / 2},
            {0, LENGTH2, 0},
        };

        /**
         * @brief Denavit Hartenberg transform for spherical wrist (Assumes offset value is zero in all cases)
         * 
         * @param theta 
         * @param length 
         * @param alpha 
         * @return Eigen::Matrix4f 
         */
        Eigen::Matrix4f denavitHartenbergTransform(float theta, float length, float alpha);

        /**
         * @brief 
         * 
         * @param theta 
         */
        inline void updateDHTable(float theta[]);
        
        public:

        Kinematic(float theta[]);

        Eigen::Matrix4f directKinematics(float theta[]);

        Eigen::MatrixXf jacobian(float theta[]);

        Eigen::Vector3f gravityTorque(float theta[]);
    };
}

#endif // _KINEMATIC_HPP_
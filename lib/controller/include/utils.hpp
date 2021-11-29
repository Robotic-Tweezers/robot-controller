#ifndef _UTILS_HPP_
#define _UTILS_HPP_
 
#include <ArduinoEigen.h>
#include <types.hpp>

namespace RobotTweezers
{
    /**
     * @brief Print function for a Eigen matrix
     * 
     */
#ifdef DEBUG
    auto print = [](auto& matrix) -> void
    {
        for (int i = 0; i < matrix.rows(); i++)
        {
            for (int j = 0; j < matrix.cols(); j++)
            {
                Serial.print(matrix(i, j));
                Serial.print(" ");
            }
            
            Serial.println(); 
        }
    };
#else
    auto print = [](auto& matrix) -> void {};
#endif // DEBUG

    class Kahan
    {
        public: 

        static float problem1(const Eigen::Vector3f& s, const Eigen::Vector3f& t);

        static float problem2(const Eigen::Vector3f& s_unit, Eigen::Vector3f& u, Eigen::Vector3f& v, bool* valid);

        static std::pair<float[2], float[2]> problem3
            (const Eigen::Vector3f& s_unit, const Eigen::Vector3f& t_unit, const Eigen::Vector3f& u, const Eigen::Vector3f& v);
        
        static float problem4(float a, float b, float c);
    };
    
    /**
     * @brief Calculates a skew symmetric matrix from the elements of a 3D vector
     * 
     * @param s A 3D input vector
     * @return Eigen::Matrix3f 
     */
    Eigen::Matrix3f skew3(const Eigen::Vector3f& s);

    /**
     * @brief Calculates a rotation matrix about the unit x-axis
     * 
     * @param alpha Rotation angle 
     * @return Eigen::Matrix3f 
     */
    Eigen::Matrix3f xRotation(float alpha);

    /**
     * @brief Calculates a rotation matrix about the unit z-axis
     * 
     * @param theta Rotation angle 
     * @return Eigen::Matrix3f 
     */
    Eigen::Matrix3f zRotation(float theta);

    /**
     * @brief Calculates a diagnol matrix from a vector
     * 
     * @param vector Input vector
     * @return Eigen::Matrix6f 
     */
    Eigen::Matrix6f vectorToDiagnol6(const float vector[]);
}

#endif // _UTILS_HPP_
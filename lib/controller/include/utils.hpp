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

        static float Problem1(const Eigen::Vector3f& s, const Eigen::Vector3f& t);

        static float Problem2(const Eigen::Vector3f& s_unit, Eigen::Vector3f& u, Eigen::Vector3f& v, bool* valid);

        static std::pair<float[2], float[2]> Problem3
            (const Eigen::Vector3f& s_unit, const Eigen::Vector3f& t_unit, const Eigen::Vector3f& u, const Eigen::Vector3f& v);
        
        static float Problem4(float a, float b, float c);
    };
    
    /**
     * @brief Calculates a skew symmetric matrix from the elements of a 3D vector
     * 
     * @param s A 3D input vector
     * @return Eigen::Matrix3f 
     */
    Eigen::Matrix3f Skew3(const Eigen::Vector3f& s);

    /**
     * @brief Calculates a rotation matrix about the unit x-axis
     * 
     * @param alpha Rotation angle 
     * @return Eigen::Matrix3f 
     */
    Eigen::Matrix3f XRotation(float alpha);

    /**
     * @brief Calculates a rotation matrix about the unit z-axis
     * 
     * @param theta Rotation angle 
     * @return Eigen::Matrix3f 
     */
    Eigen::Matrix3f ZRotation(float theta);

    /**
     * @brief Calculates a diagnol matrix from a vector
     * 
     * @param vector Input vector
     * @return Eigen::Matrix6f 
     */
    Eigen::Matrix6f VectorToDiagnol6(const float vector[]);

    /**
     * @brief Compare two floats for near equality
     * 
     * @param a operand 1
     * @param b operand 2
     * @param epsilon threshold value
     * @return true a and b are nearly equal 
     * @return false a and b are not nearly equal
     */
    bool ApproxEqual(float a, float b, float epsilon);
}

#endif // _UTILS_HPP_
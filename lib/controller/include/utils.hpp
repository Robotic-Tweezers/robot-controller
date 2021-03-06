#ifndef _UTILS_HPP_
#define _UTILS_HPP_

#include <ArduinoEigen.h>
#include <types.hpp>
#include <limits.h>

#define NaN (std::numeric_limits<float>::quiet_NaN())

namespace RobotTweezers
{
    /**
     * @brief Print function for a Eigen matrix
     *
     */
#ifdef DEBUG
    auto print = [](auto &matrix) -> void
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
    auto print = [](auto &matrix) -> void {};
#endif // DEBUG

    bool IsEqual(float a, float b, float precision);

    class Kahan
    {
    public:
        static float Problem1(const Eigen::Vector3f &s, const Eigen::Vector3f &t);

        static float Problem2(const Eigen::Vector3f &s_unit, Eigen::Vector3f &u, Eigen::Vector3f &v);

        static std::pair<Eigen::Vector2f, Eigen::Vector2f> Problem3(
            const Eigen::Vector3f &s_unit, const Eigen::Vector3f &t_unit, Eigen::Vector3f &u, Eigen::Vector3f &v);

        static float Problem4(float a, float b, float c);
    };

    /**
     * @brief Calculates a skew symmetric matrix from the elements of a 3D vector
     *
     * @param s A 3D input vector
     * @return Eigen::Matrix3f
     */
    Eigen::Matrix3f Skew3(const Eigen::Vector3f &s);

    /**
     * @brief Calculates a rotation matrix about the unit x-axis
     *
     * @param alpha Rotation angle
     * @return Eigen::Matrix3f
     */
    Eigen::Matrix3f XRotation(float alpha);

    /**
     * @brief 
     * 
     * @param angle 
     * @param axis 
     * @return Eigen::Matrix3f 
     */
    Eigen::Matrix3f Rotation(float angle, Eigen::Vector3f axis);

    /**
     * @brief Calculates a rotation matrix about the unit z-axis
     *
     * @param theta Rotation angle
     * @return Eigen::Matrix3f
     */
    Eigen::Matrix3f ZRotation(float theta);

    /**
     * @brief 
     * 
     * @param vector 
     * @return Eigen::Matrix3f 
     */
    Eigen::Matrix3f ToDiagnol3(const float x, const float y, const float z);
    
    /**
     * @brief Calculates a diagnol matrix from a vector
     *
     * @param vector Input vector
     * @return Eigen::Matrix6f
     */
    Eigen::Matrix6f ToDiagnol6(const float x, const float y, const float z, const float r, const float p, const float w);

    /**
     * @brief Calculates the rotation matrix specified by XYZ Euler Angles
     *
     * @param x X1 Euler angle
     * @param y Y2 Euler angle
     * @param z Z3 Euler angle
     * @return Eigen::Matrix3f Rotation matrix
     */
    Eigen::Matrix3f EulerXYZToRotation(const float x, const float y, const float z);

    /**
     * @brief Calculates the rotation matrix specified by XZY Euler Angles
     *
     * @param x X1 Euler angle
     * @param y Y3 Euler angle
     * @param z Z2 Euler angle
     * @return Eigen::Matrix3f Rotation matrix
     */
    Eigen::Matrix3f EulerXZYToRotation(const float x, const float y, const float z);
}

#endif // _UTILS_HPP_
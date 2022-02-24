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

    class Kahan
    {
    public:
        static float Problem1(const Eigen::Vector3f &s, const Eigen::Vector3f &t);

        static float Problem2(const Eigen::Vector3f &s_unit, Eigen::Vector3f &u, Eigen::Vector3f &v, bool *valid);

        static std::tuple<float, float, float, float> Problem3(
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

    Eigen::Matrix3f Rotation(float angle, Eigen::Vector3f axis);

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
     * @brief Calculates the rotation matrix specified by XYZ Euler Angles
     *
     * @param x X1 Euler angle
     * @param y Y2 Euler angle
     * @param z Z3 Euler angle
     * @return Eigen::Matrix3f Rotation matrix
     */
    Eigen::Matrix3f EulerXYZToRotation(const float x, const float y, const float z);
}

#endif // _UTILS_HPP_
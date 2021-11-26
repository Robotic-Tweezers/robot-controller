#ifndef _UTILS_HPP_
#define _UTILS_HPP_
 
#include <ArduinoEigen.h>

// Print an Eigen class
#define PRINT(MATRIX) \
({ \
    for (int i = 0; i < (MATRIX).rows(); i++) \
    { \
        for (int j = 0; j < (MATRIX).cols(); j++) \
        { \
            Serial.print((MATRIX)(i, j)); \
            Serial.print(" "); \
        } \
         \
        Serial.println(); \
    } \
})

namespace robot_tweezers
{
    class Kahan
    {
        public: 

        static float problem1(Eigen::Vector3f s, Eigen::Vector3f t);

        static float problem2(Eigen::Vector3f s_unit, Eigen::Vector3f u, Eigen::Vector3f v, bool* valid);

        static std::pair<float[2], float[2]> problem3(Eigen::Vector3f s_unit, Eigen::Vector3f t_unit, Eigen::Vector3f u, Eigen::Vector3f v);
        
        static float problem4(float a, float b, float c);
    };
    
    Eigen::Matrix3f skew3(Eigen::Vector3f s);

    Eigen::Matrix3f xRotation(float alpha);

    Eigen::Matrix3f zRotation(float theta);
}

#endif // _UTILS_HPP_
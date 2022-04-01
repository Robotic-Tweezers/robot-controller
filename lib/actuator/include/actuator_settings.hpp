#ifndef ACTUATOR_SETTINGS_HPP
#define ACTUATOR_SETTINGS_HPP

#include <stdint.h>

namespace RobotTweezers
{
    /**
     * @brief Contains all settings used by the TMC2209 and actuator system
     *
     */
    struct ActuatorSettings
    {
    public:
        /// @brief Min and Max motion limits for a given actuator
        struct
        {
            float min;
            float max;
        } motion_limits;
        /// @brief Actuator gear ratio 
        float gear_ratio;
        /// @brief Stall threshold for sensorless homing
        uint16_t stall_threshold;
        /// @brief Step pin
        uint8_t step;
        /// @brief Direction pin
        uint8_t direction;
        /// @brief Uart address
        uint8_t uart_address;
    };
}

#endif // ACTUATOR_SETTINGS_HPP
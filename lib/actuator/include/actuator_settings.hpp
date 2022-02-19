#ifndef _ACTUATOR_SETTINGS_HPP_
#define _ACTUATOR_SETTINGS_HPP_

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
        uint8_t step;
        uint8_t direction;
        uint8_t uart_address;
        struct
        {
            float min;
            float max;
        } motion_limits;
        float gear_ratio;
    };
}

#endif // _ACTUATOR_SETTINGS_HPP_
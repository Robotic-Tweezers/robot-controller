#ifndef _STEPPER_HPP_
#define _STEPPER_HPP_

#include <Arduino.h>
#include <stdint.h>

namespace robot_tweezers
{
    enum resolution_e
    {
        MICROSTEP8 = 0,
        MICROSTEP32,
        MICROSTEP64,
        MICROSTEP16,
    };

    class Stepper
    {
        private:

        uint8_t step;
        uint8_t direction;
        uint8_t enable;
        uint8_t microstep1;
        uint8_t microstep2;
        bool step_state;
        bool direction_state;
        resolution_e resolution;

        void configureOutputPins(uint8_t step, uint8_t direction, uint8_t enable, uint8_t microstep1, uint8_t microstep2);

        public:

        /****************************************
         * Step/Direction control
        *****************************************/
        /**
         * @brief Construct a new Stepper object, not initialized
         * 
         */
        Stepper();

        /**
         * @brief Construct a new Stepper object, uses step and direction pins only
         * 
         * @param step 
         * @param direction 
         */
        Stepper(uint8_t step, uint8_t direction);

        /**
         * @brief Construct a new Stepper object, uses step, direction and enable
         * 
         * @param step 
         * @param direction 
         */
        Stepper(uint8_t step, uint8_t direction, uint8_t enable);

        /**
         * @brief Construct a new Stepper object
         * 
         * @param step 
         * @param direction 
         */
        Stepper(uint8_t step, uint8_t direction, uint8_t enable, uint8_t microstep1, uint8_t microstep2);

        /**
         * @brief 
         * 
         */
        void stepMotor(void);

        void setResolution(resolution_e resolution);
    };
}

#endif // _STEPPER_HPP_
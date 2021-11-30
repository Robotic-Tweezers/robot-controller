#ifndef _STEPPER_HPP_
#define _STEPPER_HPP_

#include <Arduino.h>
#include <stdint.h>
#include <encoder.hpp>

// Could be higher?
#define MAX_FREQUENCY                   (4000) // Hz
#define MAX_VELOCITY(RESOLUTION)        ((float)(2 * PI * MAX_FREQUENCY / (200 * (RESOLUTION))))
#define FREQUENCY(RESOLUTION, VELOCITY) ((float)(200 * (RESOLUTION) * (VELOCITY) / (2 * PI)))

// Useful links
// https://github.com/bigtreetech/BIGTREETECH-Stepper-Motor-Driver/blob/master/TMC2209/V1.1/manual/TMC2209-V1.1-manual.pdf
// https://github.com/teemuatlut/TMCStepper
// https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC2209_Datasheet_V103.pdf

namespace RobotTweezers
{
    enum resolution_e
    {
        MICROSTEP8 = 8,
        MICROSTEP32 = 32,
        MICROSTEP64 = 64,
        MICROSTEP16 = 16,
    };

    class Stepper
    {
        private:

        uint8_t index;
        uint8_t step;
        uint8_t direction;
        uint8_t enable;
        uint8_t microstep1;
        uint8_t microstep2;

        void setDirection(bool positive);

        void setResolution(resolution_e resolution);

        public:

        Encoder encoder;

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
        Stepper(uint8_t index, uint8_t step, uint8_t direction, uint8_t encoder_a, uint8_t encoder_b);

        /**
         * @brief Construct a new Stepper object, uses step, direction and enable
         * 
         * @param step 
         * @param direction 
         */
        Stepper(uint8_t index, uint8_t step, uint8_t direction, uint8_t enable, uint8_t encoder_a, uint8_t encoder_b);

        /**
         * @brief Construct a new Stepper object
         * 
         * @param step 
         * @param direction 
         */
        Stepper(uint8_t index, uint8_t step, uint8_t direction, uint8_t enable, uint8_t microstep1, uint8_t microstep2, 
            uint8_t encoder_a, uint8_t encoder_b);

        void configureOutputPins(uint8_t step, uint8_t direction, uint8_t enable, uint8_t microstep1, uint8_t microstep2);

        void configureEncoder(uint8_t encoder_a, uint8_t encoder_b);
        
        void enableStepper(void);
        
        void disableStepper(void);

        /**
         * @brief 
         * 
         */
        void setVelocity(float velocity);
    };
}

#endif // _STEPPER_HPP_
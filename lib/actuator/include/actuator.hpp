#ifndef _ACTUATOR_HPP_
#define _ACTUATOR_HPP_

#include <stdint.h>
#include <ArduinoEigen.h>
#include <TMCStepper.h>
#include <AccelStepper.h>
#include <actuator_settings.hpp>

// Useful links
// https://github.com/bigtreetech/BIGTREETECH-Stepper-Motor-Driver/blob/master/TMC2209/V1.1/manual/TMC2209-V1.1-manual.pdf
// https://github.com/teemuatlut/TMCStepper
// https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC2209_Datasheet_V103.pdf

#define STEPS 200
#define MINIMUM_VELOCITY 0.001
#define MICROSTEP_MAX(RESOLUTION) (STEPS *(RESOLUTION))

namespace RobotTweezers
{
    /**
     * @brief A step/direction + uart control interface using the TMC actuator library
     *
     */
    class Actuator
    {
    private:


        /** @brief Motion limits */
        struct
        {
            float min;
            float max;
        } motion_limits;

        /** @brief Actuator gear ratio */
        float gear_ratio;
        
        /** @brief Current microstep resolution, stored locally to avoid repeated UART transfers */ 
        uint16_t microstep;

        /** @brief Enable pin controls all drivers simultaneously */
        static uint8_t enable;

        /**
         * @brief Construct a new default Actuator object, not initialized
         *
         */
        Actuator(void);

        /**
         * @brief Construct a new Actuator object
         * 
         * @param serial    Uart bus for serial communication
         * @param settings  Contains hardware settings
         */
        Actuator(HardwareSerial *serial, ActuatorSettings &settings);

        /**
         * @brief Set driver microstep resolution, store local copy to avoid constant uart writes 
         * 
         * @param microstep Microstep resolution
         */
        void SetMicrostepResolution(uint16_t microstep);

        /**
         * @brief Converts steps into radians for a given actuator configuration
         * 
         * @param steps         Steps to convert into radians
         * @param gear_ratio    Gear ratio
         * @param microstep     Current microstep resolution
         * @return float        Steps converted into radians
         */
        static float StepsToRadians(long steps, float gear_ratio, uint16_t microstep);

        /**
         * @brief Converts radians into steps for a given actuator configuration
         * 
         * @param radians       Radians to convert
         * @param gear_ratio    Gear ratio
         * @param microstep     Current microstep resolution
         * @return long 
         */
        static long RadiansToSteps(float radians, float gear_ratio, uint16_t microstep);

    public:
        /** @brief Provides step and direction control for each TMC2209 driver */
        AccelStepper driver;

        /** @brief Provides a UART interface to control stepper driver features */
        TMC2209Stepper uart;

        /**
         * @brief Initialize TMC2209 UART registers
         * 
         * @return true     Successful initialization
         * @return false    Unsuccessful initialization
         */
        bool Initialize(void);

        /**
         * @brief Returns the UART address of the TMC2209Stepper instance
         * 
         * @return uint8_t UART address
         */
        uint8_t Address(void);

        /**
         * @brief Set the instantaneous actuator velocity
         * 
         * @param velocity instantaneous actuator velocity (rad / s)
         */
        void SetVelocity(float velocity);

        /**
         * @brief Get the current actuator position 
         * 
         * @return float Current actuator position
         */
        float GetPosition(void);

        /**
         * @brief Set Motion Limits
         * 
         * @param min Minimum travel
         * @param max Maximum travel
         */
        void SetMotionLimits(float min, float max);

        /**
         * @brief Set Gear Ratio
         * 
         * @param gear_ratio Actuator gear ratio
         */
        void SetGearRatio(float gear_ratio);

        /**
         * @brief Creates a new Actuator object, and checks for normal comms
         * 
         * @param serial        UART Serial driver 
         * @param settings      Actuator settings
         * @return Actuator*    Returned actuator object
         */
        static Actuator *ActuatorFactory(HardwareSerial *serial, ActuatorSettings &settings);

        /**
         * @brief Set the Enable Pin acting on all Actuators
         * 
         * @param enable Enable pin
         */
        static void SetEnablePin(uint8_t enable);

        /**
         * @brief Enable all Actuators
         * 
         */
        static void Enable(void);

        /**
         * @brief Disable all Actuators
         * 
         */
        static void Disable(void);

        /**
         * @brief Delete all Actuators
         * 
         * @param actuators 
         * @param size 
         */
        static void Delete(Actuator *actuators[], uint8_t size);

        /**
         * @brief Set velocity of all actuators
         * 
         * @param actuators Array of Actuator pointers 
         * @param velocity  Vector containing Actuator velocities
         * @param size      Number of actuators
         */
        static void SetVelocity(Actuator *actuators[], const Eigen::Vector3f &velocity, uint8_t size);

        /**
         * @brief Get the Position of all Actuators
         * 
         * @param actuators         Array of Actuator pointers 
         * @param size              Number of actuators
         * @return Eigen::Vector3f  Position of each Actuator
         */
        static Eigen::Vector3f GetPosition(Actuator *actuators[], uint8_t size);
    };
}

#endif // _ACTUATOR_HPP_

#ifndef _CONFIG_HPP_
#define _CONFIG_HPP_

#define VERSION_MAJOR 1
#define VERSION_MINOR 0

/**
 * @brief Task periods
 * 
 */
#define PWM_LOOP_RATE 100 // Stepper PWM rate in us
#define CONTROLLER_LOOP_RATE 50 // Controller loop rate in ms
#define INTERFACE_LOOP_RATE 100  // Serial loop rate in ms

/**
 * @brief Interface configurations
 *
 */
#define INTERFACE_BAUDRATE 921600

/**
 * @brief Controller configurations
 *
 */
//
#define MESSAGE_QUEUE_DEPTH (1U)
#define RECIEVE_BUFFER_SIZE (128U)

// Translational control not used
#if 0
    #define POSITION_GAIN_X 0.00f // Gain for position in x
    #define POSITION_GAIN_Y 0.00f // Gain for position in y
    #define POSITION_GAIN_Z 0.00f // Gain for position in z
#endif 
#define POSITION_GAIN_R 2.00f // Gain for roll angle
#define POSITION_GAIN_P 2.00f // Gain for pitch angle
#define POSITION_GAIN_W 2.00f // Gain for yaw angle

// Translational control not used
#if 0
    #define VELOCITY_GAIN_X 0.00f // Gain for velocity in x
    #define VELOCITY_GAIN_Y 0.00f // Gain for velocity in y
    #define VELOCITY_GAIN_Z 0.00f // Gain for velocity in z
#endif 
#define VELOCITY_GAIN_R 0.40f // Gain for roll angular velocity
#define VELOCITY_GAIN_P 0.40f // Gain for pitch angular velocity
#define VELOCITY_GAIN_W 0.40f // Gain for yaw angular velocity

#define STATE_ERROR 0.10 // Defines the max position error

/**
 * @brief Actuator configurations
 *
 */
#define ACTUATORS 3
#define ACTUATOR_BAUDRATE 460800
#define ENABLE_PIN 8 // Enable pin

#define THETA0_STALL 8 // TMC2209 Driver 0 stall value for homing
#define THETA1_STALL 8 // TMC2209 Driver 1 stall value for homing
#define THETA2_STALL 8 // TMC2209 Driver 2 stall value for homing

#define THETA0_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define THETA1_ADDRESS 0b01 // TMC2209 Driver address according to MS1 and MS2
#define THETA2_ADDRESS 0b10 // TMC2209 Driver address according to MS1 and MS2

#define THETA0_STEP 3 // Teensy 4.0 step PWM pin for actuator 0
#define THETA1_STEP 5 // Teensy 4.0 step PWM pin for actuator 1
#define THETA2_STEP 7 // Teensy 4.0 step PWM pin for actuator 2

#define THETA0_DIRECTION 2  // Direction pin for actuator 0
#define THETA1_DIRECTION 4  // Direction pin for actuator 1
#define THETA2_DIRECTION 6  // Direction pin for actuator 2

#define THETA0_MOTION_MIN (-PI)
#define THETA0_MOTION_MAX (PI)
#define THETA1_MOTION_MIN (-PI)
#define THETA1_MOTION_MAX (PI)
#define THETA2_MOTION_MIN (-PI)
#define THETA2_MOTION_MAX (PI)

#define THETA0_GEAR_RATIO 1
#define THETA1_GEAR_RATIO 3
#define THETA2_GEAR_RATIO 1

// Add dummy values for linting and if we get off target testing working
#ifndef LENGTH1
#define LENGTH1 18 // cm
#endif             // LENGTH1

#ifndef LENGTH2
#define LENGTH2 9.5 // cm
#endif              // LENGTH1

#endif // _CONFIG_HPP_
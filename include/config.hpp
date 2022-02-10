#ifndef _CONFIG_HPP_
#define _CONFIG_HPP_

#define VERSION_MAJOR 0
#define VERSION_MINOR 0

#define CONTROLLER_LOOP_RATE 200 // Controller loop rate in ms
#define INTERFACE_LOOP_RATE 200 // Serial loop rate in ms

/**
 * @brief Interface configurations
 * 
 */
#define QUEUE_MAX_SIZE 100
#define MAX_JSON_SIZE 256
#define INTERFACE_BAUDRATE 921600

/**
 * @brief Controller configurations
 *
 */
#define POSITION_GAIN_X 0.00 // Gain for position in x
#define POSITION_GAIN_Y 0.00 // Gain for position in y
#define POSITION_GAIN_Z 0.00 // Gain for position in z
#define POSITION_GAIN_R 1.00 // Gain for roll angle
#define POSITION_GAIN_P 1.00 // Gain for pitch angle
#define POSITION_GAIN_W 1.00 // Gain for yaw angle

#define VELOCITY_GAIN_X 0.00 // Gain for velocity in x
#define VELOCITY_GAIN_Y 0.00 // Gain for velocity in y
#define VELOCITY_GAIN_Z 0.00 // Gain for velocity in z
#define VELOCITY_GAIN_R 0.00 // Gain for roll angular velocity
#define VELOCITY_GAIN_P 0.00 // Gain for pitch angular velocity
#define VELOCITY_GAIN_W 0.00 // Gain for yaw angular velocity

#define STATE_ERROR 0.01 // Defines the max position error 

/**
 * @brief Actuator configurations
 * 
 */
#define ACTUATORS 3
#define ACTUATOR_BAUDRATE 460800
#define ENABLE_PIN 12 // Enable pin

#define THETA0_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define THETA1_ADDRESS 0b01 // TMC2209 Driver address according to MS1 and MS2
#define THETA2_ADDRESS 0b10 // TMC2209 Driver address according to MS1 and MS2

#define THETA0_STEP 10 // Teensy 4.0 step PWM pin for actuator 0
#define THETA1_STEP 8  // Teensy 4.0 step PWM pin for actuator 1
#define THETA2_STEP 3  // Teensy 4.0 step PWM pin for actuator 2

#define THETA0_DIRECTION 7 // Direction pin for actuator 0
#define THETA1_DIRECTION 6 // Direction pin for actuator 1
#define THETA2_DIRECTION 2 // Direction pin for actuator 2

#define THETA0_STEP_COUNT 11 // Interrupt pin for actuator 0 step count
#define THETA1_STEP_COUNT 9  // Interrupt pin for actuator 1 step count
#define THETA2_STEP_COUNT 4  // Interrupt pin for actuator 2 step count

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
#endif // LENGTH1

#ifndef LENGTH2
#define LENGTH2 9.5 // cm
#endif // LENGTH1

#endif // _CONFIG_HPP_
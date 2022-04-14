#ifndef CONFIG_HPP
#define CONFIG_HPP

#define VERSION_MAJOR 1
#define VERSION_MINOR 0

/**
 * @brief Task periods
 *
 */
#define PWM_LOOP_RATE 100      // Stepper PWM rate in us
#define INTERFACE_LOOP_RATE 10 // Serial loop rate in ms

/**
 * @brief Serial mappings
 * 
 */
#define actuator_serial Serial1
#define interface_serial Serial4

/**
 * @brief Controller configurations
 *
 */
// Use an RTOS queue, but only pass one message
#define MESSAGE_QUEUE_DEPTH 1U

// Add robot lengths (used in direct kinematics)
#ifndef LENGTH1
#define LENGTH1 18 // cm
#endif             // LENGTH1

#ifndef LENGTH2
#define LENGTH2 9.5 // cm
#endif              // LENGTH1

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

#define THETA0_DIRECTION 2 // Direction pin for actuator 0
#define THETA1_DIRECTION 4 // Direction pin for actuator 1
#define THETA2_DIRECTION 6 // Direction pin for actuator 2

#define THETA0_MOTION_MIN (-PI * 3 / 4)
#define THETA0_MOTION_MAX (PI * 3 / 4)
#define THETA1_MOTION_MIN (-PI * 3 / 4)
#define THETA1_MOTION_MAX (PI * 3 / 4)
#define THETA2_MOTION_MIN (-PI * 3 / 4)
#define THETA2_MOTION_MAX (PI * 3 / 4)

#define THETA0_GEAR_RATIO -1
#define THETA1_GEAR_RATIO 3
#define THETA2_GEAR_RATIO 1

#endif // CONFIG_HPP
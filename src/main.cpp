#include <Arduino.h>
#include <FreeRTOS_TEENSY4.h>
#include <config.hpp>
#include <kinematic.hpp>
#include <coordinates.hpp>
#include <actuator.hpp>
#include <logger.hpp>

#define TIME_IN_MS(TIME) (TIME) * configTICK_RATE_HZ / 1000UL
#define TIME_IN_US(TIME) (TIME) * configTICK_RATE_HZ / 1000000UL

#define CONTROLLER_QUEUE_SIZE (1024U)

#define DEMO 1

using namespace std;
using namespace Eigen;
using namespace RobotTweezers;

typedef struct Message
{
    float roll;
    float pitch;
    float yaw;
} Message;

// Task handles
TaskHandle_t run_stepper_handle;
TaskHandle_t controller_handle;
TaskHandle_t interface_handle;

// Message queue
QueueHandle_t controller_queue;

// Constant gain matricies
const Matrix3f kp = ToDiagnol3(POSITION_GAIN_R, POSITION_GAIN_P, POSITION_GAIN_W);
const Matrix3f kv = ToDiagnol3(VELOCITY_GAIN_R, VELOCITY_GAIN_P, VELOCITY_GAIN_W);

// Controller variables
Actuator *actuators[ACTUATORS];
Vector6f joint_state;

#if DEMO
    Message demo[] = {
        Message{.roll =  3.1416, .pitch =  0.0000, .yaw =  0.0000},
        Message{.roll =  3.1416, .pitch = -0.2618, .yaw =  0.0000},
        Message{.roll =  3.1416, .pitch = -0.5236, .yaw =  0.0000},
        Message{.roll =  3.1416, .pitch = -0.7854, .yaw =  0.0000},
        Message{.roll =  3.1416, .pitch = -1.0472, .yaw =  0.0000},
        Message{.roll = -3.1416, .pitch = -1.3090, .yaw =  0.0000},
        Message{.roll =  0.0000, .pitch = -1.5708, .yaw = -3.1416},
        Message{.roll = -1.5708, .pitch = -1.3090, .yaw =  1.5708},
        Message{.roll = -1.5708, .pitch = -1.0472, .yaw =  1.5708},
        Message{.roll = -1.5708, .pitch = -0.7854, .yaw =  1.5708},
        Message{.roll = -1.5708, .pitch = -0.5236, .yaw =  1.5708},
        Message{.roll = -1.5708, .pitch = -0.2618, .yaw =  1.5708},
        Message{.roll = -1.5708, .pitch =  0.0000, .yaw =  1.5708},
        Message{.roll = -1.5708, .pitch =  0.2618, .yaw =  1.5708},
        Message{.roll = -1.5708, .pitch =  0.5236, .yaw =  1.5708},
        Message{.roll = -1.5708, .pitch =  0.7854, .yaw =  1.5708},
        Message{.roll = -1.5708, .pitch =  1.0472, .yaw =  1.5708},
        Message{.roll = -1.5708, .pitch =  1.3090, .yaw =  1.5708},
        Message{.roll =  0.0000, .pitch =  1.5708, .yaw = -0.0000},
        Message{.roll = -3.1416, .pitch =  1.3090, .yaw =  3.1416},
        Message{.roll = -3.1416, .pitch =  1.0472, .yaw =  3.1416},
        Message{.roll =  3.1416, .pitch =  0.7854, .yaw =  3.1416},
        Message{.roll =  3.1416, .pitch =  0.5236, .yaw =  3.1416},
        Message{.roll =  3.1416, .pitch =  0.2618, .yaw =  3.1416},
        Message{.roll =  3.1416, .pitch =  0.0000, .yaw =  3.1416},
    };
#endif

/// @brief The DH transformation parameters for the wrist manipulator (translational x-axis component is omitted)
float dh_table[3][3] = {
    {0, LENGTH1, PI / 2},
    {0, 0, -PI / 2},
    {0, LENGTH2, 0},
};

/**
 * @brief 
 * 
 * @param serial 
 * @return true 
 * @return false 
 */
static bool InitializeActuators(HardwareSerial *serial)
{
    bool status = true;
    ActuatorSettings settings[ACTUATORS];
    settings[0] = ActuatorSettings{
        .motion_limits = {
            .min = THETA0_MOTION_MIN,
            .max = THETA0_MOTION_MAX},
        .gear_ratio = THETA0_GEAR_RATIO,
        .stall_threshold = THETA0_STALL,
        .step = THETA0_STEP,
        .direction = THETA0_DIRECTION,
        .uart_address = THETA0_ADDRESS};
    settings[1] = ActuatorSettings{
        .motion_limits = {
            .min = THETA1_MOTION_MIN,
            .max = THETA1_MOTION_MAX},
        .gear_ratio = THETA1_GEAR_RATIO,
        .stall_threshold = THETA1_STALL,
        .step = THETA1_STEP,
        .direction = THETA1_DIRECTION,
        .uart_address = THETA1_ADDRESS};
    settings[2] = ActuatorSettings{
        .motion_limits = {
            .min = THETA2_MOTION_MIN,
            .max = THETA2_MOTION_MAX},
        .gear_ratio = THETA2_GEAR_RATIO,
        .stall_threshold = THETA2_STALL,
        .step = THETA2_STEP,
        .direction = THETA2_DIRECTION,
        .uart_address = THETA2_ADDRESS};

    for (uint8_t i = 0; i < ACTUATORS; i++)
    {
        status &= (actuators[i] = Actuator::ActuatorFactory(serial, settings[i])) != nullptr;
    }

    return status;
}

/**
 * @brief Simple decision logic to determine which of two solutions will be used for trajectory
 * 
 * @param position 
 * @param solutions 
 * @return Eigen::Vector3f 
 */
static Eigen::Vector3f Decision(Vector3f position, std::pair<Eigen::Vector3f, Eigen::Vector3f> &solutions)
{
    float distance1, distance2;
    for (int i = 0; i < position.rows(); i++)
    {
        // Check for edge case: if the desired pos is pi, change sign to match current pose
        if (IsEqual(abs(solutions.first(i)), PI, 0.01))
        {
            solutions.first(i) = (position(i) > 0) ? PI : -PI;
        }
        
        // Check for edge case: if the desired pos is pi, change sign to match current pose
        if (IsEqual(abs(solutions.second(i)), PI, 0.01))
        {
            solutions.second(i) = (position(i) > 0) ? PI : -PI;
        }
    }
    distance1 = sqrt((solutions.first - position).norm());
    distance2 = sqrt((solutions.second - position).norm());
    return (distance1 < distance2) ? solutions.first : solutions.second;
}

/**
 * @brief 
 * 
 * @param arg 
 */
static void RunSteppers(void *arg)
{
    TickType_t previous_wake = 0;
    while (true)
    {
        Actuator::Run(actuators, ACTUATORS);
        vTaskDelayUntil(&previous_wake, TIME_IN_US(PWM_LOOP_RATE));
    }
}

/**
 * @brief 
 * 
 * @param arg 
 */
static void ControlLoop(void *arg)
{
    pair<Vector3f, Vector3f> solutions;
    Vector6f desired_state, state_error;
    Message new_msg;
    TickType_t previous_wake = 0;
    
    while (true)
    {
        // Update joint positions
        joint_state.head(3) = Actuator::GetPosition(actuators, ACTUATORS);

        // State error
        state_error = desired_state - joint_state;

        // State error is at required minimum
        if (sqrt(state_error.dot(state_error)) <= STATE_ERROR)
        {
            if (xQueueReceive(controller_queue, (void *)&new_msg, 0) == pdTRUE)
            {
                // Find all possible solutions for achieving desired orientation
                solutions = Kinematic::InverseKinematics(new_msg.roll, new_msg.pitch, new_msg.yaw);
                // Select desired state, using initial state
                desired_state.head(3) = Decision(joint_state.head(3), solutions);
                
            }
            else 
            {
                Actuator::SetVelocity(actuators, Vector3f(0, 0, 0), ACTUATORS);
                vTaskSuspend(controller_handle);
            }
        }

        // Update joint velocity
        joint_state.tail(3) = (kp * state_error.head(3)) + (kv * state_error.tail(3));

        // Set new velocity
        Actuator::SetVelocity(actuators, joint_state.tail(3), ACTUATORS);

        vTaskDelayUntil(&previous_wake, TIME_IN_MS(CONTROLLER_LOOP_RATE));
    }
}

static void SerialInterface(void *arg)
{
    Stream *client_interface = static_cast<Stream *>(arg);
    TickType_t previous_wake = 0;

    while (true)
    {
        if (client_interface->available() > 0)
        {
            /*
            if (gui_command.containsKey("position"))
            {
                Coordinates new_coords;
                float roll_anlge = gui_command["position"]["roll"];
                float pitch_angle = gui_command["position"]["pitch"];
                float yaw_angle = gui_command["position"]["yaw"];
                new_coords.frame = EulerXYZToRotation(roll_anlge, pitch_angle, yaw_angle);
                if (position_queue.isEmpty())
                {
                    // Resume controller
                    vTaskResume(controller_handle);
                }

                if (position_queue.size() <= QUEUE_MAX_SIZE)
                {
                    xQueueSend(controller_queue, )
                    position_queue.push(new_coords);
                }
            }
            */
        }

        vTaskDelayUntil(&previous_wake, TIME_IN_MS(INTERFACE_LOOP_RATE));
    }
}

void setup()
{
    portBASE_TYPE status = pdPASS;
    HardwareSerial *actuator_serial = &Serial1;
    Logger logger(&Serial);

    Serial.begin(INTERFACE_BAUDRATE);
    logger.Log("Starting Robot, firmware version %d.%d", VERSION_MAJOR, VERSION_MINOR);

    controller_queue = xQueueCreate(CONTROLLER_QUEUE_SIZE, sizeof(Message));
 
    // Set pin and enable all actuators
    Actuator::SetEnablePin(ENABLE_PIN);

    actuator_serial->begin(ACTUATOR_BAUDRATE);
    status &= InitializeActuators(actuator_serial);

    // Set Kinematic information
    Kinematic::SetDegreesOfFreedom(ACTUATORS);
    Kinematic::SetBaseFrame(XRotation(PI));

    for (auto actuator : actuators)
    {
        actuator ? logger.Log("Stepper initialized with address %d", actuator->Address())
                 : logger.Log("Stepper is NULL");
    }

    if (status != pdPASS)
    {
        Actuator::Delete(actuators, ACTUATORS);
        logger.Error("Actuators did not initialize properly, check UART or power connection");
    }

    logger.Log("Actuator motors initialized successfully.");
    Actuator::Enable();

#if 0 // Need to add sensorless homeing to robot 
    status &= actuators[0]->Home(HALF_PI);
    status &= actuators[1]->Home(HALF_PI);
    status &= actuators[2]->Home(HALF_PI);
#endif

    logger.Log("Home %s", status ? "PASSED" : "FAILED");

    // Turn on LED to indicate correct operation
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    logger.Log("Set controller inputs to initial states, spawning controller.");

#if DEMO // Test trajectory used for demo
    for (unsigned int i = 0; i < sizeof(demo) / sizeof(Message); i++)
    {
        xQueueSend(controller_queue, (void *)(demo + i), 0);
    }
#endif

    // Rate Monotonic scheduling
    status &= xTaskCreate(RunSteppers, "Run Steppers", configMINIMAL_SECURE_STACK_SIZE, nullptr, 0, &run_stepper_handle);
    status &= xTaskCreate(SerialInterface, "Interface", 10 * configMINIMAL_SECURE_STACK_SIZE, &Serial, 1, &interface_handle);
    status &= xTaskCreate(ControlLoop, "Controller", 100 * configMINIMAL_SECURE_STACK_SIZE, nullptr, 2, &controller_handle);

    if (status != pdPASS)
    {
        Actuator::Delete(actuators, ACTUATORS);
        logger.Error("Creation problem");
    }
    
    vTaskStartScheduler();

    Actuator::Delete(actuators, ACTUATORS);
    logger.Error("Insufficient RAM");
}

//------------------------------------------------------------------------------
// WARNING idle loop has a very small stack (configMINIMAL_STACK_SIZE)
// loop must never block
void loop()
{
    // Not used.
}
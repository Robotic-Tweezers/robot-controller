#include <Arduino.h>
#include <FreeRTOS_TEENSY4.h>
#include <config.hpp>
#include <kinematic.hpp>
#include <coordinates.hpp>
#include <actuator.hpp>
#include <logger.hpp>

#include <protobuf.hpp>

#define TIME_IN_MS(TIME) (TIME) * configTICK_RATE_HZ / 1000UL
#define TIME_IN_US(TIME) (TIME) * configTICK_RATE_HZ / 1000000UL

using namespace std;
using namespace Eigen;
using namespace RobotTweezers;

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
    Vector3f new_position;
    OrientationMsg new_msg;
    TickType_t previous_wake = 0;

    while (true)
    {
        if (xQueueReceive(controller_queue, (void *)&new_msg, 0) == pdTRUE)
        {
            joint_state.head(3) = Actuator::GetPosition(actuators, ACTUATORS);
            // Find all possible solutions for achieving desired orientation
            solutions = Kinematic::InverseKinematics(new_msg.roll, new_msg.pitch, new_msg.yaw);
            // Select desired state, using initial state
            new_position = Decision(joint_state.head(3), solutions);

            Actuator::SetPosition(actuators, new_position, ACTUATORS);
        }

        // Sleep while drivers are moving robot
        vTaskSuspend(controller_handle);
    }
}

static void SerialInterface(void *arg)
{
    HardwareSerial *uart = static_cast<HardwareSerial *>(arg);
    OrientationMsg message;
    TickType_t previous_wake = 0;

    uart->begin(INTERFACE_BAUDRATE);

    while (true)
    {
        if (Protobuf::UartWrite(uart, &message))
        {
            if (uxQueueMessagesWaiting(controller_queue) < MESSAGE_QUEUE_DEPTH)
            {
                xQueueSend(controller_queue, (void *)&message, 0);
                // Resume controller
                vTaskResume(controller_handle);
            }
#if MESSAGE_QUEUE_DEPTH == 1
            else
            {
                // Not advised to use overwrite if the queue size is larger than 1
                xQueueOverwrite(controller_queue, (void *)&message);
            }
#else
            else if (uxQueueSpacesAvailable(controller_queue) >= 1)
            {
                xQueueSend(controller_queue, (void *)&message, 0);
            }
            else
            {
                OrientationMsg removed_msg;
                // remove oldest message
                xQueueReceive(controller_queue, (void *)&removed_msg, 0);
                // add new message
                xQueueSend(controller_queue, (void *)&message, 0);
            }
#endif
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

    controller_queue = xQueueCreate(MESSAGE_QUEUE_DEPTH, sizeof(OrientationMsg));

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

    // Rate Monotonic scheduling
    status &= xTaskCreate(RunSteppers, "Run Steppers", configMINIMAL_SECURE_STACK_SIZE, nullptr, 0, &run_stepper_handle);
    status &= xTaskCreate(SerialInterface, "Interface", 10 * configMINIMAL_SECURE_STACK_SIZE, &Serial4, 1, &interface_handle);
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
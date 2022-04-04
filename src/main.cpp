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

// Controller variables
Vector3f joint_state;
Actuator *actuators[ACTUATORS];
ActuatorSettings actuator_settings[ACTUATORS] = {
    ActuatorSettings{
        .motion_limits = {
            .min = THETA0_MOTION_MIN,
            .max = THETA0_MOTION_MAX},
        .gear_ratio = THETA0_GEAR_RATIO,
        .stall_threshold = THETA0_STALL,
        .step = THETA0_STEP,
        .direction = THETA0_DIRECTION,
        .uart_address = THETA0_ADDRESS
    },
    ActuatorSettings{
        .motion_limits = {
            .min = THETA1_MOTION_MIN,
            .max = THETA1_MOTION_MAX},
        .gear_ratio = THETA1_GEAR_RATIO,
        .stall_threshold = THETA1_STALL,
        .step = THETA1_STEP,
        .direction = THETA1_DIRECTION,
        .uart_address = THETA1_ADDRESS
    },
    ActuatorSettings{
        .motion_limits = {
            .min = THETA2_MOTION_MIN,
            .max = THETA2_MOTION_MAX},
        .gear_ratio = THETA2_GEAR_RATIO,
        .stall_threshold = THETA2_STALL,
        .step = THETA2_STEP,
        .direction = THETA2_DIRECTION,
        .uart_address = THETA2_ADDRESS
    }
};

/// @brief The DH transformation parameters for the wrist manipulator (translational x-axis component is omitted)
float dh_table[3][3] = {
    {0, LENGTH1, PI / 2},
    {0, 0, -PI / 2},
    {0, LENGTH2, 0},
};

/**
 * @brief Creates three actuator objects using settings defined in config.hpp
 *
 * @param serial    UART bus used by all TMC2209 drivers
 * @return true     Successful initialization of all three actuators
 * @return false    Unsuccessful init
 */
static bool InitializeActuators(HardwareSerial *serial)
{
    bool status = true;

    for (uint8_t i = 0; i < ACTUATORS; i++)
    {
        status &= (actuators[i] = Actuator::ActuatorFactory(serial, actuator_settings[i])) != nullptr;
    }

    return status;
}

/**
 * @brief Simple decision logic to determine which of two solutions will be used for trajectory
 *
 * @param position          Current position of the robot
 * @param solutions         Two possible orientations to achieve desired end effector orientation
 * @return Eigen::Vector3f  Chosen orientation
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
 * @brief Runs all three steppers periodically
 *
 * @param arg Unused 
 */
static void RunSteppers(void *arg)
{
    TickType_t previous_wake = 0;

    (void)arg;

    while (true)
    {
        for (uint8_t i = 0; i < ACTUATORS; i++)
        {
            actuators[i]->driver.run();
        }

        vTaskDelayUntil(&previous_wake, TIME_IN_US(PWM_LOOP_RATE));
    }
}

/**
 * @brief Calculates the joint positions needed to produce a desired end-effector orientation
 *
 * @param arg Unused 
 */
static void ControlLoop(void *arg)
{
    pair<Vector3f, Vector3f> solutions;
    Vector3f new_position;
    OrientationMsg new_msg;

    (void)arg;

    while (true)
    {
        if (xQueueReceive(controller_queue, (void *)&new_msg, 0) == pdTRUE)
        {
            for (uint8_t i = 0; i < ACTUATORS; i++)
            {
                joint_state(i) = actuators[i]->GetPosition();
            }
            
            // Find all possible solutions for achieving desired orientation
            solutions = Kinematic::InverseKinematics(new_msg.roll, new_msg.pitch, new_msg.yaw);
            // Select desired state, using initial state
            new_position = Decision(joint_state, solutions);

            for (uint8_t i = 0; i < ACTUATORS; i++)
            {
                actuators[i]->SetTargetPosition(new_position(i));
            }
        }

        // Sleep while drivers are moving robot
        vTaskSuspend(controller_handle);
    }
}

/**
 * @brief Power-on test to check if the ESP32 is connected
 * 
 * @return true     ESP32 is connected
 * @return false    ESP32 is not connected
 */
static bool TestEsp32Connection(void)
{
    UartConnection connection_msg, response;
    int8_t attempts = 5;
    bool status = false;

    connection_msg.id = TEENSY_ID;

    while (attempts--)
    {
        Protobuf::UartWrite(&interface_serial, &connection_msg);
        if (Protobuf::UartRead(&interface_serial, &response))
        {
            if (response.id == ESP32_ID)
            {
                status = true;
                break;
            }
        }
        
        delay(1000);
    }

    return status;
}

/**
 * @brief Polling loop to check for new orientation commands
 * 
 * @param arg 
 */
static void SerialInterface(void *arg)
{
    OrientationMsg message = OrientationMsg_init_default;
    TickType_t previous_wake = 0;

    (void)arg;

    while (true)
    {
        if (Protobuf::UartRead(&interface_serial, &message))
        {
            if (uxQueueMessagesWaiting(controller_queue) < MESSAGE_QUEUE_DEPTH)
            {
                xQueueSend(controller_queue, (void *)&message, 0);
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
            // Resume controller
            vTaskResume(controller_handle);
        }

        vTaskDelayUntil(&previous_wake, TIME_IN_MS(INTERFACE_LOOP_RATE));
    }
}

void setup()
{
    portBASE_TYPE status = pdPASS;
    Logger logger(&Serial);

    Serial.begin(115200);
    logger.Log("Starting Robot, firmware version %d.%d", VERSION_MAJOR, VERSION_MINOR);

    // Set pin and enable all actuators
    Actuator::SetEnablePin(ENABLE_PIN);

    actuator_serial.begin(ACTUATOR_BAUDRATE);
    interface_serial.begin(PROTOBUF_INTERFACE_BAUDRATE);

    // Small delay before checking connections
    delay(1000);

#ifdef ESP_CONNECTION
    status &= TestEsp32Connection();
#endif // ESP_CONNECTION
    status &= InitializeActuators(&actuator_serial);

    // Set actuator system pointer to use static methods
    Actuator::actuator_system = actuators;

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
        for (uint8_t i = 0; i < ACTUATORS; i++)
        {
            delete actuators[i];
        }

        logger.Error("Actuators did not initialize properly, check UART or power connection");
    }

    logger.Log("Actuator motors initialized successfully.");

    Actuator::Enable();

#if HOMING // Need to add sensorless homeing to robot 
    status &= actuators[0]->Home(HALF_PI);
    status &= actuators[1]->Home(HALF_PI);
    status &= actuators[2]->Home(HALF_PI);
#endif // HOMING

    logger.Log("Home %s", status ? "PASSED" : "FAILED");

    // Turn on LED to indicate correct operation
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    logger.Log("Set controller inputs to initial states, spawning controller.");

    // Start RTOS
    controller_queue = xQueueCreate(MESSAGE_QUEUE_DEPTH, sizeof(OrientationMsg));

    // Rate Monotonic scheduling
    status &= xTaskCreate(RunSteppers, "Run Steppers", configMINIMAL_SECURE_STACK_SIZE, nullptr, 0, &run_stepper_handle);
    status &= xTaskCreate(SerialInterface, "Interface", 10 * configMINIMAL_SECURE_STACK_SIZE, nullptr, 1, &interface_handle);
    status &= xTaskCreate(ControlLoop, "Controller", 100 * configMINIMAL_SECURE_STACK_SIZE, nullptr, 2, &controller_handle);

    if (status != pdPASS)
    {
        for (uint8_t i = 0; i < ACTUATORS; i++)
        {
            delete actuators[i];
        }
        
        logger.Error("Creation problem");
    }

    logger.Log("All tasks created successfully, starting scheduler...");

    vTaskStartScheduler();

    for (uint8_t i = 0; i < ACTUATORS; i++)
    {
        delete actuators[i];
    }

    logger.Error("Insufficient RAM");
}

//------------------------------------------------------------------------------
// WARNING idle loop has a very small stack (configMINIMAL_STACK_SIZE)
// loop must never block
void loop()
{
    // Not used.
}
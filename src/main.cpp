#include <Arduino.h>
#include <ArduinoJson.h>
#include <FreeRTOS_TEENSY4.h>
#include <config.hpp>
#include <kinematic.hpp>
#include <coordinates.hpp>
#include <actuator.hpp>
#include <logger.hpp>
#include <queue>

#define TIME_IN_MS(TIME) (TIME) * configTICK_RATE_HZ / 1000UL
#define TIME_IN_US(TIME) (TIME) * configTICK_RATE_HZ / 1000000UL

using namespace std;
using namespace Eigen;
using namespace RobotTweezers;

// Task handles
TaskHandle_t run_stepper_handle;
TaskHandle_t controller_handle;
TaskHandle_t interface_handle;

// Constant gain matricies
const Matrix6f kp = VectorToDiagnol6((float[]){
    POSITION_GAIN_X,
    POSITION_GAIN_Y,
    POSITION_GAIN_Z,
    POSITION_GAIN_R,
    POSITION_GAIN_P,
    POSITION_GAIN_W});
const Matrix6f kv = VectorToDiagnol6((float[]){
    VELOCITY_GAIN_X,
    VELOCITY_GAIN_Y,
    VELOCITY_GAIN_Z,
    VELOCITY_GAIN_R,
    VELOCITY_GAIN_P,
    VELOCITY_GAIN_W});

// Controller variables
Actuator *actuators[ACTUATORS];
queue<Coordinates> position_queue;
Coordinates desired_position;
Vector6f desired_velocity;
Vector6f joint_state;
/** @brief The DH transformation parameters for the wrist manipulator (a is omitted) */
float dh_table[3][3] = {
    {0, LENGTH1, PI / 2},
    {0, 0, -PI / 2},
    {0, LENGTH2, 0},
};

static bool InitializeActuators(HardwareSerial *serial)
{
    bool status = true;
    ActuatorSettings settings[ACTUATORS];
    settings[0] = ActuatorSettings {
        .step = THETA0_STEP,
        .direction = THETA0_DIRECTION,
        .uart_address = THETA0_ADDRESS,
        .motion_limits = {
            .min = THETA0_MOTION_MIN,
            .max = THETA0_MOTION_MAX
        },
        .gear_ratio = THETA0_GEAR_RATIO
    };
    settings[1] = ActuatorSettings {
        .step = THETA1_STEP,
        .direction = THETA1_DIRECTION,
        .uart_address = THETA1_ADDRESS,
        .motion_limits = {
            .min = THETA1_MOTION_MIN,
            .max = THETA1_MOTION_MAX
        },
        .gear_ratio = THETA1_GEAR_RATIO
    };
    settings[2] = ActuatorSettings {
        .step = THETA2_STEP,
        .direction = THETA2_DIRECTION,
        .uart_address = THETA2_ADDRESS,
        .motion_limits = {
            .min = THETA2_MOTION_MIN,
            .max = THETA2_MOTION_MAX
        },
        .gear_ratio = THETA2_GEAR_RATIO
    };

    for (uint8_t i = 0; i < ACTUATORS; i++)
    {
        status &= (actuators[i] = Actuator::ActuatorFactory(serial, settings[i])) != nullptr;
    }

    return status;
}

static void RunSteppers(void *arg)
{
    while (true)
    {
        Actuator::Run(actuators, ACTUATORS);
        vTaskDelay(TIME_IN_US(PWM_LOOP_RATE));
    }
}

static void ControlLoop(void *arg)
{
    Vector3f gravity_torque;
    Coordinates actual_position;
    Vector6f position_error, velocity_error, state_error;
    MatrixXf jacobian_matrix;

    while (true)
    {
        joint_state.head(3) = Actuator::GetPosition(actuators, ACTUATORS);

        Kinematic::UpdateDenavitHartenbergTable(dh_table, joint_state.head(3));

        // Direct kinematics of joint state (calculate end effector position)
        actual_position = Kinematic::DirectKinematics(dh_table);

        // Calculates Jacobian matrix, must be called after DirectKinematics
        jacobian_matrix = Kinematic::Jacobian(dh_table, actual_position);

        /// @TODO do we need gravity contributions here?
        // gravity_torque = Kinematic::GravityTorque(joint_state.head(3));

        // Calculate position and velocity error
        position_error = desired_position - actual_position;
        // Goes unstable here
        velocity_error = desired_velocity - (jacobian_matrix * joint_state.tail(3));

        // Total state error
        state_error = (kp * position_error) + (kv * velocity_error);

        if (sqrt(state_error.dot(state_error)) <= STATE_ERROR)
        {
            if (position_queue.empty())
            {
                Actuator::SetVelocity(actuators, Vector3f(0, 0, 0), ACTUATORS);
                vTaskSuspend(controller_handle);
            }
            else
            {
                desired_position = position_queue.front();
                position_queue.pop();
            }
        }

        // Add gravity contributions
        joint_state.tail(3) = gravity_torque + (jacobian_matrix.transpose() * state_error);

        // Resolve singularity, since theta 0 always points in -z we need to check if theta 2 does
        if (jacobian_matrix(5, 2) <= -0.98)
        {
            // Set theta 0 velocity to zero
            joint_state(3) = 0.00;
        }

        Actuator::SetVelocity(actuators, joint_state.tail(3), ACTUATORS);

        vTaskDelay(TIME_IN_MS(CONTROLLER_LOOP_RATE));
    }
}

static void SerialInterface(void *arg)
{
    Stream *client_interface = static_cast<Stream *>(arg);
    StaticJsonDocument<256> gui_command;
    String json_command, response;

    while (true)
    {
        if (client_interface->available() > 0)
        {
            json_command = client_interface->readString();
            if (deserializeJson(gui_command, json_command))
            {
                // Error in decoding message
                continue;
            }

            // If sender includes version keyword, respond with firmware verison
            if (gui_command.containsKey("version"))
            {
                char temp_buffer[24];
                sprintf(temp_buffer, "Robot Tweezers v%d.%d", VERSION_MAJOR, VERSION_MINOR);
                gui_command["version"] = temp_buffer;
                serializeJson(gui_command, response);
                client_interface->print(response);
            }

            if (gui_command.containsKey("actuators"))
            {
                for (uint8_t i = 0; i < ACTUATORS; i++)
                {
                    gui_command["actuators"][i]["position"] = actuators[i]->GetPosition();
                    serializeJson(gui_command, response);
                    client_interface->print(response);
                }
            }

            // Enable/Disable actuators
            if (gui_command.containsKey("enable_actuators"))
            {
                gui_command["enable_actuators"] ? Actuator::Enable() : Actuator::Disable();
            }

            if (gui_command.containsKey("position"))
            {
                Coordinates new_coords;
                float roll_anlge = gui_command["position"]["roll"];
                float pitch_angle = gui_command["position"]["pitch"];
                float yaw_angle = gui_command["position"]["yaw"];
                new_coords.frame = EulerXYZToRotation(roll_anlge, pitch_angle, yaw_angle);
                if (position_queue.empty())
                {
                    // Resume controller
                    vTaskResume(controller_handle);
                }

                if (position_queue.size() <= QUEUE_MAX_SIZE)
                {
                    position_queue.push(new_coords);
                }
            }
        }

        vTaskDelay(TIME_IN_MS(INTERFACE_LOOP_RATE));
    }
}

void setup()
{
    portBASE_TYPE status = pdPASS;
    HardwareSerial *actuator_serial = &Serial1;
    Logger logger(&Serial);

    Serial.begin(INTERFACE_BAUDRATE);
    logger.Log("Starting Robot, firmware version %d.%d", VERSION_MAJOR, VERSION_MINOR);

    // Set pin and enable all actuators
    Actuator::SetEnablePin(ENABLE_PIN);

    actuator_serial->begin(ACTUATOR_BAUDRATE);
    status &= InitializeActuators(actuator_serial);

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

    // Turn on LED to indicate correct operation
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    logger.Log("Set controller inputs to initial states, spawning controller.");

    desired_position.frame = XRotation(PI);

    status &= xTaskCreate(RunSteppers, nullptr, configMINIMAL_SECURE_STACK_SIZE, nullptr, 1, &run_stepper_handle);
    status &= xTaskCreate(SerialInterface, nullptr, 10 * configMINIMAL_SECURE_STACK_SIZE, &Serial, 2, &interface_handle);
    status &= xTaskCreate(ControlLoop, nullptr, 100 * configMINIMAL_SECURE_STACK_SIZE, nullptr, 3, &controller_handle);

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
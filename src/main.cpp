#include <Arduino.h>
#include <ArduinoJson.h>
#include <TimeLib.h>
#include <queue>
#include <config.hpp>
#include <kinematic.hpp>
#include <coordinates.hpp>
#include <actuator.hpp>
#include <logger.hpp>

#define TIME_IN_MS(TIME) (TIME) * configTICK_RATE_HZ / 1000UL
#define TIME_IN_US(TIME) (TIME) * configTICK_RATE_HZ / 1000000UL

using namespace std;
using namespace Eigen;
using namespace RobotTweezers;

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

// Controller subcalculations
Vector3f gravity_torque;
Kinematic wrist_kinematics;
Coordinates actual_position;
Vector6f position_error, velocity_error, state_error;
MatrixXf jacobian_matrix;

// Serial 
Stream *client_interface;
StaticJsonDocument<256> gui_command;
String json_command, response;

// Timing and syncronization
static uint32_t last_cycle = 0;
static bool controller_enable = true;

void setup(void)
{
    HardwareSerial *actuator_serial = &Serial1;
    Logger logger(&Serial);
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
    bool status = true;

    Serial.begin(INTERFACE_BAUDRATE);
    logger.Log("Starting Robot, firmware version %d.%d", VERSION_MAJOR, VERSION_MINOR);

    // Set pin and enable all actuators
    Actuator::SetEnablePin(ENABLE_PIN);

    actuator_serial->begin(ACTUATOR_BAUDRATE);

    for (uint8_t i = 0; i < ACTUATORS; i++)
    {
        if ((actuators[i] = Actuator::ActuatorFactory(actuator_serial, settings[i])) != nullptr)
        {
            logger.Log("Stepper initialized with address %d", actuators[i]->Address());
        }
        else
        {
            logger.Log("Stepper is NULL");
            status = false;
        }
    }

    if (status != true)
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

    desired_position.frame = XRotation(PI) * ZRotation(PI / 2);

    if (status != true)
    {
        Actuator::Delete(actuators, ACTUATORS);
        logger.Error("Creation problem");
    }

    client_interface = &Serial;
}

void loop(void)
{
    if (client_interface->available() > 0 && (millis() - last_cycle >= INTERFACE_LOOP_RATE))
    {
        last_cycle = millis();
        json_command = client_interface->readString();
        if (deserializeJson(gui_command, json_command) == nullptr)
        {
            // If sender includes version keyword, respond with firmware verison
            if (gui_command.containsKey("version"))
            {
                char temp_buffer[24];
                sprintf(temp_buffer, "Robot Tweezers v%d.%d", VERSION_MAJOR, VERSION_MINOR);
                gui_command["version"] = temp_buffer;
                serializeJson(gui_command, response);
                client_interface->print(response);
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
                    controller_enable = true;
                }

                if (position_queue.size() <= QUEUE_MAX_SIZE)
                {
                    position_queue.push(new_coords);
                }
            }
        }
    }

    if (controller_enable)
    {
        joint_state.head(3) = Actuator::GetPosition(actuators, ACTUATORS);

        // Direct kinematics of joint state (calculate end effector position)
        actual_position = wrist_kinematics.DirectKinematics(joint_state.head(3));

        // Calculates Jacobian matrix, must be called after DirectKinematics
        jacobian_matrix = wrist_kinematics.Jacobian();

        /// @TODO do we need gravity contributions here?
        // gravity_torque = wrist_kinematics.GravityTorque(joint_state.head(3));

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
                controller_enable = false;
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
    }
}
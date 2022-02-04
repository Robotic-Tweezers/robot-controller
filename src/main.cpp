#include <Arduino.h>
#include <ArduinoJson.h>
#include <TimeLib.h>
#include <FreeRTOS_TEENSY4.h>
#include <config.hpp>
#include <kinematic.hpp>
#include <coordinates.hpp>
#include <actuator.hpp>
#include <queue>

#define TIME_IN_MS(TIME) (TIME) * configTICK_RATE_HZ / 1000UL
#define TIME_IN_US(TIME) (TIME) * configTICK_RATE_HZ / 1000000UL

using namespace std;
using namespace Eigen;
using namespace RobotTweezers;

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

static bool InitializeActuators(HardwareSerial *serial)
{
    ActuatorSettings settings[ACTUATORS];
    settings[0] = ActuatorSettings {
        .step = THETA0_STEP,
        .direction = THETA0_DIRECTION,
        .uart_address = THETA0_ADDRESS,
        .step_counter = THETA0_STEP_COUNT,
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
        .step_counter = THETA1_STEP_COUNT,
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
        .step_counter = THETA2_STEP_COUNT,
        .motion_limits = {
            .min = THETA2_MOTION_MIN,
            .max = THETA2_MOTION_MAX
        },
        .gear_ratio = THETA2_GEAR_RATIO
    };

    for (uint8_t i = 0; i < ACTUATORS; i++)
    {
        if ((actuators[i] = Actuator::ActuatorFactory(serial, settings[i])) == nullptr)
        {
            return false;
        }
    }

    return true;
}

static void ControlLoop(void *arg)
{
    Vector3f gravity_torque;
    Kinematic wrist_kinematics;
    Coordinates actual_position;
    Vector6f position_error, velocity_error, state_error;
    MatrixXf jacobian_matrix;

    while (true)
    {
        for (uint8_t i = 0; i < ACTUATORS; i++)
        {
            joint_state(i) = actuators[i]->GetPosition();
        }

        // Direct kinematics of joint state (calculate end effector position)
        actual_position = wrist_kinematics.DirectKinematics(joint_state.head(3));

        // Calculates Jacobian matrix, must be called after DirectKinematics
        jacobian_matrix = wrist_kinematics.Jacobian();

        // @TODO do we need gravity contributions here?
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
                actuators[0]->SetVelocity(0);
                actuators[1]->SetVelocity(0);
                actuators[2]->SetVelocity(0);
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

        for (uint8_t i = 0; i < ACTUATORS; i++)
        {
            actuators[i]->SetVelocity(joint_state(3 + i));
        }

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
    // Blink built in LED to indicate problem
    auto error_state = [&](String message) -> void
    {
        Serial.println(message);

        delete actuators[0];
        delete actuators[1];
        delete actuators[2];

        while (true)
        {
            digitalWrite(LED_BUILTIN, LOW);
            delay(1000);
            digitalWrite(LED_BUILTIN, HIGH);
            delay(1000);
        }
    };

    Serial.begin(INTERFACE_BAUDRATE);
    Serial.print("Starting Robot, firmware version ");
    Serial.print(VERSION_MAJOR);
    Serial.print(".");
    Serial.println(VERSION_MINOR);

    // Set pin and enable all actuators
    Actuator::SetEnablePin(ENABLE_PIN);
    Actuator::Enable();

    actuator_serial->begin(ACTUATOR_BAUDRATE);
    status &= InitializeActuators(actuator_serial);

    if (status != pdPASS)
    {
        error_state("Actuators did not initialize properly, check UART or power connection");
    }

    REGISTER_STEP_COUNTER(0);
    REGISTER_STEP_COUNTER(1);
    REGISTER_STEP_COUNTER(2);

    Serial.println("Actuator motors initialized successfully.");

    // Turn on LED to indicate correct operation
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.println("Set controller inputs to initial states, spawning controller.");

    // Set initial desired state
    desired_position.frame = XRotation(PI);

    status &= xTaskCreate(SerialInterface, NULL, 10 * configMINIMAL_SECURE_STACK_SIZE, &Serial, 1, &interface_handle);
    status &= xTaskCreate(ControlLoop, NULL, 100 * configMINIMAL_SECURE_STACK_SIZE, NULL, 2, &controller_handle);

    if (status != pdPASS)
    {
        error_state("Creation problem");
    }

    vTaskStartScheduler();

    error_state("Insufficient RAM");
}

//------------------------------------------------------------------------------
// WARNING idle loop has a very small stack (configMINIMAL_STACK_SIZE)
// loop must never block
void loop()
{
    // Not used.
}
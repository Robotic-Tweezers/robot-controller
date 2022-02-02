#include <Arduino.h>
#include <ArduinoJson.h>
#include <TimeLib.h>
#include <FreeRTOS_TEENSY4.h>
#include <config.hpp>
#include <kinematic.hpp>
#include <coordinates.hpp>
#include <stepper.hpp>

#define TIME_IN_MS(TIME) (TIME) * configTICK_RATE_HZ / 1000UL
#define TIME_IN_US(TIME) (TIME) * configTICK_RATE_HZ / 1000000UL

using namespace Eigen;
using namespace RobotTweezers;

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
time_t rtc_time;
// Controller variables
Stepper *steppers[3];
Coordinates desired_position;
Vector6f desired_velocity;
Vector6f joint_state;

static void ControlLoop(void *arg)
{
    Vector3f gravity_torque;
    Kinematic wrist_kinematics;
    Coordinates actual_position;
    Vector6f position_error, velocity_error, state_error;
    MatrixXf jacobian_matrix;

    while (true)
    {
        for (uint8_t i = 0; i < STEPPERS; i++)
        {
            joint_state(i) = steppers[i]->GetPosition();
        }

        // Direct kinematics of joint state (calculate end effector position)
        actual_position = wrist_kinematics.DirectKinematics(joint_state.head(3));

        // Calculates Jacobian matrix, must be called after DirectKinematics
        jacobian_matrix = wrist_kinematics.Jacobian();

        print(joint_state.head(3).matrix());

        // @TODO do we need gravity contributions here?
        //gravity_torque = wrist_kinematics.GravityTorque(joint_state.head(3));

        // Calculate position and velocity error
        position_error = desired_position - actual_position;
        // Goes unstable here
        velocity_error = desired_velocity - (jacobian_matrix * joint_state.tail(3));

        // Total state error
        state_error = (kp * position_error) + (kv * velocity_error);
        
        // Add gravity contributions
        joint_state.tail(3) = gravity_torque + (jacobian_matrix.transpose() * state_error);

        // Resolve singularity, since theta 0 always points in -z we need to check if theta 2 does
        if (jacobian_matrix(5, 2) <= -0.98)
        {
            // Set theta 0 velocity to zero
            joint_state(3) = 0.00;
        }

        for (uint8_t i = 0; i < STEPPERS; i++)
        {
            steppers[i]->SetVelocity(joint_state(3 + i));
        }

        vTaskDelay(TIME_IN_MS(CONTROLLER_LOOP_RATE));
    }
}

static void SerialInterface(void *arg)
{
    Stream *client_interface = static_cast<Stream*>(arg);
    StaticJsonDocument<256> gui_command;
    String json_command, response;

    while (true)
    {
        if (Serial.available() > 0)
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

            for (uint8_t i = 0; i < STEPPERS; i++)
            {
                gui_command["steppers"][i]["position"] = steppers[i]->GetPosition();
                serializeJson(gui_command, response);
                client_interface->print(response);
            }

            // Enable/Disable steppers
            if (gui_command.containsKey("enable_steppers"))
            {
                gui_command["enable_steppers"] ? Stepper::Enable() : Stepper::Disable();
            }
        }

        vTaskDelay(TIME_IN_MS(INTERFACE_LOOP_RATE));
    }
}

void setup()
{
    portBASE_TYPE status = pdPASS;
    HardwareSerial *stepper_serial = &Serial1;
    // Blink built in LED to indicate problem
    auto error_state = [&](String message) -> void
    {
        Serial.println(message);

        delete steppers[0];
        delete steppers[1];
        delete steppers[2];

        while (true)
        {
            digitalWrite(LED_BUILTIN, LOW);
            delay(1000);
            digitalWrite(LED_BUILTIN, HIGH);
            delay(1000);
        }
    };

    Serial.begin(9600);
    Serial.print("Starting Robot, firmware version ");
    Serial.print(VERSION_MAJOR);
    Serial.print(".");
    Serial.println(VERSION_MINOR);

    // Set pin and enable all steppers
    Stepper::SetEnablePin(ENABLE_PIN);
    Stepper::Enable();

    stepper_serial->begin(460800);

    steppers[0] = Stepper::StepperFactory(stepper_serial, THETA0_ADDRESS, THETA0_STEP, THETA0_DIRECTION);
    steppers[1] = Stepper::StepperFactory(stepper_serial, THETA1_ADDRESS, THETA1_STEP, THETA1_DIRECTION);
    steppers[2] = Stepper::StepperFactory(stepper_serial, THETA2_ADDRESS, THETA2_STEP, THETA2_DIRECTION);

    steppers[0]->step_counter_pin = THETA0_STEP_COUNT;
    steppers[1]->step_counter_pin = THETA1_STEP_COUNT;
    steppers[2]->step_counter_pin = THETA2_STEP_COUNT;

    for (auto stepper : steppers)
    {
        if (!stepper)
        {
            error_state("Steppers did not initialize properly, check UART connection");
        }
    }

    REGISTER_STEP_COUNTER(0);
    REGISTER_STEP_COUNTER(1);
    REGISTER_STEP_COUNTER(2);
    
    Serial.println("Stepper motors initialized successfully.");

    // Turn on LED to indicate correct operation
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    desired_position.frame = XRotation(PI) * ZRotation(-PI / 2);

    Serial.println("Set controller inputs to initial states, spawning controller.");

    status &= xTaskCreate(SerialInterface, NULL, 10 * configMINIMAL_SECURE_STACK_SIZE, &Serial, 1, NULL);
    status &= xTaskCreate(ControlLoop, NULL, 100 * configMINIMAL_SECURE_STACK_SIZE, NULL, 2, NULL);

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
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
Stepper *steppers[3];
Coordinates desired_position;
Vector6f desired_velocity;
Vector6f joint_state;

static void ControlLoop(void *arg)
{
    Vector3f gravity_torque, applied_torque;
    Kinematic wrist_kinematics;
    Coordinates actual_position;
    Vector6f position_error, velocity_error, state_error;
    MatrixXf jacobian_matrix;

    while (true)
    {
        for (uint8_t i = 0; i < STEPPERS; i++)
        {
            if (steppers[i])
            {
                joint_state(i) = steppers[i]->GetPosition();
            }
        }

        // Direct kinematics of joint state (calculate end effector position)
        actual_position = wrist_kinematics.DirectKinematics(joint_state.head(3));

        // Calculates Jacobian matrix, must be called after DirectKinematics
        jacobian_matrix = wrist_kinematics.Jacobian();

        // @TODO do we need gravity contributions here?
        //gravity_torque = wrist_kinematics.GravityTorque(joint_state.head(3));

        // Calculate position and velocity error
        position_error = desired_position - actual_position;
        // Goes unstable here
        velocity_error = desired_velocity - (jacobian_matrix * joint_state.tail(3));

        // Total state error
        state_error = (kp * position_error) + (kv * velocity_error);

        // Add gravity contributions
        applied_torque = gravity_torque + (jacobian_matrix.transpose() * state_error);

        for (uint8_t i = 0; i < STEPPERS; i++)
        {
            if (steppers[i])
            {
                joint_state(3 + i) = applied_torque(i);
                steppers[i]->SetVelocity(joint_state(3 + i));
            }
        }

        print(position_error);

        vTaskDelay(TIME_IN_MS(CONTROLLER_LOOP_RATE));
    }
}

static void SerialInterface(void *arg)
{
    while (true)
    {
        if (Serial.available() > 0)
        {
            StaticJsonDocument<200> gui_command;
            String json_data = Serial.readString();
            String response;
            DeserializationError err = deserializeJson(gui_command, json_data);
            if (err)
            {
                // Error in decoding message
                continue;
            }

            // If sender includes version keyword, respond with firmware verison
            if (gui_command.containsKey("version"))
            {
                uint8_t stepper_count = 0;
                // gui_command["version"] = "Robot Tweezers v%d.%d", (VERSION_MAJOR, VERSION_MINOR);
                for (auto stepper : steppers)
                {
                    if (stepper)
                    {
                        stepper_count++;
                    }
                }

                gui_command["stepper_count"] = stepper_count;
            }

            // Enable/Disable steppers
            if (gui_command.containsKey("enable_steppers"))
            {
                gui_command["enable_steppers"] ? Stepper::Enable() : Stepper::Disable();
            }

            if (gui_command.containsKey("steppers"))
            {
                for (uint8_t i = 0; i < STEPPERS; i++)
                {
                    uint8_t index = gui_command["steppers"][i]["index"];
                    //desired_position(index) = gui_command["steppers"][i]["desired_velocity"];
                    // steppers[index]->SetVelocity(gui_command["steppers"][i]["desired_velocity"]);
                }
            }

            for (uint8_t i = 0; i < STEPPERS; i++)
            {
                if (steppers[i])
                {
                    gui_command["steppers"][i]["position"] = steppers[i]->GetPosition();
                }
            }

            serializeJson(gui_command, response);
            Serial.print(response);
        }

        vTaskDelay(TIME_IN_MS(100));
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

    // Set pin and enable steppers
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

    desired_position.frame = XRotation(PI) * ZRotation(PI / 2);
    desired_position.origin = Vector3f(0, 0, 0);
    desired_velocity = Vector6f(0, 0, 0, 0, 0, 0);
    joint_state = Vector6f(0, 0, 0, 0, 0, 0);

    Serial.println("Set controller inputs to initial states, spawning controller.");

    // status &= xTaskCreate(SerialInterface, NULL, 10 * configMINIMAL_SECURE_STACK_SIZE, NULL, 1, NULL);
    //status &= xTaskCreate(ControlLoop, NULL, 200 * configMINIMAL_SECURE_STACK_SIZE, NULL, 2, NULL);

    auto ISR = [](void) -> void
    {
        Serial.println("Index pin!");
    };
    pinMode(19, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(19), ISR, CHANGE);
    steppers[0]->SetVelocity(1);
    while (1)
    {

    }

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
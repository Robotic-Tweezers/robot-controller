#include <Arduino.h>
#include <ArduinoJson.h>
#include <TimeLib.h>
#include <FreeRTOS_TEENSY4.h>
#include <TMCStepper.h>
#include <config.hpp>
#include <kinematic.hpp>
#include <coordinates.hpp>

#define TIME_IN_MS(TIME) (TIME) * configTICK_RATE_HZ / 1000UL
#define TIME_IN_US(TIME) (TIME) * configTICK_RATE_HZ / 1000000UL
#define STEPPERS 3
#define ENABLE_PIN 2        // Enable pin
#define THETA1_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define THETA2_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define THETA3_ADDRESS 0b10 // TMC2209 Driver address according to MS1 and MS2

using namespace Eigen;
using namespace RobotTweezers;

time_t rtc_time;
const Matrix6f kp = vectorToDiagnol6((float[]){0, 0, 0, 1.00, 1.00, 1.00});
const Matrix6f kv = vectorToDiagnol6((float[]){0.1, 0.5, 0.1, 0.1, 0.5, 0.1});
volatile float desired_position;

void controlLoop(void *arg)
{
    const unsigned int loop_period = 1000; // ms
    float theta[3] = {0, 0, 0};
    Vector3f theta_dot, gravity_torque, applied_torque;
    Kinematic wrist_kinematics(theta);
    Coordinates position, actual_position;
    Vector6f velocity(0, 0, 0, 0, 0, 0);
    Vector6f position_error, velocity_error, state_error;
    MatrixXf jacobian_matrix;

    while (true)
    {
        jacobian_matrix = wrist_kinematics.jacobian(theta);
        gravity_torque = wrist_kinematics.gravityTorque(theta);

        // Direct kinematics of joint state (calculate end effector position)
        actual_position = wrist_kinematics.directKinematics(theta);

        // Calculate position and velocity error
        position_error = position - actual_position;
        velocity_error = velocity - (jacobian_matrix * theta_dot);

        // Total state error
        state_error = (kp * position_error) + (kv * velocity_error);

        // Add gravity contributions
        applied_torque = gravity_torque + (jacobian_matrix.transpose() * state_error);

        vTaskDelay(TIME_IN_MS(loop_period));
    }
}

void serialInterface(void *arg)
{
    while (true)
    {
        if (Serial.available() > 0)
        {
            StaticJsonDocument<200> document;
            String json_data = Serial.readString();
            DeserializationError err = deserializeJson(document, json_data);
            if (err)
            {
                // Error in decoding message
                continue;
            }

            if (document.containsKey("name"))
            {
                String response;
                document["name"] = "Robot Tweezers v0.0";
                document["stepper_count"] = STEPPERS;
                serializeJson(document, response);
                Serial.print(response);
            }
        }

        vTaskDelay(TIME_IN_MS(100));
    }
}

void setup()
{
    portBASE_TYPE status = pdPASS;
    // Blink built in LED to indicate problem
    auto error_state = [](void) -> void
    {
        while (true)
        {
            digitalWrite(LED_BUILTIN, LOW);
            delay(1000);
            digitalWrite(LED_BUILTIN, HIGH);
            delay(1000);
        }
    };

    Serial.begin(9600);
    Serial1.begin(460800);
    for (uint8_t i = 0; i < STEPPERS; i++)
    {
        steppers[i] = createStepper(&Serial1, i);
    }

    // Enable steppers
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW);

    if (status != pdPASS)
    {
        Serial.println("Steppers did not initialize properly");
        error_state();
    }

    // Turn on LED to indicate correct operation
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    //status &= xTaskCreate(controlLoop, NULL, 10 * configMINIMAL_SECURE_STACK_SIZE, NULL, 2, NULL);
    status &= xTaskCreate(serialInterface, NULL, 10 * configMINIMAL_SECURE_STACK_SIZE, NULL, 1, NULL);

    if (status != pdPASS)
    {
        Serial.println("Creation problem");
        error_state();
    }

    vTaskStartScheduler();

    Serial.println("Insufficient RAM");
    error_state();
}

//------------------------------------------------------------------------------
// WARNING idle loop has a very small stack (configMINIMAL_STACK_SIZE)
// loop must never block
void loop()
{
    // Not used.
}
#include <Arduino.h>
#include <FreeRTOS_TEENSY4.h>
#include <TimeLib.h>
#include <config.hpp>
#include <stepper.hpp>
#include <kinematic.hpp>
#include <coordinates.hpp>

#define TIME_IN_MS(TIME) ((TIME) * configTICK_RATE_HZ / 1000UL)
#define TIME_IN_US(TIME) ((TIME) * configTICK_RATE_HZ / 1000000UL)

using namespace Eigen;
using namespace RobotTweezers;

time_t rtc_time;

const float kp[6] = {0, 0, 0, 1.00, 1.00, 1.00};
const float kv[6] = {0.1, 0.5, 0.1, 0.1, 0.5, 0.1};
volatile float desired_position;

Stepper test_stepper(11, 12, 10, 9, 8, 22, 23);

static void controlLoop(void* arg)
{
    float theta[3] = {0, 0, 0};
    const unsigned int loop_period = 200; // ms
    Vector3f theta_dot, gravity_torque, applied_torque;
    Kinematic wrist_kinematics(theta);
    Coordinates position, actual_position;
    Vector6f velocity(0, 0, 0, 0, 0, 0);
    Vector6f position_error, velocity_error, state_error;
    MatrixXf jacobian_matrix;

    Matrix6f position_gain = vectorToDiagnol6(kp);
    Matrix6f velocity_gain = vectorToDiagnol6(kv);

    position.frame = xRotation(PI) * zRotation(-PI / 2);
    // position.origin << 0, 0, -25;

    while (true)
    {
        theta[0] = test_stepper.encoder.position();
        theta_dot << test_stepper.encoder.velocity((float)loop_period / 1000.0), 0, 0;
        Serial.println(theta[0]);

        jacobian_matrix = wrist_kinematics.jacobian(theta);
        gravity_torque = wrist_kinematics.gravityTorque(theta);

        // Direct kinematics of joint state (calculate end effector position)
        actual_position.setCoordinates(wrist_kinematics.directKinematics(theta));

        // Calculate position and velocity error
        position_error = Coordinates::error(position, actual_position);
        velocity_error = velocity - (jacobian_matrix * theta_dot);

        // Total state error
        state_error = (position_gain * position_error) + (velocity_gain * velocity_error);

        // Add gravity contributions
        applied_torque = gravity_torque + (jacobian_matrix.transpose() * state_error);

        // Set output torques
        test_stepper.setVelocity(applied_torque(0));

        vTaskDelay(TIME_IN_MS(loop_period));
    }
}

static void pollSerial(void* arg)
{
    while (true)
    {
        if (Serial.available() > 0)
        {
            String message = Serial.readString(128);
            desired_position = message.toFloat();
            Serial.clear();
            Serial.println(desired_position);
        }

        vTaskDelay(TIME_IN_MS(200UL));
    }
}

void setup()
{
    portBASE_TYPE status = pdPASS;

    Serial.begin(9600);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    attachInterrupt(digitalPinToInterrupt(test_stepper.encoder.getPinA()), [](void) -> void
    {
        noInterrupts();
        test_stepper.encoder.pinInterruptA();
        interrupts();
    }, CHANGE);
    attachInterrupt(digitalPinToInterrupt(test_stepper.encoder.getPinB()), [](void) -> void
    {
        noInterrupts();
        test_stepper.encoder.pinInterruptB();
        interrupts();
    }, CHANGE);

    status &= xTaskCreate(pollSerial, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    status &= xTaskCreate(controlLoop, NULL, 10 * configMINIMAL_SECURE_STACK_SIZE, NULL, 2, NULL);

    if (status != pdPASS)
    {
        Serial.println("Creation problem");
        while (true);
    }

    vTaskStartScheduler();
    
    Serial.println("Insufficient RAM");
    while (true);
}

//------------------------------------------------------------------------------
// WARNING idle loop has a very small stack (configMINIMAL_STACK_SIZE)
// loop must never block
void loop()
{
    // Not used.
}
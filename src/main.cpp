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
using namespace robot_tweezers;

time_t rtc_time;

volatile Vector6f desired_vel;
const Vector6f position_gain(0.1, 0.5, 0.2, 0.9, 0.5, 0.9);
const Vector6f velocity_gain(0.1, 0.5, 0.1, 0.1, 0.5, 0.1);

Stepper test_stepper;
uint8_t enc_a = 22;
uint8_t enc_b = 23;

AngleAxisf euler2Quaternion( const float roll, const float pitch, const float yaw )
{
    Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());

    return AngleAxisf(yawAngle * pitchAngle * rollAngle);
}

static void controlLoop(void* arg)
{
    float theta[3] = {0, PI / 2, 0}; // = read_encoder();
    Vector3f theta_dot, gravity_torque, applied_torque;
    Kinematic wrist_kinematics(theta);
    Coordinates actual_pos, desired_pos;
    Vector6f position_err, velocity_err, state_err;
    Matrix6f position_gain, velocity_gain;

    while (true)
    {
        // Direct kinematics of joint state
        actual_pos.setCoordinates(wrist_kinematics.directKinematics(theta));
        PRINT(actual_pos.origin);
        
        //Vector3f a = actual_pos.frame.eulerAngles(0, 1, 2);
        //Matrix3f b = euler2Quaternion(a(0), a(1), a(2)).matrix();
        //print(b);
        //Matrix3f::eulerAngles
        //Quaternionf::
        //position_err << 
        //    desired_pos.origin - actual_pos.origin;
        // position_err = position_error(end_effector_coords, desired_pos);
        // velocity_err = desired_vel - (wrist_kinematics.jacobian(theta) * theta_dot);
        // state_err = (position_gain * position_err) + (velocity_gain * velocity_err);

        //applied_torque = gravity_torque + (wrist_kinematics.jacobian(theta).transpose() * state_err);

        vTaskDelay(TIME_IN_MS(1000UL));
    }
}

static void pulseMotor(void* arg)
{
    unsigned int delay = 500U;
    while (true)
    {
        digitalWrite(LED_BUILTIN, digitalRead(enc_a));
        test_stepper.stepMotor();
        vTaskDelay(TIME_IN_US(delay));
    }
}

static void pollSerial(void* arg)
{
    while (true)
    {
        if (Serial.available() > 0)
        {
            String message = Serial.readString(128);
            int res = message.toInt();
            switch (res)
            {
            case 8:
                test_stepper.setResolution(MICROSTEP8);
                break;
            case 32:
                test_stepper.setResolution(MICROSTEP32);
                break;
            case 64:
                test_stepper.setResolution(MICROSTEP64);
                break;
            case 16:
                test_stepper.setResolution(MICROSTEP16);
                break;
            }

            Serial.clear();
        }

        vTaskDelay(TIME_IN_MS(200UL));
    }
}

void setup()
{
    portBASE_TYPE status = pdPASS;

    Serial.begin(9600);

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(enc_a, INPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    // analogWriteFrequency(LED_BUILTIN, 1)
    // analogWrite(LED_BUILTIN, 128);

    test_stepper = Stepper(11, 12, 10, 9, 8);

    status &= xTaskCreate(pulseMotor, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    status &= xTaskCreate(pollSerial, NULL, configMINIMAL_STACK_SIZE, NULL, 2, NULL);
    status &= xTaskCreate(controlLoop, NULL, 10 * configMINIMAL_SECURE_STACK_SIZE, NULL, 3, NULL);

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
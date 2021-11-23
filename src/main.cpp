#include <Arduino.h>
#include <FreeRTOS_TEENSY4.h>
#include <TimeLib.h>

#include <config.hpp>
#include <motor.hpp>
#include <kinematic.hpp>
#include <coordinates.hpp>

#define TIME_IN_MS(TIME) ((TIME) * configTICK_RATE_HZ / 1000UL)

using namespace Eigen;
using namespace robot_tweezers;

motor::Motor sg90(1, 11);
time_t rtc_time;

volatile int duty_cycle;
volatile Vector6f desired_vel;
const Vector6f position_gain(0.1, 0.5, 0.2, 0.9, 0.5, 0.9);
const Vector6f velocity_gain(0.1, 0.5, 0.1, 0.1, 0.5, 0.1);

void print(Vector3f& v)
{
    for (int i = 0; i < 3; i++)
    {
        Serial.print(v(i));
        Serial.print(" ");
    }
    
    Serial.println();
}

void print(Matrix3f& m)
{   
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            Serial.print(m(i, j));
            Serial.print(" ");
        }
        
        Serial.println();
    }
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
        Matrix3f::euler
        position_err << 
            desired_pos.origin - actual_pos.origin;
        // position_err = position_error(end_effector_coords, desired_pos);
        // velocity_err = desired_vel - (wrist_kinematics.jacobian(theta) * theta_dot);
        // state_err = (position_gain * position_err) + (velocity_gain * velocity_err);

        //applied_torque = gravity_torque + (wrist_kinematics.jacobian(theta).transpose() * state_err);

        vTaskDelay(TIME_IN_MS(1000UL));
    }
}

static void pulseMotor(void* arg)
{
    while (true)
    {
        sg90.pwmStep(duty_cycle);
        // sleep for 0.5 ms
        vTaskDelay(TIME_IN_MS(1000UL));
    }
}

static void pollSerial(void* arg)
{
    while (true)
    {
        if (Serial.available() > 0)
        {
            String message = Serial.readString(128);
            if (message.startsWith("-r"))
            {
                /*
                int last_space = 2;
                int theta_i = 0;

                for (unsigned int i = 3; i < message.length(); i++)
                {
                    if (message[i] == ' ')
                    {
                        theta[theta_i++] = message.substring(last_space + 1, i).toFloat();
                        last_space = i;
                    }
                }
                theta[theta_i] = message.substring(last_space + 1).toFloat();
                */
            }

            Serial.clear();
        }

        vTaskDelay(TIME_IN_MS(200UL));
    }
}

void setup()
{
    portBASE_TYPE status = pdPASS;
    duty_cycle = 2;

    Serial.begin(9600);

    // status &= xTaskCreate(pulseMotor, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    status &= xTaskCreate(pollSerial, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    status &= xTaskCreate(controlLoop, NULL, 2 * configMINIMAL_SECURE_STACK_SIZE, NULL, 2, NULL);

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
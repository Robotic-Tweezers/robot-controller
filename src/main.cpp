#include <Arduino.h>
#include <FreeRTOS_TEENSY4.h>
#include <TimeLib.h>
#include <config.hpp>
#include <stepper.hpp>
#include <kinematic.hpp>
#include <coordinates.hpp>

#define TIME_IN_MS(TIME)    ((TIME) * configTICK_RATE_HZ / 1000UL)
#define TIME_IN_US(TIME)    ((TIME) * configTICK_RATE_HZ / 1000000UL)
#define STEPPERS            (1)

using namespace Eigen;
using namespace RobotTweezers;

time_t rtc_time;

const Matrix6f kp = vectorToDiagnol6((float[]){0, 0, 0, 1.00, 1.00, 1.00});
const Matrix6f kv = vectorToDiagnol6((float[]){0.1, 0.5, 0.1, 0.1, 0.5, 0.1});
volatile float desired_position;
Stepper steppers[STEPPERS] = 
{
    Stepper(0, 11, 12, 10, 9, 8, 22, 23),
    // Stepper(1, 3, 4, 5, 6, 7, 12, 21),           // Not used yet
    // Stepper(2, 14, 15, 16, 17, 18, 19, 20),    // Not used yet
};

// Cannot use local variables to initialize interrupts (so no loop here)
static inline void configureStepperInterrupts(void)
{
    #define REGISTER_ENCODER_INTERRUPTS(IDX) ( \
    { \
        attachInterrupt(digitalPinToInterrupt(steppers[(IDX)].encoder.getPinA()), [](void) -> void \
        { \
            noInterrupts(); \
            steppers[(IDX)].encoder.pinInterruptA(); \
            interrupts(); \
        }, CHANGE); \
        attachInterrupt(digitalPinToInterrupt(steppers[(IDX)].encoder.getPinB()), [](void) -> void \
        { \
            noInterrupts(); \
            steppers[(IDX)].encoder.pinInterruptB(); \
            interrupts(); \
        }, CHANGE); \
    })
    
    REGISTER_ENCODER_INTERRUPTS(0);
    // REGISTER_ENCODER_INTERRUPTS(1);
    // REGISTER_ENCODER_INTERRUPTS(2);
}

static void readJointState(float theta[], Vector3f& theta_dot, float interval)
{
    for (uint8_t i = 0; i < (uint8_t)STEPPERS; i++)
    {
        theta[i] = steppers[i].encoder.position();
        theta_dot(i) = steppers[i].encoder.velocity(interval);
    }
}

static void getDesiredState(Coordinates& position, Vector6f& velocity, float interval)
{
    // to be used to calculate desired velocity;
    static Coordinates previous_position;

    // Hard coded setpoint
    position.frame = xRotation(PI) * zRotation(PI / 2);
}

static void controlLoop(void* arg)
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
        readJointState(theta, theta_dot, (float)loop_period / 1000.0);
        getDesiredState(position, velocity, (float)loop_period / 1000.0);

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

        // Set output torques
        for (uint8_t i = 0; i < (uint8_t)STEPPERS; i++)
        {
            steppers[i].setVelocity(applied_torque(i));
        }

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
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    configureStepperInterrupts();

    status &= xTaskCreate(pollSerial, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    // status &= xTaskCreate(controlLoop, NULL, 10 * configMINIMAL_SECURE_STACK_SIZE, NULL, 2, NULL);
    
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
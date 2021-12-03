#include <Arduino.h>
#include <FreeRTOS_TEENSY4.h>
#include <TimeLib.h>
#include <config.hpp>
#include <stepper.hpp>
#include <kinematic.hpp>
#include <coordinates.hpp>
#include <TMCStepper.h>

#define TIME_IN_MS(TIME) (TIME) * configTICK_RATE_HZ / 1000UL
#define TIME_IN_US(TIME) (TIME) * configTICK_RATE_HZ / 1000000UL
#define STEPPERS 3
#define STALL_VALUE 50      // [0... 255]
#define TOFF_VALUE 4        // [1... 15]
#define ENABLE_PIN 2        // Enable pin
#define THETA1_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define THETA2_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define THETA3_ADDRESS 0b10 // TMC2209 Driver address according to MS1 and MS2
#define ADDRESS(STEPPER)    (int8_t)((STEPPER).ms2()) << 1 | (int8_t)((STEPPER).ms1())
// Match to your driver
// SilentStepStick series use 0.11
// UltiMachine Einsy and Archim2 boards use 0.2
// Panucatt BSD2660 uses 0.1
// Watterott TMC5160 uses 0.075
#define R_SENSE 0.11f

using namespace Eigen;
using namespace RobotTweezers;

time_t rtc_time;

const Matrix6f kp = vectorToDiagnol6((float[]){0, 0, 0, 1.00, 1.00, 1.00});
const Matrix6f kv = vectorToDiagnol6((float[]){0.1, 0.5, 0.1, 0.1, 0.5, 0.1});
volatile float desired_position;
TMC2209Stepper steppers[STEPPERS] =
{
    TMC2209Stepper(&Serial1, R_SENSE, THETA1_ADDRESS),
    TMC2209Stepper(&Serial2, R_SENSE, THETA2_ADDRESS),
    TMC2209Stepper(&Serial3, R_SENSE, THETA3_ADDRESS),
};

static bool stepperInit(TMC2209Stepper& stepper)
{
    static int i = 0;
    /// @TODO Figure out what these do
    stepper.begin();
    stepper.toff(TOFF_VALUE);
    // Set velocity
    stepper.VACTUAL(0);
    /// @TODO Figure out what these do
    stepper.blank_time(24);
    stepper.rms_current(400);
    // Set microstep resolution
    stepper.microsteps(64);
    /// @TODO Figure out what these do
    stepper.TCOOLTHRS(0xFFFFF);
    stepper.semin(5);
    stepper.semax(2);
    stepper.sedn(0b01);
    stepper.SGTHRS(STALL_VALUE);

    Serial.println("Testing connection...");
    uint8_t result = stepper.test_connection();
    if (result)
    {
        char print_buffer[128];
        sprintf(print_buffer, "Failed to connect to stepper at address %d, likely cause:", i++);//ADDRESS(stepper));
        Serial.println(print_buffer);
        result == 1 ? Serial.println("loose connection") : Serial.println("no power");
        Serial.println("Fix the problem and reset board.");
        return false;
    }

    return true;
}

static void serialEvent(void* arg)
{
    float vel = 2000;
    while (true)
    {
        if (Serial.available())
        {
            switch (Serial.read())
            {
            case '+':
                Serial.println("Move forward");
                steppers[0].VACTUAL(3000);
                steppers[1].VACTUAL(3000);
                break;
            case '-':
                Serial.println("Move backward");
                steppers[0].VACTUAL(-3000);
                steppers[1].VACTUAL(-3000);
                break;
            case '0':
                Serial.println("Stop");
                steppers[0].VACTUAL(0);
                steppers[1].VACTUAL(0);
                break;
            default:
                break;
            }
            
            Serial.clear();
        }

        vTaskDelay(TIME_IN_MS(100));
    }
}
/*
static void readJointState(float theta[], Vector3f &theta_dot, float interval)
{
    for (uint8_t i = 0; i < (uint8_t)STEPPERS; i++)
    {
        theta[i] = steppers[i].encoder.position();
        theta_dot(i) = steppers[i].encoder.velocity(interval);
    }
}

static void getDesiredState(Coordinates &position, Vector6f &velocity, float interval)
{
    // to be used to calculate desired velocity;
    static Coordinates previous_position;

    // Hard coded setpoint
    position.frame = xRotation(PI) * zRotation(PI / 2);
}

static void controlLoop(void *arg)
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
*/

TMC2209Stepper driver(&Serial1, R_SENSE, 0x00);

int32_t speed = 5000;

static void stepperUartTesting(void *arg)
{
    uint16_t last_position, position;

    Serial.println("OK");

    while (true)
    {
        static uint32_t last_time = 0;

        uint32_t ms = millis();

        while (Serial.available() > 0)
        {
            int8_t read_byte = Serial.read();

            if (read_byte == '0')
            {
                Serial.print("Motor ");

                if (driver.toff() == 0)
                {
                    Serial.print("already ");
                }

                Serial.println("disabled.");
                driver.toff(0);
            }
            else if (read_byte == '1')
            {
                Serial.print("Motor ");

                if (driver.toff() != 0)
                {
                    Serial.print("already ");
                }

                Serial.println("enabled.");
                driver.toff(TOFF_VALUE);
            }
            else if (read_byte == '+')
            {
                speed += 1000;

                if (speed == 0)
                {
                    Serial.println("Hold motor.");
                }
                else
                {
                    Serial.println("Increase speed.");
                }

                driver.VACTUAL(speed);
            }
            else if (read_byte == '-')
            {
                speed -= 1000;

                if (speed == 0)
                {
                    Serial.println("Hold motor.");
                }
                else
                {
                    Serial.println("Decrease speed.");
                }

                driver.VACTUAL(speed);
            }
        }

        if ((ms - last_time) > 100 && driver.toff() != 0 && speed != 0)
        {
            position = driver.MSCNT();
            float delta = ((float)position - (float)last_position); // / ((float)ms - (float)last_time);
            last_position = position;
            last_time = ms;
            Serial.print("Status: ");
            Serial.print(driver.SG_RESULT(), DEC);
            Serial.print(" ");
            Serial.print(driver.SG_RESULT() < STALL_VALUE, DEC);
            Serial.print(" ");
            Serial.print(driver.cs2rms(driver.cs_actual()), DEC);
            Serial.print(" ");
            Serial.println(delta);
        }
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
    Serial1.begin(115200);
    Serial2.begin(115200);
    Serial3.begin(115200);

    // Enable steppers
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW);

    //for (TMC2209Stepper& stepper : steppers)
    //{
    //    status &= stepperInit(stepper);
    //}
    status &= stepperInit(steppers[0]);
    status &= stepperInit(steppers[1]);
    if (status != pdPASS)
    {
        Serial.println("Steppers did not initialize properly");
        error_state();
    }

    // Turn on LED to indicate correct operation
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    // status &= xTaskCreate(controlLoop, NULL, 10 * configMINIMAL_SECURE_STACK_SIZE, NULL, 2, NULL);
    // status &= xTaskCreate(stepperUartTesting, NULL, 10 * configMINIMAL_SECURE_STACK_SIZE, NULL, 1, NULL);
    status &= xTaskCreate(serialEvent, NULL, 10 * configMINIMAL_SECURE_STACK_SIZE, NULL, 1, NULL);

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
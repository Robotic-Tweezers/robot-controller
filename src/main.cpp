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

void swuart_calcCRC(uint8_t* datagram, uint8_t datagramLength)
{
    uint8_t* crc = datagram + (datagramLength-1); // CRC located in last byte of message
    uint8_t currentByte;

    *crc = 0;
    // Execute for all bytes of a message
    for (uint8_t i = 0; i < (datagramLength - 1); i++)
    {
        currentByte = datagram[i]; // Retrieve a byte to be sent from Array
        for (uint8_t j = 0; j < 8; j++)
        {
            if ((*crc >> 7) ^ (currentByte & 0x01)) // update CRC based result of XOR operation
            {
                *crc = (*crc << 1) ^ 0x07;
            }
            else
            {
                *crc = (*crc << 1);
            }

            currentByte = currentByte >> 1;
        } // for CRC bit
    } // for message byte
}

static void stepperUartTesting(void* arg)
{
    uint8_t step = 11;
    uint8_t direction = 12;
    uint8_t enable = 10;
    uint8_t microstep1 = 9;
    uint8_t microstep2 = 8;
    uint8_t buffer[8] = {0xA0, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00};
    char read_buffer[8];
    swuart_calcCRC(buffer, 8);

    digitalWrite(step, LOW);
    digitalWrite(direction, LOW);
    digitalWrite(enable, LOW);
    digitalWrite(microstep1, LOW);
    digitalWrite(microstep2, LOW);
    while (true)
    {
        Serial1.write(buffer, 8);
        if (Serial1.available() > 0)
        {
            Serial1.readBytes(read_buffer, 8);
            sprintf(read_buffer, "%x%x%x%x%x%x%x%x", 
                read_buffer[0], read_buffer[1], read_buffer[2], read_buffer[3], 
                read_buffer[4], read_buffer[5], read_buffer[6], read_buffer[7]);
            Serial.println(read_buffer);
            Serial1.clear();
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
    Serial1.begin(9600);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    // configureStepperInterrupts();

    // status &= xTaskCreate(pollSerial, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    // status &= xTaskCreate(controlLoop, NULL, 10 * configMINIMAL_SECURE_STACK_SIZE, NULL, 2, NULL);
    status &= xTaskCreate(stepperUartTesting, NULL, 10 * configMINIMAL_SECURE_STACK_SIZE, NULL, 1, NULL);
    
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
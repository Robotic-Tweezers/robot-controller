#include <Arduino.h>
#include <FreeRTOS_TEENSY4.h>
#include <TimeLib.h>
#include <config.hpp>
#include <stepper.hpp>
#include <kinematic.hpp>
#include <coordinates.hpp>
#include <TMCStepper.h>

#define TIME_IN_MS(TIME) ((TIME)*configTICK_RATE_HZ / 1000UL)
#define TIME_IN_US(TIME) ((TIME)*configTICK_RATE_HZ / 1000000UL)
#define STEPPERS (1)

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
#define REGISTER_ENCODER_INTERRUPTS(IDX) (                                                           \
    {                                                                                                \
        attachInterrupt(                                                                             \
            digitalPinToInterrupt(steppers[(IDX)].encoder.getPinA()), [](void) -> void { \
            noInterrupts(); \
            steppers[(IDX)].encoder.pinInterruptA(); \
            interrupts(); }, CHANGE); \
        attachInterrupt(                                                                             \
            digitalPinToInterrupt(steppers[(IDX)].encoder.getPinB()), [](void) -> void { \
            noInterrupts(); \
            steppers[(IDX)].encoder.pinInterruptB(); \
            interrupts(); }, CHANGE); \
    })

    REGISTER_ENCODER_INTERRUPTS(0);
    // REGISTER_ENCODER_INTERRUPTS(1);
    // REGISTER_ENCODER_INTERRUPTS(2);
}

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

static void pollSerial(void *arg)
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

void swuart_calcCRC(uint8_t *datagram, uint8_t datagramLength)
{
    uint8_t *crc = datagram + (datagramLength - 1); // CRC located in last byte of message
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
    }     // for message byte
}

static void manualStepperUartTesting(void* arg)
{
    uint8_t buffer[4] = {0xA0, 0x00, 0x6, 0x00}; // , 0x00, 0x00, 0x00, 0x00};
    char read_buffer[8];
    swuart_calcCRC(buffer, 4);

    // driver.begin();                 //  SPI: Init CS pins and possible SW SPI pins
                                    // UART: Init SW UART (if selected) with default 115200 baudrate
    // driver.toff(4);                 // Enables driver in software
    // driver.rms_current(2000);       // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
    // driver.microsteps(16);
    // driver.pwm_autoscale(true);     // Needed for stealthChop 

    while (true)
    {
        Serial1.write(buffer, 4);
        Serial1.flush();
        vTaskDelay(TIME_IN_MS(500));
        if (Serial1.available() > 0)
        {
            Serial1.readBytes(read_buffer, 8);
            sprintf(read_buffer, "%x%x%x%x%x%x%x%x", 
                read_buffer[0], read_buffer[1], read_buffer[2], read_buffer[3], 
                read_buffer[4], read_buffer[5], read_buffer[6], read_buffer[7]);
            Serial.println(read_buffer);
        }
        
        Serial1.clear();

        vTaskDelay(TIME_IN_MS(500));
    }
}

#define STALL_VALUE 50 // [0... 255]
#define TOFF_VALUE 4   // [1... 15]

#define EN_PIN 6 // Enable pin
#define SW_RX 2  // SoftwareSerial receive pin
#define SW_TX 3  // SoftwareSerial transmit pin
//#define SERIAL_PORT Serial1 // HardwareSerial port
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2

#define R_SENSE 0.11f // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075

TMC2209Stepper driver(&Serial1, R_SENSE, 0x00);

int32_t speed = 5000;

static void stepperUartTesting(void *arg)
{
    uint16_t last_position, position;
    driver.begin();
    driver.toff(TOFF_VALUE);
    driver.VACTUAL(speed);
    driver.blank_time(24);
    driver.rms_current(400);
    driver.microsteps(16);
    driver.TCOOLTHRS(0xFFFFF);
    driver.semin(5);
    driver.semax(2);
    driver.sedn(0b01);
    driver.SGTHRS(STALL_VALUE);
    last_position = driver.MSCNT();

    Serial.print("\nTesting connection...");
    uint8_t result = driver.test_connection();

    if (result)
    {
        Serial.println("failed!");
        Serial.print("Likely cause: ");

        switch (result)
        {
        case 1:
            Serial.println("loose connection");
            break;
        case 2:
            Serial.println("no power");
            break;
        }

        Serial.println("Fix the problem and reset board.");

        // We need this delay or messages above don't get fully printed out
        delay(100);
        abort();
    }

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
            float delta = ((float)position - (float)last_position);// / ((float)ms - (float)last_time);
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
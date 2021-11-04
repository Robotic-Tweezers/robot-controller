#include <Arduino.h>
#include <FreeRTOS_TEENSY4.h>

#include <config.hpp>
#include <motor.hpp>

motor::Motor sg90(1, 11);
volatile int duty_cycle;

static void pulseMotor(void* arg)
{
    while (true)
    {
        sg90.pwmStep(duty_cycle);
        // sleep for 0.5 ms
        vTaskDelay(configTICK_RATE_HZ / 2000L);
    }
}

static void pollSerial(void* arg)
{
    while (true)
    {
        if (Serial.available() > 0)
        {
            char c = Serial.read();
            if (c <= '9' && c >= '0')
            {
                duty_cycle = c - '0';
            }
            
            Serial.print("Recieved new duty cycle: ");
            Serial.println(duty_cycle);
            Serial.clear();
        }

        vTaskDelay(200UL * configTICK_RATE_HZ / 1000UL);
    }
}

void setup()
{
    portBASE_TYPE status = pdPASS;
    duty_cycle = 2;

    Serial.begin(9600);

    status &= xTaskCreate(pulseMotor, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    status &= xTaskCreate(pollSerial, NULL, configMINIMAL_STACK_SIZE, NULL, 2, NULL);
    
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
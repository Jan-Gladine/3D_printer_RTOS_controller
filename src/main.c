/*
 * File:   main.c
 * Author: Joost
 *
 * Adaptation of the blinky example for the Atmega2560
 * Demonstrates a couple of important concepts:
 * - Serial communication using the interrupt based functions from the library
 * - Exact periodic behavior vs relative behavior
 * - How binary semaphores can be used for synchronization.
 * Created on September 28, 2020, 11:05 AM
 */

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

/* Scheduler include files. */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* serial interface include file. */
#include "serial.h"

/* system time include file. */
#include "time.h"

/* RAMPS board macros */
#include "RAMPS_macros.h"
#include "inet.h"

/* 3D printer control */
#include "motorController.h"

/*PID controller*/
#include "PID_controller.h"

/*Arduino main library*/
#include "Arduino.h"

/*temperature table*/
#include "thermistortable_1.h"

/*-----------------------------------------------------------*/
/* Create a handle for the serial port. */
extern xComPortHandle xSerialPort;

/* Setup functions*/

/* Tasks */
static void TaskBlinkRedLED(void *pvParameters); // Main Arduino Mega 2560, Freetronics EtherMega (Red) LED Blink
static void TaskTransmitSerialDebug(void *pvParameters); // example task that periodically transmits something
//static void ButtonTask(void *pvParameters);
static void MotorTask(void *pvParameters);
static void PIDTask(void *pvParameters);
int checkTable(int thermisttor);

/*-----------------------------------------------------------*/
/*global variables*/
int global_temp;
int global_power;
int measured_temp_C;

/* Main program loop */
int main(void) __attribute__((OS_main));

int main(void)
{
    init();
    // turn on the serial port for debugging or for other USART reasons.
	//xSerialPort = xSerialPortInitMinimal( USART0, 38400, portSERIAL_BUFFER_TX, portSERIAL_BUFFER_RX); //  serial port: WantedBaud, TxQueueLength, RxQueueLength (8n1)
    xSerialPort = xSerialPortInitMinimal( USART0, 115200, portSERIAL_BUFFER_TX, portSERIAL_BUFFER_RX); //  serial port: WantedBaud, TxQueueLength, RxQueueLength (8n1)
    
	avrSerialxPrint_P( &xSerialPort, PSTR("\r\n\n\nHello World!\r\n")); // Ok, so we're alive...

    // create all the tasks
    // xTaskCreate(
    //         TaskBlinkRedLED
    //         ,  (const char *)"RedLED" // Main Arduino Mega 2560, Freetronics EtherMega (Red) LED Blink
    //         ,  256				// Tested 9 free @ 208
    //         ,  NULL
    //         ,  2 // priority of 1, the idle task has priority of 0
    //         ,  NULL ); // */
    xTaskCreate(
            TaskTransmitSerialDebug
            , (const char*)"Debug"
            , 128
            , NULL
            , 1
            , NULL
            );
    xTaskCreate(
            PIDTask
            , (const char *)"PID"
            , 256
            , NULL
            , 2
            , NULL
            );
//    xTaskCreate(
//            ButtonTask
//            , (const char*)"btn"
//            , 128
//            , NULL
//            , configMAX_PRIORITIES // give this task the max priority
//            , NULL
//            );
    xTaskCreate(
            MotorTask
            , (const char*)"mot"
            , 256
            , NULL
            , 2
            , NULL
            );
    
    // Some debug info, e.g. how much heap is still free?
	avrSerialPrintf_P(PSTR("\r\n\nFree Heap Size: %u\r\n"),xPortGetFreeHeapSize() ); // needs heap_1 or heap_2 for this function to succeed.
    // Start the scheduler. Because we use heap1, we can now only deallocate FreeRTOS objects
	vTaskStartScheduler();
    // If the scheduler returns, means we have run out of RAM, otherwise it'll run forever
	avrSerialxPrint_P( &xSerialPort, PSTR("\r\n\n\nGoodbye... no space for idle task!\r\n")); // Doh, so we're dead...
}

/*-----------------------------------------------------------*/

static void TaskBlinkRedLED(void *pvParameters) // Main Red LED Flash
{
    (void) pvParameters;

    TickType_t xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialized with the current tick
    count. Note that this is the only time the variable is written to explicitly.
    After this xLastWakeTime is automatically updated within vTaskDelayUntil(). */
	xLastWakeTime = xTaskGetTickCount();

	DDRB |= _BV(DDB7);

    while(1)
    {
    	PORTB |=  _BV(PORTB7);       // main (red IO_B7) LED on. EtherMega LED on
		vTaskDelayUntil( &xLastWakeTime, ( 100 / portTICK_PERIOD_MS ) );

		PORTB &= ~_BV(PORTB7);       // main (red IO_B7) LED off. EtherMega LED off
		vTaskDelayUntil( &xLastWakeTime, ( 400 / portTICK_PERIOD_MS ) );

		xSerialxPrintf_P( &xSerialPort, PSTR("RedLED HighWater @ %u\r\n"), uxTaskGetStackHighWaterMark(NULL));
        // What is the high watermark? Can you find it in the FreeRTOS the documentation?
    }
}

/* 
 */
static void TaskTransmitSerialDebug(void *pvParameters){
    
    (void) pvParameters;
    // going to printout "every" second
    const TickType_t xDelay1000ms = pdMS_TO_TICKS(1000);
    // keep track of activation count of the task here
    unsigned int activationCount = 0;
    // infinite loop
    for(;;)
    {   
        // using local SRAM instead of program memory for the array (not really recommended)
        const char local_data[] = "Activation nr.: %u\r\n";
        const char measured_temp[] = "measured temperature = %d \n";
        xSerialPrintf(local_data, activationCount);
        xSerialPrintf(measured_temp, global_temp);
        xSerialPrintf("temp in C: %d \n", measured_temp_C);
        xSerialPrintf("output power = %d \n", global_power);
        vTaskDelay(xDelay1000ms); // what's the difference with vTaskDelayUntil?
        ++activationCount;
    } 
}

/* Task demonstrating deferred interrupt processing with a printout
 */
//static void ButtonTask(void *pvParameters){
//    
//    TickType_t xLastWakeTime;
//	/* The xLastWakeTime variable needs to be initialized with the current tick
//    count. Note that this is the only time the variable is written to explicitly.
//    After this xLastWakeTime is automatically updated within vTaskDelayUntil(). */
//	xLastWakeTime = xTaskGetTickCount();
//    
//    for(;;){
//        
//        if(X_lim_sw_ch.rising_edge){
//            xSerialPrint_P(PSTR("Rising edge\r\n"));
//            X_lim_sw_ch.rising_edge = 0;
//        }
//        if(X_lim_sw_ch.falling_edge){
//            xSerialPrint_P(PSTR("Falling edge\r\n"));
//            X_lim_sw_ch.falling_edge = 0;
//        }
//        
//        vTaskDelayUntil( &xLastWakeTime, ( pdMS_TO_TICKS(500) ));
//    }
//}

static void PIDTask(void *pvParameters){
    digitalWrite(9, HIGH);
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(100);
    PID_controller_U.In1 = 200; //PID input = target temperature 
    //setup ADC pins
    //ADMUX |= (1<<REFS0);
    //ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);
    for (;;)
    {
        int measuredTemp = analogRead(THERM0_PIN);
        int measuredTempInC = checkTable(measuredTemp);
        PID_controller_U.In2 = measuredTempInC; //PID input = measured temperature
        PID_controller_step();
        double power = PID_controller_Y.Out1;
        int analogPower = (power/40)*255;
        if (analogPower > 255)
        {
            analogPower = 255;
        }
        analogWrite(10, analogPower); 
        global_temp = measuredTemp;    
        global_power = analogPower;   
        measured_temp_C = measuredTempInC;
        vTaskDelayUntil(&xLastWakeTime, frequency);
    }  
}

static void MotorTask(void *pvParameters){
    
    TickType_t xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialized with the current tick
    count. Note that this is the only time the variable is written to explicitly.
    After this xLastWakeTime is automatically updated within vTaskDelayUntil(). */
	xLastWakeTime = xTaskGetTickCount();
    
    initMotors();
    xSerialPrint("Homing\r\n");
    homeMotors();
    
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    //moveMotor_microsteps(&x_motor, 20, 3200, 0);
    moveMotor_mm(&x_motor, 50, 20, 1);
    moveMotor_mm(&y_motor, 50, 20, 1);
    moveMotor_mm(&z_motor, 50, 20, 1);
    
    while(x_motor.motor_state != ready && y_motor.motor_state != ready && z_motor.motor_state != ready){
        vTaskDelay(pdMS_TO_TICKS(100));
        // wait till all motors are ready again.
    }
    
    
    for(;;){
        
        // draw a small square
        for(int i = 0; i < 20; ++i){
            moveMotor_mm(&x_motor, 50, 20, 1);
            while(x_motor.motor_state != ready){vTaskDelay(pdMS_TO_TICKS(100));}
            moveMotor_mm(&y_motor, 50, 20, 1);
            while(y_motor.motor_state != ready){vTaskDelay(pdMS_TO_TICKS(100));}
            moveMotor_mm(&x_motor, 50, 20, 0);
            while(x_motor.motor_state != ready){vTaskDelay(pdMS_TO_TICKS(100));}
            moveMotor_mm(&y_motor, 50, 20, 0);
            while(y_motor.motor_state != ready){vTaskDelay(pdMS_TO_TICKS(100));}
            moveMotor_mm(&z_motor, 0.4, 5, 1);
            while(z_motor.motor_state != ready){vTaskDelay(pdMS_TO_TICKS(100));}
        }
        extrude_mm_filament(20, 5);
        
        xSerialPrintf("remsteps %u\r\n", x_motor.remaining_steps);
    }
}
int checkTable(int thermistor){
    int x1,x2,y1,y2;
    int result = 300;
    for (int i = 0; i < sizeof(temptable_1)/sizeof(temptable_1[0]); i++)
    {
        if (thermistor == temptable_1[i][0])
        {
            result = temptable_1[i][1];
        }
        else{
            x1 = temptable_1[i][0];
            y1 = temptable_1[i][1];
            x2 = temptable_1[i + 1][0];
            y2 = temptable_1[i + 1][1];
            if (thermistor >= x1 && thermistor <= x2) {
                result = y1 + ((thermistor - x1) * (y2 - y1)) / (x2 - x1);
            }
        }
    }
    return result;
}



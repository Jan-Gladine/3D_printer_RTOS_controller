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

/*-----------------------------------------------------------*/
/* Create a handle for the serial port. */
extern xComPortHandle xSerialPort;

/* store button events in a struct */
typedef struct {
    uint8_t state : 1;
    uint8_t rising_edge : 1;
    uint8_t falling_edge : 1;
}button_changes_t;

static button_changes_t X_lim_sw_ch = {0, 0, 0};

/* store motor state also in a struct */
typedef enum {
    homing,
    busy,
    ready
} motor_state_t;

typedef struct {
    motor_state_t motor_state;
    uint32_t remaining_steps;
    int32_t speed; // in steps per second or some other measure.
    uint8_t dir;
}motor_t; 

static motor_t E0_motor = {0, 0, 0, 0};

/* Setup functions*/
static void endstopConfig();
static void configMotors();
static void initMotor(motor_t* motor);
static void moveMotor(motor_t* motor);

/* Tasks */
static void TaskBlinkRedLED(void *pvParameters); // Main Arduino Mega 2560, Freetronics EtherMega (Red) LED Blink
static void TaskTransmitSerialDebug(void *pvParameters); // example task that periodically transmits something
static void ButtonTask(void *pvParameters);
static void MotorTask(void *pvParameters);

/*-----------------------------------------------------------*/

/* Main program loop */
int main(void) __attribute__((OS_main));

int main(void)
{

    // turn on the serial port for debugging or for other USART reasons.
	//xSerialPort = xSerialPortInitMinimal( USART0, 38400, portSERIAL_BUFFER_TX, portSERIAL_BUFFER_RX); //  serial port: WantedBaud, TxQueueLength, RxQueueLength (8n1)
    xSerialPort = xSerialPortInitMinimal( USART0, 115200, portSERIAL_BUFFER_TX, portSERIAL_BUFFER_RX); //  serial port: WantedBaud, TxQueueLength, RxQueueLength (8n1)
    
	avrSerialxPrint_P( &xSerialPort, PSTR("\r\n\n\nHello World!\r\n")); // Ok, so we're alive...
    
    // perform further hardware setup
    endstopConfig();
    
    // create all the tasks
    xTaskCreate(
            TaskBlinkRedLED
            ,  (const char *)"RedLED" // Main Arduino Mega 2560, Freetronics EtherMega (Red) LED Blink
            ,  256				// Tested 9 free @ 208
            ,  NULL
            ,  2 // priority of 1, the idle task has priority of 0
            ,  NULL ); // */
    xTaskCreate(
            TaskTransmitSerialDebug
            , (const char*)"Debug"
            , 128
            , NULL
            , 1
            , NULL
            );
    xTaskCreate(
            ButtonTask
            , (const char*)"btn"
            , 128
            , NULL
            , configMAX_PRIORITIES // give this task the max priority
            , NULL
            );
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
        xSerialPrintf(local_data, activationCount);
        vTaskDelay(xDelay1000ms); // what's the difference with vTaskDelayUntil?
        ++activationCount;
    } 
}

/* Task demonstrating deferred interrupt processing with a printout
 */
static void ButtonTask(void *pvParameters){
    
    TickType_t xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialized with the current tick
    count. Note that this is the only time the variable is written to explicitly.
    After this xLastWakeTime is automatically updated within vTaskDelayUntil(). */
	xLastWakeTime = xTaskGetTickCount();
    
    for(;;){
        
        if(X_lim_sw_ch.rising_edge){
            xSerialPrint_P(PSTR("Rising edge\r\n"));
            X_lim_sw_ch.rising_edge = 0;
        }
        if(X_lim_sw_ch.falling_edge){
            xSerialPrint_P(PSTR("Falling edge\r\n"));
            X_lim_sw_ch.falling_edge = 0;
        }
        
        vTaskDelayUntil( &xLastWakeTime, ( pdMS_TO_TICKS(500) ));
    }
}

/* Function that sets up the X carriage end stop and accompanying interrupt
 */
static void endstopConfig(){
    // configure the X endstop as input
    // nothing to do here, default state is input.
    
    // enable pull-up resistors
    X_MIN_PORT |= 1 << X_MIN_PIN;
    
    // The switch is NC, so not pressed = 0 or GND
    // pressing the switch = falling edge
    // releasing the switch = rising edge
    // so we use pin change interrupts
    // X is on INT5 (External INT 5), Y on PCINT 15 (PCI1) and Z on INT3 (External INT 3)
    
    // enable pin change external interupt 5
    EIMSK = 0b00100000;
    EICRB = 0b00000100;
    
    // initialize the struct to the initial values
    X_lim_sw_ch.state = (PINE & (1 << X_MIN_PIN)) == (1 << X_MIN_PIN);
}

ISR(INT5_vect){
    
    // perform processing here to decide if rising or falling edge has
    // just occurred.
    
    // example immediately goes to show that the button is really bouncy,
    // and that hardware debouncing has not been implemented (unfortunately)
    // the bouncyness shouldn't matter too much for our use case.
    // when moving in, any triggering of the interrupt suffices to stop timers.
    // when moving out, the bouncing can be ignored.
    
    uint8_t currentPinState = (PINE & (1 << X_MIN_PIN)) == (1 << X_MIN_PIN);
    
    X_lim_sw_ch.rising_edge = 1;
    X_lim_sw_ch.falling_edge = 1;
    
    if(currentPinState && (currentPinState != X_lim_sw_ch.state)){
        //rising edge
        X_lim_sw_ch.rising_edge = 1;
        X_lim_sw_ch.state = currentPinState; // code duplication, bad!
    }else if(!currentPinState && (currentPinState != X_lim_sw_ch.state)){
        //falling edge
        X_lim_sw_ch.falling_edge = 1;
        X_lim_sw_ch.state = currentPinState;
    }
    // if neither of these two cases holds true, another pin has triggered the interrupt
}

static void configMotors(){
    // initialize the timers, don't start them yet.
    // the extruder motor step port is on portA, pin 4
    DDRA |= (1 << E0_STEP_PIN) | (1 << E0_DIR_PIN) | (1 << E0_EN_PIN);
    // default state should be low, but we'll write the entire port low
    // just in case
    PORTA = PINA & 0b10101011;
    // enable is active low, so the stepper should be enabled now.
    
    // We'll use timer 1 for the extruder motor.
    // the A4988 requires at least a 1 us pulse, we'll use 2 us to be safe.
    // so, we'll config the timer such that 1 tick = 2us, so 500000 kHz
    // 500k is not possible wit the existing prescalers, so we'll use /64 prescaler
    // to get 250kHz. Use CSn bits for this, only when moves need to happen.
    // We'll use Fast PWM mode, set the TOP value with OCRA, and set OCRB to
    // half of OCRA (so 50% dutycycle)
    TCCR1A |= 0b00000011; // WGM11 and WGM10 set to 1
    TCCR1B |= 0b00011000; // WGM13 and WGM 14 set to 1, no clock selected yet.
    
    // OCRA and OCRB don't need to be set yet, but we can enable their
    // interrupts already
    TIMSK1 |= 0x00000110;
    
    return;
}

// initialize the motor, set state to ready
static void initMotor(motor_t* motor){
    motor->motor_state = ready;
}

static void moveMotor(motor_t* motor){
    // setup the timers to move the motor, but only if the motor is ready for a new move.
    // Extruder motor 
    
    // This function should also set the DIR pin right.
    if(motor->motor_state != busy){
        motor->motor_state = busy;
        // when the motor is not busy (it shouldn't be when you call this function)
        // clear the count register
        OCR1A = 250; // todo: link OCR1A to the speed parameter
        OCR1B = 125;
        TCNT1 = 0;
        // start the timer
        TCCR1B |= 0b00000011;// /64 prescaler
    }
    return;
}

ISR(TIMER1_COMPA_vect){
    // compA = counter has counted to top, so we can set out pint to 0
    E0_STEP_PORT &= ~(1 << E0_STEP_PIN);
}

ISR(TIMER1_COMPB_vect){
    // compB = counter has counted halfway, set output to 1
    E0_STEP_PORT |= (1 << E0_STEP_PIN);
    // decrement remaining steps
    E0_motor.remaining_steps -= 1;
    if(E0_motor.remaining_steps == 0){
        // when no steps remain, stop the counter (deselect the clock)
        TCCR1B &= ~0b00000111;
        // set state to no longer busy
        E0_motor.motor_state = ready;
        // Tip: you could also release a semaphore here for synchronization with the task.
    }
}

static void MotorTask(void *pvParameters){
    
    TickType_t xLastWakeTime;
	/* The xLastWakeTime variable needs to be initialized with the current tick
    count. Note that this is the only time the variable is written to explicitly.
    After this xLastWakeTime is automatically updated within vTaskDelayUntil(). */
	xLastWakeTime = xTaskGetTickCount();
    
    configMotors();
    initMotor(&E0_motor);
    
    for(;;){
        if(E0_motor.motor_state == ready){
            E0_motor.remaining_steps = 3200; // 3200 steps = one rotation
            moveMotor(&E0_motor);
            vTaskDelayUntil( &xLastWakeTime, ( pdMS_TO_TICKS(2000)));
            
        }else{
            vTaskDelay(pdMS_TO_TICKS(100));
        }   
    }
}



#include "motorController.h"
#include "RAMPS_macros.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include "FreeRTOS.h"
#include "task.h"


#include "serial.h"
extern xComPortHandle xSerialPort;

// macro to convert mm to microsteps
#define mm_to_microsteps(mm, mm_p_ms) (uint32_t)((mm)/(mm_p_ms))
// macro to convert mm/s to OCRA value (= period of pulse)
#define min(a, b)  (((a) < (b)) ? (a) : (b)) 
// 62500 is F_CPU/
#define speed_to_ocval(mm_p_s, mm_p_ms) min((uint16_t)(62500/((mm_p_s)/(mm_p_ms))), 65535)

/* 3 motors */
volatile motor_t x_motor = {initMotor, x, 0, 0, 0, X_MM_PER_MICROSTEP};
volatile motor_t y_motor = {initMotor, y, 0, 0, 0, Y_MM_PER_MICROSTEP};
volatile motor_t z_motor = {initMotor, z, 0, 0, 0, Z_MM_PER_MICROSTEP};
volatile motor_t e_motor = {initMotor, e, 0, 0, 0, E_MM_PER_MICROSTEP};

/* head position */
head_position_t head_position = {-1, -1, -1}; // -1 = not homed yet,

/* Function to init the hardware for all the motors */
void initMotors(){
    // Set endstop pins to input with pullup    
    X_MIN_PORT |= 1 << X_MIN_PIN; // pullup on
    Y_MIN_PORT |= 1 << Y_MIN_PIN; // pullup on
    Z_MIN_PORT |= 1 << Z_MIN_PIN; // pullup on
    
    xSerialPrint("init done\r\n");
    
    // Set STEP and DIR and EN pins to output, write both low
    DDRF |= 1 << X_STEP_PIN | 1 << X_DIR_PIN | 1 << Y_STEP_PIN | 1 << Y_DIR_PIN | 1 << Y_EN_PIN ;
    DDRD |= 1 << X_EN_PIN;
    DDRL |= 1 << Z_STEP_PIN | 1 << Z_DIR_PIN ;
    DDRA |= 1 << E0_STEP_PIN | 1 << E0_DIR_PIN | 1 << E0_EN_PIN ;
    DDRK |= 1 << Z_EN_PIN;
    
    PORTF &= ~(1 << X_STEP_PIN | 1 << X_DIR_PIN | 1 << Y_STEP_PIN | 1 << Y_DIR_PIN | 1 << Y_EN_PIN);
    PORTD &= ~(1 << X_EN_PIN);
    PORTL &= ~(1 << Z_STEP_PIN | 1 << Z_DIR_PIN);
    PORTA &= ~(1 << E0_STEP_PIN | 1 << E0_DIR_PIN | 1 << E0_EN_PIN);
    PORTK &= ~(1 << Z_EN_PIN);
    
    // Setup the 4 16 bit timers
    /* Setup choices for the timers described here:
     * - A4988 driver needs pulses of at least 2 us (1us high, 1us low)
     * - BUT: a bit of experimental validation shows that for the extruder you need about at least 32us pulses (half period) or 64us full period
     * - We aim to use Fast PWM, with overflow at OCA and 50%Dutycycle OCB
     * - The minimum value of OCA is 0x03, which means 4 timesteps (timer is cleared in next cycle),
     * - OCB is in our case always half of OCA
     * - to get the most from our timer, we therefore would like 0x03 to be approx 64us
     *   that would mean 16us for 1 timertick or 62500 Mhz timer frequency, which we can get with a /256 prescaler
     * - All weighted axis and extruder with filament will probably need to go slower or they'll skip steps,
     * - but we can use this for estimate speed calculations
     * 
     * Timer associations: Z axis is the only one with hardware OC pin available
     * use that one for Z axis (TC5) in case you ever want to enable the hardware output
     * remaining timers are assigned as follows: E:TC1, X:TC3, Y:TC4
     */
    TCCR1A = 0b00000011;// WGM11 and WGM10 set
    TCCR1B = 0b00011000;// WGM13 and WGM14 set
    TCCR3A = 0b00000011;
    TCCR3B = 0b00011000;
    TCCR4A = 0b00000011;
    TCCR4B = 0b00011000;
    TCCR5A = 0b00000011;
    TCCR5B = 0b00011000;
    TIMSK1 = 0b00000110;// enable COMPA and COMPB interrupts
    TIMSK3 = 0b00000110;
    TIMSK4 = 0b00000110;
    TIMSK5 = 0b00000110;
    // don't start the timers yet, or enable their interrupts yet
}

/* Function to home a motor*/
void homeMotors(){
    // Read MIN pins, those that are pushed in don't need homing
    // switches are NC, so read 0 when not pushed
    uint8_t home_x = (PINE & (1 << X_MIN_PIN)) == 0;
    uint8_t home_y = (PINJ & (1 << Y_MIN_PIN)) == 0;
    uint8_t home_z = (PIND & (1 << Z_MIN_PIN)) == 0;
    
    // Homing order: X, Y, Z, do this sequentially.
    //, Y on PCINT 15 (PCI1) and Z on INT3 (External INT 3)
    if(home_x){
//        xSerialPrint("homing X\r\n");
        // set state to homing.
        x_motor.motor_state = homing;
//        // enable pin change interrupt for the endstop switch.
//        // X is on INT5 (External INT 5)
//        //EIFR |= 1 << INTF5;// clear any pending interrupts.
//        EICRB |= 0b00000100;
//        EIMSK |= 0b00100000;
        /* The pin change interrupt was continuously triggering on it's own.
         * dont't know if its due to interference or whatever, but it wasn't
         * reliable, therefore, a simpler polling routing making use of vTaskDelay
         * is implemented instead.
         */
        // Call move function with homing speed.
//        xSerialPrint("movingMotor X\r\n");
        moveMotor_mm(&x_motor, 0, HOME_SPEED_FAST, 0);
        while((PINE & (1 << X_MIN_PIN)) == 0){
            //TODO fault fallback, what if the motor doesn't home in time
            vTaskDelay(pdMS_TO_TICKS(50)); // wait 50 ms
        }
        TCCR3B &= 0b11111000; // clear the clock selection bits.
        TIFR3 |= 0b00000110; // clear all potential interrupts
        // Motor is now approximately homed, move away 1 cm, then home again at slower speed
        x_motor.motor_state = ready;
        moveMotor_mm(&x_motor, 10, HOME_SPEED_FAST, 1);
        while(x_motor.motor_state != ready){
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        x_motor.motor_state = homing; // set state to homing again
        moveMotor_mm(&x_motor, 0, HOME_SPEED_SLOW, 0);
        while((PINE & (1 << X_MIN_PIN)) == 0){
            //TODO fault fallback, what if the motor doesn't home in time
            vTaskDelay(pdMS_TO_TICKS(50)); // wait 50 ms
        }
        TCCR3B &= 0b11111000; // clear the clock selection bits.
        TIFR3 |= 0b00000110; // clear all potential interrupts
        // Motor is now adequately homed.
    }
    if(home_y){
//        xSerialPrint("homing Y\r\n");
        // set state to homing.
        y_motor.motor_state = homing;
        // Call move function with homing speed.
//        xSerialPrint("movingMotor Y\r\n");
        moveMotor_mm(&y_motor, 0, HOME_SPEED_FAST, 0);
        while((PINJ & (1 << Y_MIN_PIN)) == 0){
            //TODO fault fallback, what if the motor doesn't home in time
            vTaskDelay(pdMS_TO_TICKS(50)); // wait 50 ms
        }
        TCCR4B &= 0b11111000; // clear the clock selection bits.
        TIFR4 |= 0b00000110; // clear all potential interrupts
        // Motor is now approximately homed, move away 1 cm, then home again at slower speed
        y_motor.motor_state = ready;
        moveMotor_mm(&y_motor, 10, HOME_SPEED_FAST, 1);
        while(y_motor.motor_state != ready){
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        y_motor.motor_state = homing; // set state to homing again
        moveMotor_mm(&y_motor, 0, HOME_SPEED_SLOW, 0);
        while((PINJ & (1 << Y_MIN_PIN)) == 0){
            //TODO fault fallback, what if the motor doesn't home in time
            vTaskDelay(pdMS_TO_TICKS(50)); // wait 50 ms
        }
        TCCR4B &= 0b11111000; // clear the clock selection bits.
        TIFR4 |= 0b00000110; // clear all potential interrupts
        // Motor is now adequately homed.
    }
    if(home_z){
//        xSerialPrint("homing Z\r\n");
        // set state to homing.
        z_motor.motor_state = homing;
        // Call move function with homing speed.
//        xSerialPrint("movingMotor Z\r\n");
        moveMotor_mm(&z_motor, 0, HOME_SPEED_FAST_Z, 0);
        while((PIND & (1 << Z_MIN_PIN)) == 0){
            //TODO fault fallback, what if the motor doesn't home in time
            vTaskDelay(pdMS_TO_TICKS(50)); // wait 50 ms
        }
        TCCR5B &= 0b11111000; // clear the clock selection bits.
        TIFR5 |= 0b00000110; // clear all potential interrupts
        // Motor is now approximately homed, move away 1 cm, then home again at slower speed
        z_motor.motor_state = ready;
        moveMotor_mm(&z_motor, 10, HOME_SPEED_FAST_Z, 1);
        while(z_motor.motor_state != ready){
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        z_motor.motor_state = homing; // set state to homing again
        moveMotor_mm(&z_motor, 0, HOME_SPEED_SLOW_Z, 0);
        while((PIND & (1 << Z_MIN_PIN)) == 0){
            //TODO fault fallback, what if the motor doesn't home in time
            vTaskDelay(pdMS_TO_TICKS(50)); // wait 50 ms
        }
        TCCR5B &= 0b11111000; // clear the clock selection bits.
        TIFR5 |= 0b00000110; // clear all potential interrupts
        // Motor is now adequately homed.
        
    }
    
    // all motors should be ready now
    x_motor.motor_state = ready;
    y_motor.motor_state = ready;
    z_motor.motor_state = ready;
    // all axes should now be homed or were already homed
    head_position.x_pos = 0;
    head_position.y_pos = 0;
    head_position.z_pos = 0;
    xSerialPrint("homing done\r\n");
    
    // Interrupts turn themselves off in the ISR, so no need to worry about those.
    return;
}

/* Function to move a motor certain number of mm, speed in mm/s */
void moveMotor_mm(volatile motor_t* motor, float mm, float speed, uint8_t dir){
    uint16_t compare_val = 0;
    uint32_t microsteps = 0;
    // depending on axis the compare val for speed and number of microsteps differs.
    if(motor->motor_state == busy){
        //if motor is somehow already busy return with error (TODO)
        return;
    } else {
        switch(motor->motor_axis){
            case x:
                compare_val = speed_to_ocval(speed, X_MM_PER_MICROSTEP);
                microsteps = mm_to_microsteps(mm, X_MM_PER_MICROSTEP);
//                xSerialPrintf("Compare val %u\r\n", compare_val);
//                xSerialPrintf("Num msteps %u\r\n", microsteps);
                break;
            case y:
                compare_val = speed_to_ocval(speed, Y_MM_PER_MICROSTEP);
                microsteps = mm_to_microsteps(mm, Y_MM_PER_MICROSTEP);
                break;
            case z:
                compare_val = speed_to_ocval(speed, Z_MM_PER_MICROSTEP);
                microsteps = mm_to_microsteps(mm, Z_MM_PER_MICROSTEP);
                break;
            default:
                return;
                break;
        }
        moveMotor_microsteps(motor, compare_val, microsteps, dir);
    } // motor considered busy from now on.)
}

/* Function to move a motor certain number of microsteps */
void moveMotor_microsteps(volatile motor_t* motor, uint16_t output_compare_val, uint32_t microsteps_to_move, uint8_t dir){
    // depending on the motor axis we must init a different timer
    motor->motor_state = busy; // motor considered busy from now on.
    motor->remaining_steps = microsteps_to_move;
    switch(motor->motor_axis){
        case x:
            { // ; works too but scopes wrongly C quirk, see https://stackoverflow.com/questions/46341364/label-can-only-be-used-as-part-of-a-statement-error
            
//            xSerialPrint("rotating_x_axis\r\n");
            
            // determine if we need to invert the DIR (0 => 1 and 1 => 0)
            uint8_t dir_final = (X_INVERT_ROT == 1) ? !dir : dir;
            // write dir pin, when 1, use OR write, when 0, clear.
            X_DIR_PORT = dir_final? X_DIR_PORT | 1 << X_DIR_PIN : X_DIR_PORT & ~(1 << X_DIR_PIN);
            // dir pin is written, remaining steps are set, moror set to busy, so one last thing to do:
            // set the output compare values and start the timer.

            OCR3A = output_compare_val;
            OCR3B = output_compare_val/2;
//            xSerialPrintf("ocr3A %u\r\n", OCR3A);
//            xSerialPrintf("ocr3B %u\r\n", OCR3B);
            TCNT3 = 0; //reset timer
            TCCR3B |= 0b00000100;
        }
            break;
        case y:
        {
            
//            xSerialPrint("rotating_y_axis\r\n");
            
            // determine if we need to invert the DIR (0 => 1 and 1 => 0)
            uint8_t dir_final = (Y_INVERT_ROT == 1) ? !dir : dir;
            // write dir pin, when 1, use OR write, when 0, clear.
            Y_DIR_PORT = dir_final? Y_DIR_PORT | 1 << Y_DIR_PIN : Y_DIR_PORT & ~(1 << Y_DIR_PIN);
            // dir pin is written, remaining steps are set, moror set to busy, so one last thing to do:
            // set the output compare values and start the timer.

            OCR4A = output_compare_val;
            OCR4B = output_compare_val/2;
//            xSerialPrintf("ocr4A %u\r\n", OCR4A);
//            xSerialPrintf("ocr4B %u\r\n", OCR4B);
            TCNT4 = 0;
            TCCR4B |= 0b00000100;
        }
            break;
        case z:
        {
            
//            xSerialPrint("rotating_z_axis\r\n");
            
            // determine if we need to invert the DIR (0 => 1 and 1 => 0)
            uint8_t dir_final = (Z_INVERT_ROT == 1) ? !dir : dir;
            // write dir pin, when 1, use OR write, when 0, clear.
            Z_DIR_PORT = dir_final? Z_DIR_PORT | 1 << Z_DIR_PIN : Z_DIR_PORT & ~(1 << Z_DIR_PIN);
            // dir pin is written, remaining steps are set, moror set to busy, so one last thing to do:
            // set the output compare values and start the timer.

            OCR5A = output_compare_val;
            OCR5B = output_compare_val/2;
//            xSerialPrintf("ocr5A %u\r\n", OCR5A);
//            xSerialPrintf("ocr5B %u\r\n", OCR5B);
            TCNT5 = 0;
            TCCR5B |= 0b00000100;
        }
            break;
        default:
            return;
            break;
    }
    return;
}

/* Function to extrude X mm of filament*/
void extrude_mm_filament(float mm, uint16_t output_compare_val){
    return;
}

/* Function to extrude filament over X amount of travel at X layer thickness*/
void extrude_mm(float mm_travel, float move_speed, float layer_height){
    return;
}

ISR(TIMER1_COMPA_vect){
    
}

ISR(TIMER1_COMPB_vect){
    
}

ISR(TIMER3_COMPA_vect){
    // COMPA  = counter has counted to top, so set out pin to 0
    X_STEP_PORT &= ~(1 << X_STEP_PIN);
    
    if(x_motor.motor_state != homing){
        // whave completed entire step, when not homing, decrement steps
        x_motor.remaining_steps -= 1;
        // if remaining_steps is 0, stop the counter, set state back to ready
        if(x_motor.remaining_steps == 0){
            TCCR3B &= ~0b00000111;
            x_motor.motor_state = ready;
        }
    }
}

ISR(TIMER3_COMPB_vect){
    // COMPB = counter has counted halfway, set output to 1 (should start at 0 and end at 0)
    X_STEP_PORT |= (1 << X_STEP_PIN);    
}

ISR(TIMER4_COMPA_vect){
        // COMPA  = counter has counted to top, so set out pin to 0
    Y_STEP_PORT &= ~(1 << Y_STEP_PIN);
    
    if(y_motor.motor_state != homing){
        // whave completed entire step, when not homing, decrement steps
        y_motor.remaining_steps -= 1;
        // if remaining_steps is 0, stop the counter, set state back to ready
        if(y_motor.remaining_steps == 0){
            TCCR4B &= ~0b00000111;
            y_motor.motor_state = ready;
        }
    }
}

ISR(TIMER4_COMPB_vect){
    // COMPB = counter has counted halfway, set output to 1 (should start at 0 and end at 0)
    Y_STEP_PORT |= (1 << Y_STEP_PIN); 
}

ISR(TIMER5_COMPA_vect){
    // COMPA  = counter has counted to top, so set out pin to 0
    Z_STEP_PORT &= ~(1 << Z_STEP_PIN);
    
    if(z_motor.motor_state != homing){
        // whave completed entire step, when not homing, decrement steps
        z_motor.remaining_steps -= 1;
        // if remaining_steps is 0, stop the counter, set state back to ready
        if(z_motor.remaining_steps == 0){
            TCCR5B &= ~0b00000111;
            z_motor.motor_state = ready;
        }
    }
}

ISR(TIMER5_COMPB_vect){
    // COMPB = counter has counted halfway, set output to 1 (should start at 0 and end at 0)
    Z_STEP_PORT |= (1 << Z_STEP_PIN); 
}

// TODO: keep interrupt enabled at all times, as failsafe for an illegal move action.
//ISR(INT5_vect){
//    xSerialPrint("btnisr\r\n");
//    // X carriage hit the endstop.
//    // Stop timer of the X motor and clear any pending interrupts
//    TCCR3B &= 0b11111000; // clear the clock selection bits.
//    TIFR3 |= 0b00000110; // clear all potential interrupts
//    // also set STEP output to 0 for correct start the next time
//    X_STEP_PORT &= ~(1 << X_STEP_PIN);
//    // Disable own interrupt
//    EIMSK &= ~0b00100000;
//    EICRB &= ~0b00000100;
//    // set state of X motor to ready
//    x_motor.motor_state = ready;
//}

ISR(INT3_vect){
    
}

ISR(PCINT1_vect){
    
}

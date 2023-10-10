/* 
 * File:   RAMPS_macros.h
 * Author: Joost
 *
 * Created on October 5, 2020, 10:01 AM
 */

#ifndef RAMPS_MACROS_H
#define	RAMPS_MACROS_H

#include <avr/io.h>

#ifdef	__cplusplus
extern "C" {
#endif
    
// Motor pin and ports (motor drivers + endstops)
// Note that the RAMPS schematic shows mapping to Arduino Pins, not Ports!
// to translate, I used: https://www.arduino.cc/en/Hacking/PinMapping2560

// X direction motor
// TODO: also add macro for PINxn register
    
#define X_STEP_PORT PORTF
#define X_STEP_PIN 0
#define X_DIR_PORT PORTF
#define X_DIR_PIN 1
#define X_MIN_PORT PORTE
#define X_MIN_PIN 5
#define X_EN_PORT PORTD
#define X_EN_PIN 7

// Y direction motor    
#define Y_STEP_PORT PORTF
#define Y_STEP_PIN 6
#define Y_DIR_PORT PORTF
#define Y_DIR_PIN 7
#define Y_MIN_PORT PORTJ
#define Y_MIN_PIN 1
#define Y_EN_PORT PORTF
#define Y_EN_PIN 2

// Sidenote: only  the Z motor actually has a timer
// associated with it's hardware pin (TC5, OCA)
// all other timers will need software mapping from the interrupt to the pin
// makes little difference.
// Z direction motor
#define Z_STEP_PORT PORTL
#define Z_STEP_PIN 3
#define Z_DIR_PORT PORTL
#define Z_DIR_PIN 1
#define Z_MIN_PORT PORTD
#define Z_MIN_PIN 3
#define Z_EN_PORT PORTK
#define Z_EN_PIN 0

// Extruder motor    
#define E0_STEP_PORT PORTA
#define E0_STEP_PIN 4
#define E0_DIR_PORT PORTA
#define E0_DIR_PIN 6
#define E0_EN_PORT PORTA
#define E0_EN_PIN 2
    
// Thermistor ports and pins
    
#define THERM_PORT PORTK
#define THERM0_PIN 5
// #define THERM1_PIN 6 // uncomment for secondary thermistor
// #define THERM2_PIN 7 // uncomment for tertiary thermistor
    
// Heater and fan ports and pins
    
#define HOT_END_PORT PORTH
#define HOT_END_PIN 5
//#define HOT_BED_PORT PORTH
//#define HOT_BED_PIN 4
//#define FANS_PORT PORTB
//#define FANS_PIN 6
    
#define MOTORSTEPS 200
#define MICROSTEPS 16
#define STEPS_PER_REV (MOTORSTEPS * MICROSTEPS)

#ifdef	__cplusplus
}
#endif

#endif	/* RAMPS_MACROS_H */


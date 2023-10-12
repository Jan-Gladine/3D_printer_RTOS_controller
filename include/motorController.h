/* 
 * File:   motorController.h
 * Author: Joost
 *
 * Created on October 7, 2020, 3:29 PM
 */

#include <avr/io.h>

#ifndef MOTORCONTROLLER_H
#define	MOTORCONTROLLER_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#define MOTOR_STEPS_PER_REV 200 // 200 steps per revolution
#define MICROSTEPS 16 // board hw configed for 16 microsteps
#define PI 3.141592653589793238463f
#define NOZZLE_DIAMETER 0.4f //mm
#define FILAMENT_DIAMETER 1.75f //mm
#define HOME_SPEED_FAST 8 // mm/s
#define HOME_SPEED_SLOW 3
#define HOME_SPEED_FAST_Z 3 // Z axis doesn't need to go as fast as the other axis.
#define HOME_SPEED_SLOW_Z 2
    
// Default assumption is move down/left/back = 0, up/right/front = 1
// if this is not the case, set the invert here to 1.
#define X_INVERT_ROT 1
#define Y_INVERT_ROT 1
#define Z_INVERT_ROT 0
    
#define E_PULLEY_DIAMETER 11.0f //mm
#define X_PULLEY_DIAMETER 12.0f //mm
#define Y_PULLEY_DIAMETER 12.0f //mm
// Z axis isn't with pulley but lead screw.
    
#define E_MM_PER_STEP (E_PULLEY_DIAMETER*PI/200.0f) //mm
#define X_MM_PER_STEP (X_PULLEY_DIAMETER*PI/200.0f) //mm
#define Y_MM_PER_STEP (Y_PULLEY_DIAMETER*PI/200.0f) //mm
#define Z_MM_PER_STEP 0.04f //mm Looked this up for the lead screw
    
#define E_MM_PER_MICROSTEP E_MM_PER_STEP/MICROSTEPS
#define X_MM_PER_MICROSTEP X_MM_PER_STEP/MICROSTEPS
#define Y_MM_PER_MICROSTEP Y_MM_PER_STEP/MICROSTEPS
#define Z_MM_PER_MICROSTEP Z_MM_PER_STEP/MICROSTEPS
    
/* store button events in a struct */
typedef struct {
    uint8_t state : 1;
    uint8_t rising_edge : 1;
    uint8_t falling_edge : 1;
}button_changes_t;

/* motor state enum */
typedef enum {
    initMotor,
    homing,
    ready,
    busy
} motor_state_t;

/* axis identifier for the motors */
typedef enum{
    x,
    y,
    z,
    e
} axis_id_t;

/* head position type to position head in XYZ coordinates*/
typedef struct{
    float x_pos;
    float y_pos;
    float z_pos;
} head_position_t;

/* motor struct */
typedef struct {
    motor_state_t motor_state;
    axis_id_t motor_axis;
    uint32_t remaining_steps;
    int32_t speed; // in steps per second
    uint8_t dir; // 0 for CW, 1 for CCW
    const float mm_per_microstep;
}motor_t;

/* 3 motors */
extern volatile motor_t x_motor;
extern volatile motor_t y_motor;
extern volatile motor_t z_motor;
extern volatile motor_t e_motor;

/* head position */
extern head_position_t head_position; // -1 = not homed yet,

/* Function to init the hardware for all the motors */
void initMotors();

/* Function to home all motors */
void homeMotors();

/* Function to move a motor certain number of mm */
void moveMotor_mm(volatile motor_t* motor, float mm, float speed, uint8_t dir);

/* Function to move a motor certain number of microsteps */
void moveMotor_microsteps(volatile motor_t* motor, uint16_t output_compare_val, uint32_t microsteps_to_move, uint8_t dir);

/* Function to extrude X mm of filament*/
void extrude_mm_filament(float mm, uint16_t output_compare_val);

/* Function to extrude filament over X amount of travel at X layer thickness and speed*/
void extrude_mm(float mm_travel, float move_speed, float layer_height);


#ifdef	__cplusplus
}
#endif

#endif	/* MOTORCONTROLLER_H */


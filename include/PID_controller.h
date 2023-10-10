/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: PID_controller.h
 *
 * Code generated for Simulink model 'PID_controller'.
 *
 * Model version                  : 1.7
 * Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
 * C/C++ source code generated on : Tue Oct 10 16:50:51 2023
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Atmel->AVR (8-bit)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_PID_controller_h_
#define RTW_HEADER_PID_controller_h_
#ifndef PID_controller_COMMON_INCLUDES_
#define PID_controller_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* PID_controller_COMMON_INCLUDES_ */

#include "PID_controller_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T Integrator_DSTATE;            /* '<S37>/Integrator' */
  real_T UD_DSTATE;                    /* '<S30>/UD' */
} DW_PID_controller_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T In1;                          /* '<Root>/In1' */
  real_T In2;                          /* '<Root>/In2' */
} ExtU_PID_controller_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T Out1;                         /* '<Root>/Out1' */
} ExtY_PID_controller_T;

/* Real-time Model Data Structure */
struct tag_RTM_PID_controller_T {
  const char_T * volatile errorStatus;
};

/* Block states (default storage) */
extern DW_PID_controller_T PID_controller_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_PID_controller_T PID_controller_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_PID_controller_T PID_controller_Y;

/* Model entry point functions */
extern void PID_controller_initialize(void);
extern void PID_controller_step(void);
extern void PID_controller_terminate(void);

/* Real-time Model object */
extern RT_MODEL_PID_controller_T *const PID_controller_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S30>/DTDup' : Unused code path elimination
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'PID_controller'
 * '<S1>'   : 'PID_controller/Discrete PID Controller'
 * '<S2>'   : 'PID_controller/Discrete PID Controller/Anti-windup'
 * '<S3>'   : 'PID_controller/Discrete PID Controller/D Gain'
 * '<S4>'   : 'PID_controller/Discrete PID Controller/Filter'
 * '<S5>'   : 'PID_controller/Discrete PID Controller/Filter ICs'
 * '<S6>'   : 'PID_controller/Discrete PID Controller/I Gain'
 * '<S7>'   : 'PID_controller/Discrete PID Controller/Ideal P Gain'
 * '<S8>'   : 'PID_controller/Discrete PID Controller/Ideal P Gain Fdbk'
 * '<S9>'   : 'PID_controller/Discrete PID Controller/Integrator'
 * '<S10>'  : 'PID_controller/Discrete PID Controller/Integrator ICs'
 * '<S11>'  : 'PID_controller/Discrete PID Controller/N Copy'
 * '<S12>'  : 'PID_controller/Discrete PID Controller/N Gain'
 * '<S13>'  : 'PID_controller/Discrete PID Controller/P Copy'
 * '<S14>'  : 'PID_controller/Discrete PID Controller/Parallel P Gain'
 * '<S15>'  : 'PID_controller/Discrete PID Controller/Reset Signal'
 * '<S16>'  : 'PID_controller/Discrete PID Controller/Saturation'
 * '<S17>'  : 'PID_controller/Discrete PID Controller/Saturation Fdbk'
 * '<S18>'  : 'PID_controller/Discrete PID Controller/Sum'
 * '<S19>'  : 'PID_controller/Discrete PID Controller/Sum Fdbk'
 * '<S20>'  : 'PID_controller/Discrete PID Controller/Tracking Mode'
 * '<S21>'  : 'PID_controller/Discrete PID Controller/Tracking Mode Sum'
 * '<S22>'  : 'PID_controller/Discrete PID Controller/Tsamp - Integral'
 * '<S23>'  : 'PID_controller/Discrete PID Controller/Tsamp - Ngain'
 * '<S24>'  : 'PID_controller/Discrete PID Controller/postSat Signal'
 * '<S25>'  : 'PID_controller/Discrete PID Controller/preSat Signal'
 * '<S26>'  : 'PID_controller/Discrete PID Controller/Anti-windup/Disc. Clamping Parallel'
 * '<S27>'  : 'PID_controller/Discrete PID Controller/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S28>'  : 'PID_controller/Discrete PID Controller/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
 * '<S29>'  : 'PID_controller/Discrete PID Controller/D Gain/Internal Parameters'
 * '<S30>'  : 'PID_controller/Discrete PID Controller/Filter/Differentiator'
 * '<S31>'  : 'PID_controller/Discrete PID Controller/Filter/Differentiator/Tsamp'
 * '<S32>'  : 'PID_controller/Discrete PID Controller/Filter/Differentiator/Tsamp/Internal Ts'
 * '<S33>'  : 'PID_controller/Discrete PID Controller/Filter ICs/Internal IC - Differentiator'
 * '<S34>'  : 'PID_controller/Discrete PID Controller/I Gain/Internal Parameters'
 * '<S35>'  : 'PID_controller/Discrete PID Controller/Ideal P Gain/Passthrough'
 * '<S36>'  : 'PID_controller/Discrete PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S37>'  : 'PID_controller/Discrete PID Controller/Integrator/Discrete'
 * '<S38>'  : 'PID_controller/Discrete PID Controller/Integrator ICs/Internal IC'
 * '<S39>'  : 'PID_controller/Discrete PID Controller/N Copy/Disabled wSignal Specification'
 * '<S40>'  : 'PID_controller/Discrete PID Controller/N Gain/Passthrough'
 * '<S41>'  : 'PID_controller/Discrete PID Controller/P Copy/Disabled'
 * '<S42>'  : 'PID_controller/Discrete PID Controller/Parallel P Gain/Internal Parameters'
 * '<S43>'  : 'PID_controller/Discrete PID Controller/Reset Signal/Disabled'
 * '<S44>'  : 'PID_controller/Discrete PID Controller/Saturation/Enabled'
 * '<S45>'  : 'PID_controller/Discrete PID Controller/Saturation Fdbk/Disabled'
 * '<S46>'  : 'PID_controller/Discrete PID Controller/Sum/Sum_PID'
 * '<S47>'  : 'PID_controller/Discrete PID Controller/Sum Fdbk/Disabled'
 * '<S48>'  : 'PID_controller/Discrete PID Controller/Tracking Mode/Disabled'
 * '<S49>'  : 'PID_controller/Discrete PID Controller/Tracking Mode Sum/Passthrough'
 * '<S50>'  : 'PID_controller/Discrete PID Controller/Tsamp - Integral/TsSignalSpecification'
 * '<S51>'  : 'PID_controller/Discrete PID Controller/Tsamp - Ngain/Passthrough'
 * '<S52>'  : 'PID_controller/Discrete PID Controller/postSat Signal/Forward_Path'
 * '<S53>'  : 'PID_controller/Discrete PID Controller/preSat Signal/Forward_Path'
 */
#endif                                 /* RTW_HEADER_PID_controller_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

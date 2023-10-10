/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: PID_controller.c
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

#include "PID_controller.h"
#include "rtwtypes.h"

/* Block states (default storage) */
DW_PID_controller_T PID_controller_DW;

/* External inputs (root inport signals with default storage) */
ExtU_PID_controller_T PID_controller_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_PID_controller_T PID_controller_Y;

/* Real-time model */
static RT_MODEL_PID_controller_T PID_controller_M_;
RT_MODEL_PID_controller_T *const PID_controller_M = &PID_controller_M_;

/* Model step function */
void PID_controller_step(void)
{
  real_T rtb_DeadZone;
  real_T rtb_IntegralGain;
  real_T rtb_Tsamp;
  int8_T tmp;
  int8_T tmp_0;

  /* Sum: '<Root>/Subtract' incorporates:
   *  Inport: '<Root>/In1'
   *  Inport: '<Root>/In2'
   */
  rtb_IntegralGain = PID_controller_U.In1 - PID_controller_U.In2;

  /* SampleTimeMath: '<S32>/Tsamp' incorporates:
   *  Gain: '<S29>/Derivative Gain'
   *
   * About '<S32>/Tsamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  rtb_Tsamp = 3.8 * rtb_IntegralGain * 10.0;

  /* Sum: '<S46>/Sum' incorporates:
   *  Delay: '<S30>/UD'
   *  DiscreteIntegrator: '<S37>/Integrator'
   *  Gain: '<S42>/Proportional Gain'
   *  Sum: '<S30>/Diff'
   */
  rtb_DeadZone = (0.6984 * rtb_IntegralGain +
                  PID_controller_DW.Integrator_DSTATE) + (rtb_Tsamp -
    PID_controller_DW.UD_DSTATE);

  /* Saturate: '<S44>/Saturation' incorporates:
   *  DeadZone: '<S28>/DeadZone'
   */
  if (rtb_DeadZone > 40.0) {
    /* Outport: '<Root>/Out1' */
    PID_controller_Y.Out1 = 40.0;
    rtb_DeadZone -= 40.0;
  } else {
    if (rtb_DeadZone < 0.0) {
      /* Outport: '<Root>/Out1' */
      PID_controller_Y.Out1 = 0.0;
    } else {
      /* Outport: '<Root>/Out1' */
      PID_controller_Y.Out1 = rtb_DeadZone;
    }

    if (rtb_DeadZone >= 0.0) {
      rtb_DeadZone = 0.0;
    }
  }

  /* End of Saturate: '<S44>/Saturation' */

  /* Gain: '<S34>/Integral Gain' */
  rtb_IntegralGain *= 0.01354;

  /* Switch: '<S26>/Switch1' incorporates:
   *  Constant: '<S26>/Clamping_zero'
   *  Constant: '<S26>/Constant'
   *  Constant: '<S26>/Constant2'
   *  RelationalOperator: '<S26>/fix for DT propagation issue'
   */
  if (rtb_DeadZone > 0.0) {
    tmp = 1;
  } else {
    tmp = -1;
  }

  /* Switch: '<S26>/Switch2' incorporates:
   *  Constant: '<S26>/Clamping_zero'
   *  Constant: '<S26>/Constant3'
   *  Constant: '<S26>/Constant4'
   *  RelationalOperator: '<S26>/fix for DT propagation issue1'
   */
  if (rtb_IntegralGain > 0.0) {
    tmp_0 = 1;
  } else {
    tmp_0 = -1;
  }

  /* Switch: '<S26>/Switch' incorporates:
   *  Constant: '<S26>/Clamping_zero'
   *  Constant: '<S26>/Constant1'
   *  Logic: '<S26>/AND3'
   *  RelationalOperator: '<S26>/Equal1'
   *  RelationalOperator: '<S26>/Relational Operator'
   *  Switch: '<S26>/Switch1'
   *  Switch: '<S26>/Switch2'
   */
  if ((rtb_DeadZone != 0.0) && (tmp == tmp_0)) {
    rtb_IntegralGain = 0.0;
  }

  /* Update for DiscreteIntegrator: '<S37>/Integrator' incorporates:
   *  Switch: '<S26>/Switch'
   */
  PID_controller_DW.Integrator_DSTATE += 0.1 * rtb_IntegralGain;

  /* Update for Delay: '<S30>/UD' */
  PID_controller_DW.UD_DSTATE = rtb_Tsamp;
}

/* Model initialize function */
void PID_controller_initialize(void)
{
  /* (no initialization code required) */
}

/* Model terminate function */
void PID_controller_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

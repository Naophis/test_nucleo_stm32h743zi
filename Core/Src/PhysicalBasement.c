/*
 * Sponsored License - for use in support of a program or activity
 * sponsored by MathWorks.  Not for government, commercial or other
 * non-sponsored organizational use.
 *
 * File: PhysicalBasement.c
 *
 * Code generated for Simulink model 'PhysicalBasement'.
 *
 * Model version                  : 1.8
 * Simulink Coder version         : 9.4 (R2020b) 29-Jul-2020
 * C/C++ source code generated on : Sun Oct  4 22:19:30 2020
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "PhysicalBasement.h"
#include "PhysicalBasement_private.h"

const t_SensorData PhysicalBasement_rtZt_SensorDat = { { 0,/* right45 */
    0,                                 /* right90 */
    0,                                 /* left45 */
    0,                                 /* left90 */
    0                                  /* front */
  },                                   /* light_sensor */
  { { 0, 0, 0, 0, 0 },                 /* gyro_raw_data */
    { 0, 0, 0, 0, 0 }                  /* gyro_data */
  },                                   /* gryo */
  { 0,                                 /* right */
    0                                  /* left */
  },                                   /* encorder */
  0.0,                                 /* ego_angle */
  0.0                                  /* ego_velocity */
};

/* Model step function */
void PhysicalBasement_step(RT_MODEL_PhysicalBasement_T *const PhysicalBasement_M,
  t_SensorData *PhysicalBasement_U_In1, real_T *PhysicalBasement_Y_Out1, real_T *
  PhysicalBasement_Y_Out2)
{
  DW_PhysicalBasement_T *PhysicalBasement_DW = PhysicalBasement_M->dwork;
  real_T rtb_Gain;

  /* Gain: '<Root>/Gain' incorporates:
   *  DataTypeConversion: '<Root>/Data Type Conversion'
   *  Inport: '<Root>/In1'
   */
  rtb_Gain = PhysicalBasement_P.Gain_Gain * (real_T)
    PhysicalBasement_U_In1->light_sensor.right45;

  /* Outport: '<Root>/Out1' incorporates:
   *  Sum: '<Root>/Add1'
   *  UnitDelay: '<Root>/Unit Delay'
   */
  *PhysicalBasement_Y_Out1 = rtb_Gain + PhysicalBasement_DW->UnitDelay_DSTATE;

  /* Outport: '<Root>/Out2' incorporates:
   *  DataTypeConversion: '<Root>/Data Type Conversion1'
   *  DataTypeConversion: '<Root>/Data Type Conversion2'
   *  Gain: '<Root>/Gain1'
   *  Inport: '<Root>/In1'
   *  Sum: '<Root>/Add'
   */
  *PhysicalBasement_Y_Out2 = ((real_T)PhysicalBasement_U_In1->encorder.right +
    (real_T)PhysicalBasement_U_In1->encorder.left) *
    PhysicalBasement_P.Gain1_Gain;

  /* Update for UnitDelay: '<Root>/Unit Delay' */
  PhysicalBasement_DW->UnitDelay_DSTATE = rtb_Gain;
}

/* Model initialize function */
void PhysicalBasement_initialize(RT_MODEL_PhysicalBasement_T *const
  PhysicalBasement_M, t_SensorData *PhysicalBasement_U_In1, real_T
  *PhysicalBasement_Y_Out1, real_T *PhysicalBasement_Y_Out2)
{
  DW_PhysicalBasement_T *PhysicalBasement_DW = PhysicalBasement_M->dwork;

  /* Registration code */

  /* states (dwork) */
  (void) memset((void *)PhysicalBasement_DW, 0,
                sizeof(DW_PhysicalBasement_T));

  /* external inputs */
  *PhysicalBasement_U_In1 = PhysicalBasement_rtZt_SensorDat;

  /* external outputs */
  (*PhysicalBasement_Y_Out1) = 0.0;
  (*PhysicalBasement_Y_Out2) = 0.0;

  /* InitializeConditions for UnitDelay: '<Root>/Unit Delay' */
  PhysicalBasement_DW->UnitDelay_DSTATE =
    PhysicalBasement_P.UnitDelay_InitialCondition;
}

/* Model terminate function */
void PhysicalBasement_terminate(RT_MODEL_PhysicalBasement_T *const
  PhysicalBasement_M)
{
  /* (no terminate code required) */
  UNUSED_PARAMETER(PhysicalBasement_M);
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

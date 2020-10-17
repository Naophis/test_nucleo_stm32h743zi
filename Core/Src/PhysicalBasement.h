/*
 * Sponsored License - for use in support of a program or activity
 * sponsored by MathWorks.  Not for government, commercial or other
 * non-sponsored organizational use.
 *
 * File: PhysicalBasement.h
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

#ifndef RTW_HEADER_PhysicalBasement_h_
#define RTW_HEADER_PhysicalBasement_h_
#include <string.h>
#ifndef PhysicalBasement_COMMON_INCLUDES_
#define PhysicalBasement_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* PhysicalBasement_COMMON_INCLUDES_ */

#include "PhysicalBasement_types.h"
#include "rt_defines.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* user code (top of header file) */
#include "header.h"

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T UnitDelay_DSTATE;             /* '<Root>/Unit Delay' */
} DW_PhysicalBasement_T;

/* Parameters (default storage) */
struct P_PhysicalBasement_T_ {
  real_T Gain_Gain;                    /* Expression: 3.14
                                        * Referenced by: '<Root>/Gain'
                                        */
  real_T UnitDelay_InitialCondition;   /* Expression: 0
                                        * Referenced by: '<Root>/Unit Delay'
                                        */
  real_T Gain1_Gain;                   /* Expression: 1
                                        * Referenced by: '<Root>/Gain1'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_PhysicalBasement_T {
  const char_T * volatile errorStatus;
  DW_PhysicalBasement_T *dwork;
};

/* Block parameters (default storage) */
extern P_PhysicalBasement_T PhysicalBasement_P;

/* External data declarations for dependent source files */
extern const t_SensorData PhysicalBasement_rtZt_SensorDat;/* t_SensorData ground */

/* Model entry point functions */
extern void PhysicalBasement_initialize(RT_MODEL_PhysicalBasement_T *const
  PhysicalBasement_M, t_SensorData *PhysicalBasement_U_In1, real_T
  *PhysicalBasement_Y_Out1, real_T *PhysicalBasement_Y_Out2);
extern void PhysicalBasement_step(RT_MODEL_PhysicalBasement_T *const
  PhysicalBasement_M, t_SensorData *PhysicalBasement_U_In1, real_T
  *PhysicalBasement_Y_Out1, real_T *PhysicalBasement_Y_Out2);
extern void PhysicalBasement_terminate(RT_MODEL_PhysicalBasement_T *const
  PhysicalBasement_M);

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
 * '<Root>' : 'PhysicalBasement'
 */
#endif                                 /* RTW_HEADER_PhysicalBasement_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: codegen_model.h
 *
 * Code generated for Simulink model 'codegen_model'.
 *
 * Model version                  : 1.11
 * Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
 * C/C++ source code generated on : Sat May 11 18:12:40 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#ifndef codegen_model_h_
#define codegen_model_h_
#ifndef codegen_model_COMMON_INCLUDES_
#define codegen_model_COMMON_INCLUDES_
#include <stdbool.h>
#include <stdint.h>
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "math.h"
#endif                                 /* codegen_model_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  double ABE;                          /* '<S1>/Rate Limiter' */
  double Integrator_DSTATE;            /* '<S38>/Integrator' */
  double Filter_DSTATE;                /* '<S33>/Filter' */
  double PrevY;                        /* '<S1>/Rate Limiter' */
  double LastMajorTime;                /* '<S1>/Rate Limiter' */
  bool PrevLimited;                    /* '<S1>/Rate Limiter' */
} DW;

/* Constant parameters (default storage) */
typedef struct {
  /* Expression: dy_mat
   * Referenced by: '<S1>/Constant3'
   */
  double Constant3_Value[1397];

  /* Expression: y{1}.Data
   * Referenced by: '<S1>/0% ABE Trajectory'
   */
  double uABETrajectory_tableData[127];

  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S1>/Constant'
   *   '<S1>/0% ABE Trajectory'
   *   '<S1>/1-D Lookup Table1'
   *   '<S1>/1-D Lookup Table2'
   *   '<S1>/1-D Lookup Table3'
   *   '<S1>/1-D Lookup Table4'
   *   '<S1>/1-D Lookup Table5'
   *   '<S1>/1-D Lookup Table6'
   *   '<S1>/1-D Lookup Table7'
   *   '<S1>/1-D Lookup Table8'
   *   '<S1>/1-D Lookup Table9'
   *   '<S1>/100% ABE Trajectory'
   */
  double pooled2[127];

  /* Expression: y{2}.Data
   * Referenced by: '<S1>/1-D Lookup Table1'
   */
  double uDLookupTable1_tableData[127];

  /* Expression: y{3}.Data
   * Referenced by: '<S1>/1-D Lookup Table2'
   */
  double uDLookupTable2_tableData[127];

  /* Expression: y{4}.Data
   * Referenced by: '<S1>/1-D Lookup Table3'
   */
  double uDLookupTable3_tableData[127];

  /* Expression: y{5}.Data
   * Referenced by: '<S1>/1-D Lookup Table4'
   */
  double uDLookupTable4_tableData[127];

  /* Expression: y{6}.Data
   * Referenced by: '<S1>/1-D Lookup Table5'
   */
  double uDLookupTable5_tableData[127];

  /* Expression: y{7}.Data
   * Referenced by: '<S1>/1-D Lookup Table8'
   */
  double uDLookupTable8_tableData[127];

  /* Expression: y{8}.Data
   * Referenced by: '<S1>/1-D Lookup Table7'
   */
  double uDLookupTable7_tableData[127];

  /* Expression: y{9}.Data
   * Referenced by: '<S1>/1-D Lookup Table6'
   */
  double uDLookupTable6_tableData[127];

  /* Expression: y{10}.Data
   * Referenced by: '<S1>/1-D Lookup Table9'
   */
  double uDLookupTable9_tableData[127];

  /* Expression: y{11}.Data
   * Referenced by: '<S1>/100% ABE Trajectory'
   */
  double u00ABETrajectory_tableData[127];
} ConstP;

/* External inputs (root inport signals with default storage) */
typedef struct {
  double Verticalvelocityinput;        /* '<Root>/Vertical velocity input' */
  double Altitudeinput;                /* '<Root>/Altitude input' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  double Airbrakesextoutput;           /* '<Root>/Air brakes ext output' */
} ExtY;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char *errorStatus;
  RTWSolverInfo solverInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_t clockTick0;
    double stepSize0;
    uint32_t clockTick1;
    SimTimeStep simTimeStep;
    double *t;
    double tArray[2];
  } Timing;
};

/* Block signals and states (default storage) */
//extern DW rtDW;
//
///* External inputs (root inport signals with default storage) */
//extern ExtU rtU;
//
///* External outputs (root outports fed by signals with default storage) */
//extern ExtY rtY;

/* Constant parameters (default storage) */
extern const ConstP rtConstP;

/* Model entry point functions */
extern void codegen_model_initialize(void);
extern void codegen_model_step(void);

/* Real-time Model object */
extern RT_MODEL *const rtM;

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
 * '<Root>' : 'codegen_model'
 * '<S1>'   : 'codegen_model/ABCS (No 3DoF model, for coding purposes)'
 * '<S2>'   : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller'
 * '<S3>'   : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/Selected trajectory to desired setpoint'
 * '<S4>'   : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/Trajectory selector'
 * '<S5>'   : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Anti-windup'
 * '<S6>'   : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/D Gain'
 * '<S7>'   : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/External Derivative'
 * '<S8>'   : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Filter'
 * '<S9>'   : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Filter ICs'
 * '<S10>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/I Gain'
 * '<S11>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Ideal P Gain'
 * '<S12>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Ideal P Gain Fdbk'
 * '<S13>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Integrator'
 * '<S14>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Integrator ICs'
 * '<S15>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/N Copy'
 * '<S16>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/N Gain'
 * '<S17>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/P Copy'
 * '<S18>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Parallel P Gain'
 * '<S19>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Reset Signal'
 * '<S20>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Saturation'
 * '<S21>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Saturation Fdbk'
 * '<S22>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Sum'
 * '<S23>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Sum Fdbk'
 * '<S24>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Tracking Mode'
 * '<S25>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Tracking Mode Sum'
 * '<S26>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Tsamp - Integral'
 * '<S27>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Tsamp - Ngain'
 * '<S28>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/postSat Signal'
 * '<S29>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/preSat Signal'
 * '<S30>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Anti-windup/Passthrough'
 * '<S31>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/D Gain/Internal Parameters'
 * '<S32>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/External Derivative/Error'
 * '<S33>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Filter/Disc. Forward Euler Filter'
 * '<S34>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Filter ICs/Internal IC - Filter'
 * '<S35>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/I Gain/Internal Parameters'
 * '<S36>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Ideal P Gain/Passthrough'
 * '<S37>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S38>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Integrator/Discrete'
 * '<S39>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Integrator ICs/Internal IC'
 * '<S40>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/N Copy/Disabled'
 * '<S41>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/N Gain/Internal Parameters'
 * '<S42>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/P Copy/Disabled'
 * '<S43>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Parallel P Gain/Internal Parameters'
 * '<S44>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Reset Signal/Disabled'
 * '<S45>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Saturation/Enabled'
 * '<S46>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Saturation Fdbk/Disabled'
 * '<S47>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Sum/Sum_PID'
 * '<S48>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Sum Fdbk/Disabled'
 * '<S49>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Tracking Mode/Disabled'
 * '<S50>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Tracking Mode Sum/Passthrough'
 * '<S51>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Tsamp - Integral/TsSignalSpecification'
 * '<S52>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Tsamp - Ngain/Passthrough'
 * '<S53>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/postSat Signal/Forward_Path'
 * '<S54>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/preSat Signal/Forward_Path'
 */
#endif                                 /* codegen_model_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

/*
 * codegen_model.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "codegen_model".
 *
 * Model version              : 1.12
 * Simulink Coder version : 23.2 (R2023b) 01-Aug-2023
 * C source code generated on : Fri Oct 11 02:16:48 2024
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_codegen_model_h_
#define RTW_HEADER_codegen_model_h_
#ifndef codegen_model_COMMON_INCLUDES_
#define codegen_model_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#endif                                 /* codegen_model_COMMON_INCLUDES_ */

#include "codegen_model_types.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"
#include <float.h>
#include <string.h>
#include <stddef.h>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetFinalTime
#define rtmGetFinalTime(rtm)           ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetRTWLogInfo
#define rtmGetRTWLogInfo(rtm)          ((rtm)->rtwLogInfo)
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTFinal
#define rtmGetTFinal(rtm)              ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

/* Block signals (default storage) */
typedef struct {
  real_T Sum;                          /* '<S1>/Sum' */
  real_T Saturation;                   /* '<S43>/Saturation' */
  real_T ABE;                          /* '<S1>/Rate Limiter' */
} B_codegen_model_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T Integrator_DSTATE;            /* '<S36>/Integrator' */
  real_T Filter_DSTATE;                /* '<S31>/Filter' */
  real_T PrevY;                        /* '<S1>/Rate Limiter' */
  real_T LastMajorTime;                /* '<S1>/Rate Limiter' */
  boolean_T PrevLimited;               /* '<S1>/Rate Limiter' */
} DW_codegen_model_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Expression: dy_mat
   * Referenced by: '<S1>/Constant3'
   */
  real_T Constant3_Value[2783];

  /* Expression: y{1}.Data
   * Referenced by: '<S1>/0% ABE Trajectory'
   */
  real_T uABETrajectory_tableData[253];

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
  real_T pooled2[253];

  /* Expression: y{2}.Data
   * Referenced by: '<S1>/1-D Lookup Table1'
   */
  real_T uDLookupTable1_tableData[253];

  /* Expression: y{3}.Data
   * Referenced by: '<S1>/1-D Lookup Table2'
   */
  real_T uDLookupTable2_tableData[253];

  /* Expression: y{4}.Data
   * Referenced by: '<S1>/1-D Lookup Table3'
   */
  real_T uDLookupTable3_tableData[253];

  /* Expression: y{5}.Data
   * Referenced by: '<S1>/1-D Lookup Table4'
   */
  real_T uDLookupTable4_tableData[253];

  /* Expression: y{6}.Data
   * Referenced by: '<S1>/1-D Lookup Table5'
   */
  real_T uDLookupTable5_tableData[253];

  /* Expression: y{7}.Data
   * Referenced by: '<S1>/1-D Lookup Table8'
   */
  real_T uDLookupTable8_tableData[253];

  /* Expression: y{8}.Data
   * Referenced by: '<S1>/1-D Lookup Table7'
   */
  real_T uDLookupTable7_tableData[253];

  /* Expression: y{9}.Data
   * Referenced by: '<S1>/1-D Lookup Table6'
   */
  real_T uDLookupTable6_tableData[253];

  /* Expression: y{10}.Data
   * Referenced by: '<S1>/1-D Lookup Table9'
   */
  real_T uDLookupTable9_tableData[253];

  /* Expression: y{11}.Data
   * Referenced by: '<S1>/100% ABE Trajectory'
   */
  real_T u00ABETrajectory_tableData[253];
} ConstP_codegen_model_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T Verticalvelocityinput;        /* '<Root>/Vertical velocity input' */
  real_T Altitudeinput;                /* '<Root>/Altitude input' */
} ExtU_codegen_model_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T Airbrakesextoutput;           /* '<Root>/Air brakes ext output' */
} ExtY_codegen_model_T;

/* Real-time Model Data Structure */
struct tag_RTM_codegen_model_T {
  const char_T *errorStatus;
  RTWLogInfo *rtwLogInfo;
  RTWSolverInfo solverInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    time_T tFinal;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Block signals (default storage) */
extern B_codegen_model_T codegen_model_B;

/* Block states (default storage) */
extern DW_codegen_model_T codegen_model_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_codegen_model_T codegen_model_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_codegen_model_T codegen_model_Y;

/* Constant parameters (default storage) */
extern const ConstP_codegen_model_T codegen_model_ConstP;

/* Model entry point functions */
extern void codegen_model_initialize(void);
extern void codegen_model_step(void);
extern void codegen_model_terminate(void);

/* Real-time Model object */
extern RT_MODEL_codegen_model_T *const codegen_model_M;

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
 * '<S7>'   : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Filter'
 * '<S8>'   : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Filter ICs'
 * '<S9>'   : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/I Gain'
 * '<S10>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Ideal P Gain'
 * '<S11>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Ideal P Gain Fdbk'
 * '<S12>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Integrator'
 * '<S13>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Integrator ICs'
 * '<S14>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/N Copy'
 * '<S15>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/N Gain'
 * '<S16>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/P Copy'
 * '<S17>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Parallel P Gain'
 * '<S18>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Reset Signal'
 * '<S19>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Saturation'
 * '<S20>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Saturation Fdbk'
 * '<S21>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Sum'
 * '<S22>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Sum Fdbk'
 * '<S23>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Tracking Mode'
 * '<S24>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Tracking Mode Sum'
 * '<S25>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Tsamp - Integral'
 * '<S26>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Tsamp - Ngain'
 * '<S27>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/postSat Signal'
 * '<S28>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/preSat Signal'
 * '<S29>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Anti-windup/Passthrough'
 * '<S30>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/D Gain/Internal Parameters'
 * '<S31>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Filter/Disc. Forward Euler Filter'
 * '<S32>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Filter ICs/Internal IC - Filter'
 * '<S33>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/I Gain/Internal Parameters'
 * '<S34>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Ideal P Gain/Passthrough'
 * '<S35>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S36>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Integrator/Discrete'
 * '<S37>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Integrator ICs/Internal IC'
 * '<S38>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/N Copy/Disabled'
 * '<S39>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/N Gain/Internal Parameters'
 * '<S40>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/P Copy/Disabled'
 * '<S41>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Parallel P Gain/Internal Parameters'
 * '<S42>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Reset Signal/Disabled'
 * '<S43>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Saturation/Enabled'
 * '<S44>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Saturation Fdbk/Disabled'
 * '<S45>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Sum/Sum_PID'
 * '<S46>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Sum Fdbk/Disabled'
 * '<S47>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Tracking Mode/Disabled'
 * '<S48>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Tracking Mode Sum/Passthrough'
 * '<S49>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Tsamp - Integral/TsSignalSpecification'
 * '<S50>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/Tsamp - Ngain/Passthrough'
 * '<S51>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/postSat Signal/Forward_Path'
 * '<S52>'  : 'codegen_model/ABCS (No 3DoF model, for coding purposes)/PID Controller/preSat Signal/Forward_Path'
 */
#endif                                 /* RTW_HEADER_codegen_model_h_ */

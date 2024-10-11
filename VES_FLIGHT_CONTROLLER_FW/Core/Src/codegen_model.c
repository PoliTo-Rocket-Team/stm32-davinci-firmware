/*
 * codegen_model.c
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

#include "codegen_model.h"
#include <math.h>
#include "rtwtypes.h"
#include <string.h>
#include "codegen_model_private.h"
#include "rt_nonfinite.h"

/* Block signals (default storage) */
B_codegen_model_T codegen_model_B;

/* Block states (default storage) */
DW_codegen_model_T codegen_model_DW;

/* External inputs (root inport signals with default storage) */
ExtU_codegen_model_T codegen_model_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_codegen_model_T codegen_model_Y;

/* Real-time model */
static RT_MODEL_codegen_model_T codegen_model_M_;
RT_MODEL_codegen_model_T *const codegen_model_M = &codegen_model_M_;

/* Lookup Binary Search Utility BINARYSEARCH_real_T */
void BINARYSEARCH_real_T(uint32_T *piLeft, uint32_T *piRght, real_T u, const
  real_T *pData, uint32_T iHi)
{
  /* Find the location of current input value in the data table. */
  *piLeft = 0U;
  *piRght = iHi;
  if (u <= pData[0] ) {
    /* Less than or equal to the smallest point in the table. */
    *piRght = 0U;
  } else if (u >= pData[iHi] ) {
    /* Greater than or equal to the largest point in the table. */
    *piLeft = iHi;
  } else {
    uint32_T i;

    /* Do a binary search. */
    while (( *piRght - *piLeft ) > 1U ) {
      /* Get the average of the left and right indices using to Floor rounding. */
      i = (*piLeft + *piRght) >> 1;

      /* Move either the right index or the left index so that */
      /*  LeftDataPoint <= CurrentValue < RightDataPoint */
      if (u < pData[i] ) {
        *piRght = i;
      } else {
        *piLeft = i;
      }
    }
  }
}

/* Lookup Utility LookUp_real_T_real_T */
void LookUp_real_T_real_T(real_T *pY, const real_T *pYData, real_T u, const
  real_T *pUData, uint32_T iHi)
{
  uint32_T iLeft;
  uint32_T iRght;
  BINARYSEARCH_real_T( &(iLeft), &(iRght), u, pUData, iHi);

  {
    real_T lambda;
    if (pUData[iRght] > pUData[iLeft] ) {
      real_T num;
      real_T den;
      den = pUData[iRght];
      den -= pUData[iLeft];
      num = u;
      num -= pUData[iLeft];
      lambda = num / den;
    } else {
      lambda = 0.0;
    }

    {
      real_T yLeftCast;
      real_T yRghtCast;
      yLeftCast = pYData[iLeft];
      yRghtCast = pYData[iRght];
      yLeftCast += lambda * ( yRghtCast - yLeftCast );
      (*pY) = yLeftCast;
    }
  }
}

real_T look1_binlxpw(real_T u0, const real_T bp0[], const real_T table[],
                     uint32_T maxIndex)
{
  real_T frac;
  real_T yL_0d0;
  uint32_T iLeft;

  /* Column-major Lookup 1-D
     Search method: 'binary'
     Use previous index: 'off'
     Interpolation method: 'Linear point-slope'
     Extrapolation method: 'Linear'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Linear'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = (u0 - bp0[0U]) / (bp0[1U] - bp0[0U]);
  } else if (u0 < bp0[maxIndex]) {
    uint32_T bpIdx;
    uint32_T iRght;

    /* Binary Search */
    bpIdx = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex - 1U;
    frac = (u0 - bp0[maxIndex - 1U]) / (bp0[maxIndex] - bp0[maxIndex - 1U]);
  }

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'portable wrapping'
   */
  yL_0d0 = table[iLeft];
  return (table[iLeft + 1U] - yL_0d0) * frac + yL_0d0;
}

/* Model step function */
void codegen_model_step(void)
{
  /* local block i/o variables */
  real_T rtb_FilterCoefficient;
  real_T rtb_IntegralGain;
  real_T rtb_TmpSignalConversionAtSFunct[11];
  const real_T *rtb_trajectory_vel_0;
  real_T difference;
  real_T rtb_Clock1;
  real_T rtb_Highesttrajectory;
  real_T rtb_LookupTableDynamic;
  real_T rtb_Lowesttrajectory;
  real_T rtb_undhighesttrajectory;
  real_T rtb_undlowesttrajectory;
  int32_T i;
  int32_T rtb_index;
  boolean_T limitedCache;

  /* Clock: '<S1>/Clock' incorporates:
   *  Clock: '<S1>/Clock1'
   */
  rtb_LookupTableDynamic = codegen_model_M->Timing.t[0];

  /* Lookup_n-D: '<S1>/0% ABE Trajectory' incorporates:
   *  S-Function (sfix_look1_dyn): '<S1>/Lookup Table Dynamic'
   */
  rtb_Lowesttrajectory = look1_binlxpw(rtb_LookupTableDynamic,
    codegen_model_ConstP.pooled2, codegen_model_ConstP.uABETrajectory_tableData,
    252U);

  /* Lookup_n-D: '<S1>/1-D Lookup Table1' incorporates:
   *  S-Function (sfix_look1_dyn): '<S1>/Lookup Table Dynamic'
   */
  rtb_undlowesttrajectory = look1_binlxpw(rtb_LookupTableDynamic,
    codegen_model_ConstP.pooled2, codegen_model_ConstP.uDLookupTable1_tableData,
    252U);

  /* Lookup_n-D: '<S1>/1-D Lookup Table2' incorporates:
   *  S-Function (sfix_look1_dyn): '<S1>/Lookup Table Dynamic'
   */
  rtb_Clock1 = look1_binlxpw(rtb_LookupTableDynamic,
    codegen_model_ConstP.pooled2, codegen_model_ConstP.uDLookupTable2_tableData,
    252U);

  /* SignalConversion generated from: '<S4>/ SFunction ' incorporates:
   *  Inport: '<Root>/Altitude input'
   *  MATLAB Function: '<S1>/Trajectory selector'
   *  Sum: '<S1>/Add3'
   */
  rtb_TmpSignalConversionAtSFunct[2] = codegen_model_U.Altitudeinput -
    rtb_Clock1;

  /* Lookup_n-D: '<S1>/1-D Lookup Table3' incorporates:
   *  S-Function (sfix_look1_dyn): '<S1>/Lookup Table Dynamic'
   */
  rtb_Clock1 = look1_binlxpw(rtb_LookupTableDynamic,
    codegen_model_ConstP.pooled2, codegen_model_ConstP.uDLookupTable3_tableData,
    252U);

  /* SignalConversion generated from: '<S4>/ SFunction ' incorporates:
   *  Inport: '<Root>/Altitude input'
   *  MATLAB Function: '<S1>/Trajectory selector'
   *  Sum: '<S1>/Add4'
   */
  rtb_TmpSignalConversionAtSFunct[3] = codegen_model_U.Altitudeinput -
    rtb_Clock1;

  /* Lookup_n-D: '<S1>/1-D Lookup Table4' incorporates:
   *  S-Function (sfix_look1_dyn): '<S1>/Lookup Table Dynamic'
   */
  rtb_Clock1 = look1_binlxpw(rtb_LookupTableDynamic,
    codegen_model_ConstP.pooled2, codegen_model_ConstP.uDLookupTable4_tableData,
    252U);

  /* SignalConversion generated from: '<S4>/ SFunction ' incorporates:
   *  Inport: '<Root>/Altitude input'
   *  MATLAB Function: '<S1>/Trajectory selector'
   *  Sum: '<S1>/Add5'
   */
  rtb_TmpSignalConversionAtSFunct[4] = codegen_model_U.Altitudeinput -
    rtb_Clock1;

  /* Lookup_n-D: '<S1>/1-D Lookup Table5' incorporates:
   *  S-Function (sfix_look1_dyn): '<S1>/Lookup Table Dynamic'
   */
  rtb_Clock1 = look1_binlxpw(rtb_LookupTableDynamic,
    codegen_model_ConstP.pooled2, codegen_model_ConstP.uDLookupTable5_tableData,
    252U);

  /* SignalConversion generated from: '<S4>/ SFunction ' incorporates:
   *  Inport: '<Root>/Altitude input'
   *  MATLAB Function: '<S1>/Trajectory selector'
   *  Sum: '<S1>/Add6'
   */
  rtb_TmpSignalConversionAtSFunct[5] = codegen_model_U.Altitudeinput -
    rtb_Clock1;

  /* Lookup_n-D: '<S1>/1-D Lookup Table8' incorporates:
   *  S-Function (sfix_look1_dyn): '<S1>/Lookup Table Dynamic'
   */
  rtb_Clock1 = look1_binlxpw(rtb_LookupTableDynamic,
    codegen_model_ConstP.pooled2, codegen_model_ConstP.uDLookupTable8_tableData,
    252U);

  /* SignalConversion generated from: '<S4>/ SFunction ' incorporates:
   *  Inport: '<Root>/Altitude input'
   *  MATLAB Function: '<S1>/Trajectory selector'
   *  Sum: '<S1>/Add9'
   */
  rtb_TmpSignalConversionAtSFunct[6] = codegen_model_U.Altitudeinput -
    rtb_Clock1;

  /* Lookup_n-D: '<S1>/1-D Lookup Table7' incorporates:
   *  S-Function (sfix_look1_dyn): '<S1>/Lookup Table Dynamic'
   */
  rtb_Clock1 = look1_binlxpw(rtb_LookupTableDynamic,
    codegen_model_ConstP.pooled2, codegen_model_ConstP.uDLookupTable7_tableData,
    252U);

  /* SignalConversion generated from: '<S4>/ SFunction ' incorporates:
   *  Inport: '<Root>/Altitude input'
   *  MATLAB Function: '<S1>/Trajectory selector'
   *  Sum: '<S1>/Add8'
   */
  rtb_TmpSignalConversionAtSFunct[7] = codegen_model_U.Altitudeinput -
    rtb_Clock1;

  /* Lookup_n-D: '<S1>/1-D Lookup Table6' incorporates:
   *  S-Function (sfix_look1_dyn): '<S1>/Lookup Table Dynamic'
   */
  rtb_Clock1 = look1_binlxpw(rtb_LookupTableDynamic,
    codegen_model_ConstP.pooled2, codegen_model_ConstP.uDLookupTable6_tableData,
    252U);

  /* Lookup_n-D: '<S1>/1-D Lookup Table9' incorporates:
   *  S-Function (sfix_look1_dyn): '<S1>/Lookup Table Dynamic'
   */
  rtb_undhighesttrajectory = look1_binlxpw(rtb_LookupTableDynamic,
    codegen_model_ConstP.pooled2, codegen_model_ConstP.uDLookupTable9_tableData,
    252U);

  /* Lookup_n-D: '<S1>/100% ABE Trajectory' incorporates:
   *  S-Function (sfix_look1_dyn): '<S1>/Lookup Table Dynamic'
   */
  rtb_Highesttrajectory = look1_binlxpw(rtb_LookupTableDynamic,
    codegen_model_ConstP.pooled2,
    codegen_model_ConstP.u00ABETrajectory_tableData, 252U);

  /* SignalConversion generated from: '<S4>/ SFunction ' incorporates:
   *  Inport: '<Root>/Altitude input'
   *  MATLAB Function: '<S1>/Trajectory selector'
   *  Sum: '<S1>/Add1'
   *  Sum: '<S1>/Add10'
   *  Sum: '<S1>/Add11'
   *  Sum: '<S1>/Add2'
   *  Sum: '<S1>/Add7'
   */
  rtb_TmpSignalConversionAtSFunct[0] = codegen_model_U.Altitudeinput -
    rtb_Lowesttrajectory;
  rtb_TmpSignalConversionAtSFunct[1] = codegen_model_U.Altitudeinput -
    rtb_undlowesttrajectory;
  rtb_TmpSignalConversionAtSFunct[8] = codegen_model_U.Altitudeinput -
    rtb_Clock1;
  rtb_TmpSignalConversionAtSFunct[9] = codegen_model_U.Altitudeinput -
    rtb_undhighesttrajectory;
  rtb_TmpSignalConversionAtSFunct[10] = codegen_model_U.Altitudeinput -
    rtb_Highesttrajectory;

  /* MATLAB Function: '<S1>/Trajectory selector' */
  rtb_Clock1 = (rtInf);
  rtb_index = 0;
  for (i = 0; i < 11; i++) {
    difference = fabs(rtb_TmpSignalConversionAtSFunct[i]);
    if (difference < rtb_Clock1) {
      rtb_Clock1 = difference;
      rtb_index = i + 1;
    }
  }

  /* MATLAB Function: '<S1>/Selected trajectory to desired setpoint' incorporates:
   *  Constant: '<S1>/Constant3'
   */
  rtb_trajectory_vel_0 = &codegen_model_ConstP.Constant3_Value[rtb_index * 253 +
    -253];

  /* Clock: '<S1>/Clock1' */
  rtb_Clock1 = rtb_LookupTableDynamic;

  /* S-Function (sfix_look1_dyn): '<S1>/Lookup Table Dynamic' incorporates:
   *  Constant: '<S1>/Constant'
   */
  /* Dynamic Look-Up Table Block: '<S1>/Lookup Table Dynamic'
   * Input0  Data Type:  Floating Point real_T
   * Input1  Data Type:  Floating Point real_T
   * Input2  Data Type:  Floating Point real_T
   * Output0 Data Type:  Floating Point real_T
   * Lookup Method: Linear_Endpoint
   *
   */
  LookUp_real_T_real_T( &(rtb_LookupTableDynamic), &rtb_trajectory_vel_0[0],
                       rtb_Clock1, codegen_model_ConstP.pooled2, 252U);

  /* Sum: '<S1>/Sum' incorporates:
   *  Inport: '<Root>/Vertical velocity input'
   */
  codegen_model_B.Sum = codegen_model_U.Verticalvelocityinput -
    rtb_LookupTableDynamic;

  /* Gain: '<S39>/Filter Coefficient' incorporates:
   *  DiscreteIntegrator: '<S31>/Filter'
   *  Gain: '<S30>/Derivative Gain'
   *  Sum: '<S31>/SumD'
   */
  rtb_FilterCoefficient = (92.8732132811108 * codegen_model_B.Sum -
    codegen_model_DW.Filter_DSTATE) * 138.62756104986786;

  /* Sum: '<S45>/Sum' incorporates:
   *  DiscreteIntegrator: '<S36>/Integrator'
   *  Gain: '<S41>/Proportional Gain'
   */
  codegen_model_B.Saturation = (300.0 * codegen_model_B.Sum +
    codegen_model_DW.Integrator_DSTATE) + rtb_FilterCoefficient;

  /* Saturate: '<S43>/Saturation' */
  if (codegen_model_B.Saturation > 1.0) {
    /* Sum: '<S45>/Sum' incorporates:
     *  Saturate: '<S43>/Saturation'
     */
    codegen_model_B.Saturation = 1.0;
  } else if (codegen_model_B.Saturation < 0.0) {
    /* Sum: '<S45>/Sum' incorporates:
     *  Saturate: '<S43>/Saturation'
     */
    codegen_model_B.Saturation = 0.0;
  }

  /* End of Saturate: '<S43>/Saturation' */

  /* Switch: '<S1>/Switch1' incorporates:
   *  Inport: '<Root>/Altitude input'
   */
  if (codegen_model_U.Altitudeinput >= 2000.0) {
    /* Switch: '<S1>/Switch4' incorporates:
     *  Gain: '<S1>/High margin mult'
     *  Gain: '<S1>/Low margin mult'
     *  Sum: '<S1>/Sum1'
     *  Sum: '<S1>/Sum2'
     *  Sum: '<S1>/Sum3'
     *  Sum: '<S1>/Sum4'
     *  Sum: '<S1>/Sum5'
     *  Sum: '<S1>/Sum6'
     *  Switch: '<S1>/Switch2'
     */
    if (codegen_model_U.Altitudeinput - ((rtb_Highesttrajectory -
          rtb_undhighesttrajectory) * 50.0 + rtb_Highesttrajectory) >= 0.0) {
      /* Switch: '<S1>/Switch1' incorporates:
       *  Constant: '<S1>/100% ABE'
       */
      codegen_model_B.ABE = 1.0;
    } else if (codegen_model_U.Altitudeinput - (rtb_Lowesttrajectory -
                (rtb_undlowesttrajectory - rtb_Lowesttrajectory) * 25.0) >= 0.0)
    {
      /* Switch: '<S1>/Switch1' incorporates:
       *  Switch: '<S1>/Switch2'
       */
      codegen_model_B.ABE = codegen_model_B.Saturation;
    } else {
      /* Switch: '<S1>/Switch1' incorporates:
       *  Constant: '<S1>/0% ABE'
       *  Switch: '<S1>/Switch2'
       */
      codegen_model_B.ABE = 0.0;
    }
  } else {
    /* Switch: '<S1>/Switch1' incorporates:
     *  Constant: '<S1>/Constant1'
     *  Switch: '<S1>/Switch4'
     */
    codegen_model_B.ABE = 0.0;
  }

  /* End of Switch: '<S1>/Switch1' */

  /* RateLimiter: '<S1>/Rate Limiter' */
  if (!(codegen_model_DW.LastMajorTime == (rtInf))) {
    rtb_undlowesttrajectory = codegen_model_M->Timing.t[0];
    rtb_Lowesttrajectory = rtb_undlowesttrajectory -
      codegen_model_DW.LastMajorTime;
    if (codegen_model_DW.LastMajorTime == rtb_undlowesttrajectory) {
      if (codegen_model_DW.PrevLimited) {
        /* Switch: '<S1>/Switch1' incorporates:
         *  RateLimiter: '<S1>/Rate Limiter'
         */
        codegen_model_B.ABE = codegen_model_DW.PrevY;
      }
    } else {
      rtb_undhighesttrajectory = rtb_Lowesttrajectory * 1.85;
      rtb_undlowesttrajectory = codegen_model_B.ABE - codegen_model_DW.PrevY;
      if (rtb_undlowesttrajectory > rtb_undhighesttrajectory) {
        /* Switch: '<S1>/Switch1' incorporates:
         *  RateLimiter: '<S1>/Rate Limiter'
         */
        codegen_model_B.ABE = codegen_model_DW.PrevY + rtb_undhighesttrajectory;
        limitedCache = true;
      } else {
        rtb_Lowesttrajectory *= -1.85;
        if (rtb_undlowesttrajectory < rtb_Lowesttrajectory) {
          /* Switch: '<S1>/Switch1' incorporates:
           *  RateLimiter: '<S1>/Rate Limiter'
           */
          codegen_model_B.ABE = codegen_model_DW.PrevY + rtb_Lowesttrajectory;
          limitedCache = true;
        } else {
          limitedCache = false;
        }
      }

      if (rtsiIsModeUpdateTimeStep(&codegen_model_M->solverInfo)) {
        codegen_model_DW.PrevLimited = limitedCache;
      }
    }
  }

  /* End of RateLimiter: '<S1>/Rate Limiter' */

  /* Outport: '<Root>/Air brakes ext output' */
  codegen_model_Y.Airbrakesextoutput = codegen_model_B.ABE;

  /* Gain: '<S33>/Integral Gain' */
  rtb_IntegralGain = codegen_model_B.Sum;

  /* Matfile logging */
  rt_UpdateTXYLogVars(codegen_model_M->rtwLogInfo, (codegen_model_M->Timing.t));

  /* Update for DiscreteIntegrator: '<S36>/Integrator' */
  codegen_model_DW.Integrator_DSTATE += 0.01 * rtb_IntegralGain;

  /* Update for DiscreteIntegrator: '<S31>/Filter' */
  codegen_model_DW.Filter_DSTATE += 0.01 * rtb_FilterCoefficient;

  /* Update for RateLimiter: '<S1>/Rate Limiter' */
  codegen_model_DW.PrevY = codegen_model_B.ABE;
  codegen_model_DW.LastMajorTime = codegen_model_M->Timing.t[0];

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.0s, 0.0s] */
    if ((rtmGetTFinal(codegen_model_M)!=-1) &&
        !((rtmGetTFinal(codegen_model_M)-codegen_model_M->Timing.t[0]) >
          codegen_model_M->Timing.t[0] * (DBL_EPSILON))) {
      rtmSetErrorStatus(codegen_model_M, "Simulation finished");
    }
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++codegen_model_M->Timing.clockTick0)) {
    ++codegen_model_M->Timing.clockTickH0;
  }

  codegen_model_M->Timing.t[0] = codegen_model_M->Timing.clockTick0 *
    codegen_model_M->Timing.stepSize0 + codegen_model_M->Timing.clockTickH0 *
    codegen_model_M->Timing.stepSize0 * 4294967296.0;

  {
    /* Update absolute timer for sample time: [0.01s, 0.0s] */
    /* The "clockTick1" counts the number of times the code of this task has
     * been executed. The resolution of this integer timer is 0.01, which is the step size
     * of the task. Size of "clockTick1" ensures timer will not overflow during the
     * application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick1 and the high bits
     * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
     */
    codegen_model_M->Timing.clockTick1++;
    if (!codegen_model_M->Timing.clockTick1) {
      codegen_model_M->Timing.clockTickH1++;
    }
  }
}

/* Model initialize function */
void codegen_model_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)codegen_model_M, 0,
                sizeof(RT_MODEL_codegen_model_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&codegen_model_M->solverInfo,
                          &codegen_model_M->Timing.simTimeStep);
    rtsiSetTPtr(&codegen_model_M->solverInfo, &rtmGetTPtr(codegen_model_M));
    rtsiSetStepSizePtr(&codegen_model_M->solverInfo,
                       &codegen_model_M->Timing.stepSize0);
    rtsiSetErrorStatusPtr(&codegen_model_M->solverInfo, (&rtmGetErrorStatus
      (codegen_model_M)));
    rtsiSetRTModelPtr(&codegen_model_M->solverInfo, codegen_model_M);
  }

  rtsiSetSimTimeStep(&codegen_model_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetSolverName(&codegen_model_M->solverInfo,"FixedStepDiscrete");
  rtmSetTPtr(codegen_model_M, &codegen_model_M->Timing.tArray[0]);
  rtmSetTFinal(codegen_model_M, 30.0);
  codegen_model_M->Timing.stepSize0 = 0.01;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = (NULL);
    codegen_model_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(codegen_model_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(codegen_model_M->rtwLogInfo, (NULL));
    rtliSetLogT(codegen_model_M->rtwLogInfo, "tout");
    rtliSetLogX(codegen_model_M->rtwLogInfo, "");
    rtliSetLogXFinal(codegen_model_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(codegen_model_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(codegen_model_M->rtwLogInfo, 4);
    rtliSetLogMaxRows(codegen_model_M->rtwLogInfo, 0);
    rtliSetLogDecimation(codegen_model_M->rtwLogInfo, 1);
    rtliSetLogY(codegen_model_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(codegen_model_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(codegen_model_M->rtwLogInfo, (NULL));
  }

  /* block I/O */
  (void) memset(((void *) &codegen_model_B), 0,
                sizeof(B_codegen_model_T));

  /* states (dwork) */
  (void) memset((void *)&codegen_model_DW, 0,
                sizeof(DW_codegen_model_T));

  /* external inputs */
  (void)memset(&codegen_model_U, 0, sizeof(ExtU_codegen_model_T));

  /* external outputs */
  codegen_model_Y.Airbrakesextoutput = 0.0;

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(codegen_model_M->rtwLogInfo, 0.0,
    rtmGetTFinal(codegen_model_M), codegen_model_M->Timing.stepSize0,
    (&rtmGetErrorStatus(codegen_model_M)));

  /* InitializeConditions for DiscreteIntegrator: '<S36>/Integrator' */
  codegen_model_DW.Integrator_DSTATE = 0.0;

  /* InitializeConditions for DiscreteIntegrator: '<S31>/Filter' */
  codegen_model_DW.Filter_DSTATE = 0.0;

  /* InitializeConditions for RateLimiter: '<S1>/Rate Limiter' */
  codegen_model_DW.LastMajorTime = (rtInf);
}

/* Model terminate function */
void codegen_model_terminate(void)
{
  /* (no terminate code required) */
}

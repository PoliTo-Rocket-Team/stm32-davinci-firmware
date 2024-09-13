/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: codegen_model.c
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

#include "codegen_model.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "math.h"

/* Private macros used by the generated code to access rtModel */
#ifndef rtmIsMajorTimeStep
#define rtmIsMajorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
#define rtmIsMinorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmSetTPtr
#define rtmSetTPtr(rtm, val)           ((rtm)->Timing.t = (val))
#endif

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;
static void BINARYSEARCH_double(uint32_t *piLeft, uint32_t *piRght, double u,
  const double *pData, uint32_t iHi);
static void LookUp_double_double(double *pY, const double *pYData, double u,
  const double *pUData, uint32_t iHi);
static double look1_binlx(double u0, const double bp0[], const double table[],
  uint32_t maxIndex);
//static double rtGetInf(void);
//static float rtGetInfF(void);
//static double rtGetMinusInf(void);
//static float rtGetMinusInfF(void);
extern double rtInf;
extern double rtMinusInf;
extern double rtNaN;
extern float rtInfF;
extern float rtMinusInfF;
extern float rtNaNF;
//static bool rtIsInf(double value);
//static bool rtIsInfF(float value);
//static bool rtIsNaN(double value);
//static bool rtIsNaNF(float value);
double rtNaN = -(double)NAN;
double rtInf = (double)INFINITY;
double rtMinusInf = -(double)INFINITY;
float rtNaNF = -(float)NAN;
float rtInfF = (float)INFINITY;
float rtMinusInfF = -(float)INFINITY;

/* Return rtInf needed by the generated code. */
//static double rtGetInf(void)
//{
//  return rtInf;
//}
//
///* Get rtInfF needed by the generated code. */
//static float rtGetInfF(void)
//{
//  return rtInfF;
//}
//
///* Return rtMinusInf needed by the generated code. */
//static double rtGetMinusInf(void)
//{
//  return rtMinusInf;
//}
//
///* Return rtMinusInfF needed by the generated code. */
//static float rtGetMinusInfF(void)
//{
//  return rtMinusInfF;
//}
//
///* Test if value is infinite */
//static bool rtIsInf(double value)
//{
//  return (bool)((value==rtInf || value==rtMinusInf) ? 1U : 0U);
//}
//
///* Test if single-precision value is infinite */
//static bool rtIsInfF(float value)
//{
//  return (bool)(((value)==rtInfF || (value)==rtMinusInfF) ? 1U : 0U);
//}
//
///* Test if value is not a number */
//static bool rtIsNaN(double value)
//{
//  return (bool)(isnan(value) != 0);
//}
//
///* Test if single-precision value is not a number */
//static bool rtIsNaNF(float value)
//{
//  return (bool)(isnan(value) != 0);
//}

/* Lookup Binary Search Utility BINARYSEARCH_double */
static void BINARYSEARCH_double(uint32_t *piLeft, uint32_t *piRght, double u,
  const double *pData, uint32_t iHi)
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
    uint32_t i;

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

/* Lookup Utility LookUp_double_double */
static void LookUp_double_double(double *pY, const double *pYData, double u,
  const double *pUData, uint32_t iHi)
{
  uint32_t iLeft;
  uint32_t iRght;
  BINARYSEARCH_double( &(iLeft), &(iRght), u, pUData, iHi);

  {
    double lambda;
    if (pUData[iRght] > pUData[iLeft] ) {
      double num;
      double den;
      den = pUData[iRght];
      den -= pUData[iLeft];
      num = u;
      num -= pUData[iLeft];
      lambda = num / den;
    } else {
      lambda = 0.0;
    }

    {
      double yLeftCast;
      double yRghtCast;
      yLeftCast = pYData[iLeft];
      yRghtCast = pYData[iRght];
      yLeftCast += lambda * ( yRghtCast - yLeftCast );
      (*pY) = yLeftCast;
    }
  }
}

static double look1_binlx(double u0, const double bp0[], const double table[],
  uint32_t maxIndex)
{
  double frac;
  double yL_0d0;
  uint32_t iLeft;

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
    uint32_t bpIdx;
    uint32_t iRght;

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
     Overflow mode: 'wrapping'
   */
  yL_0d0 = table[iLeft];
  return (table[iLeft + 1U] - yL_0d0) * frac + yL_0d0;
}

/* Model step function */
void codegen_model_step(void)
{
  /* local block i/o variables */
  double rtb_FilterCoefficient;
  double rtb_IntegralGain;
  double rtb_TmpSignalConversionAtSFunct[11];
  const double *rtb_trajectory_vel_0;
  double rtb_Add4;
  double rtb_Clock1;
  double rtb_Highesttrajectory;
  double rtb_LookupTableDynamic;
  double rtb_Lowesttrajectory;
  double rtb_undhighesttrajectory;
  double rtb_undlowesttrajectory;
  int32_t i;
  int32_t rtb_Add3;
  bool limitedCache;

  /* Clock: '<S1>/Clock' incorporates:
   *  Clock: '<S1>/Clock1'
   */
  rtb_LookupTableDynamic = rtM->Timing.t[0];

  /* Lookup_n-D: '<S1>/0% ABE Trajectory' incorporates:
   *  S-Function (sfix_look1_dyn): '<S1>/Lookup Table Dynamic'
   */
  rtb_Lowesttrajectory = look1_binlx(rtb_LookupTableDynamic, rtConstP.pooled2,
    rtConstP.uABETrajectory_tableData, 126U);

  /* Lookup_n-D: '<S1>/1-D Lookup Table1' incorporates:
   *  S-Function (sfix_look1_dyn): '<S1>/Lookup Table Dynamic'
   */
  rtb_undlowesttrajectory = look1_binlx(rtb_LookupTableDynamic, rtConstP.pooled2,
    rtConstP.uDLookupTable1_tableData, 126U);

  /* Lookup_n-D: '<S1>/1-D Lookup Table2' incorporates:
   *  S-Function (sfix_look1_dyn): '<S1>/Lookup Table Dynamic'
   */
  rtb_Clock1 = look1_binlx(rtb_LookupTableDynamic, rtConstP.pooled2,
    rtConstP.uDLookupTable2_tableData, 126U);

  /* SignalConversion generated from: '<S4>/ SFunction ' incorporates:
   *  Inport: '<Root>/Altitude input'
   *  MATLAB Function: '<S1>/Trajectory selector'
   *  Sum: '<S1>/Add3'
   */
  rtb_TmpSignalConversionAtSFunct[2] = rtU.Altitudeinput - rtb_Clock1;

  /* Lookup_n-D: '<S1>/1-D Lookup Table3' incorporates:
   *  S-Function (sfix_look1_dyn): '<S1>/Lookup Table Dynamic'
   */
  rtb_Clock1 = look1_binlx(rtb_LookupTableDynamic, rtConstP.pooled2,
    rtConstP.uDLookupTable3_tableData, 126U);

  /* SignalConversion generated from: '<S4>/ SFunction ' incorporates:
   *  Inport: '<Root>/Altitude input'
   *  MATLAB Function: '<S1>/Trajectory selector'
   *  Sum: '<S1>/Add4'
   */
  rtb_TmpSignalConversionAtSFunct[3] = rtU.Altitudeinput - rtb_Clock1;

  /* Lookup_n-D: '<S1>/1-D Lookup Table4' incorporates:
   *  S-Function (sfix_look1_dyn): '<S1>/Lookup Table Dynamic'
   */
  rtb_Clock1 = look1_binlx(rtb_LookupTableDynamic, rtConstP.pooled2,
    rtConstP.uDLookupTable4_tableData, 126U);

  /* SignalConversion generated from: '<S4>/ SFunction ' incorporates:
   *  Inport: '<Root>/Altitude input'
   *  MATLAB Function: '<S1>/Trajectory selector'
   *  Sum: '<S1>/Add5'
   */
  rtb_TmpSignalConversionAtSFunct[4] = rtU.Altitudeinput - rtb_Clock1;

  /* Lookup_n-D: '<S1>/1-D Lookup Table5' incorporates:
   *  S-Function (sfix_look1_dyn): '<S1>/Lookup Table Dynamic'
   */
  rtb_Clock1 = look1_binlx(rtb_LookupTableDynamic, rtConstP.pooled2,
    rtConstP.uDLookupTable5_tableData, 126U);

  /* SignalConversion generated from: '<S4>/ SFunction ' incorporates:
   *  Inport: '<Root>/Altitude input'
   *  MATLAB Function: '<S1>/Trajectory selector'
   *  Sum: '<S1>/Add6'
   */
  rtb_TmpSignalConversionAtSFunct[5] = rtU.Altitudeinput - rtb_Clock1;

  /* Lookup_n-D: '<S1>/1-D Lookup Table8' incorporates:
   *  S-Function (sfix_look1_dyn): '<S1>/Lookup Table Dynamic'
   */
  rtb_Clock1 = look1_binlx(rtb_LookupTableDynamic, rtConstP.pooled2,
    rtConstP.uDLookupTable8_tableData, 126U);

  /* SignalConversion generated from: '<S4>/ SFunction ' incorporates:
   *  Inport: '<Root>/Altitude input'
   *  MATLAB Function: '<S1>/Trajectory selector'
   *  Sum: '<S1>/Add9'
   */
  rtb_TmpSignalConversionAtSFunct[6] = rtU.Altitudeinput - rtb_Clock1;

  /* Lookup_n-D: '<S1>/1-D Lookup Table7' incorporates:
   *  S-Function (sfix_look1_dyn): '<S1>/Lookup Table Dynamic'
   */
  rtb_Clock1 = look1_binlx(rtb_LookupTableDynamic, rtConstP.pooled2,
    rtConstP.uDLookupTable7_tableData, 126U);

  /* SignalConversion generated from: '<S4>/ SFunction ' incorporates:
   *  Inport: '<Root>/Altitude input'
   *  MATLAB Function: '<S1>/Trajectory selector'
   *  Sum: '<S1>/Add8'
   */
  rtb_TmpSignalConversionAtSFunct[7] = rtU.Altitudeinput - rtb_Clock1;

  /* Lookup_n-D: '<S1>/1-D Lookup Table6' incorporates:
   *  S-Function (sfix_look1_dyn): '<S1>/Lookup Table Dynamic'
   */
  rtb_Clock1 = look1_binlx(rtb_LookupTableDynamic, rtConstP.pooled2,
    rtConstP.uDLookupTable6_tableData, 126U);

  /* Lookup_n-D: '<S1>/1-D Lookup Table9' incorporates:
   *  S-Function (sfix_look1_dyn): '<S1>/Lookup Table Dynamic'
   */
  rtb_undhighesttrajectory = look1_binlx(rtb_LookupTableDynamic,
    rtConstP.pooled2, rtConstP.uDLookupTable9_tableData, 126U);

  /* Lookup_n-D: '<S1>/100% ABE Trajectory' incorporates:
   *  S-Function (sfix_look1_dyn): '<S1>/Lookup Table Dynamic'
   */
  rtb_Highesttrajectory = look1_binlx(rtb_LookupTableDynamic, rtConstP.pooled2,
    rtConstP.u00ABETrajectory_tableData, 126U);

  /* SignalConversion generated from: '<S4>/ SFunction ' incorporates:
   *  Inport: '<Root>/Altitude input'
   *  MATLAB Function: '<S1>/Trajectory selector'
   *  Sum: '<S1>/Add1'
   *  Sum: '<S1>/Add10'
   *  Sum: '<S1>/Add11'
   *  Sum: '<S1>/Add2'
   *  Sum: '<S1>/Add7'
   */
  rtb_TmpSignalConversionAtSFunct[0] = rtU.Altitudeinput - rtb_Lowesttrajectory;
  rtb_TmpSignalConversionAtSFunct[1] = rtU.Altitudeinput -
    rtb_undlowesttrajectory;
  rtb_TmpSignalConversionAtSFunct[8] = rtU.Altitudeinput - rtb_Clock1;
  rtb_TmpSignalConversionAtSFunct[9] = rtU.Altitudeinput -
    rtb_undhighesttrajectory;
  rtb_TmpSignalConversionAtSFunct[10] = rtU.Altitudeinput -
    rtb_Highesttrajectory;

  /* MATLAB Function: '<S1>/Trajectory selector' */
  rtb_Clock1 = (rtInf);
  rtb_Add3 = 0;
  for (i = 0; i < 11; i++) {
    rtb_Add4 = fabs(rtb_TmpSignalConversionAtSFunct[i]);
    if (rtb_Add4 < rtb_Clock1) {
      rtb_Clock1 = rtb_Add4;
      rtb_Add3 = i + 1;
    }
  }

  /* MATLAB Function: '<S1>/Selected trajectory to desired setpoint' incorporates:
   *  Constant: '<S1>/Constant3'
   */
  rtb_trajectory_vel_0 = &rtConstP.Constant3_Value[rtb_Add3 * 127 + -127];

  /* Clock: '<S1>/Clock1' */
  rtb_Clock1 = rtb_LookupTableDynamic;

  /* S-Function (sfix_look1_dyn): '<S1>/Lookup Table Dynamic' incorporates:
   *  Constant: '<S1>/Constant'
   */
  /* Dynamic Look-Up Table Block: '<S1>/Lookup Table Dynamic'
   * Input0  Data Type:  Floating Point double
   * Input1  Data Type:  Floating Point double
   * Input2  Data Type:  Floating Point double
   * Output0 Data Type:  Floating Point double
   * Lookup Method: Linear_Endpoint
   *
   */
  LookUp_double_double( &(rtb_LookupTableDynamic), &rtb_trajectory_vel_0[0],
                       rtb_Clock1, rtConstP.pooled2, 126U);

  /* Sum: '<S1>/Sum' incorporates:
   *  Inport: '<Root>/Vertical velocity input'
   */
  rtb_LookupTableDynamic = rtU.Verticalvelocityinput - rtb_LookupTableDynamic;

  /* Gain: '<S41>/Filter Coefficient' incorporates:
   *  DiscreteIntegrator: '<S33>/Filter'
   *  Gain: '<S31>/Derivative Gain'
   *  Sum: '<S33>/SumD'
   */
  rtb_FilterCoefficient = (9.8046696918563914 * rtb_LookupTableDynamic -
    rtDW.Filter_DSTATE) * 50.551815401727268;

  /* Switch: '<S1>/Switch1' incorporates:
   *  Constant: '<S1>/Constant1'
   *  Inport: '<Root>/Altitude input'
   */
  if (rtU.Altitudeinput >= 1500.0) {
    /* Switch: '<S1>/Switch4' incorporates:
     *  Constant: '<S1>/0% ABE'
     *  Constant: '<S1>/100% ABE'
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
    if (rtU.Altitudeinput - ((rtb_Highesttrajectory - rtb_undhighesttrajectory) *
         50.0 + rtb_Highesttrajectory) >= 0.0) {
      rtb_Lowesttrajectory = 1.0;
    } else if (rtU.Altitudeinput - (rtb_Lowesttrajectory -
                (rtb_undlowesttrajectory - rtb_Lowesttrajectory) * 25.0) >= 0.0)
    {
      /* Sum: '<S47>/Sum' incorporates:
       *  DiscreteIntegrator: '<S38>/Integrator'
       *  Gain: '<S43>/Proportional Gain'
       */
      rtb_Lowesttrajectory = (18.439618771030755 * rtb_LookupTableDynamic +
        rtDW.Integrator_DSTATE) + rtb_FilterCoefficient;

      /* Saturate: '<S45>/Saturation' */
      if (rtb_Lowesttrajectory > 1.0) {
        /* Switch: '<S1>/Switch2' */
        rtb_Lowesttrajectory = 1.0;
      } else if (rtb_Lowesttrajectory < 0.0) {
        /* Switch: '<S1>/Switch2' */
        rtb_Lowesttrajectory = 0.0;
      }

      /* End of Saturate: '<S45>/Saturation' */
    } else {
      rtb_Lowesttrajectory = 0.0;
    }

    /* End of Switch: '<S1>/Switch4' */
  } else {
    rtb_Lowesttrajectory = 0.0;
  }

  /* End of Switch: '<S1>/Switch1' */

  /* RateLimiter: '<S1>/Rate Limiter' */
  if (rtDW.LastMajorTime == (rtInf)) {
    /* RateLimiter: '<S1>/Rate Limiter' */
    rtDW.ABE = rtb_Lowesttrajectory;
  } else {
    rtb_undhighesttrajectory = rtM->Timing.t[0];
    rtb_undlowesttrajectory = rtb_undhighesttrajectory - rtDW.LastMajorTime;
    if (rtDW.LastMajorTime == rtb_undhighesttrajectory) {
      if (rtDW.PrevLimited) {
        /* RateLimiter: '<S1>/Rate Limiter' */
        rtDW.ABE = rtDW.PrevY;
      } else {
        /* RateLimiter: '<S1>/Rate Limiter' */
        rtDW.ABE = rtb_Lowesttrajectory;
      }
    } else {
      rtb_undhighesttrajectory = rtb_Lowesttrajectory - rtDW.PrevY;
      if (rtb_undhighesttrajectory > rtb_undlowesttrajectory) {
        /* RateLimiter: '<S1>/Rate Limiter' */
        rtDW.ABE = rtDW.PrevY + rtb_undlowesttrajectory;
        limitedCache = true;
      } else {
        rtb_undlowesttrajectory = -rtb_undlowesttrajectory;
        if (rtb_undhighesttrajectory < rtb_undlowesttrajectory) {
          /* RateLimiter: '<S1>/Rate Limiter' */
          rtDW.ABE = rtDW.PrevY + rtb_undlowesttrajectory;
          limitedCache = true;
        } else {
          /* RateLimiter: '<S1>/Rate Limiter' */
          rtDW.ABE = rtb_Lowesttrajectory;
          limitedCache = false;
        }
      }

      if (rtsiIsModeUpdateTimeStep(&rtM->solverInfo)) {
        rtDW.PrevLimited = limitedCache;
      }
    }
  }

  /* End of RateLimiter: '<S1>/Rate Limiter' */

  /* Outport: '<Root>/Air brakes ext output' */
  rtY.Airbrakesextoutput = rtDW.ABE;

  /* Gain: '<S35>/Integral Gain' */
  rtb_IntegralGain = 0.32557239105898234 * rtb_LookupTableDynamic;

  /* Update for DiscreteIntegrator: '<S38>/Integrator' */
  rtDW.Integrator_DSTATE += 0.01 * rtb_IntegralGain;

  /* Update for DiscreteIntegrator: '<S33>/Filter' */
  rtDW.Filter_DSTATE += 0.01 * rtb_FilterCoefficient;

  /* Update for RateLimiter: '<S1>/Rate Limiter' */
  rtDW.PrevY = rtDW.ABE;
  rtDW.LastMajorTime = rtM->Timing.t[0];

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   */
  rtM->Timing.t[0] =
    ((double)(++rtM->Timing.clockTick0)) * rtM->Timing.stepSize0;

  {
    /* Update absolute timer for sample time: [0.01s, 0.0s] */
    /* The "clockTick1" counts the number of times the code of this task has
     * been executed. The resolution of this integer timer is 0.01, which is the step size
     * of the task. Size of "clockTick1" ensures timer will not overflow during the
     * application lifespan selected.
     */
    rtM->Timing.clockTick1++;
  }
}

/* Model initialize function */
void codegen_model_initialize(void)
{
  /* Registration code */
  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&rtM->solverInfo, &rtM->Timing.simTimeStep);
    rtsiSetTPtr(&rtM->solverInfo, &rtmGetTPtr(rtM));
    rtsiSetStepSizePtr(&rtM->solverInfo, &rtM->Timing.stepSize0);
    rtsiSetErrorStatusPtr(&rtM->solverInfo, (&rtmGetErrorStatus(rtM)));
    rtsiSetRTModelPtr(&rtM->solverInfo, rtM);
  }

  rtsiSetSimTimeStep(&rtM->solverInfo, MAJOR_TIME_STEP);
  rtsiSetIsMinorTimeStepWithModeChange(&rtM->solverInfo, false);
  rtsiSetIsContModeFrozen(&rtM->solverInfo, false);
  rtsiSetSolverName(&rtM->solverInfo,"FixedStepDiscrete");
  rtmSetTPtr(rtM, &rtM->Timing.tArray[0]);
  rtM->Timing.stepSize0 = 0.01;

  /* InitializeConditions for RateLimiter: '<S1>/Rate Limiter' */
  rtDW.LastMajorTime = (rtInf);
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

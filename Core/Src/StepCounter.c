/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: StepCounter.c
 *
 * Code generated for Simulink model 'StepCounter'.
 *
 * Model version                  : 1.30
 * Simulink Coder version         : 26.1 (R2026a) 20-Nov-2025
 * C/C++ source code generated on : Wed Jul  1 11:01:54 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: STMicroelectronics->ST10/Super10
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "StepCounter.h"
#include <math.h>
#include "rtwtypes.h"

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

#ifndef UCHAR_MAX
#include <limits.h>
#endif

#if ( UCHAR_MAX != (0xFFU) ) || ( SCHAR_MAX != (0x7F) )
#error Code was generated for compiler with different sized uchar/char. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( USHRT_MAX != (0xFFFFU) ) || ( SHRT_MAX != (0x7FFF) )
#error Code was generated for compiler with different sized ushort/short. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( UINT_MAX != (0xFFFFU) ) || ( INT_MAX != (0x7FFF) )
#error Code was generated for compiler with different sized uint/int. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( ULONG_MAX != (0xFFFFFFFFUL) ) || ( LONG_MAX != (0x7FFFFFFFL) )
#error Code was generated for compiler with different sized ulong/long. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

/* Skipping ulong_long/long_long check: insufficient preprocessor integer range. */

/* Continuous states */
X rtX;

/* Disabled State Vector */
XDis rtXDis;

/* Block signals and states (default storage) */
DW rtDW;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;
extern uint32_T MWDSP_EPH_R_D(real_T evt, uint32_T *sta);

/* private model entry point functions */
extern void StepCounter_derivatives(void);

/*
 * This function updates continuous states using the ODE3 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  /* Solver Matrices */
  static const real_T rt_ODE3_A[3] = {
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3] = {
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE3_IntgData *id = (ODE3_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T hB[3];
  int_T i;
  int_T nXc = 6;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  StepCounter_derivatives();

  /* f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*)); */
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  StepCounter_step();
  StepCounter_derivatives();

  /* f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*)); */
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  StepCounter_step();
  StepCounter_derivatives();

  /* tnew = t + hA(3);
     ynew = y + f*hB(:,3); */
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

uint32_T MWDSP_EPH_R_D(real_T evt, uint32_T *sta)
{
  uint32_T previousState;
  uint32_T retVal;
  int16_T curState;
  int16_T lastzcevent;
  int16_T newState;
  int16_T newStateR;

  /* S-Function (sdspcount2): '<S3>/Counter' */
  /* Detect rising edge events */
  previousState = *sta;
  retVal = 0UL;
  lastzcevent = 0;
  newState = 5;
  newStateR = 5;
  if (evt > 0.0) {
    curState = 2;
  } else {
    curState = !(evt < 0.0);
  }

  if (*sta == 5UL) {
    newStateR = curState;
  } else if ((uint32_T)curState != *sta) {
    if (*sta == 3UL) {
      if (curState == 1) {
        newStateR = 1;
      } else {
        lastzcevent = 2;
        previousState = 1UL;
      }
    }

    if (previousState == 4UL) {
      if (curState == 1) {
        newStateR = 1;
      } else {
        lastzcevent = 3;
        previousState = 1UL;
      }
    }

    if ((previousState == 1UL) && (curState == 2)) {
      retVal = 2UL;
    }

    if (previousState == 0UL) {
      retVal = 2UL;
    }

    if ((uint16_T)retVal == (uint16_T)lastzcevent) {
      retVal = 0UL;
    }

    if ((curState == 1) && (retVal == 2UL)) {
      newState = 3;
    } else {
      newState = curState;
    }
  }

  if (newStateR != 5) {
    *sta = (uint32_T)newStateR;
    retVal = 0UL;
  }

  if (newState != 5) {
    *sta = (uint32_T)newState;
  }

  /* End of S-Function (sdspcount2): '<S3>/Counter' */
  return retVal;
}

/* Model step function */
void StepCounter_step(void)
{
  real_T TmpSignalConversionAtIntegrat_2[9];
  real_T rtb_TransferFcn_0[9];
  real_T rtb_TransferFcn_1[9];
  real_T TmpSignalConversionAtIntegrat_0;
  real_T TmpSignalConversionAtIntegrat_1;
  real_T TmpSignalConversionAtIntegrator;
  real_T rtb_TransferFcn1;
  real_T rtb_TransferFcn2;
  real_T rtb_TransferFcn_2;
  real_T rtb_y;
  int16_T TmpSignalConversionAtIntegrat_3;
  int16_T i;
  int16_T i_0;
  int16_T rtb_TransferFcn_tmp;
  uint16_T rtb_Counter1;
  static const int8_T b[3] = { 0, 0, 1 };

  static const int8_T c[3] = { 0, 1, 0 };

  static const int8_T d[3] = { 1, 0, 0 };

  static const int8_T b_b[3] = { -1, 0, 0 };

  if (rtmIsMajorTimeStep(rtM)) {
    /* set solver stop time */
    rtsiSetSolverStopTime(&rtM->solverInfo,((rtM->Timing.clockTick0+1)*
      rtM->Timing.stepSize0));
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(rtM)) {
    rtM->Timing.t[0] = rtsiGetT(&rtM->solverInfo);
  }

  /* TransferFcn: '<S1>/Transfer Fcn1' */
  rtb_TransferFcn1 = 10.0 * rtX.TransferFcn1_CSTATE;

  /* TransferFcn: '<S1>/Transfer Fcn2' */
  rtb_TransferFcn2 = 10.0 * rtX.TransferFcn2_CSTATE;

  /* MATLAB Function: '<S2>/angle extrapolation' incorporates:
   *  Integrator: '<S2>/Integrator'
   *  MATLAB Function: '<S2>/obtainment of the trajectory of g'
   */
  TmpSignalConversionAtIntegrator = tan(rtX.Integrator_CSTATE[1]);
  rtb_y = cos(rtX.Integrator_CSTATE[0]);
  TmpSignalConversionAtIntegrat_0 = sin(rtX.Integrator_CSTATE[0]);
  TmpSignalConversionAtIntegrat_1 = cos(rtX.Integrator_CSTATE[1]);

  /* SignalConversion generated from: '<S2>/Integrator' incorporates:
   *  MATLAB Function: '<S2>/angle extrapolation'
   *  TransferFcn: '<S1>/Transfer Fcn'
   */
  rtDW.TmpSignalConversionAtIntegrator[0] = (TmpSignalConversionAtIntegrat_0 *
    TmpSignalConversionAtIntegrator * rtb_TransferFcn1 + 10.0 *
    rtX.TransferFcn_CSTATE) + rtb_y * TmpSignalConversionAtIntegrator *
    rtb_TransferFcn2;
  rtDW.TmpSignalConversionAtIntegrator[1] = rtb_y * rtb_TransferFcn1 -
    TmpSignalConversionAtIntegrat_0 * rtb_TransferFcn2;
  rtDW.TmpSignalConversionAtIntegrator[2] = TmpSignalConversionAtIntegrat_0 /
    TmpSignalConversionAtIntegrat_1 * rtb_TransferFcn1 + rtb_y /
    TmpSignalConversionAtIntegrat_1 * rtb_TransferFcn2;

  /* MATLAB Function: '<S2>/obtainment of the trajectory of g' incorporates:
   *  Integrator: '<S2>/Integrator'
   */
  TmpSignalConversionAtIntegrator = sin(rtX.Integrator_CSTATE[2]);
  rtb_TransferFcn2 = cos(rtX.Integrator_CSTATE[2]);
  rtb_TransferFcn1 = sin(rtX.Integrator_CSTATE[1]);
  rtb_TransferFcn_0[0] = rtb_TransferFcn2;
  rtb_TransferFcn_0[3] = -TmpSignalConversionAtIntegrator;
  rtb_TransferFcn_0[6] = 0.0;
  rtb_TransferFcn_0[1] = TmpSignalConversionAtIntegrator;
  rtb_TransferFcn_0[4] = rtb_TransferFcn2;
  rtb_TransferFcn_0[7] = 0.0;
  TmpSignalConversionAtIntegrat_2[0] = TmpSignalConversionAtIntegrat_1;
  TmpSignalConversionAtIntegrat_2[3] = 0.0;
  TmpSignalConversionAtIntegrat_2[6] = rtb_TransferFcn1;
  TmpSignalConversionAtIntegrat_2[2] = -rtb_TransferFcn1;
  TmpSignalConversionAtIntegrat_2[5] = 0.0;
  TmpSignalConversionAtIntegrat_2[8] = TmpSignalConversionAtIntegrat_1;
  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_TransferFcn_tmp = 3 * i_0 + 2;
    rtb_TransferFcn_0[rtb_TransferFcn_tmp] = b[i_0];
    TmpSignalConversionAtIntegrat_3 = 3 * i_0 + 1;
    TmpSignalConversionAtIntegrat_2[TmpSignalConversionAtIntegrat_3] = c[i_0];
    rtb_TransferFcn_1[3 * i_0] = 0.0;
    rtb_TransferFcn_1[TmpSignalConversionAtIntegrat_3] = 0.0;
    rtb_TransferFcn_1[rtb_TransferFcn_tmp] = 0.0;
  }

  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_TransferFcn2 = rtb_TransferFcn_1[3 * i_0];
    rtb_TransferFcn_tmp = 3 * i_0 + 1;
    rtb_TransferFcn1 = rtb_TransferFcn_1[rtb_TransferFcn_tmp];
    TmpSignalConversionAtIntegrat_3 = 3 * i_0 + 2;
    TmpSignalConversionAtIntegrator =
      rtb_TransferFcn_1[TmpSignalConversionAtIntegrat_3];
    for (i = 0; i < 3; i++) {
      TmpSignalConversionAtIntegrat_1 = TmpSignalConversionAtIntegrat_2[3 * i_0
        + i];
      rtb_TransferFcn2 += rtb_TransferFcn_0[3 * i] *
        TmpSignalConversionAtIntegrat_1;
      rtb_TransferFcn1 += rtb_TransferFcn_0[3 * i + 1] *
        TmpSignalConversionAtIntegrat_1;
      TmpSignalConversionAtIntegrator += rtb_TransferFcn_0[3 * i + 2] *
        TmpSignalConversionAtIntegrat_1;
    }

    rtb_TransferFcn_1[TmpSignalConversionAtIntegrat_3] =
      TmpSignalConversionAtIntegrator;
    rtb_TransferFcn_1[rtb_TransferFcn_tmp] = rtb_TransferFcn1;
    rtb_TransferFcn_1[3 * i_0] = rtb_TransferFcn2;
  }

  TmpSignalConversionAtIntegrat_2[1] = 0.0;
  TmpSignalConversionAtIntegrat_2[4] = rtb_y;
  TmpSignalConversionAtIntegrat_2[7] = -TmpSignalConversionAtIntegrat_0;
  TmpSignalConversionAtIntegrat_2[2] = 0.0;
  TmpSignalConversionAtIntegrat_2[5] = TmpSignalConversionAtIntegrat_0;
  TmpSignalConversionAtIntegrat_2[8] = rtb_y;
  rtb_TransferFcn2 = 0.0;
  rtb_TransferFcn1 = 0.0;
  TmpSignalConversionAtIntegrator = 0.0;
  for (i_0 = 0; i_0 < 3; i_0++) {
    TmpSignalConversionAtIntegrat_2[3 * i_0] = d[i_0];
    rtb_y = 0.0;
    TmpSignalConversionAtIntegrat_0 = 0.0;
    rtb_TransferFcn_2 = 0.0;
    for (i = 0; i < 3; i++) {
      TmpSignalConversionAtIntegrat_1 = TmpSignalConversionAtIntegrat_2[3 * i_0
        + i];
      rtb_y += rtb_TransferFcn_1[3 * i] * TmpSignalConversionAtIntegrat_1;
      TmpSignalConversionAtIntegrat_0 += rtb_TransferFcn_1[3 * i + 1] *
        TmpSignalConversionAtIntegrat_1;
      rtb_TransferFcn_2 += rtb_TransferFcn_1[3 * i + 2] *
        TmpSignalConversionAtIntegrat_1;
    }

    rtb_TransferFcn_tmp = 3 * i_0 + 2;
    rtb_TransferFcn_0[rtb_TransferFcn_tmp] = rtb_TransferFcn_2;
    TmpSignalConversionAtIntegrat_3 = 3 * i_0 + 1;
    rtb_TransferFcn_0[TmpSignalConversionAtIntegrat_3] =
      TmpSignalConversionAtIntegrat_0;
    rtb_TransferFcn_0[3 * i_0] = rtb_y;
    i = b_b[i_0];
    rtb_TransferFcn2 += rtb_TransferFcn_0[3 * i_0] * (real_T)i;
    rtb_TransferFcn1 += rtb_TransferFcn_0[TmpSignalConversionAtIntegrat_3] *
      (real_T)i;
    TmpSignalConversionAtIntegrator += rtb_TransferFcn_0[rtb_TransferFcn_tmp] *
      (real_T)i;
  }

  /* MATLAB Function: '<S3>/MATLAB Function' incorporates:
   *  Gain: '<S3>/Gain1'
   *  Inport: '<Root>/In2'
   *  MATLAB Function: '<S2>/obtainment of the trajectory of g'
   *  Sum: '<Root>/Add'
   */
  rtDW.y = (((rtIn2[1] - 9.81 * rtb_TransferFcn1) * 0.0 + (rtIn2[0] - 9.81 *
              rtb_TransferFcn2)) + (rtIn2[2] - 9.81 *
             TmpSignalConversionAtIntegrator) * 0.0 < -2.5);
  if (rtmIsMajorTimeStep(rtM) &&
      rtM->Timing.TaskCounters.TID[1] == 0) {
    /* S-Function (sdspcount2): '<S3>/Counter1' */
    if (rtDW.y != 0.0) {
      rtDW.Counter1_Count = 0U;
    }

    rtb_Counter1 = rtDW.Counter1_Count;
    if (rtDW.Counter1_Count < 9999U) {
      rtDW.Counter1_Count++;
    } else {
      rtDW.Counter1_Count = 0U;
    }

    /* End of S-Function (sdspcount2): '<S3>/Counter1' */

    /* S-Function (sdspcount2): '<S3>/Counter' incorporates:
     *  Constant: '<S3>/Zero'
     *  MATLAB Function: '<S3>/MATLAB Function1'
     */
    if (MWDSP_EPH_R_D(0.0, &rtDW.Counter_RstEphState) != 0UL) {
      rtDW.Counter_Count = 0U;
    }

    if (MWDSP_EPH_R_D((real_T)(rtb_Counter1 > 20U), &rtDW.Counter_ClkEphState)
        != 0UL) {
      if (rtDW.Counter_Count < 9999U) {
        rtDW.Counter_Count++;
      } else {
        rtDW.Counter_Count = 0U;
      }
    }

    /* Outport: '<Root>/step number' incorporates:
     *  S-Function (sdspcount2): '<S3>/Counter'
     */
    rtY.stepnumber = rtDW.Counter_Count;
  }

  if (rtmIsMajorTimeStep(rtM)) {
    rt_ertODEUpdateContinuousStates(&rtM->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     */
    ++rtM->Timing.clockTick0;
    rtM->Timing.t[0] = rtsiGetSolverStopTime(&rtM->solverInfo);

    {
      /* Update absolute timer for sample time: [0.01s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.01, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       */
      rtM->Timing.clockTick1++;
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void StepCounter_derivatives(void)
{
  XDot *_rtXdot;
  _rtXdot = ((XDot *) rtM->derivs);

  /* Derivatives for Integrator: '<S2>/Integrator' */
  _rtXdot->Integrator_CSTATE[0] = rtDW.TmpSignalConversionAtIntegrator[0];
  _rtXdot->Integrator_CSTATE[1] = rtDW.TmpSignalConversionAtIntegrator[1];
  _rtXdot->Integrator_CSTATE[2] = rtDW.TmpSignalConversionAtIntegrator[2];

  /* Derivatives for TransferFcn: '<S1>/Transfer Fcn' incorporates:
   *  Inport: '<Root>/In1'
   */
  _rtXdot->TransferFcn_CSTATE = -10.0 * rtX.TransferFcn_CSTATE;
  _rtXdot->TransferFcn_CSTATE += rtIn1[0];

  /* Derivatives for TransferFcn: '<S1>/Transfer Fcn1' incorporates:
   *  Inport: '<Root>/In1'
   */
  _rtXdot->TransferFcn1_CSTATE = -10.0 * rtX.TransferFcn1_CSTATE;
  _rtXdot->TransferFcn1_CSTATE += rtIn1[1];

  /* Derivatives for TransferFcn: '<S1>/Transfer Fcn2' incorporates:
   *  Inport: '<Root>/In1'
   */
  _rtXdot->TransferFcn2_CSTATE = -10.0 * rtX.TransferFcn2_CSTATE;
  _rtXdot->TransferFcn2_CSTATE += rtIn1[2];
}

/* Model initialize function */
void StepCounter_initialize(void)
{
  /* Registration code */
  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&rtM->solverInfo, &rtM->Timing.simTimeStep);
    rtsiSetTPtr(&rtM->solverInfo, &rtmGetTPtr(rtM));
    rtsiSetStepSizePtr(&rtM->solverInfo, &rtM->Timing.stepSize0);
    rtsiSetdXPtr(&rtM->solverInfo, &rtM->derivs);
    rtsiSetContStatesPtr(&rtM->solverInfo, (real_T **) &rtM->contStates);
    rtsiSetNumContStatesPtr(&rtM->solverInfo, &rtM->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&rtM->solverInfo,
      &rtM->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&rtM->solverInfo,
      &rtM->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&rtM->solverInfo,
      &rtM->periodicContStateRanges);
    rtsiSetContStateDisabledPtr(&rtM->solverInfo, (boolean_T**)
      &rtM->contStateDisabled);
    rtsiSetErrorStatusPtr(&rtM->solverInfo, (&rtmGetErrorStatus(rtM)));
    rtsiSetRTModelPtr(&rtM->solverInfo, rtM);
  }

  rtsiSetSimTimeStep(&rtM->solverInfo, MAJOR_TIME_STEP);
  rtsiSetIsMinorTimeStepWithModeChange(&rtM->solverInfo, false);
  rtsiSetIsContModeFrozen(&rtM->solverInfo, false);
  rtM->intgData.y = rtM->odeY;
  rtM->intgData.f[0] = rtM->odeF[0];
  rtM->intgData.f[1] = rtM->odeF[1];
  rtM->intgData.f[2] = rtM->odeF[2];
  rtM->contStates = ((X *) &rtX);
  rtM->contStateDisabled = ((XDis *) &rtXDis);
  rtM->Timing.tStart = (0.0);
  rtsiSetSolverData(&rtM->solverInfo, (void *)&rtM->intgData);
  rtsiSetSolverName(&rtM->solverInfo,"ode3");
  rtmSetTPtr(rtM, &rtM->Timing.tArray[0]);
  rtM->Timing.stepSize0 = 0.01;

  /* InitializeConditions for Integrator: '<S2>/Integrator' */
  rtX.Integrator_CSTATE[0] = 0.0;
  rtX.Integrator_CSTATE[1] = 0.0;
  rtX.Integrator_CSTATE[2] = 0.0;

  /* InitializeConditions for TransferFcn: '<S1>/Transfer Fcn' */
  rtX.TransferFcn_CSTATE = 0.0;

  /* InitializeConditions for TransferFcn: '<S1>/Transfer Fcn1' */
  rtX.TransferFcn1_CSTATE = 0.0;

  /* InitializeConditions for TransferFcn: '<S1>/Transfer Fcn2' */
  rtX.TransferFcn2_CSTATE = 0.0;

  /* InitializeConditions for S-Function (sdspcount2): '<S3>/Counter' */
  rtDW.Counter_ClkEphState = 5UL;
  rtDW.Counter_RstEphState = 5UL;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: StepCounter.c
 *
 * Code generated for Simulink model 'StepCounter'.
 *
 * Model version                  : 1.33
 * Simulink Coder version         : 26.1 (R2026a) 20-Nov-2025
 * C/C++ source code generated on : Wed Jul  1 18:27:25 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 *
 * ============================================================================
 * MANUAL MODIFICATION WARNING:
 * This file has been manually modified to include a robust rising-edge 
 * detection filter in the state machine to prevent stationary step counting. 
 * DO NOT directly overwrite this file with raw Simulink generation without 
 * migrating the custom rising-edge logic.
 * ============================================================================
 */

#include "StepCounter.h"
#include <math.h>
#include "StepCounter_private.h"
#include "rtwtypes.h"

/* Block signals (default storage) */
B_StepCounter_T StepCounter_B;

/* Continuous states */
X_StepCounter_T StepCounter_X;

/* Disabled State Vector */
XDis_StepCounter_T StepCounter_XDis;

/* Block states (default storage) */
DW_StepCounter_T StepCounter_DW;

/* External inputs (root inport signals with default storage) */
ExtU_StepCounter_T StepCounter_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_StepCounter_T StepCounter_Y;

/* Real-time model */
static RT_MODEL_StepCounter_T StepCounter_M_;
RT_MODEL_StepCounter_T *const StepCounter_M = &StepCounter_M_;

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
  uint32_T curState;
  uint32_T lastzcevent;
  uint32_T newState;
  uint32_T newStateR;
  uint32_T previousState;
  uint32_T retVal;

  /* S-Function (sdspcount2): '<S3>/Counter' */
  /* Detect rising edge events */
  previousState = *sta;
  retVal = 0U;
  lastzcevent = 0U;
  newState = 5U;
  newStateR = 5U;
  if (evt > 0.0) {
    curState = 2U;
  } else {
    curState = (uint32_T)!(evt < 0.0);
  }

  if (*sta == 5U) {
    newStateR = curState;
  } else if (curState != *sta) {
    if (*sta == 3U) {
      if (curState == 1U) {
        newStateR = 1U;
      } else {
        lastzcevent = 2U;
        previousState = 1U;
      }
    }

    if (previousState == 4U) {
      if (curState == 1U) {
        newStateR = 1U;
      } else {
        lastzcevent = 3U;
        previousState = 1U;
      }
    }

    if ((previousState == 1U) && (curState == 2U)) {
      retVal = 2U;
    }

    if (previousState == 0U) {
      retVal = 2U;
    }

    if (retVal == lastzcevent) {
      retVal = 0U;
    }

    if ((curState == 1U) && (retVal == 2U)) {
      newState = 3U;
    } else {
      newState = curState;
    }
  }

  if (newStateR != 5U) {
    *sta = newStateR;
    retVal = 0U;
  }

  if (newState != 5U) {
    *sta = newState;
  }

  /* End of S-Function (sdspcount2): '<S3>/Counter' */
  return retVal;
}

/* Model step function */
void StepCounter_step(void)
{
  real_T TmpSignalConversionAtIntegrat_0;
  real_T TmpSignalConversionAtIntegrator;
  real_T y_tmp;
  int32_T TmpSignalConversionAtIntegrat_1;
  int32_T i;
  int32_T rtb_Counter1;
  int32_T y_tmp_tmp;
  static const int8_T b[3] = { 0, 0, 1 };

  static const int8_T c[3] = { 0, 1, 0 };

  static const int8_T d[3] = { 1, 0, 0 };

  static const int8_T b_b[3] = { -1, 0, 0 };

  if (rtmIsMajorTimeStep(StepCounter_M)) {
    /* set solver stop time */
    rtsiSetSolverStopTime(&StepCounter_M->solverInfo,
                          ((StepCounter_M->Timing.clockTick0+1)*
      StepCounter_M->Timing.stepSize0));
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(StepCounter_M)) {
    StepCounter_M->Timing.t[0] = rtsiGetT(&StepCounter_M->solverInfo);
  }

  /* TransferFcn: '<S1>/Transfer Fcn1' */
  StepCounter_B.TransferFcn1 = 10.0 * StepCounter_X.TransferFcn1_CSTATE;

  /* TransferFcn: '<S1>/Transfer Fcn2' */
  StepCounter_B.TransferFcn2 = 10.0 * StepCounter_X.TransferFcn2_CSTATE;

  /* MATLAB Function: '<S2>/angle extrapolation' incorporates:
   *  Integrator: '<S2>/Integrator'
   *  MATLAB Function: '<S2>/obtainment of the trajectory of g'
   */
  StepCounter_B.TmpSignalConversionAtIntegrat_k = tan
    (StepCounter_X.Integrator_CSTATE[1]);
  StepCounter_B.TmpSignalConversionAtIntegrat_c = cos
    (StepCounter_X.Integrator_CSTATE[0]);
  TmpSignalConversionAtIntegrator = sin(StepCounter_X.Integrator_CSTATE[0]);
  TmpSignalConversionAtIntegrat_0 = cos(StepCounter_X.Integrator_CSTATE[1]);

  /* SignalConversion generated from: '<S2>/Integrator' incorporates:
   *  MATLAB Function: '<S2>/angle extrapolation'
   *  TransferFcn: '<S1>/Transfer Fcn'
   */
  StepCounter_B.TmpSignalConversionAtIntegrator[0] =
    (TmpSignalConversionAtIntegrator *
     StepCounter_B.TmpSignalConversionAtIntegrat_k * StepCounter_B.TransferFcn1
     + 10.0 * StepCounter_X.TransferFcn_CSTATE) +
    StepCounter_B.TmpSignalConversionAtIntegrat_c *
    StepCounter_B.TmpSignalConversionAtIntegrat_k * StepCounter_B.TransferFcn2;
  StepCounter_B.TmpSignalConversionAtIntegrator[1] =
    StepCounter_B.TmpSignalConversionAtIntegrat_c * StepCounter_B.TransferFcn1 -
    TmpSignalConversionAtIntegrator * StepCounter_B.TransferFcn2;
  StepCounter_B.TmpSignalConversionAtIntegrator[2] =
    TmpSignalConversionAtIntegrator / TmpSignalConversionAtIntegrat_0 *
    StepCounter_B.TransferFcn1 + StepCounter_B.TmpSignalConversionAtIntegrat_c /
    TmpSignalConversionAtIntegrat_0 * StepCounter_B.TransferFcn2;

  /* MATLAB Function: '<S2>/obtainment of the trajectory of g' incorporates:
   *  Integrator: '<S2>/Integrator'
   */
  StepCounter_B.TransferFcn1 = sin(StepCounter_X.Integrator_CSTATE[2]);
  StepCounter_B.TransferFcn2 = cos(StepCounter_X.Integrator_CSTATE[2]);
  StepCounter_B.TmpSignalConversionAtIntegrat_k = sin
    (StepCounter_X.Integrator_CSTATE[1]);
  StepCounter_B.y_tmp[0] = StepCounter_B.TransferFcn2;
  StepCounter_B.y_tmp[3] = -StepCounter_B.TransferFcn1;
  StepCounter_B.y_tmp[6] = 0.0;
  StepCounter_B.y_tmp[1] = StepCounter_B.TransferFcn1;
  StepCounter_B.y_tmp[4] = StepCounter_B.TransferFcn2;
  StepCounter_B.y_tmp[7] = 0.0;
  StepCounter_B.TmpSignalConversionAtIntegrat_m[0] =
    TmpSignalConversionAtIntegrat_0;
  StepCounter_B.TmpSignalConversionAtIntegrat_m[3] = 0.0;
  StepCounter_B.TmpSignalConversionAtIntegrat_m[6] =
    StepCounter_B.TmpSignalConversionAtIntegrat_k;
  StepCounter_B.TmpSignalConversionAtIntegrat_m[2] =
    -StepCounter_B.TmpSignalConversionAtIntegrat_k;
  StepCounter_B.TmpSignalConversionAtIntegrat_m[5] = 0.0;
  StepCounter_B.TmpSignalConversionAtIntegrat_m[8] =
    TmpSignalConversionAtIntegrat_0;
  for (rtb_Counter1 = 0; rtb_Counter1 < 3; rtb_Counter1++) {
    y_tmp_tmp = 3 * rtb_Counter1 + 2;
    StepCounter_B.y_tmp[y_tmp_tmp] = b[rtb_Counter1];
    TmpSignalConversionAtIntegrat_1 = 3 * rtb_Counter1 + 1;
    StepCounter_B.TmpSignalConversionAtIntegrat_m[TmpSignalConversionAtIntegrat_1]
      = c[rtb_Counter1];
    StepCounter_B.y_tmp_c[3 * rtb_Counter1] = 0.0;
    StepCounter_B.y_tmp_c[TmpSignalConversionAtIntegrat_1] = 0.0;
    StepCounter_B.y_tmp_c[y_tmp_tmp] = 0.0;
  }

  for (rtb_Counter1 = 0; rtb_Counter1 < 3; rtb_Counter1++) {
    StepCounter_B.TransferFcn1 = StepCounter_B.y_tmp_c[3 * rtb_Counter1];
    y_tmp_tmp = 3 * rtb_Counter1 + 1;
    StepCounter_B.TransferFcn2 = StepCounter_B.y_tmp_c[y_tmp_tmp];
    TmpSignalConversionAtIntegrat_1 = 3 * rtb_Counter1 + 2;
    StepCounter_B.TmpSignalConversionAtIntegrat_k =
      StepCounter_B.y_tmp_c[TmpSignalConversionAtIntegrat_1];
    for (i = 0; i < 3; i++) {
      TmpSignalConversionAtIntegrat_0 =
        StepCounter_B.TmpSignalConversionAtIntegrat_m[3 * rtb_Counter1 + i];
      StepCounter_B.TransferFcn1 += StepCounter_B.y_tmp[3 * i] *
        TmpSignalConversionAtIntegrat_0;
      StepCounter_B.TransferFcn2 += StepCounter_B.y_tmp[3 * i + 1] *
        TmpSignalConversionAtIntegrat_0;
      StepCounter_B.TmpSignalConversionAtIntegrat_k += StepCounter_B.y_tmp[3 * i
        + 2] * TmpSignalConversionAtIntegrat_0;
    }

    StepCounter_B.y_tmp_c[TmpSignalConversionAtIntegrat_1] =
      StepCounter_B.TmpSignalConversionAtIntegrat_k;
    StepCounter_B.y_tmp_c[y_tmp_tmp] = StepCounter_B.TransferFcn2;
    StepCounter_B.y_tmp_c[3 * rtb_Counter1] = StepCounter_B.TransferFcn1;
  }

  StepCounter_B.TmpSignalConversionAtIntegrat_m[1] = 0.0;
  StepCounter_B.TmpSignalConversionAtIntegrat_m[4] =
    StepCounter_B.TmpSignalConversionAtIntegrat_c;
  StepCounter_B.TmpSignalConversionAtIntegrat_m[7] =
    -TmpSignalConversionAtIntegrator;
  StepCounter_B.TmpSignalConversionAtIntegrat_m[2] = 0.0;
  StepCounter_B.TmpSignalConversionAtIntegrat_m[5] =
    TmpSignalConversionAtIntegrator;
  StepCounter_B.TmpSignalConversionAtIntegrat_m[8] =
    StepCounter_B.TmpSignalConversionAtIntegrat_c;
  StepCounter_B.TransferFcn1 = 0.0;
  StepCounter_B.TransferFcn2 = 0.0;
  StepCounter_B.TmpSignalConversionAtIntegrat_k = 0.0;
  for (rtb_Counter1 = 0; rtb_Counter1 < 3; rtb_Counter1++) {
    StepCounter_B.TmpSignalConversionAtIntegrat_m[3 * rtb_Counter1] =
      d[rtb_Counter1];
    StepCounter_B.TmpSignalConversionAtIntegrat_c = 0.0;
    TmpSignalConversionAtIntegrator = 0.0;
    y_tmp = 0.0;
    for (i = 0; i < 3; i++) {
      TmpSignalConversionAtIntegrat_0 =
        StepCounter_B.TmpSignalConversionAtIntegrat_m[3 * rtb_Counter1 + i];
      StepCounter_B.TmpSignalConversionAtIntegrat_c += StepCounter_B.y_tmp_c[3 *
        i] * TmpSignalConversionAtIntegrat_0;
      TmpSignalConversionAtIntegrator += StepCounter_B.y_tmp_c[3 * i + 1] *
        TmpSignalConversionAtIntegrat_0;
      y_tmp += StepCounter_B.y_tmp_c[3 * i + 2] *
        TmpSignalConversionAtIntegrat_0;
    }

    y_tmp_tmp = 3 * rtb_Counter1 + 2;
    StepCounter_B.y_tmp[y_tmp_tmp] = y_tmp;
    TmpSignalConversionAtIntegrat_1 = 3 * rtb_Counter1 + 1;
    StepCounter_B.y_tmp[TmpSignalConversionAtIntegrat_1] =
      TmpSignalConversionAtIntegrator;
    StepCounter_B.y_tmp[3 * rtb_Counter1] =
      StepCounter_B.TmpSignalConversionAtIntegrat_c;
    i = b_b[rtb_Counter1];
    StepCounter_B.TransferFcn1 += StepCounter_B.y_tmp[3 * rtb_Counter1] *
      (real_T)i;
    StepCounter_B.TransferFcn2 +=
      StepCounter_B.y_tmp[TmpSignalConversionAtIntegrat_1] * (real_T)i;
    StepCounter_B.TmpSignalConversionAtIntegrat_k +=
      StepCounter_B.y_tmp[y_tmp_tmp] * (real_T)i;
  }

  /* MATLAB Function: '<S3>/MATLAB Function' incorporates:
   *  Gain: '<S3>/Gain1'
   *  Inport: '<Root>/In2'
   *  MATLAB Function: '<S2>/obtainment of the trajectory of g'
   *  Sum: '<Root>/Add'
   */
  StepCounter_B.y = (((StepCounter_U.In2[1] - 9.81 * StepCounter_B.TransferFcn2)
                      * 0.0 + (StepCounter_U.In2[0] - 9.81 *
    StepCounter_B.TransferFcn1)) + (StepCounter_U.In2[2] - 9.81 *
    StepCounter_B.TmpSignalConversionAtIntegrat_k) * 0.0 < -2.5);
  if (rtmIsMajorTimeStep(StepCounter_M)) {
    /* Custom Debounce Logic: Count the step immediately on rising edge if cooldown has passed */
    static real_T prev_y = 0.0;
    int rising_edge = (StepCounter_B.y != 0.0 && prev_y == 0.0);
    prev_y = StepCounter_B.y;
    
    if (rising_edge && StepCounter_DW.Counter1_Count > 20) {
      if (StepCounter_DW.Counter_Count < 9999) {
        StepCounter_DW.Counter_Count++;
      } else {
        StepCounter_DW.Counter_Count = 0U;
      }
      /* Reset cooldown timer */
      StepCounter_DW.Counter1_Count = 0U;
    } else {
      /* Increment cooldown timer */
      if (StepCounter_DW.Counter1_Count <= 20) {
        StepCounter_DW.Counter1_Count++;
      }
    }

    /* Outport: '<Root>/step number' incorporates:
     *  S-Function (sdspcount2): '<S3>/Counter'
     */
    StepCounter_Y.stepnumber = StepCounter_DW.Counter_Count;
  }

  if (rtmIsMajorTimeStep(StepCounter_M)) {
    rt_ertODEUpdateContinuousStates(&StepCounter_M->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     */
    ++StepCounter_M->Timing.clockTick0;
    StepCounter_M->Timing.t[0] = rtsiGetSolverStopTime
      (&StepCounter_M->solverInfo);

    {
      /* Update absolute timer for sample time: [0.01s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.01, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       */
      StepCounter_M->Timing.clockTick1++;
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void StepCounter_derivatives(void)
{
  XDot_StepCounter_T *_rtXdot;
  _rtXdot = ((XDot_StepCounter_T *) StepCounter_M->derivs);

  /* Derivatives for Integrator: '<S2>/Integrator' */
  _rtXdot->Integrator_CSTATE[0] = StepCounter_B.TmpSignalConversionAtIntegrator
    [0];
  _rtXdot->Integrator_CSTATE[1] = StepCounter_B.TmpSignalConversionAtIntegrator
    [1];
  _rtXdot->Integrator_CSTATE[2] = StepCounter_B.TmpSignalConversionAtIntegrator
    [2];

  /* Derivatives for TransferFcn: '<S1>/Transfer Fcn' incorporates:
   *  Inport: '<Root>/In1'
   */
  _rtXdot->TransferFcn_CSTATE = -10.0 * StepCounter_X.TransferFcn_CSTATE;
  _rtXdot->TransferFcn_CSTATE += StepCounter_U.In1[0];

  /* Derivatives for TransferFcn: '<S1>/Transfer Fcn1' incorporates:
   *  Inport: '<Root>/In1'
   */
  _rtXdot->TransferFcn1_CSTATE = -10.0 * StepCounter_X.TransferFcn1_CSTATE;
  _rtXdot->TransferFcn1_CSTATE += StepCounter_U.In1[1];

  /* Derivatives for TransferFcn: '<S1>/Transfer Fcn2' incorporates:
   *  Inport: '<Root>/In1'
   */
  _rtXdot->TransferFcn2_CSTATE = -10.0 * StepCounter_X.TransferFcn2_CSTATE;
  _rtXdot->TransferFcn2_CSTATE += StepCounter_U.In1[2];
}

/* Model initialize function */
void StepCounter_initialize(void)
{
  /* Registration code */
  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&StepCounter_M->solverInfo,
                          &StepCounter_M->Timing.simTimeStep);
    rtsiSetTPtr(&StepCounter_M->solverInfo, &rtmGetTPtr(StepCounter_M));
    rtsiSetStepSizePtr(&StepCounter_M->solverInfo,
                       &StepCounter_M->Timing.stepSize0);
    rtsiSetdXPtr(&StepCounter_M->solverInfo, &StepCounter_M->derivs);
    rtsiSetContStatesPtr(&StepCounter_M->solverInfo, (real_T **)
                         &StepCounter_M->contStates);
    rtsiSetNumContStatesPtr(&StepCounter_M->solverInfo,
      &StepCounter_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&StepCounter_M->solverInfo,
      &StepCounter_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&StepCounter_M->solverInfo,
      &StepCounter_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&StepCounter_M->solverInfo,
      &StepCounter_M->periodicContStateRanges);
    rtsiSetContStateDisabledPtr(&StepCounter_M->solverInfo, (boolean_T**)
      &StepCounter_M->contStateDisabled);
    rtsiSetErrorStatusPtr(&StepCounter_M->solverInfo, (&rtmGetErrorStatus
      (StepCounter_M)));
    rtsiSetRTModelPtr(&StepCounter_M->solverInfo, StepCounter_M);
  }

  rtsiSetSimTimeStep(&StepCounter_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetIsMinorTimeStepWithModeChange(&StepCounter_M->solverInfo, false);
  rtsiSetIsContModeFrozen(&StepCounter_M->solverInfo, false);
  StepCounter_M->intgData.y = StepCounter_M->odeY;
  StepCounter_M->intgData.f[0] = StepCounter_M->odeF[0];
  StepCounter_M->intgData.f[1] = StepCounter_M->odeF[1];
  StepCounter_M->intgData.f[2] = StepCounter_M->odeF[2];
  StepCounter_M->contStates = ((X_StepCounter_T *) &StepCounter_X);
  StepCounter_M->contStateDisabled = ((XDis_StepCounter_T *) &StepCounter_XDis);
  StepCounter_M->Timing.tStart = (0.0);
  rtsiSetSolverData(&StepCounter_M->solverInfo, (void *)&StepCounter_M->intgData);
  rtsiSetSolverName(&StepCounter_M->solverInfo,"ode3");
  rtmSetTPtr(StepCounter_M, &StepCounter_M->Timing.tArray[0]);
  StepCounter_M->Timing.stepSize0 = 0.01;

  /* InitializeConditions for Integrator: '<S2>/Integrator' */
  StepCounter_X.Integrator_CSTATE[0] = 0.0;
  StepCounter_X.Integrator_CSTATE[1] = 0.0;
  StepCounter_X.Integrator_CSTATE[2] = 0.0;

  /* InitializeConditions for TransferFcn: '<S1>/Transfer Fcn' */
  StepCounter_X.TransferFcn_CSTATE = 0.0;

  /* InitializeConditions for TransferFcn: '<S1>/Transfer Fcn1' */
  StepCounter_X.TransferFcn1_CSTATE = 0.0;

  /* InitializeConditions for TransferFcn: '<S1>/Transfer Fcn2' */
  StepCounter_X.TransferFcn2_CSTATE = 0.0;

  /* InitializeConditions for S-Function (sdspcount2): '<S3>/Counter' */
  StepCounter_DW.Counter_ClkEphState = 5U;
  StepCounter_DW.Counter_RstEphState = 5U;
}

/* Model terminate function */
void StepCounter_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

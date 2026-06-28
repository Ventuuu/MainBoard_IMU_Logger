/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: StepCounter.h
 *
 * Code generated for Simulink model 'StepCounter'.
 *
 * Model version                  : 1.27
 * Simulink Coder version         : 26.1 (R2026a) 20-Nov-2025
 * C/C++ source code generated on : Sat Jun 27 23:03:37 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef StepCounter_h_
#define StepCounter_h_
#ifndef StepCounter_COMMON_INCLUDES_
#define StepCounter_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "math.h"
#endif                                 /* StepCounter_COMMON_INCLUDES_ */

#include "StepCounter_types.h"
#include <math.h>
#include <string.h>

/* Macros for accessing real-time model data structure */
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

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

#ifndef rtmGetTStart
#define rtmGetTStart(rtm)              ((rtm)->Timing.tStart)
#endif

/* Block signals (default storage) */
typedef struct {
  real_T TmpSignalConversionAtIntegrator[3];
  real_T FromWorkspace[3];             /* '<Root>/From Workspace' */
  real_T y;                            /* '<S3>/MATLAB Function' */
} B_StepCounter_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  struct {
    void *TimePtr;
    void *DataPtr;
    void *RSimInfoPtr;
  } FromWorkspace1_PWORK;              /* '<Root>/From Workspace1' */

  struct {
    void *TimePtr;
    void *DataPtr;
    void *RSimInfoPtr;
  } FromWorkspace_PWORK;               /* '<Root>/From Workspace' */

  struct {
    int_T PrevIndex;
  } FromWorkspace1_IWORK;              /* '<Root>/From Workspace1' */

  struct {
    int_T PrevIndex;
  } FromWorkspace_IWORK;               /* '<Root>/From Workspace' */

  uint16_T Counter1_Count;             /* '<S3>/Counter1' */
} DW_StepCounter_T;

/* Continuous states (default storage) */
typedef struct {
  real_T Integrator_CSTATE[3];         /* '<S2>/Integrator' */
  real_T TransferFcn_CSTATE;           /* '<S1>/Transfer Fcn' */
  real_T TransferFcn1_CSTATE;          /* '<S1>/Transfer Fcn1' */
  real_T TransferFcn2_CSTATE;          /* '<S1>/Transfer Fcn2' */
} X_StepCounter_T;

/* State derivatives (default storage) */
typedef struct {
  real_T Integrator_CSTATE[3];         /* '<S2>/Integrator' */
  real_T TransferFcn_CSTATE;           /* '<S1>/Transfer Fcn' */
  real_T TransferFcn1_CSTATE;          /* '<S1>/Transfer Fcn1' */
  real_T TransferFcn2_CSTATE;          /* '<S1>/Transfer Fcn2' */
} XDot_StepCounter_T;

/* State disabled  */
typedef struct {
  boolean_T Integrator_CSTATE[3];      /* '<S2>/Integrator' */
  boolean_T TransferFcn_CSTATE;        /* '<S1>/Transfer Fcn' */
  boolean_T TransferFcn1_CSTATE;       /* '<S1>/Transfer Fcn1' */
  boolean_T TransferFcn2_CSTATE;       /* '<S1>/Transfer Fcn2' */
} XDis_StepCounter_T;

#ifndef ODE3_INTG
#define ODE3_INTG

/* ODE3 Integration Data */
typedef struct {
  real_T *y;                           /* output */
  real_T *f[3];                        /* derivatives */
} ODE3_IntgData;

#endif

/* Real-time Model Data Structure */
struct tag_RTM_StepCounter_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_StepCounter_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  XDis_StepCounter_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[6];
  real_T odeF[3][6];
  ODE3_IntgData intgData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    time_T stepSize0;
    uint32_T clockTick1;
    struct {
      uint8_T TID[2];
    } TaskCounters;

    time_T tStart;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Block signals (default storage) */
extern B_StepCounter_T StepCounter_B;

/* Continuous states (default storage) */
extern X_StepCounter_T StepCounter_X;

/* Disabled states (default storage) */
extern XDis_StepCounter_T StepCounter_XDis;

/* Block states (default storage) */
extern DW_StepCounter_T StepCounter_DW;

/* Model entry point functions */
extern void StepCounter_initialize(void);
extern void StepCounter_step(void);
extern void StepCounter_terminate(void);

/* Real-time Model object */
extern RT_MODEL_StepCounter_T *const StepCounter_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<Root>/Gain' : Unused code path elimination
 * Block '<Root>/Gain1' : Unused code path elimination
 * Block '<Root>/Scope' : Unused code path elimination
 * Block '<Root>/Scope2' : Unused code path elimination
 * Block '<Root>/Scope3' : Unused code path elimination
 * Block '<Root>/Scope5' : Unused code path elimination
 * Block '<Root>/Scope6' : Unused code path elimination
 * Block '<Root>/Scope8' : Unused code path elimination
 * Block '<S3>/Counter' : Unused code path elimination
 * Block '<S3>/Zero' : Unused code path elimination
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
 * '<Root>' : 'StepCounter'
 * '<S1>'   : 'StepCounter/filtering of the gyro'
 * '<S2>'   : 'StepCounter/gravity direction approximator'
 * '<S3>'   : 'StepCounter/step counter'
 * '<S4>'   : 'StepCounter/gravity direction approximator/angle extrapolation'
 * '<S5>'   : 'StepCounter/gravity direction approximator/obtainment of the trajectory of g'
 * '<S6>'   : 'StepCounter/step counter/MATLAB Function'
 * '<S7>'   : 'StepCounter/step counter/MATLAB Function1'
 */
#endif                                 /* StepCounter_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

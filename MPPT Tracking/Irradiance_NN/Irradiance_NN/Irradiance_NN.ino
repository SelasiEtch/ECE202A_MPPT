#include "Trained_Irradiation_NN.h"
#include "Trained_Irradiation_NN_private.h"
#include "Trained_Irradiation_NN_types.h"

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}


/*
 * Trained_Irradiation_NN.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "Trained_Irradiation_NN".
 *
 * Model version              : 1.1
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C source code generated on : Tue Nov  8 09:32:18 2022
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "Trained_Irradiation_NN.h"
#include "rtwtypes.h"
#include <math.h>
#include "Trained_Irradiation_NN_private.h"
#include "rt_nonfinite.h"

/* Real-time model */
static RT_MODEL_Trained_Irradiation__T Trained_Irradiation_NN_M_;
RT_MODEL_Trained_Irradiation__T *const Trained_Irradiation_NN_M =
  &Trained_Irradiation_NN_M_;

/* Model step function */
void Trained_Irradiation_NN_step(void)
{
  real_T tmp[10];
  real_T rtb_Addminy_d;
  real_T rtb_DotProduct;
  real_T rtb_DotProduct_0;
  real_T rtb_DotProduct_a_0;
  real_T rtb_DotProduct_f_0;
  real_T rtb_DotProduct_fc_0;
  real_T rtb_DotProduct_g_0;
  real_T rtb_DotProduct_j_0;
  real_T rtb_DotProduct_k_0;
  real_T rtb_DotProduct_ku_0;
  real_T rtb_DotProduct_m_0;
  real_T tmp_0;
  int32_T i;

  /* Gain: '<S23>/range y // range x' */
  rtb_DotProduct = Trained_Irradiation_NN_P.mapminmax_ymax -
    Trained_Irradiation_NN_P.mapminmax_ymin;

  /* Bias: '<S23>/Add min y' incorporates:
   *  Bias: '<S23>/Subtract min x'
   *  Constant: '<Root>/x1'
   *  Gain: '<S23>/range y // range x'
   */
  rtb_Addminy_d = rtb_DotProduct / (Trained_Irradiation_NN_P.mapminmax_xmax[0] -
    Trained_Irradiation_NN_P.mapminmax_xmin[0]) *
    (Trained_Irradiation_NN_P.x1_Value[0] -
     Trained_Irradiation_NN_P.mapminmax_xmin[0]) +
    Trained_Irradiation_NN_P.mapminmax_ymin;

  /* DotProduct: '<S9>/Dot Product' incorporates:
   *  Constant: '<S7>/IW{1,1}(1,:)''
   */
  rtb_DotProduct_0 = Trained_Irradiation_NN_P.IW111_Value[0] * rtb_Addminy_d;

  /* DotProduct: '<S11>/Dot Product' incorporates:
   *  Constant: '<S7>/IW{1,1}(2,:)''
   */
  rtb_DotProduct_k_0 = Trained_Irradiation_NN_P.IW112_Value[0] * rtb_Addminy_d;

  /* DotProduct: '<S12>/Dot Product' incorporates:
   *  Constant: '<S7>/IW{1,1}(3,:)''
   */
  rtb_DotProduct_g_0 = Trained_Irradiation_NN_P.IW113_Value[0] * rtb_Addminy_d;

  /* DotProduct: '<S13>/Dot Product' incorporates:
   *  Constant: '<S7>/IW{1,1}(4,:)''
   */
  rtb_DotProduct_m_0 = Trained_Irradiation_NN_P.IW114_Value[0] * rtb_Addminy_d;

  /* DotProduct: '<S14>/Dot Product' incorporates:
   *  Constant: '<S7>/IW{1,1}(5,:)''
   */
  rtb_DotProduct_a_0 = Trained_Irradiation_NN_P.IW115_Value[0] * rtb_Addminy_d;

  /* DotProduct: '<S15>/Dot Product' incorporates:
   *  Constant: '<S7>/IW{1,1}(6,:)''
   */
  rtb_DotProduct_f_0 = Trained_Irradiation_NN_P.IW116_Value[0] * rtb_Addminy_d;

  /* DotProduct: '<S16>/Dot Product' incorporates:
   *  Constant: '<S7>/IW{1,1}(7,:)''
   */
  rtb_DotProduct_fc_0 = Trained_Irradiation_NN_P.IW117_Value[0] * rtb_Addminy_d;

  /* DotProduct: '<S17>/Dot Product' incorporates:
   *  Constant: '<S7>/IW{1,1}(8,:)''
   */
  rtb_DotProduct_j_0 = Trained_Irradiation_NN_P.IW118_Value[0] * rtb_Addminy_d;

  /* DotProduct: '<S18>/Dot Product' incorporates:
   *  Constant: '<S7>/IW{1,1}(9,:)''
   */
  rtb_DotProduct_ku_0 = Trained_Irradiation_NN_P.IW119_Value[0] * rtb_Addminy_d;

  /* DotProduct: '<S10>/Dot Product' incorporates:
   *  Constant: '<S7>/IW{1,1}(10,:)''
   */
  tmp_0 = Trained_Irradiation_NN_P.IW1110_Value[0] * rtb_Addminy_d;

  /* Bias: '<S23>/Add min y' incorporates:
   *  Bias: '<S23>/Subtract min x'
   *  Constant: '<Root>/x1'
   *  Gain: '<S23>/range y // range x'
   */
  rtb_Addminy_d = rtb_DotProduct / (Trained_Irradiation_NN_P.mapminmax_xmax[1] -
    Trained_Irradiation_NN_P.mapminmax_xmin[1]) *
    (Trained_Irradiation_NN_P.x1_Value[1] -
     Trained_Irradiation_NN_P.mapminmax_xmin[1]) +
    Trained_Irradiation_NN_P.mapminmax_ymin;

  /* DotProduct: '<S9>/Dot Product' incorporates:
   *  Constant: '<S7>/IW{1,1}(1,:)''
   */
  rtb_DotProduct_0 += Trained_Irradiation_NN_P.IW111_Value[1] * rtb_Addminy_d;

  /* DotProduct: '<S11>/Dot Product' incorporates:
   *  Constant: '<S7>/IW{1,1}(2,:)''
   */
  rtb_DotProduct_k_0 += Trained_Irradiation_NN_P.IW112_Value[1] * rtb_Addminy_d;

  /* DotProduct: '<S12>/Dot Product' incorporates:
   *  Constant: '<S7>/IW{1,1}(3,:)''
   */
  rtb_DotProduct_g_0 += Trained_Irradiation_NN_P.IW113_Value[1] * rtb_Addminy_d;

  /* DotProduct: '<S13>/Dot Product' incorporates:
   *  Constant: '<S7>/IW{1,1}(4,:)''
   */
  rtb_DotProduct_m_0 += Trained_Irradiation_NN_P.IW114_Value[1] * rtb_Addminy_d;

  /* DotProduct: '<S14>/Dot Product' incorporates:
   *  Constant: '<S7>/IW{1,1}(5,:)''
   */
  rtb_DotProduct_a_0 += Trained_Irradiation_NN_P.IW115_Value[1] * rtb_Addminy_d;

  /* DotProduct: '<S15>/Dot Product' incorporates:
   *  Constant: '<S7>/IW{1,1}(6,:)''
   */
  rtb_DotProduct_f_0 += Trained_Irradiation_NN_P.IW116_Value[1] * rtb_Addminy_d;

  /* DotProduct: '<S16>/Dot Product' incorporates:
   *  Constant: '<S7>/IW{1,1}(7,:)''
   */
  rtb_DotProduct_fc_0 += Trained_Irradiation_NN_P.IW117_Value[1] * rtb_Addminy_d;

  /* DotProduct: '<S17>/Dot Product' incorporates:
   *  Constant: '<S7>/IW{1,1}(8,:)''
   */
  rtb_DotProduct_j_0 += Trained_Irradiation_NN_P.IW118_Value[1] * rtb_Addminy_d;

  /* DotProduct: '<S18>/Dot Product' incorporates:
   *  Constant: '<S7>/IW{1,1}(9,:)''
   */
  rtb_DotProduct_ku_0 += Trained_Irradiation_NN_P.IW119_Value[1] * rtb_Addminy_d;

  /* DotProduct: '<S10>/Dot Product' incorporates:
   *  Constant: '<S7>/IW{1,1}(10,:)''
   */
  tmp_0 += Trained_Irradiation_NN_P.IW1110_Value[1] * rtb_Addminy_d;

  /* Bias: '<S23>/Add min y' incorporates:
   *  Bias: '<S23>/Subtract min x'
   *  Constant: '<Root>/x1'
   *  Gain: '<S23>/range y // range x'
   */
  rtb_Addminy_d = rtb_DotProduct / (Trained_Irradiation_NN_P.mapminmax_xmax[2] -
    Trained_Irradiation_NN_P.mapminmax_xmin[2]) *
    (Trained_Irradiation_NN_P.x1_Value[2] -
     Trained_Irradiation_NN_P.mapminmax_xmin[2]) +
    Trained_Irradiation_NN_P.mapminmax_ymin;

  /* DotProduct: '<S10>/Dot Product' incorporates:
   *  Constant: '<S7>/IW{1,1}(10,:)''
   */
  rtb_DotProduct = Trained_Irradiation_NN_P.IW1110_Value[2] * rtb_Addminy_d +
    tmp_0;

  /* Sum: '<S2>/netsum' incorporates:
   *  Constant: '<S2>/b{1}'
   *  Constant: '<S7>/IW{1,1}(1,:)''
   *  Constant: '<S7>/IW{1,1}(2,:)''
   *  Constant: '<S7>/IW{1,1}(3,:)''
   *  Constant: '<S7>/IW{1,1}(4,:)''
   *  Constant: '<S7>/IW{1,1}(5,:)''
   *  Constant: '<S7>/IW{1,1}(6,:)''
   *  Constant: '<S7>/IW{1,1}(7,:)''
   *  Constant: '<S7>/IW{1,1}(8,:)''
   *  Constant: '<S7>/IW{1,1}(9,:)''
   *  DotProduct: '<S11>/Dot Product'
   *  DotProduct: '<S12>/Dot Product'
   *  DotProduct: '<S13>/Dot Product'
   *  DotProduct: '<S14>/Dot Product'
   *  DotProduct: '<S15>/Dot Product'
   *  DotProduct: '<S16>/Dot Product'
   *  DotProduct: '<S17>/Dot Product'
   *  DotProduct: '<S18>/Dot Product'
   *  DotProduct: '<S9>/Dot Product'
   *  Gain: '<S8>/Gain'
   */
  tmp[0] = ((Trained_Irradiation_NN_P.IW111_Value[2] * rtb_Addminy_d +
             rtb_DotProduct_0) + Trained_Irradiation_NN_P.b1_Value[0]) *
    Trained_Irradiation_NN_P.Gain_Gain;
  tmp[1] = ((Trained_Irradiation_NN_P.IW112_Value[2] * rtb_Addminy_d +
             rtb_DotProduct_k_0) + Trained_Irradiation_NN_P.b1_Value[1]) *
    Trained_Irradiation_NN_P.Gain_Gain;
  tmp[2] = ((Trained_Irradiation_NN_P.IW113_Value[2] * rtb_Addminy_d +
             rtb_DotProduct_g_0) + Trained_Irradiation_NN_P.b1_Value[2]) *
    Trained_Irradiation_NN_P.Gain_Gain;
  tmp[3] = ((Trained_Irradiation_NN_P.IW114_Value[2] * rtb_Addminy_d +
             rtb_DotProduct_m_0) + Trained_Irradiation_NN_P.b1_Value[3]) *
    Trained_Irradiation_NN_P.Gain_Gain;
  tmp[4] = ((Trained_Irradiation_NN_P.IW115_Value[2] * rtb_Addminy_d +
             rtb_DotProduct_a_0) + Trained_Irradiation_NN_P.b1_Value[4]) *
    Trained_Irradiation_NN_P.Gain_Gain;
  tmp[5] = ((Trained_Irradiation_NN_P.IW116_Value[2] * rtb_Addminy_d +
             rtb_DotProduct_f_0) + Trained_Irradiation_NN_P.b1_Value[5]) *
    Trained_Irradiation_NN_P.Gain_Gain;
  tmp[6] = ((Trained_Irradiation_NN_P.IW117_Value[2] * rtb_Addminy_d +
             rtb_DotProduct_fc_0) + Trained_Irradiation_NN_P.b1_Value[6]) *
    Trained_Irradiation_NN_P.Gain_Gain;
  tmp[7] = ((Trained_Irradiation_NN_P.IW118_Value[2] * rtb_Addminy_d +
             rtb_DotProduct_j_0) + Trained_Irradiation_NN_P.b1_Value[7]) *
    Trained_Irradiation_NN_P.Gain_Gain;
  tmp[8] = ((Trained_Irradiation_NN_P.IW119_Value[2] * rtb_Addminy_d +
             rtb_DotProduct_ku_0) + Trained_Irradiation_NN_P.b1_Value[8]) *
    Trained_Irradiation_NN_P.Gain_Gain;
  tmp[9] = (rtb_DotProduct + Trained_Irradiation_NN_P.b1_Value[9]) *
    Trained_Irradiation_NN_P.Gain_Gain;

  /* DotProduct: '<S22>/Dot Product' incorporates:
   *  Constant: '<S20>/IW{2,1}(1,:)''
   *  Constant: '<S8>/one'
   *  Constant: '<S8>/one1'
   *  Gain: '<S8>/Gain1'
   *  Math: '<S8>/Exp'
   *  Math: '<S8>/Reciprocal'
   *  Sum: '<S8>/Sum'
   *  Sum: '<S8>/Sum1'
   *
   * About '<S8>/Exp':
   *  Operator: exp
   *
   * About '<S8>/Reciprocal':
   *  Operator: reciprocal
   */
  tmp_0 = 0.0;
  for (i = 0; i < 10; i++) {
    tmp_0 += (1.0 / (exp(tmp[i]) + Trained_Irradiation_NN_P.one_Value) *
              Trained_Irradiation_NN_P.Gain1_Gain -
              Trained_Irradiation_NN_P.one1_Value) *
      Trained_Irradiation_NN_P.IW211_Value[i];
  }

  /* Bias: '<S24>/Add min x' incorporates:
   *  Bias: '<S24>/Subtract min y'
   *  Constant: '<S3>/b{2}'
   *  DotProduct: '<S22>/Dot Product'
   *  Gain: '<S24>/Divide by range y'
   *  Sum: '<S3>/netsum'
   */
  rtb_DotProduct = (Trained_Irradiation_NN_P.mapminmax_reverse_xmax -
                    Trained_Irradiation_NN_P.mapminmax_reverse_xmin) /
    (Trained_Irradiation_NN_P.mapminmax_reverse_ymax -
     Trained_Irradiation_NN_P.mapminmax_reverse_ymin) * ((tmp_0 +
    Trained_Irradiation_NN_P.b2_Value) -
    Trained_Irradiation_NN_P.mapminmax_reverse_ymin) +
    Trained_Irradiation_NN_P.mapminmax_reverse_xmin;

  /* Matfile logging */
  rt_UpdateTXYLogVars(Trained_Irradiation_NN_M->rtwLogInfo,
                      (&Trained_Irradiation_NN_M->Timing.taskTime0));

  /* signal main to stop simulation */
  {                                    /* Sample time: [1.0s, 0.0s] */
    if ((rtmGetTFinal(Trained_Irradiation_NN_M)!=-1) &&
        !((rtmGetTFinal(Trained_Irradiation_NN_M)-
           Trained_Irradiation_NN_M->Timing.taskTime0) >
          Trained_Irradiation_NN_M->Timing.taskTime0 * (DBL_EPSILON))) {
      rtmSetErrorStatus(Trained_Irradiation_NN_M, "Simulation finished");
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
  if (!(++Trained_Irradiation_NN_M->Timing.clockTick0)) {
    ++Trained_Irradiation_NN_M->Timing.clockTickH0;
  }

  Trained_Irradiation_NN_M->Timing.taskTime0 =
    Trained_Irradiation_NN_M->Timing.clockTick0 *
    Trained_Irradiation_NN_M->Timing.stepSize0 +
    Trained_Irradiation_NN_M->Timing.clockTickH0 *
    Trained_Irradiation_NN_M->Timing.stepSize0 * 4294967296.0;
}

/* Model initialize function */
void Trained_Irradiation_NN_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)Trained_Irradiation_NN_M, 0,
                sizeof(RT_MODEL_Trained_Irradiation__T));
  rtmSetTFinal(Trained_Irradiation_NN_M, 10.0);
  Trained_Irradiation_NN_M->Timing.stepSize0 = 1.0;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = (NULL);
    Trained_Irradiation_NN_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(Trained_Irradiation_NN_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(Trained_Irradiation_NN_M->rtwLogInfo, (NULL));
    rtliSetLogT(Trained_Irradiation_NN_M->rtwLogInfo, "tout");
    rtliSetLogX(Trained_Irradiation_NN_M->rtwLogInfo, "");
    rtliSetLogXFinal(Trained_Irradiation_NN_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(Trained_Irradiation_NN_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(Trained_Irradiation_NN_M->rtwLogInfo, 4);
    rtliSetLogMaxRows(Trained_Irradiation_NN_M->rtwLogInfo, 0);
    rtliSetLogDecimation(Trained_Irradiation_NN_M->rtwLogInfo, 1);
    rtliSetLogY(Trained_Irradiation_NN_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(Trained_Irradiation_NN_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(Trained_Irradiation_NN_M->rtwLogInfo, (NULL));
  }

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(Trained_Irradiation_NN_M->rtwLogInfo, 0.0,
    rtmGetTFinal(Trained_Irradiation_NN_M),
    Trained_Irradiation_NN_M->Timing.stepSize0, (&rtmGetErrorStatus
    (Trained_Irradiation_NN_M)));
}

/* Model terminate function */
void Trained_Irradiation_NN_terminate(void)
{
  /* (no terminate code required) */
}


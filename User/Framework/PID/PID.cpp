
// Created by 42038 on 2026/8/10.
//

/*
 *初始化
 */
#include "PID.hpp"

void PIDc::PID_Init(PID_t *WhichPID,float Kp,float Ki,float Kd)
{
    WhichPID->Kp1 = Kp;
    WhichPID->Ki1 = Ki;
    WhichPID->Kd1 = Kd;

    WhichPID->PID_Err_now=0;
    WhichPID->PID_Err_last=0;
    WhichPID->PID_Err_all=0;

    WhichPID->PID_Out=0;
    WhichPID->PID_lastout=0;
    WhichPID->PID_Target=0;//目标输入
    WhichPID->PID_Input=0;//反馈量

    WhichPID->PID_Precision = PID_DEFAULT_PRECISION;
    WhichPID->PID_ErrAllMax = PID_DEFAULT_ERRALL_MAX;
    WhichPID->PID_OutMax = PID_DEFAULT_OUTPUT_MAX;
    WhichPID->PID_OutStep = PID_DEFAULT_OUTPUT_STEP_MAX;
}

/*
 *PID更新
 */


void PIDc::PID_Update(PID_t *WhichPID,float NowInput,float Target)
{
    WhichPID->PID_Input = NowInput;
    WhichPID->PID_Target = Target;

    WhichPID->PID_Err_last = WhichPID->PID_Err_now;//差值更新
    WhichPID->PID_Err_now = WhichPID->PID_Target - WhichPID->PID_Input;//误差更新

    if (WhichPID->PID_Err_now < WhichPID->PID_Precision && WhichPID->PID_Err_now > -WhichPID->PID_Precision){
        WhichPID->PID_Err_now = 0;
    }

    WhichPID->PID_Err_all += WhichPID->PID_Err_now;//求和

    if (WhichPID->PID_Err_all > WhichPID->PID_ErrAllMax){
        WhichPID->PID_Err_all = WhichPID->PID_ErrAllMax;
    }
    else if (WhichPID->PID_Err_all < -WhichPID->PID_ErrAllMax){
        WhichPID->PID_Err_all = -WhichPID->PID_ErrAllMax;
    }
}

/*
 *PID计算
 */

float PIDc::PID_Calculate(PID_t *WhichPID){
    WhichPID->PID_Out =
        WhichPID->Kp1 * WhichPID->PID_Err_now +
        WhichPID->Kd1 * (WhichPID->PID_Err_now - WhichPID->PID_Err_last)+
            (WhichPID->PID_Err_all * WhichPID->Ki1);

    if (WhichPID->PID_Out >= WhichPID->PID_OutMax)
        WhichPID->PID_Out = WhichPID->PID_OutMax;//pid总限幅
    if (WhichPID->PID_Out <= -WhichPID->PID_OutMax)
        WhichPID->PID_Out = -WhichPID->PID_OutMax;

    if (WhichPID->PID_Out - WhichPID->PID_lastout > WhichPID->PID_OutStep)//步长限幅
        WhichPID->PID_Out = WhichPID->PID_lastout + WhichPID->PID_OutStep;
    if (WhichPID->PID_Out - WhichPID->PID_lastout < -WhichPID->PID_OutStep)
        WhichPID->PID_Out = WhichPID->PID_lastout + -WhichPID->PID_OutStep;

    WhichPID->PID_lastout = WhichPID->PID_Out;
    return WhichPID->PID_Out;
}

/*
 *串级PID
 */

float PIDc::Pos_Spd_PID(PID_t *Spdparam,PID_t *Posparam, float Pos_Target, float Pos_Input, const float Spd_Input){
    PID_Update(Posparam,Pos_Input,Pos_Target);
    PID_Calculate(Posparam);//位置环PID
    PID_Update(Spdparam,Spd_Input,Posparam->PID_Out);  //Target如此赋值是为让位置环的Error_Now和input相等，Error_Now=Target - NowInput
    PID_Calculate(Spdparam);//速度环PID
    return Spdparam->PID_Out;
}

/*
 * 单环PID
 */
float PIDc::SingleLoop_PID(PID_t *Which_PID, float Pos_Input, float Pos_Target){
    PID_Update(Which_PID,Pos_Input,Pos_Target);
    PID_Calculate(Which_PID);

    return Which_PID->PID_Out;
}

/*
 * 清零所有PID数据
 */
void PIDc::PID_Clear(){
    SpdParam.PID_Err_now = 0.0f;
    SpdParam.PID_Err_all = 0.0f;
    SpdParam.PID_Err_last = 0.0f;
    SpdParam.PID_Out = 0.0f;

    PosParam.PID_Err_now = 0.0f;
    PosParam.PID_Err_all = 0.0f;
    PosParam.PID_Err_last = 0.0f;
    PosParam.PID_Out = 0.0f;
}
//
// Created by 42038 on 2026/8/15.

#include "Motor.hpp"

Motorc Yaw(yaw,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f);
Motorc Pitch(pitch,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f);

/*
 * 定义class时写入相应变量值
 */
Motorc::Motorc(Motor_Name_t name,
    float p_kp, float p_ki, float p_kd,
    float s_kp, float s_ki, float s_kd)
:Motor_Name(name),
    Command_Data(0),Target_Data(0.0f),Angle_min(0.0f),Angle_max(0.0f){
    PID.PID_Init(&PID.PosParam,p_kp,p_ki,p_kd);
    PID.PID_Init(&PID.SpdParam,s_kp,s_ki,s_kd);
}

/*
 * 电机初始化，设置对应角度限位值
 */
void Motorc::Init(float angle_min,float angle_max){
    Angle_min = angle_min;
    Angle_max = angle_max;
}

/*
 * 电机控制环，读取目标值和电机反馈值、PID计算等环节
 */
void Motorc::Loop(float target){
    Get_Data(target);
    Angle_Clamp();
    Motor_Calculate();
}

/*
 * 将发送给电机的值封装成一个函数方便读取
 */
int16_t Motorc::Command_Send(){
    return Command_Data;
}

/*
 * @brief 读取CAN接收的电机反馈值和电机目标值
 */
void Motorc::Get_Data(float target){
    Motor_Data_Read(&Receive_Data,Motor_Name);
    Target_Data = target;
}

/*
 * 电机目标值限
 * 如果电机的实际角度值超限位，也在前面回调函数那里直接进保护模式
 *
 */
void Motorc::Angle_Clamp(){
    Target_Data = std::clamp(Target_Data,Angle_min,Angle_max);
}

/*
 * 电机保护函数，CAN指令清零，目标值清零
 *
 */
void Motorc::Protect(){
    Target_Data = Receive_Data.Angle;
    Command_Data = 0;
    PID.PID_Clear();
}

/*
 * 串级PID运算
 */
void Motorc::Motor_Calculate(){
    Command_Data = (int16_t)PID.Pos_Spd_PID(&PID.SpdParam, &PID.PosParam,Target_Data,Receive_Data.Angle,Receive_Data.Speed);
}
//
// Created by 42038 on 2026/8/15.

#include "Motor.hpp"

Motorc Yaw(yaw,10.0f,0.0f,0.0f,10.0f,0.0f,0.0f);
Motorc Pitch(pitch,10.0f,0.0f,0.0f,10.0f,0.0f,0.0f);

/*
 * 定义class时写入相应变量值
 */
Motorc::Motorc(Motor_Name_t name,
    float p_kp, float p_ki, float p_kd,
    float s_kp, float s_ki, float s_kd)
: Motor_Name(name),Command_Data(0),Target_Data(0.0f),Angle_min(0.0f),Angle_max(0.0f),Zero_Offset(0.0f){
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
float Motorc::Get_Mechanical_Angle()
{
    float angle =
        ((float)Receive_Data.Angle - Zero_Offset)
        * 360.0f / 8192.0f;
    // 转换到 [-180, 180]
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}
/*
 * 电机控制环，读取目标值和电机反馈值、PID计算等环节
 */
void Motorc::Loop(float target){
    Get_Data(target);
    if (Work_Mode == MOTOR_SPEED)
    {Target_Speed = target;}
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
 * 计算过圈最短路径误差 (-180-180度)
 */
float Motorc::Calculate_Shortest_Path_Err(
    float target,
    float current)
{float err = target - current;
    while (err > 180.0f)
        err -= 360.0f;
    while (err < -180.0f)
        err += 360.0f;
    return err;
}
/*
 * 串级PID运算
 */
void Motorc::Motor_Calculate() {
    switch (Work_Mode) {
        case MOTOR_PROTECT: {
            Command_Data = 0;
            PID.PID_Clear();
            break;
        }
        case MOTOR_SPEED: {
            // 单环速度 PID
            PID.PID_Update(&PID.SpdParam, Receive_Data.Speed, Target_Speed);
            Command_Data = (int16_t)PID.PID_Calculate(&PID.SpdParam);
            break;
        }
        case MOTOR_POSITION: {
            // 位置模式
            float real_angle = Get_Mechanical_Angle();
            float angle_err =Calculate_Shortest_Path_Err(Target_Data,real_angle);
            // 将最短误差注入位置环计算
            PID.PID_Update(&PID.PosParam, 0.0f, angle_err); // 此时 Target=angle_err, Input=0
            PID.PID_Calculate(&PID.PosParam);

            // 速度环跟随位置环输出
            PID.PID_Update(&PID.SpdParam, Receive_Data.Speed, PID.PosParam.PID_Out);
            Command_Data = (int16_t)PID.PID_Calculate(&PID.SpdParam);
            break;
        }
        default: {
            Command_Data = 0;
            PID.PID_Clear();
            break;
        }
    }
}
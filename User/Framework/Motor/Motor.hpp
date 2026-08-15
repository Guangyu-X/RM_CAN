//
// Created by 42038 on 2026/8/15.
//

#ifndef RM_CAN_MOTOR_HPP
#define RM_CAN_MOTOR_HPP

#include "stm32f4xx.h"
#include "PID.hpp"
#include "CAN.hpp"
#include <algorithm>

class Motorc {
private:
    PIDc PID;    //PID的类
    Motor_Receive_Data_t Receive_Data;//接收电机反馈报文
    Motor_Name_t Motor_Name;//电机类型的枚举变量
    int16_t Command_Data = 0;//can发送给电机的参数
    float Target_Data = 0.0f;//角度闭环的目标值
    float Angle_min = 0.0f;//角度最大值限幅
    float Angle_max = 0.0f;//角度最小值限幅
    void Get_Data(float target);
    void Angle_Clamp();
    void Motor_Calculate();
public:
    Motorc(Motor_Name_t name,
           float p_kp, float p_ki, float p_kd,
           float s_kp, float s_ki, float s_kd);
    void Init(float angle_min,float angle_max);
    void Loop(float target);
    int16_t Command_Send();
    void Protect();
};

extern Motorc Yaw,Pitch;

#endif //RM_CAN_MOTOR_HPP
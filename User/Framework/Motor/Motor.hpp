//
// Created by 42038 on 2026/8/15.
//

#ifndef RM_CAN_MOTOR_HPP
#define RM_CAN_MOTOR_HPP

#include "stm32f4xx.h"
#include "PID.hpp"
#include "CAN.hpp"
#include <algorithm>

typedef enum {
    MOTOR_PROTECT = 0, // 保护模式
    MOTOR_SPEED, // 速度模式单环速度PID
    MOTOR_POSITION // 位置模式串级PID+最短路径
} Motor_Mode_t;

class Motorc {
private:
    Motor_Mode_t Work_Mode = MOTOR_PROTECT;
    float Target_Speed = 0.0f;
    // 电机编码器零点
    float Zero_Offset = 0.0f;
    // 机械角度限位
    float Angle_min = 0.0f;
    float Angle_max = 0.0f;
    float Calculate_Shortest_Path_Err(float target, float current);
    PIDc PID;
    Motor_Receive_Data_t Receive_Data{};
    Motor_Name_t Motor_Name;
    int16_t Command_Data = 0;
    float Target_Data = 0.0f;
    void Get_Data(float target);
    void Angle_Clamp();
    void Motor_Calculate();
public:
    Motorc(Motor_Name_t name,
           float p_kp, float p_ki, float p_kd,
           float s_kp, float s_ki, float s_kd);
    void Set_Mode(Motor_Mode_t mode) {Work_Mode = mode;}
    void Set_Speed_Target(float spd) {Target_Speed = spd;}
    void Init(float angle_min, float angle_max);
    // 设置编码器零点
    void Set_Zero_Offset(float offset) {Zero_Offset = offset;}
    void Loop(float target);
    int16_t Command_Send();
    void Protect();
    float Get_Target_Angle() {return Target_Data;}
    float Get_Real_Angle() {return Get_Mechanical_Angle();}
    float Get_Target_Speed() {return Target_Speed;}
    float Get_Real_Speed() {return (float)Receive_Data.Speed;}
    float Get_Mechanical_Angle();
    uint16_t Get_Encoder_Angle() {return Receive_Data.Angle;}
};

extern Motorc Yaw,Pitch;

#endif //RM_CAN_MOTOR_HPP
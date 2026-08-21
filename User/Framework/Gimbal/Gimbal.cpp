//
// Created by 42038 on 2026/8/15.
//
#include "Gimbal.hpp"
#include "Remote.hpp"

Gimbalc Gimbal;

/*
 * @brief 初始化云台，包括设置yaw和pitch的角度软件限位
 */
void Gimbalc::Init() {
    Yaw.Init(-180.0f,180.0f);
    Pitch.Init(-180.0f,180.0f);
    Yaw.Set_Zero_Offset(4096.0f);
    Pitch.Set_Zero_Offset(4096.0f);
}

/*
 * @brief 将两个电机的数据拼接成一段CAN报文
 * 拼接顺序是按照先取低再取高，写零是因为没这两ID的电机
 * 电流模式
 */
void Gimbalc::Command_Send(){
    Can_Send(COMMAND_ID,Pitch.Command_Send(),Yaw.Command_Send(),0,0);
}

/*
 * 获得yaw和pitch的目标值
 */
void Gimbalc::Get_Target() {
    Yaw_Target = Remote.Yaw_Target_Vofa;
    Pitch_Target = Remote.Pitch_Target_Vofa;
    // 根据上位机传来的模式编号，动态设置 Yaw 电机的工作模式
    switch (Remote.Mode_Vofa) {
        case 0:
        {Yaw.Set_Mode(MOTOR_PROTECT);
            Pitch.Set_Mode(MOTOR_PROTECT);
            break;}

        case 1:
        {Yaw.Set_Mode(MOTOR_SPEED);
            Pitch.Set_Mode(MOTOR_SPEED);
            break;}

        case 2:
        {Yaw.Set_Mode(MOTOR_POSITION);
            Pitch.Set_Mode(MOTOR_POSITION);
            break;}

        default:
        {Yaw.Set_Mode(MOTOR_PROTECT);
            Pitch.Set_Mode(MOTOR_PROTECT);
            break;}
    }
}


/*
 * 云台控制环，控制Yaw和Pitch
 * Yaw_Target和Pitch_Target目前是直接写到private里面的，后需要用上位机赋值
 * 保护模式添加到CAN通信回调函数那里
 */
void Gimbalc::Loop() {
    if(Gimbal_Status){  // 默认true，云台正常工作
        Get_Target();     // 这里面会更新目标值并切换模式
        Yaw.Loop(Yaw_Target);
        Pitch.Loop(Pitch_Target);
    }
    else{
        Protect();
    }
    Command_Send();
}
/*
 * 云台保护函数：当失能、过热或超限位时调用
 */
void Gimbalc::Protect(){
    Yaw.Protect();
    Pitch.Protect();
}
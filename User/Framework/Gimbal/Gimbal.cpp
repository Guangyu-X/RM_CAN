//
// Created by 42038 on 2026/8/15.
//
#include "Gimbal.hpp"

Gimbalc Gimbal;

/*
 * @brief 初始化云台，包括设置yaw和pitch的角度软件限位
 */
void Gimbalc::Init() {
    Yaw.Init(-90.0f,90.0f);
    Pitch.Init(-15.0f,45.0f);
}

/*
 * @brief 将两个电机的数据拼接成一段CAN报文
 * 拼接顺序是按照先取低再取高，写零是因为没这两ID的电机
 * 电流模式
 */
void Gimbalc::Command_Send(){
    Can_Send(COMMAND_ID,Yaw.Command_Send(),Pitch.Command_Send(),0,0);
}

/*
 * 获得yaw和pitch的目标值
 */
void Gimbalc::Get_Target(){
    Yaw_Target = 20.0f;
    Pitch_Target = 10.0f;;//先写死吧（）
}

void Gimbalc::Protect(){
    Yaw.Protect();
    Pitch.Protect();
}


/*
 * 云台控制环，控制Yaw和Pitch
 * Yaw_Target和Pitch_Target目前是直接写到private里面的，后需要用上位机赋值
 * 保护模式添加到CAN通信回调函数那里
 */
void Gimbalc::Loop() {
    if(Gimbal_Status){  //默认true，云台正常工作
        Get_Target();
        Yaw.Loop(Yaw_Target);
        Pitch.Loop(Pitch_Target);
    }
    else{//过热，超限位的时候给false，在回调函数赋值
        Protect();
    }
    Command_Send();
}
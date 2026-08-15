//
// Created by 42038 on 2026/8/15.
//

#ifndef RM_CAN_CAN_HPP
#define RM_CAN_CAN_HPP

#include "can.h"

#define PITCH_RECEIVE_ID  0x205
#define YAW_RECEIVE_ID    0x206

typedef struct {
    uint16_t Angle;
    int16_t Speed;
    int16_t Current;
    int16_t Temperature;
} Motor_Receive_Data_t;

enum Motor_Name_t {//枚举类型，列举电机类型
    yaw,
    pitch
};

void Can_Init();
void Can_Send(int16_t ID, int16_t Mess_1, int16_t Mess_2, int16_t Mess_3, int16_t Mess_4);
void Can_Receive();
void Motor_Data_Read(Motor_Receive_Data_t *data, Motor_Name_t name);

#endif //RM_CAN_CAN_HPP
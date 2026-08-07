//
// Created by 42038 on 2026/8/7.
//

#ifndef RM_CAN_CANIO_H
#define RM_CAN_CANIO_H

#include "stm32f4xx_hal.h"

#define YAW_RECEIVE_ID 0X206

void Can_Init();
void Can_Send(int16_t ID,int16_t Mess_1,int16_t Mess_2,int16_t Mess_3,int16_t Mess_4);
void Can_Receive();

typedef struct {
    uint16_t Angle;
    int16_t Speed;
    int16_t Current;
    uint8_t Temperature;
}Motor_Data;

#endif //RM_CAN_CANIO_H
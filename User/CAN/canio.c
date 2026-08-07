//
// Created by luvsic on 2026/8/7.
//

#include "canio.h"
#include "can.h"

void Can_Init(){
        CAN_FilterTypeDef can_filter_st;
        can_filter_st.FilterBank = 0;                  //选择过滤器序号
        can_filter_st.FilterActivation = CAN_FILTER_ENABLE;  //启用过滤器
        can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;    //使用掩码
        can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;   //选择过滤器长度，32Bit or 16Bit
        can_filter_st.FilterIdHigh = 0x0000;           //过滤器接受ID的高16位
        can_filter_st.FilterIdLow = 0x0000;            //过滤器接受ID的低16位
        can_filter_st.FilterMaskIdHigh = 0x0000;       //过滤器掩码的高16位
        can_filter_st.FilterMaskIdLow = 0x0000;        //过滤器掩码的低16位
        can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;  //使用FIFO0队列
        can_filter_st.SlaveStartFilterBank = 0;

        HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);       //写入CAN过滤器配置
        HAL_CAN_Start(&hcan1);                              //开启CAN外设
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);  //开启FIFO0队列
    }

void Can_Send(){

}

void Can_Receive(){

}
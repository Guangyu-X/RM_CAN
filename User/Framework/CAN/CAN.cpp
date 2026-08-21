//
// Created by 42038 on 2026/8/15.
//

#include "CAN.hpp"

#include "Debug.hpp"
#include "Gimbal.hpp"

uint8_t rx_buf[8];
Motor_Receive_Data_t Yaw_Data,Pitch_Data;

void Can_Init(){
        CAN_FilterTypeDef can_filter_st;
        can_filter_st.FilterBank = 0;//选择过滤器序号
        can_filter_st.FilterActivation = CAN_FILTER_ENABLE;//启用过滤器
        can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;//使用掩码
        can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;//选择过滤器长度，32Bit or 16Bit
        can_filter_st.FilterIdHigh = 0x0000;//过滤器接受ID的高16位
        can_filter_st.FilterIdLow = 0x0000;//过滤器接受ID的低16位
        can_filter_st.FilterMaskIdHigh = 0x0000;//过滤器掩码的高16位
        can_filter_st.FilterMaskIdLow = 0x0000;//过滤器掩码的低16位
        can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;//使用FIFO0队列
        can_filter_st.SlaveStartFilterBank = 0;
        HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);//写入CAN过滤器配置
        HAL_CAN_Start(&hcan1);//开启CAN外设
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);//开启FIFO0队列
    }

void Can_Send(int16_t ID,int16_t Mess_1,int16_t Mess_2,int16_t Mess_3,int16_t Mess_4){
    uint32_t Send_Mail_Box = 0;
    CAN_TxHeaderTypeDef Tx_Message;
    uint8_t can_send_message[8];

    Tx_Message.DLC = 0x08;//数据长度，对照6020手册
    Tx_Message.IDE = CAN_ID_STD;//标准帧
    Tx_Message.StdId = ID;
    Tx_Message.RTR = CAN_RTR_DATA;//遥控帧或数据帧

    can_send_message[0] = Mess_1 >> 8;//右移8位，16位的长度默认取低的八位，所以需要拆开
    can_send_message[1] = Mess_1;
    can_send_message[2] = Mess_2 >> 8;
    can_send_message[3] = Mess_2;
    can_send_message[4] = Mess_3 >> 8;
    can_send_message[5] = Mess_3;
    can_send_message[6] = Mess_4 >> 8;
    can_send_message[7] = Mess_4;

    HAL_StatusTypeDef tx_status = HAL_CAN_AddTxMessage(&hcan1, &Tx_Message, can_send_message, &Send_Mail_Box);
    if (tx_status != HAL_OK) {
        usart_printf("TX ID=0x%03X M1=%d M2=%d M3=%d M4=%d status=%d\r\n",
              ID,
              Mess_1,
              Mess_2,
              Mess_3,
              Mess_4,
              tx_status);
    }
}

void Can_Receive(){
    CAN_RxHeaderTypeDef rx_header;
    static int16_t ID;
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_buf);
    ID = (int16_t)rx_header.StdId;
    switch(ID)
    {
        case YAW_RECEIVE_ID:{
            Yaw_Data.Angle = rx_buf[0] << 8 | rx_buf[1];//与发送对应，这边是拼
            Yaw_Data.Speed = rx_buf[2] << 8 | rx_buf[3];//把高八位左移以后进行“按位或|”
            Yaw_Data.Current = rx_buf[4] << 8 | rx_buf[5];
            Yaw_Data.Temperature = rx_buf[6];
            break;
        }
        case PITCH_RECEIVE_ID:{
            Pitch_Data.Angle = rx_buf[0] << 8 | rx_buf[1];
            Pitch_Data.Speed = rx_buf[2] << 8 | rx_buf[3];
            Pitch_Data.Current = rx_buf[4] << 8 | rx_buf[5];
            Pitch_Data.Temperature = rx_buf[6];
            break;
        }
        default:
            break;
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    Can_Receive();//类似于串口那个callback
    if (Yaw_Data.Temperature > 80 || Pitch_Data.Temperature > 80
    ||std::abs((int)Yaw_Data.Speed) > 8000 ||std::abs((int)Pitch_Data.Speed) > 8000) // 绝对值
        {
        Gimbal.Gimbal_Status = false;
    } else {
        Gimbal.Gimbal_Status = true;
    }
}

void Motor_Data_Read(Motor_Receive_Data_t *data, Motor_Name_t name){
    switch (name){
        case Motor_Name_t::yaw:
            *data = Yaw_Data;
            break;
        case Motor_Name_t::pitch:
            *data = Pitch_Data;
            break;
        default:
            break;
    }
}
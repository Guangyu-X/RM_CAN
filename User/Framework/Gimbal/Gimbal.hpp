//
// Created by 42038 on 2026/8/15.
//

#ifndef RM_CAN_GIMBAL_HPP
#define RM_CAN_GIMBAL_HPP

#include "Motor.hpp"

#define COMMAND_ID   0x1FE//电流模式

class Gimbalc {
private:
    float Yaw_Target = 0.0f,Pitch_Target = 0.0f;
    void Command_Send();
    void Get_Target();
    void Protect();
public:
    bool Gimbal_Status = true; //云台正常工作时true，保护时false
    void Init();
    void Loop();
};

extern Gimbalc Gimbal;

#endif //RM_CAN_GIMBAL_HPP
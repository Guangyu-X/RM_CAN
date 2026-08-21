
// Created by 42038 on 2026/8/21.
//

#ifndef RM_CAN_REMOTE_HPP
#define RM_CAN_REMOTE_HPP
#include <sys/_stdint.h>

class Remotec {
public:
    float Yaw_Target_Vofa = 0.0f;
    float Pitch_Target_Vofa = 0.0f;
    int Mode_Vofa = 2; // 默认给 2（位置模式）

    void Vofa_Data_Process(char *rx_str);
};

extern Remotec Remote;
#ifdef __cplusplus
extern "C" {
#endif
    void Vofa_Data_Process_C(uint8_t *rx_buf, uint16_t size);
#ifdef __cplusplus
}
#endif
#endif //RM_CAN_REMOTE_HPP





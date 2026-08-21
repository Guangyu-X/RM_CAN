//
// Created by 42038 on 2026/8/15.
//

#include "Remote.hpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>

Remotec Remote;

/*
 * @brief 解析 Vofa+ 字符串，格式: "目标Yaw,目标Pitch,模式\n"
 */
void Remotec::Vofa_Data_Process(char *rx_str) {
    if (rx_str == nullptr) return;

    // 1. 提取第一个参数：Yaw 目标值
    char *token = strtok(rx_str, ",");
    if (token != nullptr) {
        Yaw_Target_Vofa = (float)atof(token);
    }

    // 2. 提取第二个参数：Pitch 目标值
    token = strtok(nullptr, ",");
    if (token != nullptr) {
        Pitch_Target_Vofa = (float)atof(token);
    }

    // 3. 提取第三个参数：模式选择
    token = strtok(nullptr, ",");
    if (token != nullptr) {
        Mode_Vofa = atoi(token);
    }
}

// C 桥接函数实现
extern "C" {
    void Vofa_Data_Process_C(uint8_t *rx_buf, uint16_t size) {
        if (rx_buf != nullptr) {
            rx_buf[size] = '\0'; // 加上字符串结束符
            Remote.Vofa_Data_Process((char *)rx_buf);
        }
    }
}
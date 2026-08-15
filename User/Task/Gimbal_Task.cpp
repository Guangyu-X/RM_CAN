//
// Created by 42038 on 2026/8/15.
//
#include "FreeRTOS.h"
#include "task.h"
#include "Gimbal.hpp"
#include "IWDG.hpp"

void Gimbal_Task(void const * argument)
{
    portTickType CurrentTime;//定义一个变量读取tick时间
    Gimbal.Init();
    for(;;)
    {
        CurrentTime = xTaskGetTickCount();//读取当前时间下tick
        Gimbal.Loop();//执行云台控制环
        Feed_Dog();//喂狗
        vTaskDelayUntil(&CurrentTime, 4 / portTICK_RATE_MS); //每4ms执行一次，将程序阻塞至CurrentTime+4ms
    }
}
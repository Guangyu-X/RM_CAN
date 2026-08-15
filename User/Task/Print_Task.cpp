//
// Created by 42038 on 2026/8/15.
//
#include "FreeRTOS.h"
#include "task.h"
#include "Debug.hpp"

/*
 * 串口打印任务
 */
void Print_Task(void const * argument)
{
    /* USER CODE BEGIN Print_Task */
    portTickType CurrentTime;
    /* Infinite loop */
    for(;;)
    {
        CurrentTime = xTaskGetTickCount();
        usart_printf("Hello RoboMaster\r\n");
        vTaskDelayUntil(&CurrentTime, 20 / portTICK_RATE_MS);//每20ms执行一次
    }
    /* USER CODE END Print_Task */
}
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
        usart_printf("RM2027\r\n");
        vTaskDelayUntil(&CurrentTime, 20 / portTICK_RATE_MS);//50Hz
    }
    /* USER CODE END Print_Task */
}
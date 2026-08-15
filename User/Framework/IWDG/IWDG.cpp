#include "IWDG.hpp"
#include "iwdg.h"
//
// Created by 42038 on 2026/8/15.
//
/*
 *喂狗喂狗
 */
 void Feed_Dog() {
     HAL_IWDG_Refresh(&hiwdg);
 }
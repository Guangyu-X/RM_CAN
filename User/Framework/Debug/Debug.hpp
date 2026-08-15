//
// Created by 42038 on 2026/8/15.
//

#ifndef RM_CAN_DEBUG_HPP
#define RM_CAN_DEBUG_HPP

#include "usart.h"
#include <cstdarg>
#include <string>

void usart_printf(const char *format, ...);//const char别写成char了

#endif //RM_CAN_DEBUG_HPP
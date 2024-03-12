//
// Created by Cyril Lemarié on 09/03/2024.
//

#ifndef BLUEPILL_TEMPLATE_BSP_H
#define BLUEPILL_TEMPLATE_BSP_H

#include "main.h"

typedef enum  {
    LED_ON = 0x00,
    LED_OFF = 0x01,
}__attribute__ ((__packed__)) ledState_t;

#define LED_G(x) HAL_GPIO_WritePin(LED_V_GPIO_Port,LED_V_Pin,x)
#define LED_B(x) HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,x)
#define LED_R(x) HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,x)



#endif //BLUEPILL_TEMPLATE_BSP_H

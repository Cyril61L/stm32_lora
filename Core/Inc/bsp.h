//
// Created by Cyril Lemarié on 09/03/2024.
//

#ifndef BLUEPILL_TEMPLATE_BSP_H
#define BLUEPILL_TEMPLATE_BSP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"


typedef enum  {
    LED_ON = 0x00,
    LED_OFF = 0x01,
}__attribute__ ((__packed__)) ledState_t;

#define LED_G(x) HAL_GPIO_WritePin(LED_V_GPIO_Port,LED_V_Pin,x)
#define LED_B(x) HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,x)
#define LED_R(x) HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,x)

#define EN_LORA_PWR(x) HAL_GPIO_WritePin(EN_LORA_PWR_GPIO_Port,EN_LORA_PWR_Pin,x)
#define LORA_RESET(x) HAL_GPIO_WritePin(LORA_RST_GPIO_Port,LORA_RST_Pin,x)
#define READ_BUSY HAL_GPIO_ReadPin(LORA_BUSY_GPIO_Port,LORA_BUSY_Pin)
#define LORA_TXEN(x) HAL_GPIO_WritePin(TXEN_GPIO_Port,TXEN_Pin,x)
#define LORA_RXEN(x) HAL_GPIO_WritePin(RXEN_GPIO_Port,RXEN_Pin,x)
#define EN_PIR_PWR(x) HAL_GPIO_WritePin(EN_PIR_PWR_GPIO_Port,EN_PIR_PWR_Pin,x)



#define BIT0 (0x0001 << 0)
#define BIT1 (0x0001 << 1)
#define BIT2 (0x0001 << 2)
#define BIT3 (0x0001 << 3)
#define BIT4 (0x0001 << 4)
#define BIT5 (0x0001 << 5)
#define BIT6 (0x0001 << 6)
#define BIT7 (0x0001 << 7)




#endif //BLUEPILL_TEMPLATE_BSP_H

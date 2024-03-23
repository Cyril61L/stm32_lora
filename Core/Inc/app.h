/**
 * @file app.h
 * @brief 
 * @author cyril
 * @version 1.0.1
 * @date 23/03/2024
 * @copyright ${ORGANIZATION_NAME}
 * @project lora_pir
 **/

#ifndef LORA_PIR_APP_H
#define LORA_PIR_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp.h"
#include "main.h"
#include "cmsis_os2.h"

extern osEventFlagsId_t eventFlagHandle;

typedef enum{
    PIR_EVENT = BIT0,
    TB_EVENT = BIT1,
    REBOOT_EVENT = BIT2,
    PERIODIC_EVENT = BIT3,
}event_t;

void AppTask(void *argument);

#endif //LORA_PIR_APP_H

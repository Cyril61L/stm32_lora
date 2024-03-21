//
// Created by Cyril Lemarié on 12/03/2024.
//

#ifndef LORA_PIR_IHM_H
#define LORA_PIR_IHM_H

#include "cmsis_os2.h"

extern osThreadId_t ihmHandle;
extern const osThreadAttr_t ihm_attributes;

void IhmTask(void *argument);

#endif //LORA_PIR_IHM_H

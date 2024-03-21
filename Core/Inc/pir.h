//
// Created by Cyril Lemarié on 12/03/2024.
//

#ifndef LORA_PIR_PIR_H
#define LORA_PIR_PIR_H

#include "cmsis_os2.h"

extern osThreadId_t pirHandle;
extern const osThreadAttr_t pir_attributes;


void PirTask(void *argument);

#endif //LORA_PIR_PIR_H

//
// Created by Cyril Lemarié on 12/03/2024.
//

#include "pir.h"
#include "cmsis_os2.h"
#include "printf.h"

#define TAG "[PIR]"

/* Definitions for pir */
osThreadId_t pirHandle;
const osThreadAttr_t pir_attributes = {
        .name = "pir",
        .stack_size = 128 * 4,
        .priority = (osPriority_t) osPriorityLow,
};


void PirTask(void *argument)
{
    // Init
    uint8_t status;
    printf("%s Init pir sensor \n",TAG);


    // Infinite loop
    for(;;)
    {
        printf("%s loop\n",TAG);
        osDelay(1000);
    }
}

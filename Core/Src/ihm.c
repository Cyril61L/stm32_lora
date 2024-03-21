//
// Created by Cyril Lemarié on 12/03/2024.
//

#include "ihm.h"
#include "cmsis_os2.h"
#include "printf.h"


#define TAG "[IHM]"

/* Definitions for ihm */
osThreadId_t ihmHandle;
const osThreadAttr_t ihm_attributes = {
        .name = "ihm",
        .stack_size = 128 * 4,
        .priority = (osPriority_t) osPriorityLow,
};


void IhmTask(void *argument)
{
    // Init
    uint8_t status;
    printf("%s Init IHM \n",TAG);


    // Infinite loop
    for(;;)
    {
        printf("%s loop\n",TAG);
        osDelay(2000);
    }
}
/**
 * @file lora_transceiver.h
 * @brief 
 * @author cyril
 * @version 1.0.1
 * @date 21/03/2024
 * @copyright ${ORGANIZATION_NAME}
 * @project lora_pir
 **/

#ifndef LORA_PIR_LORA_TRANSCEIVER_H
#define LORA_PIR_LORA_TRANSCEIVER_H

#include "driver_llcc68_interface.h"
#include "cmsis_os2.h"


/* Definitions for myQueue01 */
extern osMessageQueueId_t loraQueueHandle;



struct time{
    uint8_t hour;
    uint8_t minute;
    uint8_t seconds;
};

/**
 * @brief
 */
typedef struct {
    uint8_t ID;
    uint8_t data[27];
    uint8_t len;
    struct time timeStamp;
}message_t;

// Task object
extern osThreadId_t radioHandle;
extern const osThreadAttr_t radio_attributes;


void lora_task(void *argument);
uint8_t lora_init(void (*callback)(uint16_t type, uint8_t *buf, uint16_t len));
uint8_t lora_irq_handler(void);
uint8_t lora_wake_up(void);
uint8_t lora_sleep(void);

#endif //LORA_PIR_LORA_TRANSCEIVER_H

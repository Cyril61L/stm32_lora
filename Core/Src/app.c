/**
 * @file app.c
 * @brief 
 * @author cyril
 * @version 1.0.1
 * @date 23/03/2024
 * @copyright ${ORGANIZATION_NAME}
 * @project lora_pir
 **/

#include <stdbool.h>
#include "app.h"
#include "lora_transceiver.h"
#include "printf.h"


static volatile bool minFlag = false;
static volatile uint16_t size = 0;
static RTC_DateTypeDef date;
static RTC_TimeTypeDef time;




/**
  * @brief  Function implementing the app thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_AppTask */
void AppTask(void *argument)
{
    /* USER CODE BEGIN 5 */
    message_t message;
    osStatus_t osStatus;
    uint32_t flags;

    EN_PIR_PWR(true);

    /* Infinite loop */
    for(;;)
    {
        // Block task until an event is trigged
        flags = osEventFlagsWait(eventFlagHandle,PIR_EVENT | TB_EVENT | PERIODIC_EVENT | REBOOT_EVENT,osFlagsWaitAny,osWaitForever);
        LED_R(LED_ON);
        switch (flags) {
            case PERIODIC_EVENT:
                printf("Periodic event\n");
                message.ID = 1;
                strncpy(message.data, "Coucou", sizeof("Coucou"));
                osStatus = osMessageQueuePut(loraQueueHandle, &message, 1, 500);
                if (osStatus != osOK)
                    printf("Error queue\n");
                break;
            case TB_EVENT:
                printf("Touch button event\n");
                break;
            case PIR_EVENT:
                printf("Pir detection event\n");
                message.ID = 1;
                strncpy(message.data, "Detection", sizeof("Detection"));
                osStatus = osMessageQueuePut(loraQueueHandle, &message, 1, 500);
                if (osStatus != osOK)
                    printf("Error queue\n");
                break;
                break;

        }
        LED_R(LED_OFF);
    }

}

/**
 * @brief
 * @param GPIO_Pin
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch(GPIO_Pin)
    {
        case LORA_DIO1_Pin:
            lora_irq_handler();
            break;
        case TB1_Pin:
            osEventFlagsSet(eventFlagHandle,TB_EVENT);
            break;
        case PIR_IN_Pin:
            osEventFlagsSet(eventFlagHandle,PIR_EVENT);
            break;
    }
}


/**
 * @brief
 * @param hrtc
 */
void HAL_RTCEx_RTCEventCallback(RTC_HandleTypeDef *hrtc)
{

    HAL_RTC_GetTime(hrtc,&time,RTC_FORMAT_BIN);
    HAL_RTC_GetDate(hrtc,&date,RTC_FORMAT_BIN);
    if(time.Seconds == 0)
    {
        minFlag = true;
        osEventFlagsSet(eventFlagHandle,PERIODIC_EVENT);
        printf_("%u:%u:%u\n",time.Hours,time.Minutes,time.Seconds);
    }
}

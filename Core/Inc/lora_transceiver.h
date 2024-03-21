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


// Task object
extern osThreadId_t radioHandle;
extern const osThreadAttr_t radio_attributes;

/**
 * @brief llcc68 lora  configuration
 */
#define LLCC68_LORA_DEFAULT_STOP_TIMER_ON_PREAMBLE      LLCC68_BOOL_FALSE                 /**< disable stop timer on preamble */
#define LLCC68_LORA_DEFAULT_REGULATOR_MODE              LLCC68_REGULATOR_MODE_DC_DC_LDO   /**< only ldo */
#define LLCC68_LORA_DEFAULT_PA_CONFIG_DUTY_CYCLE        0x02                              /**< set +17dBm power */
#define LLCC68_LORA_DEFAULT_PA_CONFIG_HP_MAX            0x03                              /**< set +17dBm power */
#define LLCC68_LORA_DEFAULT_TX_DBM                      20                                /**< +17dBm */
#define LLCC68_LORA_DEFAULT_RAMP_TIME                   LLCC68_RAMP_TIME_40US             /**< set ramp time 10 us */
#define LLCC68_LORA_DEFAULT_SF                          LLCC68_LORA_SF_9                  /**< sf9 */
#define LLCC68_LORA_DEFAULT_BANDWIDTH                   LLCC68_LORA_BANDWIDTH_125_KHZ     /**< 125khz */
#define LLCC68_LORA_DEFAULT_CR                          LLCC68_LORA_CR_4_5                /**< cr4/5 */
#define LLCC68_LORA_DEFAULT_LOW_DATA_RATE_OPTIMIZE      LLCC68_BOOL_FALSE                 /**< disable low data rate optimize */
#define LLCC68_LORA_DEFAULT_RF_FREQUENCY                868000000U                        /**< 480000000Hz */
#define LLCC68_LORA_DEFAULT_SYMB_NUM_TIMEOUT            0                                 /**< 0 */
#define LLCC68_LORA_DEFAULT_SYNC_WORD                   0x1424U                           /**< public network */
#define LLCC68_LORA_DEFAULT_RX_GAIN                     0x94                              /**< common rx gain */
#define LLCC68_LORA_DEFAULT_OCP                         0x38                              /**< 140 mA */
#define LLCC68_LORA_DEFAULT_PREAMBLE_LENGTH             50                                /**< 12 */
#define LLCC68_LORA_DEFAULT_HEADER                      LLCC68_LORA_HEADER_EXPLICIT       /**< explicit header */
#define LLCC68_LORA_DEFAULT_BUFFER_SIZE                 255                               /**< 255 */
#define LLCC68_LORA_DEFAULT_CRC_TYPE                    LLCC68_LORA_CRC_TYPE_ON           /**< crc on */
#define LLCC68_LORA_DEFAULT_INVERT_IQ                   LLCC68_BOOL_FALSE                 /**< disable invert iq */
#define LLCC68_LORA_DEFAULT_CAD_SYMBOL_NUM              LLCC68_LORA_CAD_SYMBOL_NUM_2      /**< 2 symbol */
#define LLCC68_LORA_DEFAULT_CAD_DET_PEAK                24                                /**< 24 */
#define LLCC68_LORA_DEFAULT_CAD_DET_MIN                 10                                /**< 10 */
#define LLCC68_LORA_DEFAULT_START_MODE                  LLCC68_START_MODE_WARM            /**< warm mode */
#define LLCC68_LORA_DEFAULT_RTC_WAKE_UP                 LLCC68_BOOL_TRUE                  /**< enable rtc wake up */





void lora_task(void *argument);
uint8_t lora_init(void (*callback)(uint16_t type, uint8_t *buf, uint16_t len));
uint8_t lora_irq_handler(void);
uint8_t lora_wake_up(void);
uint8_t lora_sleep(void);

#endif //LORA_PIR_LORA_TRANSCEIVER_H

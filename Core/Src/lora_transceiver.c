/**
 * @file lora_transceiver.c
 * @brief 
 * @author cyril
 * @version 1.0.1
 * @date 21/03/2024
 * @copyright ${ORGANIZATION_NAME}
 * @project lora_pir
 **/

#include <stdbool.h>
#include "cmsis_os.h"
#include "lora_transceiver.h"
#include "driver_llcc68.h"
#include "bsp.h"
#include "printf.h"


#define TAG "[RADIO]"
#define TX_BUFFER_SIZE  32


// tx buffer
static uint8_t tx_buffer[TX_BUFFER_SIZE];

// llcc68 handle
static llcc68_handle_t gs_handle;
static uint8_t gs_rx_done;




static uint8_t lora_start(void);
uint8_t lora_sent(uint8_t *data,uint8_t len, uint16_t timeoutUs);
uint8_t lora_receive(uint16_t s);

/**
 * @brief
 * @param argument
 */
void lora_task(void *argument)
{
    // Init task
    lora_init(llcc68_interface_receive_callback);
    HAL_Delay(500);
    message_t message;
    // Endless loop
    for(;;)
    {
        if(osMessageQueueGet(loraQueueHandle,&message,NULL,100000) == osOK)
        {
            printf_("%s lora receive message %s\n",TAG,message.data);
            lora_start();
            switch (message.ID) {
                case 1:
                    lora_sent(message.data,message.len,0);
                    break;
            }
        }
        osDelay(1000);
        //lora_start();
        //lora_receive(100);
    }
}

/**
 * @brief
 * @param callback
 * @return
 */
uint8_t lora_init(void (*callback)(uint16_t type, uint8_t *buf, uint16_t len))
{
    /* link interface function */
    DRIVER_LLCC68_LINK_INIT(&gs_handle, llcc68_handle_t);
    DRIVER_LLCC68_LINK_SPI_INIT(&gs_handle, llcc68_interface_spi_init);
    DRIVER_LLCC68_LINK_SPI_DEINIT(&gs_handle, llcc68_interface_spi_deinit);
    DRIVER_LLCC68_LINK_SPI_WRITE_READ(&gs_handle, llcc68_interface_spi_write_read);
    DRIVER_LLCC68_LINK_RESET_GPIO_INIT(&gs_handle, llcc68_interface_reset_gpio_init);
    DRIVER_LLCC68_LINK_RESET_GPIO_DEINIT(&gs_handle, llcc68_interface_reset_gpio_deinit);
    DRIVER_LLCC68_LINK_RESET_GPIO_WRITE(&gs_handle, llcc68_interface_reset_gpio_write);
    DRIVER_LLCC68_LINK_BUSY_GPIO_INIT(&gs_handle, llcc68_interface_busy_gpio_init);
    DRIVER_LLCC68_LINK_BUSY_GPIO_DEINIT(&gs_handle, llcc68_interface_busy_gpio_deinit);
    DRIVER_LLCC68_LINK_BUSY_GPIO_READ(&gs_handle, llcc68_interface_busy_gpio_read);
    DRIVER_LLCC68_LINK_DELAY_MS(&gs_handle, llcc68_interface_delay_ms);
    DRIVER_LLCC68_LINK_DEBUG_PRINT(&gs_handle, llcc68_interface_debug_print);
    DRIVER_LLCC68_LINK_RECEIVE_CALLBACK(&gs_handle, callback);
    return 0;
}


/**
 * @brief
 * @return
 */
uint8_t lora_irq_handler(void)
{
    if (llcc68_irq_handler(&gs_handle) != 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief
 * @return
 */
uint8_t lora_wake_up(void)
{

}

/**
 * @brief
 * @return
 */
uint8_t lora_sleep(void)
{

}


/**
 * @brief
 * @return
 */
static uint8_t lora_start(void)
{
    uint8_t res;
    uint32_t reg;
    uint8_t modulation;
    uint8_t config;

    EN_LORA_PWR(true);

    /* start sent test */
    llcc68_interface_debug_print("llcc68: start lora\n");

    /* init the llcc68 */
    res = llcc68_init(&gs_handle);
    if (res != 0)
    {
        llcc68_interface_debug_print("llcc68: init failed.\n");

        return 1;
    }

    /* enter standby */
    res = llcc68_set_standby(&gs_handle, LLCC68_CLOCK_SOURCE_XTAL_32MHZ);
    if (res != 0)
    {
        llcc68_interface_debug_print("llcc68: set standby failed.\n");
        (void)llcc68_deinit(&gs_handle);

        return 1;
    }

    res = llcc68_set_calibration_image(&gs_handle,0xD7,0xDB);
    if (res != 0)
    {
        llcc68_interface_debug_print("llcc68: calibration failed.\n");
        (void)llcc68_deinit(&gs_handle);

        return 1;
    }

    /* disable stop timer on preamble */
    res = llcc68_set_stop_timer_on_preamble(&gs_handle, LLCC68_BOOL_FALSE);
    if (res != 0)
    {
        llcc68_interface_debug_print("llcc68: stop timer on preamble failed.\n");
        (void)llcc68_deinit(&gs_handle);

        return 1;
    }

    /* set dc dc ldo */
    res = llcc68_set_regulator_mode(&gs_handle, LLCC68_REGULATOR_MODE_DC_DC_LDO);
    if (res != 0)
    {
        llcc68_interface_debug_print("llcc68: set regulator mode failed.\n");
        (void)llcc68_deinit(&gs_handle);

        return 1;
    }

    /* set +17dBm power */
    res = llcc68_set_pa_config(&gs_handle, 0x02, 0x03);
    if (res != 0)
    {
        llcc68_interface_debug_print("llcc68: set pa config failed.\n");
        (void)llcc68_deinit(&gs_handle);

        return 1;
    }

    /* enter to stdby rc mode */
    res = llcc68_set_rx_tx_fallback_mode(&gs_handle, LLCC68_RX_TX_FALLBACK_MODE_STDBY_XOSC);
    if (res != 0)
    {
        llcc68_interface_debug_print("llcc68: set rx tx fallback mode failed.\n");
        (void)llcc68_deinit(&gs_handle);

        return 1;
    }

    /* set dio irq */
    res = llcc68_set_dio_irq_params(&gs_handle, 0x03FFU, 0x03FFU, 0x0000, 0x0000);
    if (res != 0)
    {
        llcc68_interface_debug_print("llcc68: set dio irq params failed.\n");
        (void)llcc68_deinit(&gs_handle);

        return 1;
    }

    /* clear irq status */
    res = llcc68_clear_irq_status(&gs_handle, 0x03FFU);
    if (res != 0)
    {
        llcc68_interface_debug_print("llcc68: clear irq status failed.\n");
        (void)llcc68_deinit(&gs_handle);

        return 1;
    }

    /* set lora mode */
    res = llcc68_set_packet_type(&gs_handle, LLCC68_PACKET_TYPE_LORA);
    if (res != 0)
    {
        llcc68_interface_debug_print("llcc68: set packet type failed.\n");
        (void)llcc68_deinit(&gs_handle);

        return 1;
    }

    /* +17dBm */
    res = llcc68_set_tx_params(&gs_handle, 20, LLCC68_RAMP_TIME_40US);
    if (res != 0)
    {
        llcc68_interface_debug_print("llcc68: set tx params failed.\n");
        (void)llcc68_deinit(&gs_handle);

        return 1;
    }

    /* sf9, 125khz, cr4/5, disable low data rate optimize */
    res = llcc68_set_lora_modulation_params(&gs_handle, LLCC68_LORA_SF_9, LLCC68_LORA_BANDWIDTH_125_KHZ,
                                            LLCC68_LORA_CR_4_5, LLCC68_BOOL_FALSE);
    if (res != 0)
    {
        llcc68_interface_debug_print("llcc68: set lora modulation params failed.\n");
        (void)llcc68_deinit(&gs_handle);

        return 1;
    }

    /* convert the frequency */
    res = llcc68_frequency_convert_to_register(&gs_handle, 868000000U, (uint32_t *)&reg);
    if (res != 0)
    {
        llcc68_interface_debug_print("llcc68: convert to register failed.\n");
        (void)llcc68_deinit(&gs_handle);

        return 1;
    }

    /* set the frequency */
    res = llcc68_set_rf_frequency(&gs_handle, reg);
    if (res != 0)
    {
        llcc68_interface_debug_print("llcc68: set rf frequency failed.\n");
        (void)llcc68_deinit(&gs_handle);

        return 1;
    }

    /* set base address */
    res = llcc68_set_buffer_base_address(&gs_handle, 0x00, 0x00);
    if (res != 0)
    {
        llcc68_interface_debug_print("llcc68: set buffer base address failed.\n");
        (void)llcc68_deinit(&gs_handle);

        return 1;
    }

    /* 1 lora symb num */
    res = llcc68_set_lora_symb_num_timeout(&gs_handle, 0);
    if (res != 0)
    {
        llcc68_interface_debug_print("llcc68: set lora symb num timeout failed.\n");
        (void)llcc68_deinit(&gs_handle);

        return 1;
    }

    /* reset stats */
    res = llcc68_reset_stats(&gs_handle, 0x0000, 0x0000, 0x0000);
    if (res != 0)
    {
        llcc68_interface_debug_print("llcc68: reset stats failed.\n");
        (void)llcc68_deinit(&gs_handle);

        return 1;
    }

    /* clear device errors */
    res = llcc68_clear_device_errors(&gs_handle);
    if (res != 0)
    {
        llcc68_interface_debug_print("llcc68: clear device errors failed.\n");
        (void)llcc68_deinit(&gs_handle);

        return 1;
    }

    /* set the lora sync word */
    res = llcc68_set_lora_sync_word(&gs_handle, 0x1424U);
    if (res != 0)
    {
        llcc68_interface_debug_print("llcc68: set lora sync word failed.\n");
        (void)llcc68_deinit(&gs_handle);

        return 1;
    }

    /* get tx modulation */
    res = llcc68_get_tx_modulation(&gs_handle, (uint8_t *)&modulation);
    if (res != 0)
    {
        llcc68_interface_debug_print("llcc68: get tx modulation failed.\n");
        (void)llcc68_deinit(&gs_handle);

        return 1;
    }
    modulation |= 0x04;

    /* set the tx modulation */
    res = llcc68_set_tx_modulation(&gs_handle, modulation);
    if (res != 0)
    {
        llcc68_interface_debug_print("llcc68: set tx modulation failed.\n");
        (void)llcc68_deinit(&gs_handle);

        return 1;
    }

    /* set the rx gain */
    res = llcc68_set_rx_gain(&gs_handle, 0x94);
    if (res != 0)
    {
        llcc68_interface_debug_print("llcc68: set rx gain failed.\n");
        (void)llcc68_deinit(&gs_handle);

        return 1;
    }

    /* set the ocp */
    res = llcc68_set_ocp(&gs_handle, 0x38);
    if (res != 0)
    {
        llcc68_interface_debug_print("llcc68: set ocp failed.\n");
        (void)llcc68_deinit(&gs_handle);

        return 1;
    }

    /* get the tx clamp config */
    res = llcc68_get_tx_clamp_config(&gs_handle, (uint8_t *)&config);
    if (res != 0)
    {
        llcc68_interface_debug_print("llcc68: get tx clamp config failed.\n");
        (void)llcc68_deinit(&gs_handle);

        return 1;
    }
    config |= 0x1E;

    /* set the tx clamp config */
    res = llcc68_set_tx_clamp_config(&gs_handle, config);
    if (res != 0)
    {
        llcc68_interface_debug_print("llcc68: set tx clamp config failed.\n");
        (void)llcc68_deinit(&gs_handle);

        return 1;
    }
    llcc68_interface_debug_print("llcc68: transceiver ready\n");
}

 /**
  * @brief
  * @param data
  * @param len
  * @param timeoutUs
  * @return status code - 0 success - 1 test failed
  */
uint8_t lora_sent(uint8_t *data,uint8_t len, uint16_t timeoutUs)
{
    uint8_t res;

    /* sent the data */
    res = llcc68_lora_transmit(&gs_handle, LLCC68_CLOCK_SOURCE_XTAL_32MHZ,
                               50, LLCC68_LORA_HEADER_EXPLICIT,
                               LLCC68_LORA_CRC_TYPE_ON, LLCC68_BOOL_FALSE,
                               (uint8_t *)data, len, 0);

    if (res != 0)
    {
        llcc68_interface_debug_print("llcc68: lora sent failed.\n");
        (void)llcc68_deinit(&gs_handle);

        return 1;
    }

    /* finish sent test */
    llcc68_interface_debug_print("llcc68: finish sent test.\n");

    /* deinit */
    (void)llcc68_deinit(&gs_handle);

    return 0;
}

/**
 * @brief
 * @param s
 * @return
 */
uint8_t lora_receive(uint16_t s)
{
    uint8_t res;
    uint8_t setup;

    /* set lora packet params */
    res = llcc68_set_lora_packet_params(&gs_handle, 50,
                                        LLCC68_LORA_HEADER_EXPLICIT, 255,
                                        LLCC68_LORA_CRC_TYPE_ON, LLCC68_BOOL_FALSE);
    if (res != 0)
    {
        llcc68_interface_debug_print("llcc68: set lora packet params failed.\n");
        (void)llcc68_deinit(&gs_handle);

        return 1;
    }

    /* get iq polarity */
    res = llcc68_get_iq_polarity(&gs_handle, (uint8_t *)&setup);
    if (res != 0)
    {
        llcc68_interface_debug_print("llcc68: get iq polarity failed.\n");
        (void)llcc68_deinit(&gs_handle);

        return 1;
    }
    setup |= 1 << 2;

    /* set the iq polarity */
    res = llcc68_set_iq_polarity(&gs_handle, setup);
    if (res != 0)
    {
        llcc68_interface_debug_print("llcc68: set iq polarity failed.\n");
        (void)llcc68_deinit(&gs_handle);

        return 1;
    }

    /* start receive */
    res = llcc68_continuous_receive(&gs_handle);
    if (res != 0)
    {
        llcc68_interface_debug_print("llcc68: lora continuous receive failed.\n");
        (void)llcc68_deinit(&gs_handle);

        return 1;
    }

    /* start receiving */
    llcc68_interface_debug_print("llcc68: start receiving...\n");
    gs_rx_done = 0;

    while ((s != 0) && (gs_rx_done == 0))
    {
        s--;
        llcc68_interface_delay_ms(1000);
    }
    if (gs_rx_done == 0)
    {
        /* receive timeout */
        llcc68_interface_debug_print("llcc68: receive timeout.\n");
        (void)llcc68_deinit(&gs_handle);

        return 1;
    }
    else
    {
        /* finish receive test */
        llcc68_interface_debug_print("llcc68: finish receive test.\n");
        (void)llcc68_deinit(&gs_handle);
    }

    return 0;
}




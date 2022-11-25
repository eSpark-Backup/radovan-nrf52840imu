/********************************************************************************************************
 * $Id$        usbd_drv.c
 * @file       usbd_drv.c
 * @brief      USB Driver Source file
 * @date       09 June, 2021
 * @author     Jitendra Zala
 ********************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

#include "nrf.h"
#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_power.h"

#include "app_error.h"
#include "app_util.h"
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"

#include "boards.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define LED_USB_RESUME      (BSP_BOARD_LED_0)
#define LED_CDC_ACM_OPEN    (BSP_BOARD_LED_1)
#define LED_CDC_ACM_RX      (BSP_BOARD_LED_2)
#define LED_CDC_ACM_TX      (BSP_BOARD_LED_3)

/**
 * @brief Enable power USB detection
 *
 * Configure if example supports USB port connection
 */
#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif


static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event);

#define CDC_ACM_COMM_INTERFACE  0
#define CDC_ACM_COMM_EPIN       NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE  1
#define CDC_ACM_DATA_EPIN       NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT      NRF_DRV_USBD_EPOUT1


/**
 * @brief CDC_ACM class instance
 * */
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
                            cdc_acm_user_ev_handler,
                            CDC_ACM_COMM_INTERFACE,
                            CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN,
                            CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250
);

#define READ_SIZE 1

static char m_rx_buffer[READ_SIZE];
static char m_tx_buffer[NRF_DRV_USBD_EPSIZE];
static bool m_send_flag = 1;
static bool m_usb_ready=false;

/**@brief User event handler for CDC_ACM
 *
 * @details This funtion handles all CDC_ACM events
 *
 * @param[in]	 CDC_ACM class and event
 * @param[out]   none
 */
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);

    switch (event)
    {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
        {
            NRF_LOG_INFO("USB Port Open"); NRF_LOG_FLUSH();
            m_usb_ready=true;
            bsp_board_led_on(LED_CDC_ACM_OPEN);
            /*Setup first transfer*/
            ret_code_t ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                                   m_rx_buffer,
                                                   READ_SIZE);
            UNUSED_VARIABLE(ret);
            break;
        }
        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
            NRF_LOG_INFO("USB Port Close"); NRF_LOG_FLUSH();
            bsp_board_led_off(LED_CDC_ACM_OPEN);
            m_usb_ready=false;
            break;
        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
            bsp_board_led_invert(LED_CDC_ACM_TX);
            break;
        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
        {
            ret_code_t ret;
            NRF_LOG_INFO("Bytes waiting: %d", app_usbd_cdc_acm_bytes_stored(p_cdc_acm)); NRF_LOG_FLUSH();
            do
            {
                /*Get amount of data transfered*/
                size_t size = app_usbd_cdc_acm_rx_size(p_cdc_acm);
                NRF_LOG_INFO("RX: size: %lu char: %c", size, m_rx_buffer[0]); NRF_LOG_FLUSH();

                /* Fetch data until internal buffer is empty */
                ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                            m_rx_buffer,
                                            READ_SIZE);
            } while (ret == NRF_SUCCESS);
            bsp_board_led_invert(LED_CDC_ACM_RX);
            break;
        }
        default:
            break;
    }
}

/**@brief User event handler for USB
 *
 * @details This funtion handles all USB events
 *
 * @param[in]	 usb event
 * @param[out]   none
 */
static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_DRV_SUSPEND:
            bsp_board_led_off(LED_USB_RESUME);
            break;
        case APP_USBD_EVT_DRV_RESUME:
            bsp_board_led_on(LED_USB_RESUME);
            break;
        case APP_USBD_EVT_STARTED:
            break;
        case APP_USBD_EVT_STOPPED:
            m_usb_ready=false;
            app_usbd_disable();
            bsp_board_leds_off();
            break;
        case APP_USBD_EVT_POWER_DETECTED:
            NRF_LOG_INFO("USB power detected");NRF_LOG_FLUSH();

            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            break;
        case APP_USBD_EVT_POWER_REMOVED:
            NRF_LOG_INFO("USB power removed"); NRF_LOG_FLUSH();
            app_usbd_stop();
            m_usb_ready=false;
            break;
        case APP_USBD_EVT_POWER_READY:
            NRF_LOG_INFO("USB ready"); NRF_LOG_FLUSH();
            app_usbd_start();
            break;
        default:
            break;
    }
}

/**@brief Send data on USB COM port
 *
 * @details This funtion can be used to send data on USB COM Port
 *
 * @param[in]	 data and datalength
 * @param[out]   none
 */
void send_data_str(char *tx_data, size_t tx_data_len)
{
    if(m_usb_ready){
		ret_code_t ret = app_usbd_cdc_acm_write(&m_app_cdc_acm, &tx_data[0], tx_data_len);
		/*if (ret == NRF_SUCCESS)
		{
			NRF_LOG_INFO("USBD -> data transmition success.");NRF_LOG_FLUSH();
		}
		else
		{
			NRF_LOG_INFO("USBD -> data transmition failed !!!");NRF_LOG_FLUSH();
		}*/
     }
}



/**@brief USB Power ON Event
 *
 * @details This funtion keeps USB powered ON
 *
 * @param[in]	 none
 * @param[out]   none
 */
void usbd_drv_keep_powered(void)
{
		while (app_usbd_event_queue_process())
		{
				// Nothing to do //
		}
}

/**@brief Enable USB Power
 *
 * @details This funtion enavbles USB Power and start USB Event
 *
 * @param[in]	 none
 * @param[out]   none
 */
void usbd_drv_power_enable(void)
{
		ret_code_t ret;

    if (USBD_POWER_DETECTION)
    {
        ret = app_usbd_power_events_enable();
        APP_ERROR_CHECK(ret);
    }
    else
    {
        NRF_LOG_INFO("No USB power detection enabled\r\nStarting USB now"); NRF_LOG_FLUSH();

        app_usbd_enable();
        app_usbd_start();
    }
}

/**@brief USB Init
 *
 * @details This funtion initialize USB and its clock source
 *
 * @param[in]	 none
 * @param[out]   none
 */
void usbd_drv_init(void)
{
    ret_code_t ret;
    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler
    };

     /* Configure LEDs */
    bsp_board_init(BSP_INIT_LEDS);

    ret = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret);

    nrf_drv_clock_lfclk_request(NULL);

    while(!nrf_drv_clock_lfclk_is_running())
    {
        /* Just waiting */
    }

    app_usbd_serial_num_generate();
     
    nrf_delay_ms(2000);
 
    ret = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(ret);

    app_usbd_class_inst_t const * class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    ret = app_usbd_class_append(class_cdc_acm);
    APP_ERROR_CHECK(ret);
    nrf_delay_ms(2000);

    if (USBD_POWER_DETECTION)
    {
        ret = app_usbd_power_events_enable();
        APP_ERROR_CHECK(ret);
    }
    else
    {
        NRF_LOG_INFO("No USB power detection enabled\r\nStarting USB now"); NRF_LOG_FLUSH();

        app_usbd_enable();
        app_usbd_start();
        nrf_delay_ms(2000);
    }
}
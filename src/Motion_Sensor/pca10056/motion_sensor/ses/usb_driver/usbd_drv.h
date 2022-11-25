/********************************************************************************************************
 * $Id$        usbd_drv.h
 * @file       usbd_drv.h
 * @brief      USB Driver Header file
 * @date       09 June, 2021
 * @author     Jitendra Zala
 ********************************************************************************************************/
#ifndef USBD_DRV_H
#define USBD_DRV_H


void usbd_drv_keep_powered(void);
void usbd_drv_power_enable(void);
void usbd_drv_init(void);
void send_data_str(char *tx_data, size_t tx_data_len);

#endif /* USBD_DRV_H */

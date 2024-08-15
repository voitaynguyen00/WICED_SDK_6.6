/*
 * Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of Cypress Semiconductor 
 *  Corporation. All rights reserved. This software, including source code, documentation and  related 
 * materials ("Software"), is owned by Cypress Semiconductor  Corporation or one of its 
 *  subsidiaries ("Cypress") and is protected by and subject to worldwide patent protection  
 * (United States and foreign), United States copyright laws and international treaty provisions. 
 * Therefore, you may use this Software only as provided in the license agreement accompanying the 
 * software package from which you obtained this Software ("EULA"). If no EULA applies, Cypress 
 * hereby grants you a personal, nonexclusive, non-transferable license to  copy, modify, and 
 * compile the Software source code solely for use in connection with Cypress's  integrated circuit 
 * products. Any reproduction, modification, translation, compilation,  or representation of this 
 * Software except as specified above is prohibited without the express written permission of 
 * Cypress. Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO  WARRANTY OF ANY KIND, EXPRESS 
 * OR IMPLIED, INCLUDING,  BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY 
 * AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the right to make changes to 
 * the Software without notice. Cypress does not assume any liability arising out of the application 
 * or use of the Software or any product or circuit  described in the Software. Cypress does 
 * not authorize its products for use in any products where a malfunction or failure of the 
 * Cypress product may reasonably be expected to result  in significant property damage, injury 
 * or death ("High Risk Product"). By including Cypress's product in a High Risk Product, the 
 *  manufacturer of such system or application assumes  all risk of such use and in doing so agrees 
 * to indemnify Cypress against all liability.
 */

/** @file
 *
 * This file provides the private interface definitions for hci_control app
 *
 */
#ifndef HCI_CONTROL_H
#define HCI_CONTROL_H

#include "wiced_bt_dev.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_trace.h"

#include "hci_control_api.h"
#include "wiced_bt_cfg.h"


/*****************************************************************************
 *  Constants
 *****************************************************************************/
#define TRANS_UART_BUFFER_SIZE            1024

/* NVRAM Ids */
#define HCI_CONTROL_FIRST_VALID_NVRAM_ID  0x10
#define HCI_CONTROL_INVALID_NVRAM_ID      0x00


/*****************************************************************************
 *  Data types
 *****************************************************************************/

/* The main application control block */
typedef struct
{
    uint8_t pairing_allowed;
} hci_control_cb_t;


/*****************************************************************************
 *  Global data
 *****************************************************************************/

/* control block declaration */
#if BTA_DYNAMIC_MEMORY == FALSE
extern hci_control_cb_t hci_control_cb;
#else
extern hci_control_cb_t *hci_control_cb_ptr;
#define hci_control_cb(*hci_control_cb_ptr)
#endif


/*****************************************************************************
 *  Function prototypes
 *****************************************************************************/

/* main functions */
extern void     hci_control_send_command_status_evt(uint16_t code, uint8_t status);

/* LE control interface */
extern void     hci_control_le_enable(void);
extern void     hci_control_le_handle_command(uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len);

extern int      hci_control_read_nvram(int nvram_id, void *p_data, int data_len);
extern int      hci_control_write_nvram(int nvram_id, int data_len, void *p_data, wiced_bool_t from_host);
extern int      hci_control_find_nvram_id(uint8_t *p_data, int len);
extern void     hci_control_delete_nvram(int nvram_id, wiced_bool_t from_host);
extern int      hci_control_alloc_nvram_id(void);

extern void hci_control_le_scan_state_changed(wiced_bt_ble_scan_type_t state);
extern void hci_control_le_advert_state_changed(wiced_bt_ble_advert_mode_t mode);
extern uint8_t wiced_bt_get_number_of_buffer_pools ( void );

#endif /* BTA_HS_INT_H */


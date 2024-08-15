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
#pragma once

#include "wiced_transport.h"

/*****************************************************************************
**  Constants that define the capabilities and configuration
*****************************************************************************/
#define WICED_BUFF_MAX_SIZE             264
#define BSG_TRANS_MAX_BUFFERS           10
#define TRANS_UART_BUFFER_SIZE          1024

#include "wiced_bt_dev.h"
#include "wiced_bt_trace.h"
#include "wiced_hci.h"
#include "wiced_bt_dev.h"
#include "hci_control_api.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_ble_hidh.h"


/* NVRAM Ids */
#define HCI_CONTROL_FIRST_VALID_NVRAM_ID        0x16
#define HCI_CONTROL_INVALID_NVRAM_ID            0x00

extern wiced_transport_buffer_pool_t *p_transport_pool;

/*****************************************************************************
**  Function prototypes
*****************************************************************************/
void hci_control_init( void );

int hci_control_nvram_write_ble_hidh_gatt_cache(wiced_bt_device_address_t bdaddr,
        wiced_bt_ble_hidh_gatt_cache_t *p_gatt_cache, wiced_bool_t send_to_host);
int hci_control_nvram_read_ble_hidh_gatt_cache( wiced_bt_device_address_t bdaddr,
        wiced_bt_ble_hidh_gatt_cache_t *p_gatt_cache);
void hci_control_send_command_status_evt( uint16_t code, uint8_t status );
wiced_bt_ble_address_type_t hci_control_get_address_type(wiced_bt_device_address_t bdaddr);
wiced_bt_gatt_status_t wiced_bt_ble_hidh_get_descriptor(uint16_t handle);


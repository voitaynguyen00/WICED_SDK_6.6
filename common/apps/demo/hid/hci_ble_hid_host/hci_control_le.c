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

/**
 *
 * This file implement BLE application controlled over UART.
 *
 */
#include "wiced.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_ble_hidh_gattc.h"
#include "wiced_app_cfg.h"
#include "wiced_timer.h"
#include "hci_control.h"
#include "wiced_memory.h"

#include "hci_control.h"
#include "hci_control_le.h"

/******************************************************
 *                     Constants
 ******************************************************/
#define HCI_CONTROL_LE_SCAN_RESULT_MAX  20

/******************************************************
 *                     Structures
 ******************************************************/
typedef struct
{
    wiced_bool_t in_use;
    wiced_bt_device_address_t address;
    wiced_bt_ble_address_type_t address_type;
} hci_control_le_scanned_device_t;

typedef struct
{
    hci_control_le_scanned_device_t scanned_devices[HCI_CONTROL_LE_SCAN_RESULT_MAX];
} hci_control_le_cb_t;


/******************************************************
 *                     Global variables
 ******************************************************/
static hci_control_le_cb_t hci_control_le_cb;

/******************************************************
 *                     Local functions
 ******************************************************/
static void hci_control_le_handle_scan_cmd( wiced_bool_t enable, wiced_bool_t filter_duplicates );
static void hci_control_le_scan_result_cback( wiced_bt_ble_scan_results_t *p_scan_result,
        uint8_t *p_adv_data );
static void hci_control_le_send_advertisement_report( wiced_bt_ble_scan_results_t *p_scan_result,
        uint8_t *p_adv_data );
static void hci_control_le_send_scan_state_event( uint8_t status );
static void hci_control_le_scan_result_save( wiced_bt_ble_scan_results_t *p_scan_result);

/*
 * hci_control_le_init
 * Handle various LE GAP and GATT commands received from MCU.
 */
void hci_control_le_init( void )
{
    memset(&hci_control_le_cb, 0, sizeof(hci_control_le_cb));

}
/*
 * hci_control_le_handle_command
 * Handle various LE GAP and GATT commands received from MCU.
 */
void hci_control_le_handle_command( uint16_t cmd_opcode, uint8_t* p, uint32_t data_len )
{
    switch( cmd_opcode )
    {
    case HCI_CONTROL_LE_COMMAND_SCAN:
        hci_control_le_handle_scan_cmd( (wiced_bool_t)p[0], (wiced_bool_t)p[1] );
        break;

    default:
        hci_control_send_command_status_evt( HCI_CONTROL_GATT_EVENT_COMMAND_STATUS,
                HCI_CONTROL_STATUS_UNKNOWN_COMMAND );
        break;
    }
}

/*
 * handle scan command from UART
 */
static void hci_control_le_handle_scan_cmd( wiced_bool_t enable, wiced_bool_t filter_duplicates )
{
    wiced_result_t status;
    if ( enable )
    {
        memset(hci_control_le_cb.scanned_devices, 0, sizeof(hci_control_le_cb.scanned_devices));
        status = wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_HIGH_DUTY, filter_duplicates, hci_control_le_scan_result_cback );
    }
    else
    {
        status = wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_NONE, filter_duplicates, hci_control_le_scan_result_cback );
    }
    WICED_BT_TRACE( "hci_control_le_handle_scan_cmd:%d status:%x\n", enable, status );

    if( ( status == WICED_BT_SUCCESS ) || ( status == WICED_BT_PENDING) )
    {
        status = HCI_CONTROL_STATUS_SUCCESS;
    }
    else
    {
        status = HCI_CONTROL_STATUS_FAILED;
    }

    hci_control_send_command_status_evt( HCI_CONTROL_LE_EVENT_COMMAND_STATUS, status );
}

/*
 * hci_control_le_scan_result_cback
 * Process advertisement packet received
 */
static void hci_control_le_scan_result_cback( wiced_bt_ble_scan_results_t *p_scan_result,
        uint8_t *p_adv_data )
{
    if ( p_scan_result )
    {
        WICED_BT_TRACE( "BLE Scan Result Device:%B AddrType:%d\n", p_scan_result->remote_bd_addr,
                p_scan_result->ble_addr_type);
        hci_control_le_scan_result_save(p_scan_result);
        hci_control_le_send_advertisement_report( p_scan_result, p_adv_data );
    }
    else
    {
        WICED_BT_TRACE( "Scan completed\n" );
    }
}

/*
 * Stack runs the scan state machine switching between high duty, low
 * duty, no scan, based on the wiced_cfg.  All changes are notified
 * through this callback.
 */
void hci_control_le_scan_state_changed( wiced_bt_ble_scan_type_t state )
{
    uint8_t hci_control_le_event = HCI_CONTROL_SCAN_EVENT_NO_SCAN;

    WICED_BT_TRACE( "Scan State Changed:%d\n", state );

    switch ( state )
    {
    case BTM_BLE_SCAN_TYPE_NONE:
        hci_control_le_event = HCI_CONTROL_SCAN_EVENT_NO_SCAN;
        break;
    case BTM_BLE_SCAN_TYPE_HIGH_DUTY:
        hci_control_le_event = HCI_CONTROL_SCAN_EVENT_HIGH_SCAN;
        break;
    case BTM_BLE_SCAN_TYPE_LOW_DUTY:
        hci_control_le_event = HCI_CONTROL_SCAN_EVENT_LOW_SCAN;
        break;
    }
    hci_control_le_send_scan_state_event( hci_control_le_event );
}

/*
 *  transfer scan event to UART
 */
static void hci_control_le_send_scan_state_event( uint8_t status )
{
    wiced_transport_send_data ( HCI_CONTROL_LE_EVENT_SCAN_STATUS, &status, 1 );
}

/*
 * hci_control_le_send_advertisement_report
 *  transfer advertisement report event to UART
 */
static void hci_control_le_send_advertisement_report( wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data )
{
    int       i;
    uint8_t   tx_buf[70];
    uint8_t   *p = tx_buf;
    uint8_t   len;

    *p++ = p_scan_result->ble_evt_type;
    *p++ = p_scan_result->ble_addr_type;
    for ( i = 0; i < 6; i++ )
        *p++ = p_scan_result->remote_bd_addr[5 - i];
    *p++ = p_scan_result->rssi;

    // currently callback does not pass the data of the adv data, need to go through the data
    // zero len in the LTV means that there is no more data
    while ( ( len = *p_adv_data ) != 0 )
    {
        for ( i = 0; i < len + 1; i++ )
            *p++ = *p_adv_data++;
    }
    wiced_transport_send_data ( HCI_CONTROL_LE_EVENT_ADVERTISEMENT_REPORT, tx_buf, ( int )( p - tx_buf ) );
}

/*
 * hci_control_le_get_address_type
 * Used to get a BLE Address type (from either scan results or connected devices)
 */
wiced_bt_ble_address_type_t hci_control_le_get_address_type(wiced_bt_device_address_t bdaddr)
{
    int i;
    hci_control_le_scanned_device_t *p_dev;

    /* Check if this device is in the Scan Result database */
    p_dev = &hci_control_le_cb.scanned_devices[0];
    for ( i = 0 ; i < HCI_CONTROL_LE_SCAN_RESULT_MAX ; i++, p_dev++)
    {
        /* Device found */
        if ((p_dev->in_use) &&
            (memcmp(p_dev->address, bdaddr, BD_ADDR_LEN) == 0))
        {
            return p_dev->address_type;
        }
    }

    /* If we reach this point, this means that the device is not in the Scan Result database */
    WICED_BT_TRACE( "Err: hci_control_le_get_address_type Failed" );
    return 0xFF;
}

/*
 * hci_control_le_scan_result_cback
 * Process advertisement packet received
 */
static void hci_control_le_scan_result_save( wiced_bt_ble_scan_results_t *p_scan_result)
{
    int i;
    hci_control_le_scanned_device_t *p_dev;

    /* Check if this device is already in the Scan Result database */
    p_dev = &hci_control_le_cb.scanned_devices[0];
    for ( i = 0 ; i < HCI_CONTROL_LE_SCAN_RESULT_MAX ; i++, p_dev++)
    {
        /* Device found */
        if ((p_dev->in_use) &&
            (memcmp(p_dev->address, p_scan_result->remote_bd_addr, BD_ADDR_LEN) == 0))
        {
            p_dev->address_type = p_scan_result->ble_addr_type;
            return;
        }
    }

    /* If we reach this point, this means that the device is not in the Scan Result database */
    /* Find a free entry and save it */
    p_dev = &hci_control_le_cb.scanned_devices[0];
    for ( i = 0 ; i < HCI_CONTROL_LE_SCAN_RESULT_MAX ; i++, p_dev++)
    {
        if (p_dev->in_use == WICED_FALSE)
        {
            p_dev->in_use = WICED_TRUE;
            memcpy(p_dev->address, p_scan_result->remote_bd_addr, BD_ADDR_LEN);
            p_dev->address_type = p_scan_result->ble_addr_type;
            WICED_BT_TRACE( "hci_control_le_scan_result_save index:%d", i);
            return;
        }
    }

    /* If we reach this point, this means that the Scan Result database is full */
    WICED_BT_TRACE( "Err: hci_control_le_scan_result_save Mem Full" );
}

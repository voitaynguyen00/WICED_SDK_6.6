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
#include "wiced_app_cfg.h"
#include "wiced_timer.h"
#include "hci_control.h"
#include "wiced_memory.h"
#include "hci_control_le.h"


/*****************************************************************************
**  Constants
*****************************************************************************/

/*****************************************************************************
**  Structures
*****************************************************************************/

/******************************************************
 *               Variables Definitions
 ******************************************************/


/******************************************************
 *             Local Function Declarations
 ******************************************************/
static wiced_bt_gatt_status_t wiced_le_gatt_callback(wiced_bt_gatt_evt_t event,
        wiced_bt_gatt_event_data_t *p_event_data);

/*
 * app_gatt_init
 * This function initializes the GATT global variables (stored in a Control Block structure)
 */
void wiced_le_gatt_init(void)
{
    wiced_bt_gatt_status_t     gatt_status;

    WICED_BT_TRACE("wiced_le_gatt_init\n");

    /* GATT registration */
    gatt_status = wiced_bt_gatt_register(wiced_le_gatt_callback);
    WICED_BT_TRACE("wiced_bt_gatt_register status %d\n", gatt_status);
}


/*
 * wiced_le_gatt_callback
 * This function handles GATT events
 */
static wiced_bt_gatt_status_t wiced_le_gatt_callback(wiced_bt_gatt_evt_t event,
        wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_SUCCESS;

    WICED_BT_TRACE("wiced_le_gatt_callback event:%d\n", event);

    switch( event )
    {
    case GATT_CONNECTION_STATUS_EVT:
        /* BLE/GATT Connection/Disconnection events */
        WICED_BT_TRACE("wiced_le_gatt_callback Up/Down:%d BdAddr:%B\n",
                p_data->connection_status.connected, p_data->connection_status.bd_addr);
        if (p_data->connection_status.connected)
        {
            wiced_bt_ble_hidh_up(&p_data->connection_status);

            /* Audio RC require MTU to be at least > 173 bytes */
            if(wiced_bt_cfg_settings.gatt_cfg.max_attr_len > GATT_DEF_BLE_MTU_SIZE)
            {
                result = wiced_bt_gatt_configure_mtu(p_data->connection_status.conn_id,
                        wiced_bt_cfg_settings.gatt_cfg.max_attr_len);
                if (result != WICED_BT_GATT_SUCCESS)
                    WICED_BT_TRACE("wiced_bt_gatt_configure_mtu failed :%d\n", result);
            }
        }
        else
        {
            wiced_bt_ble_hidh_down(&p_data->connection_status);
        }
        break;

    case GATT_OPERATION_CPLT_EVT:
        /* GATT Read/Write complete events */
        WICED_BT_TRACE("wiced_le_gatt_callback OpComplete conn_id:%d\n",
                p_data->operation_complete.conn_id);
        result = wiced_bt_ble_hidh_gatt_operation_complete(&p_data->operation_complete);
        break;

    case GATT_DISCOVERY_RESULT_EVT:
        /* GATT Discovery events */
        WICED_BT_TRACE("wiced_le_gatt_callback DiscResult conn_id:%d\n",
                p_data->discovery_result.conn_id);
        result = wiced_bt_ble_hidh_gatt_discovery_result(&p_data->discovery_result);
        break;

    case GATT_DISCOVERY_CPLT_EVT:
        /* GATT Discovery Complete events */
        WICED_BT_TRACE("wiced_le_gatt_callback DiscComplete conn_id:%d\n",
                p_data->discovery_complete.conn_id);
        result = wiced_bt_ble_hidh_gatt_discovery_complete(&p_data->discovery_complete);
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        /* GATT Request events (received from peer, Client, device */
        /* Unused in this BLE HID host Application */
        break;

    default:
        break;
    }

    return result;
}


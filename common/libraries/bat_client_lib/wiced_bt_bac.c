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
 * Battery Status Client Profile library.
 *
 */

#include "wiced.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_trace.h"
#include "wiced_memory.h"
#include "wiced_timer.h"
#include "wiced_bt_bac.h"
#include "wiced_result.h"
#include "string.h"
#include "wiced_bt_gatt_util.h"

#ifdef WICED_BT_TRACE_ENABLE
#define     BAC_LIB_TRACE                          WICED_BT_TRACE
#else
#define     BAC_LIB_TRACE(...)
#endif

/******************************************************
 *                  Constants
 ******************************************************/
/* service discovery states */
enum
{
    BAC_CLIENT_STATE_IDLE                       = 0x00,
    BAC_CLIENT_STATE_CONNECTED                  = 0x01,
    BAC_CLIENT_STATE_DISCOVER_CHARACTERISTIC    = 0x02,
    BAC_CLIENT_STATE_DISCOVER_BATTERY_LEVEL_CCD = 0x03,
};

/******************************************************
 *                  Structures
 ******************************************************/

typedef struct
{
    wiced_bt_bac_callback_t *p_callback;    /* Application BAC Callback */

    uint16_t conn_id;                       /* connection identifier */
    uint16_t bac_e_handle;                  /* Battery Service discovery end handle */
    uint8_t  bac_current_state;             /* to avoid other requests during execution of previous request*/

    /* during discovery below gets populated and gets used later on application request in connection state */
    uint16_t battery_service_char_handle;       /* characteristic handle */
    uint16_t battery_service_char_value_handle; /* characteristic value handle */
    uint16_t battery_service_cccd_handle;       /* descriptor handle */
} wiced_bt_bac_cb_t;


/******************************************************
 *                Variables Definitions
 ******************************************************/
static wiced_bt_bac_cb_t wiced_bt_bac_cb;

/******************************************************
 *              Function Prototypes
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t wiced_bt_bac_init(wiced_bt_bac_callback_t *p_callback)
{
    memset(&wiced_bt_bac_cb , 0, sizeof(wiced_bt_bac_cb) );

    wiced_bt_bac_cb.p_callback = p_callback;

    return WICED_SUCCESS;
}

void wiced_bt_bac_client_connection_up(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    wiced_bt_bac_cb.conn_id = p_conn_status->conn_id;
    wiced_bt_bac_cb.bac_current_state = BAC_CLIENT_STATE_CONNECTED;
}

void wiced_bt_bac_client_connection_down(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    /* Reinitialize BAC Library (but keep the Callback) */
    wiced_bt_bac_init(wiced_bt_bac_cb.p_callback);
}

wiced_bt_gatt_status_t wiced_bt_bac_discover( uint16_t conn_id, uint16_t start_handle, uint16_t end_handle)
{
    if ((start_handle == 0) || (end_handle == 0))
        return WICED_BT_GATT_INVALID_HANDLE;

    wiced_bt_bac_cb.bac_e_handle = end_handle;
    wiced_bt_bac_cb.bac_current_state = BAC_CLIENT_STATE_DISCOVER_CHARACTERISTIC;

    return wiced_bt_util_send_gatt_discover(conn_id, GATT_DISCOVER_CHARACTERISTICS, 0, start_handle, end_handle);
}

/*
 * While application performs GATT discovery it shall pass discovery results for
 * for the handles that belong to BAC service, to this function.
 * Process discovery results from the stack.
 */
void wiced_bt_bac_discovery_result(wiced_bt_gatt_discovery_result_t *p_data)
{
    BAC_LIB_TRACE("[%s]\n",__FUNCTION__);

    if (p_data->discovery_type == GATT_DISCOVER_CHARACTERISTICS)
    {
        // Result for characteristic discovery.  Save appropriate handle baced on the UUID.
        wiced_bt_gatt_char_declaration_t *p_char = &p_data->discovery_data.characteristic_declaration;
        if(p_char->char_uuid.len == LEN_UUID_16)
        {
            if( p_char->char_uuid.uu.uuid16 == UUID_CHARACTERISTIC_BATTERY_LEVEL )
            {
                wiced_bt_bac_cb.battery_service_char_handle = p_char->handle;
                wiced_bt_bac_cb.battery_service_char_value_handle = p_char->val_handle;
                BAC_LIB_TRACE("control hdl:%04x-%04x\n",
                        wiced_bt_bac_cb.battery_service_char_handle,
                        wiced_bt_bac_cb.battery_service_char_value_handle);
            }
        }
    }
    else if((p_data->discovery_type == GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS) &&
        (p_data->discovery_data.char_descr_info.type.len == 2) &&
        (p_data->discovery_data.char_descr_info.type.uu.uuid16 == UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION))
    {
        // result for descriptor discovery, save appropriate handle baced on the state
        if( wiced_bt_bac_cb.bac_current_state == BAC_CLIENT_STATE_DISCOVER_BATTERY_LEVEL_CCD )
        {
            wiced_bt_bac_cb.battery_service_cccd_handle = p_data->discovery_data.char_descr_info.handle;
            BAC_LIB_TRACE("battery service cccd_hdl hdl:%04x\n", wiced_bt_bac_cb.battery_service_cccd_handle );
        }
    }

}

void wiced_bt_bac_client_discovery_complete(wiced_bt_gatt_discovery_complete_t *p_data)
{
    wiced_bt_bac_event_data_t data_event;

    BAC_LIB_TRACE("[%s] state:%d\n", __FUNCTION__, wiced_bt_bac_cb.bac_current_state);

    if( p_data->disc_type == GATT_DISCOVER_CHARACTERISTICS )
    {
        // done with BAC characteristics, start reading descriptor handles
        // make sure that all characteristics are present
        if ( (wiced_bt_bac_cb.battery_service_char_handle == 0) ||
             (wiced_bt_bac_cb.battery_service_char_value_handle == 0) )
        {
            // something is very wrong
            BAC_LIB_TRACE("[%s] failed\n", __FUNCTION__);
            wiced_bt_bac_cb.bac_current_state = BAC_CLIENT_STATE_IDLE;

            /* Reinitialize BAC Library (but keep the Callback) */
            wiced_bt_bac_init(wiced_bt_bac_cb.p_callback);

            /* Tell the application that the GATT Discovery failed */
            data_event.discovery.conn_id = p_data->conn_id;
            data_event.discovery.status = WICED_BT_GATT_NOT_FOUND;
            wiced_bt_bac_cb.p_callback(WICED_BT_BAC_EVENT_DISCOVERY_COMPLETE,&data_event);
            return;
        }

        wiced_bt_bac_cb.bac_current_state = BAC_CLIENT_STATE_DISCOVER_BATTERY_LEVEL_CCD;
        wiced_bt_util_send_gatt_discover(p_data->conn_id, GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS,
                UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                wiced_bt_bac_cb.battery_service_char_handle + 1, wiced_bt_bac_cb.bac_e_handle);
    }
    else if(p_data->disc_type == GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS)
    {
         if ( wiced_bt_bac_cb.bac_current_state == BAC_CLIENT_STATE_DISCOVER_BATTERY_LEVEL_CCD )
         {
            wiced_bt_bac_cb.bac_current_state = BAC_CLIENT_STATE_CONNECTED;
            /* Tell the application that Battery Notification is supported */
            data_event.discovery.notification_supported = WICED_TRUE;
         }
         else
         {
            BAC_LIB_TRACE("No CCD found in the server\n");
            wiced_bt_bac_cb.bac_current_state = BAC_CLIENT_STATE_CONNECTED;
            /* Tell the application that Battery Notification is not supported */
            data_event.discovery.notification_supported = WICED_FALSE;
         }
         /* Tell the application that the GATT Discovery was successful */
         data_event.discovery.conn_id = p_data->conn_id;
         data_event.discovery.status = WICED_BT_GATT_SUCCESS;
         wiced_bt_bac_cb.p_callback(WICED_BT_BAC_EVENT_DISCOVERY_COMPLETE,&data_event);
    }
}

wiced_bt_gatt_status_t wiced_bt_bac_read_battery_level(uint16_t conn_id)
{
    wiced_bt_gatt_read_param_t read_req;
    wiced_bt_gatt_status_t status;

    if( wiced_bt_bac_cb.bac_current_state != BAC_CLIENT_STATE_CONNECTED )
    {
        BAC_LIB_TRACE("Illegal State: %d\n",wiced_bt_bac_cb.bac_current_state);
        wiced_bt_bac_cb.bac_current_state = BAC_CLIENT_STATE_IDLE;
        return WICED_BT_GATT_ERROR;
    }

    memset( &read_req, 0, sizeof( wiced_bt_gatt_read_param_t ) );
    read_req.by_handle.auth_req = GATT_AUTH_REQ_NONE;
    read_req.by_handle.handle = wiced_bt_bac_cb.battery_service_char_value_handle;

    status = wiced_bt_gatt_send_read(conn_id, GATT_READ_BY_HANDLE, &read_req);

    return status;
}

wiced_bt_gatt_status_t wiced_bt_bac_enable_notification( uint16_t conn_id )
{
    wiced_bt_gatt_status_t status;

    // verify that CCCD has been discovered
    if ((wiced_bt_bac_cb.battery_service_cccd_handle == 0))
    {
        return WICED_BT_GATT_NOT_FOUND;
    }

    status = wiced_bt_util_set_gatt_client_config_descriptor( conn_id,
            wiced_bt_bac_cb.battery_service_cccd_handle, GATT_CLIENT_CONFIG_NOTIFICATION );

    return status;
}

wiced_bt_gatt_status_t wiced_bt_bac_disable_notification(uint16_t conn_id)
{
    wiced_bt_gatt_status_t status;

    // verify that CCCD has been discovered
    if ((wiced_bt_bac_cb.battery_service_cccd_handle == 0))
    {
        return WICED_BT_GATT_NOT_FOUND;
    }

    status = wiced_bt_util_set_gatt_client_config_descriptor( conn_id,
            wiced_bt_bac_cb.battery_service_cccd_handle, GATT_CLIENT_CONFIG_NONE );

    return status;
}

/*
 * Process read response from the stack.
 * Application passes it here if handle belongs to our service.
 */
void wiced_bt_bac_read_rsp(wiced_bt_gatt_operation_complete_t *p_data)
{
    wiced_bt_bac_event_data_t data_event;

    BAC_LIB_TRACE("[%s] state:%02x rc:%d handle: %x\n", __FUNCTION__, wiced_bt_bac_cb.bac_current_state,
            p_data->status, p_data->response_data.att_value.handle );

    if( p_data->response_data.att_value.handle == wiced_bt_bac_cb.battery_service_char_value_handle )
    {
        BAC_LIB_TRACE("[%s] Read Battery Value: handle: %x\n", __FUNCTION__,
                p_data->response_data.att_value.handle);

        /* Send Battery Level Response to the application */
        data_event.battery_level_rsp.conn_id = p_data->conn_id;
        data_event.battery_level_rsp.status = p_data->status;
        data_event.battery_level_rsp.battery_level = p_data->response_data.att_value.p_data[0];
        wiced_bt_bac_cb.p_callback(WICED_BT_BAC_EVENT_BATTERY_LEVEL_RSP,&data_event);
    }
}

void wiced_bt_bac_process_notification(wiced_bt_gatt_operation_complete_t *p_data)
{
    wiced_bt_bac_event_data_t data_event;
    uint16_t handle = p_data->response_data.att_value.handle;

    BAC_LIB_TRACE("[%s] state:%02x rc:%d handle: %x\n", __FUNCTION__, wiced_bt_bac_cb.bac_current_state,
            p_data->status, p_data->response_data.att_value.handle );

    if( handle == wiced_bt_bac_cb.battery_service_char_value_handle )
    {
        /* Send Battery Level Notification to the application */
        data_event.battery_level_notification.conn_id = p_data->conn_id;
        data_event.battery_level_notification.battery_level = p_data->response_data.att_value.p_data[0];
        wiced_bt_bac_cb.p_callback(WICED_BT_BAC_EVENT_BATTERY_LEVEL_NOTIFICATION,&data_event);
    }
}


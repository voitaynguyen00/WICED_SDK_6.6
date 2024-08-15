/*
* Copyright 2021, Cypress Semiconductor Corporation or a subsidiary of 
 * Cypress Semiconductor Corporation. All Rights Reserved.
 * 
 * This software, associated documentation and materials ("Software"),
 * is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
*/


/**
 * @file wiced_hci_bt_rfcomm.c
 *
 * This handles the RFCOMM api functions when USE_WICED_HCI is defined.
 *
 */

#include "wiced_bt_rfcomm.h"
#include <string.h>
#include <stdlib.h>
#include "wiced_hci.h"
#include "wwd_debug.h"

 /******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Structures
 ******************************************************/

typedef struct _wiced_hci_bt_rfcomm_context
{
        wiced_bt_port_mgmt_cback_t *rfcomm_mgmt_cb;
        wiced_bt_rfcomm_data_cback_t *rfcomm_data_cb;
        wiced_bt_port_event_cback_t *rfcomm_port_event_cb;
        wiced_hci_cb  rfcomm_context_cb;
} wiced_hci_bt_rfcomm_context_t;


/******************************************************
 *               Static Function Declarations
 ******************************************************/
 static void wiced_hci_rfcomm_cb(uint16_t command, uint8_t* payload, uint32_t len);

/******************************************************
 *               External Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/
 wiced_hci_bt_rfcomm_context_t wh_bt_rfcomm_context;

/******************************************************
 *               Function Definitions
 ******************************************************/

 wiced_bt_rfcomm_result_t wiced_bt_rfcomm_create_connection (uint16_t uuid, uint8_t scn,
                                            wiced_bool_t is_server, uint16_t mtu,
                                            wiced_bt_device_address_t bd_addr,
                                            uint16_t *p_handle,
                                            wiced_bt_port_mgmt_cback_t *p_mgmt_cb)
{
    uint8_t*        data  = NULL;
    uint16_t      length  = 0;
    uint8_t*      ptr = NULL;
    wiced_result_t result = WICED_SUCCESS;

    length = sizeof(wiced_bt_device_address_t);
    data = (uint8_t*)calloc(length, sizeof(uint8_t));
    ptr = data;

    wh_bt_rfcomm_context.rfcomm_mgmt_cb = p_mgmt_cb;
    wh_bt_rfcomm_context.rfcomm_context_cb = wiced_hci_rfcomm_cb;
    wiced_hci_set_event_callback(SPP, wh_bt_rfcomm_context.rfcomm_context_cb);

    BDADDR_TO_STREAM(ptr,bd_addr);

    wiced_hci_send( HCI_CONTROL_SPP_COMMAND_CONNECT, data, length );
    free(data);

    return result;
}

wiced_bt_rfcomm_result_t wiced_bt_rfcomm_remove_connection ( uint16_t handle, wiced_bool_t remove_server )
{
    uint8_t*        data  = NULL;
    uint16_t      length  = 0;
    uint8_t*      ptr = NULL;
    wiced_result_t result = WICED_SUCCESS;

    length = sizeof(uint16_t);
    data = (uint8_t*)calloc(length, sizeof(uint8_t));
    ptr = data;

    UINT16_TO_STREAM( ptr, handle );

    wiced_hci_send( HCI_CONTROL_SPP_COMMAND_DISCONNECT, data, length );
    free(data);

    return result;
}


wiced_bt_rfcomm_result_t wiced_bt_rfcomm_write_data (uint16_t handle, char *p_data, uint16_t max_len, uint16_t *p_len)
{
    uint8_t *data = NULL;
    uint16_t length = 0;
    uint8_t *ptr = NULL;
    wiced_result_t result = WICED_SUCCESS;

    length += sizeof(uint16_t) + sizeof(char) + (max_len*sizeof(uint16_t));

    data = (uint8_t*)calloc(length, sizeof(uint8_t));
    ptr = data;

    UINT16_TO_STREAM(ptr,handle);
    memcpy(ptr, p_data, max_len);
    ptr += max_len;

    wiced_hci_send( HCI_CONTROL_SPP_COMMAND_DATA, data, length);
    free(data);

    return result;
}

wiced_bt_rfcomm_result_t wiced_bt_rfcomm_set_data_callback (uint16_t port_handle, wiced_bt_rfcomm_data_cback_t *p_cb)
{
    wiced_result_t result = WICED_SUCCESS;
    wh_bt_rfcomm_context.rfcomm_data_cb = p_cb;
    return result;
}

wiced_bt_rfcomm_result_t wiced_bt_rfcomm_set_event_mask (uint16_t port_handle, wiced_bt_rfcomm_port_event_t mask)
{
    wiced_result_t result = WICED_SUCCESS;
    return result;

}

wiced_bt_rfcomm_result_t wiced_bt_rfcomm_set_event_callback (uint16_t port_handle, wiced_bt_port_event_cback_t *p_port_cb)
{
    wiced_result_t result = WICED_SUCCESS;
    wh_bt_rfcomm_context.rfcomm_port_event_cb = p_port_cb;
    return result;
}

static void wiced_hci_rfcomm_cb(uint16_t command, uint8_t* payload, uint32_t len)
{
     switch(command)
     {
         case HCI_CONTROL_SPP_EVENT_CONNECTED:
         {
            wiced_bt_device_address_t bd_addr;
            uint16_t handle;

            WPRINT_APP_INFO(("HCI_CONTROL_SPP_EVENT_CONNECTED\n"));
            STREAM_TO_BDADDR( bd_addr,payload );
            STREAM_TO_UINT16( handle, payload );

            wh_bt_rfcomm_context.rfcomm_mgmt_cb( WICED_BT_RFCOMM_SUCCESS, handle );
         }
         break;

         case HCI_CONTROL_SPP_EVENT_SERVICE_NOT_FOUND:
         {
            WPRINT_APP_INFO(("HCI_CONTROL_SPP_EVENT_SERVICE_NOT_FOUND\n"));
         }
         break;

         case HCI_CONTROL_SPP_EVENT_CONNECTION_FAILED:
         {
            WPRINT_APP_INFO(("HCI_CONTROL_SPP_EVENT_CONNECTION_FAILED\n"));
         }
         break;

         case HCI_CONTROL_SPP_EVENT_DISCONNECTED:
         {
            WPRINT_APP_INFO(("HCI_CONTROL_SPP_EVENT_DISCONNECTED\n"));
         }
         break;

         case HCI_CONTROL_SPP_EVENT_TX_COMPLETE:
         {
            uint16_t handle;
            uint8_t result;

            WPRINT_APP_INFO(("HCI_CONTROL_SPP_EVENT_TX_COMPLETE\n"));
            STREAM_TO_UINT16(handle,payload);
            STREAM_TO_UINT8(result, payload);
            if(result == WICED_BT_RFCOMM_SUCCESS)
            {
                WPRINT_APP_INFO(("SPP TX event complete\n"));
            }
            else
            {
                WPRINT_APP_INFO(("result = %d handle = %x\n",result,handle));
            }
         }
         break;

         case HCI_CONTROL_SPP_EVENT_RX_DATA:
         {
            uint16_t handle;
            uint8_t *data = NULL;
            uint8_t *ptr = NULL;

            WPRINT_APP_INFO( (" HCI_CONTROL_SPP_EVENT_RX_DATA\n ") );

            data = (uint8_t*)calloc(len, sizeof(uint8_t));
            ptr = payload;

            STREAM_TO_UINT16( handle, ptr );
            memcpy( data, ptr, len-2 );
            /* send it to the application */
            WPRINT_APP_INFO( ( "%x %x\n",data[0],data[1] ) );
            wh_bt_rfcomm_context.rfcomm_data_cb( handle, data, len-2 );
            free(data);
         }
         break;

         case HCI_CONTROL_SPP_EVENT_COMMAND_STATUS:
         {
            uint8_t status;

            WPRINT_APP_INFO(("HCI_CONTROL_SPP_EVENT_COMMAND_STATUS\n"));
            STREAM_TO_UINT8(status,payload);
            WPRINT_APP_INFO(("Status: %d\n",status));
         }
         break;

         default:
         {
            WPRINT_APP_INFO(("this is the default case: %d\n", command));
         }
     }
}


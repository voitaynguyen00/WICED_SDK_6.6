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
 *
 * This file shows how to create a device which implements mesh on/off client model.
 */

#ifdef WICED_BT_MESH_MODEL_DEFAULT_TRANSITION_TIME_CLIENT_INCLUDED

#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_mesh_models.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_mesh_app.h"

#ifdef HCI_CONTROL
#include "wiced_transport.h"
#include "hci_control_api.h"
#endif

#define MESH_DEFAULT_TRANSITION_TIME_CLIENT_ELEMENT_INDEX   0

/******************************************************
 *          Structures
 ******************************************************/

/******************************************************
 *          Function Prototypes
 ******************************************************/
void mesh_default_transition_time_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);
static void mesh_default_transition_time_client_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_default_transition_time_client_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_default_transition_time_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_default_transition_time_data_t *p_data);

/******************************************************
 *          Variables Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/
#if 0
void mesh_app_init(wiced_bool_t is_provisioned)
{
    wiced_bt_mesh_model_default_transition_time_client_init(MESH_DEFAULT_TRANSITION_TIME_CLIENT_ELEMENT_INDEX, mesh_default_transition_time_client_message_handler, is_provisioned);
}
#endif

/*
 * Process event received from the Default Transition Time Server.
 */
void mesh_default_transition_time_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data)
{
#if defined HCI_CONTROL
    wiced_bt_mesh_hci_event_t *p_hci_event;
#endif
    switch (event)
    {
    case WICED_BT_MESH_TX_COMPLETE:
        WICED_BT_TRACE("tx complete status:%d\n", p_event->tx_status);
#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            wiced_bt_mesh_send_hci_tx_complete(p_hci_event, p_event);
#endif
        break;

    case WICED_BT_MESH_DEFAULT_TRANSITION_TIME_STATUS:
        WICED_BT_TRACE("deftt time:%d\n", ((wiced_bt_mesh_default_transition_time_data_t *)p_data)->time);

#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            mesh_default_transition_time_hci_event_send(p_hci_event, (wiced_bt_mesh_default_transition_time_data_t *)p_data);
#endif
        break;

    default:
        break;
    }
    wiced_bt_mesh_release_event(p_event);
}

/*
 * In 2 chip solutions MCU can send commands to change onoff state.
 */
uint32_t mesh_default_transition_time_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length)
{
#ifdef HCI_CONTROL
    wiced_bt_mesh_event_t *p_event;

    switch (opcode)
    {
    case HCI_CONTROL_MESH_COMMAND_DEF_TRANS_TIME_GET:
    case HCI_CONTROL_MESH_COMMAND_DEF_TRANS_TIME_SET:
        break;

    default:
        return WICED_FALSE;
    }
    p_event = wiced_bt_mesh_create_event_from_wiced_hci(opcode, MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_DEFTT_CLNT, &p_data, &length);
    if (p_event == NULL)
    {
        WICED_BT_TRACE("bad hdr\n");
        return WICED_TRUE;
    }
    switch (opcode)
    {
    case HCI_CONTROL_MESH_COMMAND_DEF_TRANS_TIME_GET:
        mesh_default_transition_time_client_get(p_event, p_data, length);
        break;

    case HCI_CONTROL_MESH_COMMAND_DEF_TRANS_TIME_SET:
        mesh_default_transition_time_client_set(p_event, p_data, length);
        break;
    }
#endif
    return WICED_TRUE;
}

/*
 * Send default_transition_time get command
 */
void mesh_default_transition_time_client_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_model_default_transition_time_client_send_get(p_event);
}

/*
 * Send default_transition_time set command
 */
void mesh_default_transition_time_client_set(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_default_transition_time_data_t set_data;

    STREAM_TO_UINT32(set_data.time, p_data);

    wiced_bt_mesh_model_default_transition_time_client_send_set(p_event, &set_data);
}

#ifdef HCI_CONTROL
/*
 * Send default_transition_time Status event over transport
 */
void mesh_default_transition_time_hci_event_send(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_default_transition_time_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT32_TO_STREAM(p, p_data->time);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_DEF_TRANS_TIME_STATUS, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}
#endif

#endif

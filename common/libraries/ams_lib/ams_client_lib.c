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
 * Apple Media Service (AMS) Library.  See
 * https://developer.apple.com/library/ios/documentation/CoreBluetooth/Reference/AppleMediaService_Reference/Introduction/Introduction.html
 *
 * AMS library provides functionality of the BLE AMS client that can interact
 * with an iOS device to control a media player.  The functionality include
 *  - GATT discovery and registration for the AMS notifications from the player.
 *  - Processing to receive various notifications from the player on the iOS device.
 *  - Passing notifications to the client application.
 *  - Passing AMS commands to iOS device.
 */
#include "wiced.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_gatt_util.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_ams.h"
#include "string.h"

#ifdef WICED_BT_TRACE_ENABLE
#define     AMS_TRACE                          WICED_BT_TRACE
#else
#define     AMS_TRACE(...)
#endif


/* Media attributes that can be enabled if needed */
#define AMS_SUPPORT_PLAYER_NAME     0  /* Player name */
#define AMS_SUPPORT_TRACK_POSITION  0  /* Track position */
#define AMS_SUPPORT_PLAYLIST_INFO   0  /* Track index and track count */

/******************************************************
 *                      Constants
 ******************************************************/

// service discovery states
enum
{
    AMS_CLIENT_STATE_IDLE                                           = 0x00,
    AMS_CLIENT_STATE_DISCOVER_ENTITY_UPDATE_CCCD                    = 0x01,
    AMS_CLIENT_STATE_WRITE_ENTITY_SET_CCCD                          = 0x02,
    AMS_CLIENT_STATE_WRITE_ENTITY_UPDATE_PLAYER                     = 0x03,
    AMS_CLIENT_STATE_WRITE_ENTITY_UPDATE_QUEUE                      = 0x04,
    AMS_CLIENT_STATE_WRITE_ENTITY_UPDATE_TRACK                      = 0x05,
    AMS_CLIENT_STATE_WRITE_ENTITY_RESET_CCCD                        = 0x06,
};

/******************************************************
 *                     Structures
 ******************************************************/
typedef struct
{
    uint8_t   state;
    uint16_t  conn_id;
    uint16_t  ams_e_handle;
    uint16_t  remote_control_char_hdl;
    uint16_t  remote_control_val_hdl;
    uint16_t  entity_update_char_hdl;
    uint16_t  entity_update_val_hdl;
    uint16_t  entity_update_cccd_hdl;
    uint16_t  entity_attribute_char_hdl;
    uint16_t  entity_attribute_val_hdl;
} ams_client_t;

/******************************************************
 *               Variables Definitions
 ******************************************************/
ams_client_t  ams_client;

wiced_bt_ams_reg_t  reg;

// following is the list of notification attributes that we are going
// to request for entity player.  Compile out attribute of no interest.
uint8_t  ams_client_player_notification_attribute[] =
{
#if AMS_SUPPORT_PLAYER_NAME
    AMS_PLAYER_ATTRIBUTE_ID_NAME,
#endif
    AMS_PLAYER_ATTRIBUTE_ID_PLAYBACK_INFO,
    AMS_PLAYER_ATTRIBUTE_ID_VOLUME
};

// following is the list of notification attributes that we are going
// to request for entity track.  Compile out attribute of no interest.
uint8_t  ams_client_track_notification_attribute[] =
{
    AMS_TRACK_ATTRIBUTE_ID_ARTIST,
    AMS_TRACK_ATTRIBUTE_ID_ALBUM,
    AMS_TRACK_ATTRIBUTE_ID_TITLE,
#if AMS_SUPPORT_TRACK_POSITION
    AMS_TRACK_ATTRIBUTE_ID_DURATION
#endif
};

// following is the list of notification attributes that we are going
// to request for entity queue.  Compile out attribute of no interest.
uint8_t  ams_client_queue_notification_attribute[] =
{
#if AMS_SUPPORT_PLAYLIST_INFO
    AMS_QUEUE_ATTRIBUTE_ID_INDEX,
    AMS_QUEUE_ATTRIBUTE_ID_COUNT,
#endif
    AMS_QUEUE_ATTRIBUTE_ID_SHUFFLE_MODE,
    AMS_QUEUE_ATTRIBUTE_ID_REPEAT_MODE
};


/******************************************************
 *               Function Prototypes
 ******************************************************/
static wiced_bt_gatt_status_t    ams_client_entity_update_write(uint8_t entity_id, uint8_t *p_attributes, int num_attributes);

/******************************************************
 *               Function Definitions
 ******************************************************/

/*
 * Connection up event from the main application
 */
void wiced_bt_ams_client_connection_up(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    ams_client.conn_id = p_conn_status->conn_id;
}

/*
 * Connection down event from the main application
 */
void wiced_bt_ams_client_connection_down(wiced_bt_gatt_connection_status_t *p_conn_status)
{
	memset (&ams_client, 0, sizeof (ams_client));
}

/*
 * Initialize the AMS Client library.  Application passes the registration structure
 * with callbacks.
 */
void wiced_bt_ams_client_initialize(wiced_bt_ams_reg_t *p_reg)
{
    reg = *p_reg;

    memset (&ams_client, 0, sizeof (ams_client));
}

/*
 * Command from the main app to start search for characteristics
 */
wiced_bool_t wiced_bt_ams_client_discover(uint16_t conn_id, uint16_t s_handle, uint16_t e_handle)
{
    if ((s_handle == 0) || (e_handle == 0))
        return WICED_FALSE;

    ams_client.conn_id      = conn_id;
    ams_client.ams_e_handle = e_handle;
    ams_client.state        = AMS_CLIENT_STATE_IDLE;

    wiced_bt_util_send_gatt_discover(conn_id, GATT_DISCOVER_CHARACTERISTICS, 0, s_handle, e_handle);
    return WICED_TRUE;
}

/*
 * Command from the main app to start the client. The function registers with the server
 * to receive notifications.
 */
wiced_bool_t wiced_bt_ams_client_start(uint16_t conn_id)
{
	if (ams_client.entity_update_cccd_hdl == 0)
	{
		return WICED_FALSE;
	}
    ams_client.state = AMS_CLIENT_STATE_WRITE_ENTITY_SET_CCCD;
    wiced_bt_util_set_gatt_client_config_descriptor(conn_id, ams_client.entity_update_cccd_hdl, GATT_CLIENT_CONFIG_NOTIFICATION);
	return WICED_FALSE;
}

/*
 * Command from the main app to stop the client. The function registers with the server
 * to receive notifications.
 */
wiced_bool_t wiced_bt_ams_client_stop(uint16_t conn_id)
{
    if (ams_client.entity_update_cccd_hdl == 0)
    {
        return WICED_FALSE;
    }
    ams_client.state = AMS_CLIENT_STATE_WRITE_ENTITY_RESET_CCCD;
    wiced_bt_util_set_gatt_client_config_descriptor(conn_id, ams_client.entity_update_cccd_hdl, GATT_CLIENT_CONFIG_NONE);
    return WICED_FALSE;
}

/*
 * While application performs GATT discovery it shall pass discovery results for 
 * the AMS service to the AMS Library.  The library needs to find 3 characteristics
 * remote control, entity update, and entity attribute.  The second has client
 * configuration descriptor (CCCD).
 */
void wiced_bt_ams_client_discovery_result(wiced_bt_gatt_discovery_result_t *p_data)
{
    AMS_TRACE("[%s]\n", __FUNCTION__);

    if (p_data->discovery_type == GATT_DISCOVER_CHARACTERISTICS)
    {
        // Result for characteristic discovery.  Save appropriate handle based on the UUID.
        wiced_bt_gatt_char_declaration_t *p_char = &p_data->discovery_data.characteristic_declaration;
        if (p_char->char_uuid.len == 16)
        {
            if (memcmp(p_char->char_uuid.uu.uuid128, AMS_REMOTE_CONTROL, 16) == 0)
            {
                ams_client.remote_control_char_hdl = p_char->handle;
                ams_client.remote_control_val_hdl  = p_char->val_handle;
                AMS_TRACE("remote control hdl:%04x-%04x", ams_client.remote_control_char_hdl, ams_client.remote_control_val_hdl);
            }
            else if (memcmp(p_char->char_uuid.uu.uuid128, AMS_ENTITY_UPDATE, 16) == 0)
            {
                ams_client.entity_update_char_hdl = p_char->handle;
                ams_client.entity_update_val_hdl  = p_char->val_handle;
                AMS_TRACE("entity update hdl:%04x-%04x", ams_client.entity_update_char_hdl, ams_client.entity_update_val_hdl);
            }
            else if (memcmp(p_char->char_uuid.uu.uuid128, AMS_ENTITY_ATTRIBUTE, 16) == 0)
            {
                ams_client.entity_attribute_char_hdl = p_char->handle;
                ams_client.entity_attribute_val_hdl  = p_char->val_handle;
                AMS_TRACE("entity attribute hdl:%04x-%04x", ams_client.entity_attribute_char_hdl, ams_client.entity_attribute_val_hdl);
            }
        }
    }
    else if ((p_data->discovery_type == GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS) &&
             (p_data->discovery_data.char_descr_info.type.len == 2) &&
             (p_data->discovery_data.char_descr_info.type.uu.uuid16 == UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION))
    {
        if (ams_client.state == AMS_CLIENT_STATE_DISCOVER_ENTITY_UPDATE_CCCD)
        {
            ams_client.entity_update_cccd_hdl = p_data->discovery_data.char_descr_info.handle;
            AMS_TRACE("entity_update_cccd_hdl hdl:%04x", ams_client.entity_update_cccd_hdl);
        }
    }
}

/*
 * While application performs GATT discovery it shall pass discovery complete callbacks
 * for the AMS service to the AMS Library. This function initiates the next discovery
 * request or write request to configure the AMS service on the iOS device.
 */
void wiced_bt_ams_client_discovery_complete(wiced_bt_gatt_discovery_complete_t *p_data)
{
    uint16_t end_handle;

    AMS_TRACE("[%s] state:%d\n", __FUNCTION__, ams_client.state);

    if (p_data->disc_type == GATT_DISCOVER_CHARACTERISTICS)
    {
        // done with AMS characteristics, start reading descriptor handles
        // make sure that all characteristics are present
        if ((ams_client.remote_control_char_hdl   == 0)  ||
            (ams_client.remote_control_val_hdl    == 0)  ||
            (ams_client.entity_update_char_hdl    == 0)  ||
            (ams_client.entity_update_val_hdl     == 0)  ||
            (ams_client.entity_attribute_char_hdl == 0)  ||
            (ams_client.entity_attribute_val_hdl  == 0))
        {
            // something is very wrong
            AMS_TRACE("[%s] failed\n", __FUNCTION__);
            ams_client.state = AMS_CLIENT_STATE_IDLE;
            memset (&ams_client, 0, sizeof (ams_client));
            (*reg.p_discovery_complete_callback)(p_data->conn_id, WICED_FALSE);
            return;
        }

        // search for descriptor from the characteristic value handle until the end of the
        // service or until the start of the next characteristic
        end_handle = ams_client.ams_e_handle;
        if (ams_client.remote_control_char_hdl > ams_client.entity_update_char_hdl)
            end_handle = ams_client.remote_control_char_hdl - 1;
        if ((ams_client.entity_attribute_char_hdl > ams_client.entity_update_char_hdl) && (ams_client.entity_attribute_char_hdl < end_handle))
            end_handle = ams_client.entity_attribute_char_hdl - 1;

        ams_client.state = AMS_CLIENT_STATE_DISCOVER_ENTITY_UPDATE_CCCD;
        wiced_bt_util_send_gatt_discover(p_data->conn_id, GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                         ams_client.entity_update_val_hdl + 1, end_handle);
    }
    else if (p_data->disc_type == GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS)
    {
        if (ams_client.state == AMS_CLIENT_STATE_DISCOVER_ENTITY_UPDATE_CCCD)
        {
            // done with descriptor discovery.
            ams_client.state = AMS_CLIENT_STATE_IDLE;
            (*reg.p_discovery_complete_callback)(p_data->conn_id, WICED_TRUE);
        }
    }
}

/*
 * Process write response from the stack.
 * Application passes it here if handle belongs to our service.
 */
void wiced_bt_ams_client_write_rsp(wiced_bt_gatt_operation_complete_t *p_data)
{
    AMS_TRACE("[%s] state:%02x status:%d\n", __FUNCTION__, ams_client.state, p_data->status);

    // If result is failure, we are probably not paired
    if (p_data->status != WICED_BT_GATT_SUCCESS)
    {
        ams_client.state = AMS_CLIENT_STATE_IDLE;
        if (ams_client.state == AMS_CLIENT_STATE_WRITE_ENTITY_RESET_CCCD)
        {
            (*reg.p_stop_complete_callback)(p_data->conn_id, WICED_FALSE);
        }
        else
        {
            (*reg.p_start_complete_callback)(p_data->conn_id, WICED_TRUE);
        }
    }

    // Write response should be processed according to what we were writing, i.e. current state.
    switch (ams_client.state)
    {
    case AMS_CLIENT_STATE_WRITE_ENTITY_SET_CCCD:
        // if we were writing to client configuration descriptor, start registration
        // for specific attributes
        ams_client.state = AMS_CLIENT_STATE_WRITE_ENTITY_UPDATE_PLAYER;
        if (sizeof(ams_client_player_notification_attribute) != 0)
        {
            ams_client_entity_update_write(AMS_ENTITY_ID_PLAYER, ams_client_player_notification_attribute, sizeof(ams_client_player_notification_attribute));
        }
        break;

    case AMS_CLIENT_STATE_WRITE_ENTITY_UPDATE_PLAYER:
        ams_client.state = AMS_CLIENT_STATE_WRITE_ENTITY_UPDATE_QUEUE;
        if (sizeof(ams_client_queue_notification_attribute) != 0)
        {
            ams_client_entity_update_write(AMS_ENTITY_ID_QUEUE, ams_client_queue_notification_attribute, sizeof(ams_client_queue_notification_attribute));
        }
        break;

    case AMS_CLIENT_STATE_WRITE_ENTITY_UPDATE_QUEUE:
        ams_client.state = AMS_CLIENT_STATE_WRITE_ENTITY_UPDATE_TRACK;
        if (sizeof(ams_client_track_notification_attribute) != 0)
        {
            ams_client_entity_update_write(AMS_ENTITY_ID_TRACK, ams_client_track_notification_attribute, sizeof(ams_client_track_notification_attribute));
        }
        break;

    case AMS_CLIENT_STATE_WRITE_ENTITY_UPDATE_TRACK:
        ams_client.state = AMS_CLIENT_STATE_IDLE;
        (*reg.p_start_complete_callback)(p_data->conn_id, WICED_TRUE);
        break;

    case AMS_CLIENT_STATE_WRITE_ENTITY_RESET_CCCD:
        ams_client.state = AMS_CLIENT_STATE_IDLE;
        (*reg.p_stop_complete_callback)(p_data->conn_id, WICED_TRUE);
        break;
    }
}

/*
 * Process GATT Notifications from the iOS device.  Application passes the received notification
 * to this function if the handle belongs to the AMS service.  This function verifies the data and if
 * successful fills the information in the event.
 */
wiced_bool_t wiced_bt_ams_client_process_notification(wiced_bt_gatt_operation_complete_t *p_data, wiced_bt_ams_event_t *p_event)
{
    uint16_t    handle = p_data->response_data.att_value.handle;
    uint8_t     *data  = p_data->response_data.att_value.p_data;
    uint16_t    len    = p_data->response_data.att_value.len;
    uint16_t    opcode;

    // this service should only receive notifications for entity update characteristic
    // first 3 bytes are hardcoded at Entity ID, Attribute ID and EntityUpdateFlags
    if ((handle != ams_client.entity_update_val_hdl) || (len < 3) || (p_event == NULL))
    {
        AMS_TRACE("AMS Notification bad handle:%02x, %d\n", (uint16_t )handle, len);
        return WICED_FALSE;
    }

    p_event->entity_id      = data[0];
    p_event->attribute_id   = data[1];
    p_event->flags          = data[2];
    p_event->attribute_str  = &data[3];
    p_event->attribute_len  = len - 3;

    return (WICED_TRUE);
}

/*
 * Send command to iOS device to indicate which attributes are interested in for specific entity.
 */
wiced_bt_gatt_status_t ams_client_entity_update_write(uint8_t entity_id, uint8_t *p_attributes, int num_attributes)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;
    uint8_t                buf[sizeof(wiced_bt_gatt_value_t) + 10];
    wiced_bt_gatt_value_t *p_write = (wiced_bt_gatt_value_t*)buf;

    // Allocating a buffer to send the write request
    memset(buf, 0, sizeof(buf));

    p_write->handle   = ams_client.entity_update_val_hdl;
    p_write->len      = num_attributes + 1;
    p_write->auth_req = GATT_AUTH_REQ_NONE;
    p_write->value[0] = entity_id;
    memcpy (&p_write->value[1], p_attributes, num_attributes);

    status = wiced_bt_gatt_send_write(ams_client.conn_id, GATT_WRITE, p_write);

    AMS_TRACE("wiced_bt_gatt_send_write conn_id:%d %d\n", ams_client.conn_id, status);
    return status;
}

/*
 * Send remote control command to iOS server.
 */
wiced_bt_gatt_status_t wiced_bt_ams_send_remote_command(uint16_t conn_id, uint16_t ams_command)
{
    wiced_bt_gatt_value_t  write;
    wiced_bt_gatt_status_t status;

    // Allocating a buffer to send the write request
    memset(&write, 0, sizeof(wiced_bt_gatt_value_t));

    write.handle   = ams_client.remote_control_val_hdl;
    write.len      = 1;
    write.auth_req = GATT_AUTH_REQ_NONE;

    write.value[0] = ams_command;

    status = wiced_bt_gatt_send_write(conn_id, GATT_WRITE, &write);
    AMS_TRACE("wiced_bt_gatt_send_write conn_id:%d %d\n", conn_id, status);

    return (status);
}


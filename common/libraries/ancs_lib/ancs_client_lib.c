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
 * Apple Notification Center Service (ANCS) library.
 *
 * For reference and ANCS definitions see
 * https://developer.apple.com/library/IOS/documentation/CoreBluetooth/Reference/AppleNotificationCenterServiceSpecification/Specification/Specification.html
 *
 * Features demonstrated
 *  - GATT discovery and registration for the ANCS notifications from the ANCS server.
 *  - Processing to receive various notifications from the server on the iOS device.
 *  - Passing notifications to the client application.
 *  - Passing ANCS action commands to iOS device.
 */
#include "wiced.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_trace.h"
#include "wiced_memory.h"
#include "wiced_timer.h"
#include "wiced_bt_ancs.h"
#include "wiced_bt_gatt_util.h"
#include "string.h"

#ifdef WICED_BT_TRACE_ENABLE
#define     ANCS_TRACE                          WICED_BT_TRACE
#else
#define     ANCS_TRACE(...)
#endif

#define ANCS_ADDITIONAL_TRACE           0

/******************************************************
 *                      Constants
 ******************************************************/

#ifdef WICED_BT_TRACE_ENABLE
#ifdef ANCS_ADDITIONAL_TRACE
static char *EventId[] =
{
    "Added",
    "Modified",
    "Removed",
    "Unknown"
};

#define ANCS_CATEGORY_ID_MAX    12
static char *CategoryId[] =
{
    "Other",
    "IncomingCall",
    "MissedCall",
    "Voicemail",
    "Social",
    "Schedule",
    "Email",
    "News",
    "HealthAndFitness",
    "BusinessAndFinance",
    "Location",
    "Entertainment",
    "Unknown"
};

static char *NotificationAttributeID[] =
{
    "AppIdentifier",
    "Title",
    "Subtitle",
    "Message",
    "MessageSize",
    "Date",
    "PositiveActLabel",
    "NegativeActLabel",
    "Unknown"
};
#endif
#endif

#define ANCS_COMMAND_ID_GET_NOTIFICATION_ATTRIBUTES     0
#define ANCS_COMMAND_ID_GET_APP_ATTRIBUTES              1
#define ANCS_COMMAND_ID_PERFORM_NOTIFICATION_ACTION     2

// service discovery states
enum
{
    ANCS_CLIENT_STATE_IDLE                                           = 0x00,
    ANCS_CLIENT_STATE_DISCOVER_NOTIFICATION_SOURCE_CCCD              = 0x01,
    ANCS_CLIENT_STATE_DISCOVER_DATA_SOURCE_CCCD                      = 0x02,
    ANCS_CLIENT_STATE_SET_DATA_SOURCE_CCCD                           = 0x03,
    ANCS_CLIENT_STATE_SET_NOTIFICATION_SOURCE_CCCD                   = 0x04,
    ANCS_CLIENT_STATE_RESET_DATA_SOURCE_CCCD                         = 0x05,
    ANCS_CLIENT_STATE_RESET_NOTIFICATION_SOURCE_CCCD                 = 0x06,
};


/******************************************************
 *                     Structures
 ******************************************************/

typedef struct t_ANCS_CLIENT
{
    uint8_t   state;
    uint8_t   notification_attribute_inx;
    uint16_t  conn_id;
    uint16_t  ancs_e_handle;
    uint16_t  notification_source_char_hdl;
    uint16_t  notification_source_val_hdl;
    uint16_t  notification_source_cccd_hdl;
    uint16_t  control_point_char_hdl;
    uint16_t  control_point_val_hdl;
    uint16_t  data_source_char_hdl;
    uint16_t  data_source_val_hdl;
    uint16_t  data_source_cccd_hdl;

    ancs_event_t* p_first_event;

    uint16_t  data_left_to_read;
    uint16_t  data_source_buffer_offset;
    uint8_t   data_source_buffer[256];
} ancs_client_t;

// ANCS queued event description. Notification is queued while
// we are busy retrieving data from the current notification.
typedef struct
{
    void        *p_next;
    uint32_t    notification_uid;
    uint8_t     command;
    uint8_t     flags;
    uint8_t     category;
} ancs_queued_event_t;

/******************************************************
 *               Variables Definitions
 ******************************************************/
ancs_client_t   ancs_client;

wiced_bt_ancs_reg_t  reg;

wiced_timer_t   ancs_retry_timer;

uint8_t *       ancs_client_event_pool = NULL;

/******************************************************
 *               Function Prototypes
 ******************************************************/

static  void                    ancs_retry_timeout(uint32_t count);
static  wiced_bt_gatt_status_t  ancs_client_send_next_get_notification_attributes_command(uint32_t uid);

/******************************************************
 *               Function Definitions
 ******************************************************/

/*
 * Initialize the ANCS Client library.  Application passes the registration structure
 * with callbacks.
 */
void wiced_bt_ancs_client_initialize(wiced_bt_ancs_reg_t *p_reg)
{
    reg = *p_reg;
    memset (&ancs_client, 0, sizeof (ancs_client));

    /* Initialize retry timer */
    wiced_init_timer (&ancs_retry_timer, &ancs_retry_timeout, 0, WICED_SECONDS_TIMER);
}

/*
 * Connection up event from the main application
 */
void wiced_bt_ancs_client_connection_up(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    ancs_client.conn_id = p_conn_status->conn_id;
}

/*
 * Connection down event from the main application
 */
void wiced_bt_ancs_client_connection_down(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    /* Stop retry timer */
    wiced_stop_timer (&ancs_retry_timer);

    memset (&ancs_client, 0, sizeof (ancs_client));
}

/*
 * Command from the main app to start search for characteristics
 */
wiced_bool_t wiced_bt_ancs_client_discover(uint16_t conn_id, uint16_t s_handle, uint16_t e_handle)
{
    if ((s_handle == 0) || (e_handle == 0))
        return WICED_FALSE;

    ancs_client.ancs_e_handle = e_handle;
    ancs_client.state         = ANCS_CLIENT_STATE_IDLE;

    wiced_bt_util_send_gatt_discover(conn_id, GATT_DISCOVER_CHARACTERISTICS, 0, s_handle, e_handle);
    return WICED_TRUE;
}

/*
 * Command from the main app to start the client. The function registers with the server
 * to receive notifications.
 */
wiced_bt_gatt_status_t wiced_bt_ancs_client_start(uint16_t conn_id)
{
    // verify that CCCD has been discovered
    if ((ancs_client.data_source_cccd_hdl == 0) || (ancs_client.notification_source_cccd_hdl == 0))
    {
        return WICED_BT_GATT_NOT_FOUND;
    }
    // Register for notifications
    ancs_client.state = ANCS_CLIENT_STATE_SET_DATA_SOURCE_CCCD;
    return (wiced_bt_util_set_gatt_client_config_descriptor(conn_id, ancs_client.data_source_cccd_hdl, GATT_CLIENT_CONFIG_NOTIFICATION));
}

/*
 * Command from the main app to stop the client. The function registers with the server
 * to receive notifications.
 */
wiced_bt_gatt_status_t wiced_bt_ancs_client_stop(uint16_t conn_id)
{
    if ((ancs_client.data_source_cccd_hdl == 0) || (ancs_client.notification_source_cccd_hdl == 0))
    {
        return WICED_BT_GATT_NOT_FOUND;
    }
    ancs_client.state = ANCS_CLIENT_STATE_RESET_DATA_SOURCE_CCCD;
    return (wiced_bt_util_set_gatt_client_config_descriptor(conn_id, ancs_client.data_source_cccd_hdl, GATT_CLIENT_CONFIG_NONE));
}

/*
 * While application performs GATT discovery it shall pass discovery results for 
 * for the handles that belong to ANCS service, to this function.
 * Process discovery results from the stack.  We are looking for 3 characteristics
 * notification source, data source, and control point.  First 2 have client
 * configuration descriptors (CCCD).
 */
void wiced_bt_ancs_client_discovery_result(wiced_bt_gatt_discovery_result_t *p_data)
{
    ANCS_TRACE("[%s]\n", __FUNCTION__);

    if (p_data->discovery_type == GATT_DISCOVER_CHARACTERISTICS)
    {
        // Result for characteristic discovery.  Save appropriate handle based on the UUID.
        wiced_bt_gatt_char_declaration_t *p_char = &p_data->discovery_data.characteristic_declaration;
        if (p_char->char_uuid.len == 16)
        {
            if (memcmp(p_char->char_uuid.uu.uuid128, ANCS_NOTIFICATION_SOURCE, 16) == 0)
            {
                ancs_client.notification_source_char_hdl = p_char->handle;
                ancs_client.notification_source_val_hdl  = p_char->val_handle;
                ANCS_TRACE("notification source hdl:%04x-%04x", ancs_client.notification_source_char_hdl, ancs_client.notification_source_val_hdl);
            }
            else if (memcmp(p_char->char_uuid.uu.uuid128, ANCS_CONTROL_POINT, 16) == 0)
            {
                ancs_client.control_point_char_hdl = p_char->handle;
                ancs_client.control_point_val_hdl  = p_char->val_handle;
                ANCS_TRACE("control hdl:%04x-%04x", ancs_client.control_point_char_hdl, ancs_client.control_point_val_hdl);
            }
            else if (memcmp(p_char->char_uuid.uu.uuid128, ANCS_DATA_SOURCE, 16) == 0)
            {
                ancs_client.data_source_char_hdl = p_char->handle;
                ancs_client.data_source_val_hdl  = p_char->val_handle;
                ANCS_TRACE("data source hdl:%04x-%04x", ancs_client.data_source_char_hdl, ancs_client.data_source_val_hdl);
            }
        }
    }
    else if ((p_data->discovery_type == GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS) &&
             (p_data->discovery_data.char_descr_info.type.len == 2) &&
             (p_data->discovery_data.char_descr_info.type.uu.uuid16 == UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION))
    {
        // result for descriptor discovery, save appropriate handle based on the state
        if (ancs_client.state == ANCS_CLIENT_STATE_DISCOVER_NOTIFICATION_SOURCE_CCCD)
        {
            ancs_client.notification_source_cccd_hdl = p_data->discovery_data.char_descr_info.handle;
            ANCS_TRACE("notification_source_cccd_hdl hdl:%04x", ancs_client.notification_source_cccd_hdl);
        }
        else if (ancs_client.state == ANCS_CLIENT_STATE_DISCOVER_DATA_SOURCE_CCCD)
        {
            ancs_client.data_source_cccd_hdl = p_data->discovery_data.char_descr_info.handle;
            ANCS_TRACE("data_source_cccd_hdl hdl:%04x", ancs_client.data_source_cccd_hdl);
        }
    }
}

/*
 * While application performs GATT discovery it shall pass discovery complete callbacks
 * for the ANCS service to the ANCS Library. This function initiates the next discovery
 * request or write request to configure the ANCS service on the iOS device.
 */
void wiced_bt_ancs_client_discovery_complete(wiced_bt_gatt_discovery_complete_t *p_data)
{
    uint16_t end_handle;

    ANCS_TRACE("[%s] state:%d\n", __FUNCTION__, ancs_client.state);

    if (p_data->disc_type == GATT_DISCOVER_CHARACTERISTICS)
    {
        // done with ANCS characteristics, start reading descriptor handles
        // make sure that all characteristics are present
        if ((ancs_client.notification_source_char_hdl == 0) ||
            (ancs_client.notification_source_val_hdl == 0) ||
            (ancs_client.control_point_char_hdl == 0) ||
            (ancs_client.control_point_val_hdl == 0) ||
            (ancs_client.data_source_char_hdl == 0) ||
            (ancs_client.data_source_val_hdl == 0))
        {
            // something is very wrong
            ANCS_TRACE("[%s] failed\n", __FUNCTION__);
            ancs_client.state = ANCS_CLIENT_STATE_IDLE;
            memset (&ancs_client, 0, sizeof (ancs_client));
            (*reg.p_discovery_complete_callback)(p_data->conn_id, WICED_FALSE);
            return;
        }

        // search for descriptor from the characteristic characteristic until the end of the
        // service or until the start of the next characteristic
        end_handle = ancs_client.ancs_e_handle;
        if (ancs_client.control_point_char_hdl > ancs_client.notification_source_char_hdl)
            end_handle = ancs_client.control_point_char_hdl - 1;
        if ((ancs_client.data_source_char_hdl > ancs_client.notification_source_char_hdl) && (ancs_client.data_source_char_hdl < end_handle))
            end_handle = ancs_client.data_source_char_hdl - 1;

        ancs_client.state = ANCS_CLIENT_STATE_DISCOVER_NOTIFICATION_SOURCE_CCCD;
        wiced_bt_util_send_gatt_discover(p_data->conn_id, GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                         ancs_client.notification_source_val_hdl + 1, end_handle);
    }
    else if (p_data->disc_type == GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS)
    {
        if (ancs_client.state == ANCS_CLIENT_STATE_DISCOVER_NOTIFICATION_SOURCE_CCCD)
        {
            // search for descriptor from the characteristic characteristic until the end of the
            // service or until the handle of the next characteristic
            end_handle = ancs_client.ancs_e_handle;
            if (ancs_client.control_point_char_hdl > ancs_client.data_source_char_hdl)
                end_handle = ancs_client.control_point_char_hdl - 1;
            if ((ancs_client.notification_source_char_hdl > ancs_client.data_source_char_hdl) && (ancs_client.notification_source_char_hdl < end_handle))
                end_handle = ancs_client.notification_source_char_hdl - 1;

            ancs_client.state = ANCS_CLIENT_STATE_DISCOVER_DATA_SOURCE_CCCD;
            ANCS_TRACE("send discover ancs_client_state:%02x %04x %04x\n", ancs_client.state, ancs_client.data_source_val_hdl + 1, end_handle - 1);
            wiced_bt_util_send_gatt_discover(p_data->conn_id, GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                             ancs_client.data_source_val_hdl + 1, end_handle);
        }
        else if (ancs_client.state == ANCS_CLIENT_STATE_DISCOVER_DATA_SOURCE_CCCD)
        {
            // done with descriptor discovery.
            ancs_client.state = ANCS_CLIENT_STATE_IDLE;
            (*reg.p_discovery_complete_callback)(p_data->conn_id, WICED_TRUE);
        }
    }
}

/*
 * Process write response from the stack.
 * Application passes it here if handle belongs to our service.
 */
void wiced_bt_ancs_client_write_rsp(wiced_bt_gatt_operation_complete_t *p_data)
{
    ANCS_TRACE("[%s] state:%02x rc:%d\n", __FUNCTION__, ancs_client.state, p_data->status);

    // if we were writing 1 to notification source, still need to write 1 to data source
    switch (ancs_client.state)
    {
    case ANCS_CLIENT_STATE_SET_DATA_SOURCE_CCCD:
        if (p_data->status != 0)
        {
            ancs_client.state = ANCS_CLIENT_STATE_IDLE;
            (*reg.p_start_complete_callback)(p_data->conn_id, p_data->status);
        }
        else
        {
            ancs_client.state = ANCS_CLIENT_STATE_SET_NOTIFICATION_SOURCE_CCCD;
            wiced_bt_util_set_gatt_client_config_descriptor(p_data->conn_id, ancs_client.notification_source_cccd_hdl, GATT_CLIENT_CONFIG_NOTIFICATION);
        }
        break;

    // if we were writing 1 to data source, done with initialization
    case ANCS_CLIENT_STATE_SET_NOTIFICATION_SOURCE_CCCD:
        ancs_client.state = ANCS_CLIENT_STATE_IDLE;
        (*reg.p_start_complete_callback)(p_data->conn_id, p_data->status);
        break;

    case ANCS_CLIENT_STATE_RESET_DATA_SOURCE_CCCD:
        if (p_data->status != 0)
        {
            ancs_client.state = ANCS_CLIENT_STATE_IDLE;
            (*reg.p_start_complete_callback)(p_data->conn_id, p_data->status);
        }
        else
        {
            ancs_client.state = ANCS_CLIENT_STATE_RESET_NOTIFICATION_SOURCE_CCCD;
            wiced_bt_util_set_gatt_client_config_descriptor(p_data->conn_id, ancs_client.notification_source_cccd_hdl, GATT_CLIENT_CONFIG_NONE);
        }
        break;

    // if we were writing 1 to data source, done with initialization
    case ANCS_CLIENT_STATE_RESET_NOTIFICATION_SOURCE_CCCD:
        ancs_client.state = ANCS_CLIENT_STATE_IDLE;
        (*reg.p_stop_complete_callback)(p_data->conn_id, p_data->status);
        break;
    }
}

/*
 * The function invoked on retry timeout retry sending attribute request
 */
void ancs_retry_timeout(uint32_t count)
{
    wiced_bt_gatt_status_t status;

    ANCS_TRACE("%s\n", __FUNCTION__);

    /* Stop retry timer */
    wiced_stop_timer (&ancs_retry_timer);

    if (ancs_client.p_first_event != 0)
    {
        status = ancs_client_send_next_get_notification_attributes_command(ancs_client.p_first_event->notification_uid);
        if (status == WICED_BT_GATT_BUSY)
        {
            // another GATT procedure is currently active, retry in a second
            ANCS_TRACE("busy retrieve:%d\n", ancs_client.p_first_event->notification_uid);
            wiced_start_timer(&ancs_retry_timer, 1);
        }
    }
}

/*
 * Send command to the phone to get notification attributes.
 */
wiced_bt_gatt_status_t ancs_client_send_next_get_notification_attributes_command(uint32_t uid)
{
    uint8_t                buf[sizeof(wiced_bt_gatt_value_t) + 10];
    wiced_bt_gatt_value_t  *p_write = (wiced_bt_gatt_value_t *)buf;
    uint8_t                *p_command = p_write->value;
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;

    // Allocating a buffer to send the write request
    memset(buf, 0, sizeof(buf));

    p_write->handle   = ancs_client.control_point_val_hdl;
    p_write->offset   = 0;
    p_write->auth_req = GATT_AUTH_REQ_NONE;

    *p_command++ = ANCS_COMMAND_ID_GET_NOTIFICATION_ATTRIBUTES;
    *p_command++ = uid & 0xff;
    *p_command++ = (uid >> 8) & 0xff;
    *p_command++ = (uid >> 16) & 0xff;
    *p_command++ = (uid >> 24) & 0xff;

    *p_command++ = ancs_client_notification_attribute[ancs_client.notification_attribute_inx];
    if (ancs_client_notification_attribute_length[ancs_client.notification_attribute_inx] != 0)
    {
        *p_command++ = ancs_client_notification_attribute_length[ancs_client.notification_attribute_inx] & 0xff;
        *p_command++ = (ancs_client_notification_attribute_length[ancs_client.notification_attribute_inx] >> 8) & 0xff;
    }
    p_write->len      = (uint8_t)(p_command - p_write->value);
    status = wiced_bt_gatt_send_write (ancs_client.conn_id, GATT_WRITE, p_write);

    ANCS_TRACE("%s status:%d", __FUNCTION__, status);
    return status;
}

/*
 * Send command to the phone to perform specified action
 */
wiced_bt_gatt_status_t wiced_bt_ancs_perform_action(uint16_t conn_id, uint32_t uid, uint32_t action_id)
{
    uint8_t                buf[sizeof(wiced_bt_gatt_value_t) + 10];
    wiced_bt_gatt_value_t  *p_write = (wiced_bt_gatt_value_t *)buf;
    uint8_t                *p_command = p_write->value;
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;

    ANCS_TRACE("%s uid:%d action:%d\n", __FUNCTION__, uid, action_id);

    if (ancs_client.control_point_val_hdl == 0)
    {
        return WICED_BT_GATT_NOT_FOUND;
    }
    // Allocating a buffer to send the write request
    memset(buf, 0, sizeof(buf));

    p_write->handle   = ancs_client.control_point_val_hdl;
    p_write->offset   = 0;
    p_write->auth_req = GATT_AUTH_REQ_NONE;

    *p_command++ = ANCS_COMMAND_ID_PERFORM_NOTIFICATION_ACTION;
    *p_command++ = uid & 0xff;
    *p_command++ = (uid >> 8) & 0xff;
    *p_command++ = (uid >> 16) & 0xff;
    *p_command++ = (uid >> 24) & 0xff;

    *p_command++ = action_id;

    p_write->len      = (uint8_t)(p_command - p_write->value);
    status = wiced_bt_gatt_send_write (conn_id, GATT_WRITE, p_write);

    ANCS_TRACE("%s status:%d", __FUNCTION__, status);
    return (status);
}

/*
 * Process Notification Source messages from the phone.
 * If it is a new or modified notification start reading attributes.
 */
void ancs_client_process_notification_source(uint8_t *data, int len)
{
    wiced_bt_gatt_status_t status;
    ancs_event_t *         p_ancs_event;
    ancs_event_t *         p_prev = NULL;

    // Skip all pre-existing events
    if (data[1] & ANCS_EVENT_FLAG_PREEXISTING)
    {
        ANCS_TRACE("skipped preexisting event UID:%d\n", data[4] + (data[5] << 8) + (data[6] << 16) + (data[7] << 24));
        return;
    }

    switch (data[0])
    {
    case ANCS_EVENT_ID_NOTIFICATION_ADDED:
    case ANCS_EVENT_ID_NOTIFICATION_MODIFIED:
    case ANCS_EVENT_ID_NOTIFICATION_REMOVED:
        break;

    default:
        ANCS_TRACE("unknown command:%d\n", data[0]);
        return;
    }

    // if it is first notification, get the buffer to fill all information
    // if we are just queuing the notification, allocate small buffer from the pool
    if (ancs_client.p_first_event == NULL)
    {
        if ((p_ancs_event = (ancs_event_t *) wiced_bt_get_buffer(sizeof(ancs_event_t))) == NULL)
        {
            ANCS_TRACE("Failed to get buf\n");
            return;
        }
        memset (p_ancs_event, 0, sizeof(ancs_event_t));
    }
    else
    {
        if ((p_ancs_event = (ancs_event_t *) wiced_bt_get_buffer_from_pool((wiced_bt_buffer_pool_t*)ancs_client_event_pool)) == NULL)
        {
            ANCS_TRACE("Failed to get pool buf pool:%d\n", ancs_client_event_pool);
            return;
        }
        ANCS_TRACE("buf from pool:%d\n", p_ancs_event);
        memset (p_ancs_event, 0, sizeof(ancs_queued_event_t));
    }
    p_ancs_event->notification_uid = data[4] + (data[5] << 8) + (data[6] << 16) + (data[7] << 24);

#if ANCS_ADDITIONAL_TRACE
    ANCS_TRACE ("ANCS Notification EventID:%s EventFlags:%04x CategoryID:%s CategoryCount:%d UID:%04x",
            (uint32_t)(data[0] < ANCS_EVENT_ID_MAX) ? EventId[data[0]] : EventId[ANCS_EVENT_ID_MAX],
            data[1],
            (uint32_t)(data[2] < ANCS_CATEGORY_ID_MAX) ? CategoryId[data[2]] : CategoryId[ANCS_CATEGORY_ID_MAX],
            data[3],
            p_ancs_event->notification_uid);
    if (len > 8)
        wiced_trace_array(&data[8], len - 8);
#endif


    ANCS_TRACE("notification type:%d, uuid:%d\n", data[0], p_ancs_event->notification_uid);

    p_ancs_event->command  = data[0];
    p_ancs_event->flags    = data[1];
    p_ancs_event->category = data[2];

    // if we do not need to get details, and if there is nothing in the queue, can ship it out now
    if ((p_ancs_event->command == ANCS_EVENT_ID_NOTIFICATION_REMOVED) && (ancs_client.p_first_event == NULL))
    {
        reg.p_notification_callback(ancs_client.conn_id, p_ancs_event);
        return;
    }
    // enqueue new event at the end of the queue
    if (ancs_client.p_first_event == NULL)
        ancs_client.p_first_event = p_ancs_event;
    else
    {
        for (p_prev = ancs_client.p_first_event; p_prev->p_next != NULL; p_prev = p_prev->p_next)
            ;
        p_prev->p_next = p_ancs_event;
    }

    if ((p_ancs_event->command == ANCS_EVENT_ID_NOTIFICATION_ADDED) || (p_ancs_event->command == ANCS_EVENT_ID_NOTIFICATION_MODIFIED))
    {
        // if we could not send previous request, need to wait for timer to expire.
        if (wiced_is_timer_in_use(&ancs_retry_timer))
        {
            return;
        }
        // if we are currently in process of dealing with another event just return
        if (ancs_client.p_first_event == p_ancs_event)
        {
            status = ancs_client_send_next_get_notification_attributes_command(p_ancs_event->notification_uid);
            if (status == WICED_BT_GATT_BUSY)
            {
                // another GATT procedure is currently active, retry in a second
                ANCS_TRACE("busy retrieve:%d\n", p_ancs_event->notification_uid);
                wiced_start_timer(&ancs_retry_timer, 1);
            }
            else if (status != WICED_BT_GATT_SUCCESS)
            {
                ANCS_TRACE("ancs gatt failed:%02x uid:%d\n", status, p_ancs_event->notification_uid);
                wiced_bt_free_buffer(p_ancs_event);
                if (ancs_client.p_first_event == p_ancs_event)
                {
                    ancs_client.p_first_event = NULL;
                }
                else
                {
                    if (p_prev)
                        p_prev->p_next = p_ancs_event;
                }
            }
        }
        else
        {
            ANCS_TRACE("will retrieve details later\n");
        }
    }
}

/*
 * Process additional message attributes.  The header file defines which attributes
 * we are asking for.
 */
void ancs_client_process_event_attribute(uint8_t  *data, int len)
{
    uint8_t                 type = data[0];
    uint16_t                length = data[1] + (data[2] << 8);
    uint8_t *               p_event_data = &data[3];
    ancs_event_t *          p_event = ancs_client.p_first_event;
    ancs_queued_event_t *   p_queued_event;
    wiced_bt_gatt_status_t  status;

    ancs_client.data_left_to_read         = 0;
    ancs_client.data_source_buffer_offset = 0;

    switch(type)
    {
#ifdef ANCS_NOTIFICATION_ATTR_ID_APP_ID
    case ANCS_NOTIFICATION_ATTR_ID_APP_ID:
        break;
#endif
#ifdef ANCS_NOTIFICATION_ATTR_ID_TITLE
    case ANCS_NOTIFICATION_ATTR_ID_TITLE:
        memcpy(p_event->title, p_event_data, (length < (sizeof(p_event->title) - 1) ? length : (sizeof(p_event->title) - 1)));
        break;
#endif
#ifdef ANCS_NOTIFICATION_ATTR_ID_SUBTITLE
    case ANCS_NOTIFICATION_ATTR_ID_SUBTITLE:
        break;
#endif
#ifdef ANCS_NOTIFICATION_ATTR_ID_MESSAGE_SIZE
    case ANCS_NOTIFICATION_ATTR_ID_MESSAGE_SIZE:
        break;
#endif
#ifdef ANCS_NOTIFICATION_ATTR_ID_MESSAGE
    case ANCS_NOTIFICATION_ATTR_ID_MESSAGE:
        memcpy(p_event->message, p_event_data, (length < (sizeof(p_event->message) - 1) ? length : (sizeof(p_event->message) - 1)));
        break;
#endif
#ifdef ANCS_NOTIFICATION_ATTR_ID_DATE
    case ANCS_NOTIFICATION_ATTR_ID_DATE:
        break;
#endif
#ifdef ANCS_NOTIFICATION_ATTR_ID_POSITIVE_ACTION_LABEL
    case ANCS_NOTIFICATION_ATTR_ID_POSITIVE_ACTION_LABEL:
        memcpy(p_event->positive_action_label, p_event_data, (length < (sizeof(p_event->positive_action_label) - 1) ? length : (sizeof(p_event->positive_action_label) - 1)));
        break;
#endif
#ifdef ANCS_NOTIFICATION_ATTR_ID_NEGATIVE_ACTION_LABEL
    case ANCS_NOTIFICATION_ATTR_ID_NEGATIVE_ACTION_LABEL:
        memcpy(p_event->negative_action_label, p_event_data, (length < (sizeof(p_event->negative_action_label) - 1) ? length : (sizeof(p_event->negative_action_label) - 1)));
        break;
#endif
    }

    // if we are not done with attributes, request the next one
    if (ancs_client_notification_attribute[++ancs_client.notification_attribute_inx] != 0)
    {
        status = ancs_client_send_next_get_notification_attributes_command(p_event->notification_uid);
        if (status == WICED_BT_GATT_BUSY)
        {
            // another GATT procedure is currently active, retry in a second
            ANCS_TRACE("busy retrieve:%d\n", p_event->notification_uid);
            wiced_start_timer(&ancs_retry_timer, 1);
        }
    }
    else
    {
        // Done with attributes for current event
        ancs_client.notification_attribute_inx = 0;

        p_event = ancs_client.p_first_event;

        if ((p_queued_event = ancs_client.p_first_event->p_next) != NULL)
        {
            if ((ancs_client.p_first_event = (ancs_event_t *) wiced_bt_get_buffer(sizeof(ancs_event_t))) == NULL)
            {
                ANCS_TRACE("Failed to get buf to copy\n");
                return;
            }
            memset (ancs_client.p_first_event, 0, sizeof(ancs_event_t));

            ancs_client.p_first_event->p_next           = p_queued_event->p_next;
            ancs_client.p_first_event->notification_uid = p_queued_event->notification_uid;
            ancs_client.p_first_event->command          = p_queued_event->command;
            ancs_client.p_first_event->flags            = p_queued_event->flags;
            ancs_client.p_first_event->category         = p_queued_event->category;

            wiced_bt_free_buffer(p_queued_event);
        }
        else
        {
            ancs_client.p_first_event = NULL;
        }
        // ship current event to the application
        reg.p_notification_callback(ancs_client.conn_id, p_event);

        // if next event in the queue is "Removed" ship it out right away
        while (ancs_client.p_first_event != NULL)
        {
            if (ancs_client.p_first_event->command == ANCS_EVENT_ID_NOTIFICATION_REMOVED)
            {
                p_event = ancs_client.p_first_event;
                ancs_client.p_first_event = p_event->p_next;

                reg.p_notification_callback(ancs_client.conn_id, p_event);
            }
            else
            {
                // start reading attributes for the next message
                status = ancs_client_send_next_get_notification_attributes_command(ancs_client.p_first_event->notification_uid);
                if (status == WICED_BT_GATT_BUSY)
                {
                    // another GATT procedure is currently active, retry in a second
                    ANCS_TRACE("busy retrieve:%d\n", ancs_client.p_first_event->notification_uid);
                    wiced_start_timer(&ancs_retry_timer, 1);
                }
                break;
            }
        }
    }
}

/*
 * Process Data Source messages from the phone.
 * This can be new or continuation of the previous message
 */
void ancs_client_process_data_source(uint8_t  *data, int len)
{
    ancs_event_t *p_event;
    uint8_t      attr_id;
    uint8_t      attr_len;

//    ANCS_TRACE("Data source left to read:%d len:%d\n", ancs_client.data_left_to_read, len);

    // check if this is a continuation of the previous message
    if (ancs_client.data_left_to_read)
    {
        memcpy(&ancs_client.data_source_buffer[ancs_client.data_source_buffer_offset], data, len);
        ancs_client.data_source_buffer_offset += len;
        ancs_client.data_left_to_read -= len;
        if (ancs_client.data_left_to_read <= 0)
        {
            ancs_client_process_event_attribute(&ancs_client.data_source_buffer[5], ancs_client.data_source_buffer_offset - 5);
        }
    }
    else
    {
        // start of the new message
        attr_id  = data[5];
        attr_len = data[6] + (data[7] << 8);
        ANCS_TRACE("ANCS Data Notification Attribute:%04x len %d\n", attr_id, attr_len);
        if (attr_len <= len - 8)
        {
            ancs_client_process_event_attribute((uint8_t *)&data[5], len - 5);
        }
        else
        {
            // whole message did not fit into the message, phone should send addition data
            memcpy(&ancs_client.data_source_buffer[0], data, len);
            ancs_client.data_source_buffer_offset = len;
            ancs_client.data_left_to_read = attr_len - len + 8;
        }
    }
    UNUSED_VARIABLE(attr_id);
}

/*
 * Process GATT Notifications from the client.  Application passes it here only
 * if the handle belongs to this service.
 */
void wiced_bt_ancs_client_process_notification(wiced_bt_gatt_operation_complete_t *p_data)
{
    uint16_t handle = p_data->response_data.att_value.handle;
    uint8_t  *data  = p_data->response_data.att_value.p_data;
    uint16_t len    = p_data->response_data.att_value.len;

    // We can receive notifications on Notification Source or Data Source
    // Phone also can send several notifications on the data source if it did not fit.
    if (ancs_client.data_left_to_read || (handle == ancs_client.data_source_val_hdl))
    {
        ancs_client_process_data_source(data, len);
    }
    else if (handle == ancs_client.notification_source_val_hdl)
    {
        ancs_client_process_notification_source(data, len);
    }
    else
    {
        ANCS_TRACE("ANCS Notification bad handle:%02x, %d\n", (uint16_t)handle, len);
    }
}


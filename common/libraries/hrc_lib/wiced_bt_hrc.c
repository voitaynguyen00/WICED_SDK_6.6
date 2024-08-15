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
 * Heart Rate Client (HRC) library.
 *
 * Implements Heart Rate Clinet role.
 *
 * Features demonstrated
 *  - Heart Rate Characteristics and associated CCC decriptors.
 *  - Configure for Heart Rate notifications on application request.
 *  - Parse Heart Rate notifications from client and sends notification to application.
 */
#include "wiced.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_hrp.h"
#include "wiced_bt_hrc.h"
#include "wiced_bt_gatt_util.h"
#include "string.h"

#ifdef WICED_BT_TRACE_ENABLE
#define     HRC_LIB_TRACE                          WICED_BT_TRACE
#else
#define     HRC_LIB_TRACE(...)
#endif

#define HRC_HEART_RATE_VALUE_UINT8_FORMAT       0
#define HRC_HEART_RATE_VALUE_UINT16_FORMAT      (0x1 << 0)
#define HRC_SENSOR_CONTACT_DETECTED             (0x1 << 1)
#define HRC_SENSOR_CONTACT_FEATURE_SUPPORTED    (0x1 << 2)
#define HRC_ENERGY_EXPENDED_FIELD_PRESENT       (0x1 << 3)
#define HRC_ENERGY_RR_INTERVAL_FIELD_PRESENT    (0x1 << 4)

/******************************************************
 *                      Constants
 ******************************************************/
enum
{
    HRC_STATE_IDLE              = 0,
    HRC_STATE_DISCOVER          = 1,
    HRC_STATE_DISCOVERED        = 2,    /* Succesfully discovered Heart Service characteristics and descriptors */
    HRC_STATE_START             = 3,    /* intiate configuration to start heart rate notifications from Server */
    HRC_STATE_STOP              = 4     /* intiate configuration to stop heart rate notifications from Server  */
};

/******************************************************
 *                     Structures
 ******************************************************/
typedef struct
{
    uint8_t   state;
    uint16_t  conn_id;

    uint16_t  heart_rate_service_end_handle;

    uint16_t  heart_rate_measure_char_hdl;
    uint16_t  heart_rate_measure_char_val_hdl;
    uint16_t  heart_rate_measure_cccd_hdl;

    uint16_t  sensor_location_char_hdl;
    uint16_t  sensor_location_char_val_hdl;

    uint16_t  control_point_char_hdl;
    uint16_t  control_point_char_val_hdl;
} wiced_bt_hrc_connection_t;

typedef struct
{
    wiced_bt_hcr_callback_t     *p_callback;
    wiced_bt_hrc_connection_t   connection;
} wiced_bt_hrc_cb_t;

/******************************************************
 *               Variables Definitions
 ******************************************************/
static wiced_bt_hrc_cb_t wiced_bt_hrc_cb;

static void hrc_lib_process_heart_rate_notification(wiced_bt_gatt_operation_complete_t *p_data);
static void wiced_bt_hrc_connection_reset(void);

/******************************************************
 *               Function Definitions
 ******************************************************/

/*
 * Initialize the HRC Client library.  Application passes the application's callback.
 */
wiced_result_t wiced_bt_hrc_init(wiced_bt_hcr_callback_t *p_callback)
{
    memset (&wiced_bt_hrc_cb, 0, sizeof (wiced_bt_hrc_cb));
    wiced_bt_hrc_cb.p_callback = p_callback;

    return WICED_SUCCESS;
}

/*
 * Connection up event from the main application
 */
void wiced_bt_hrc_connection_up(uint16_t conn_id)
{
    wiced_bt_hrc_cb.connection.conn_id = conn_id;
}

/*
 * Connection down event from the main application
 */
void wiced_bt_hrc_connection_down(uint16_t conn_id)
{
    wiced_bt_hrc_connection_reset();
}

/*
 * Command from the main app to start search for characteristics
 */
wiced_bt_gatt_status_t wiced_bt_hrc_discover(uint16_t conn_id, uint16_t s_handle, uint16_t e_handle)
{
    wiced_bt_gatt_status_t status;
    
    if ((s_handle == 0) || (e_handle == 0))
        return WICED_BT_GATT_INVALID_HANDLE;

    wiced_bt_hrc_cb.connection.heart_rate_service_end_handle = e_handle;

    status = wiced_bt_util_send_gatt_discover(conn_id, GATT_DISCOVER_CHARACTERISTICS, 0, s_handle, e_handle);
    if (status == WICED_BT_GATT_SUCCESS)
    {
        wiced_bt_hrc_cb.connection.state   = HRC_STATE_DISCOVER;
        return WICED_TRUE;
    }
    else
    {
        return WICED_FALSE;
    }
}

/*
 * Command from the main app to start the client. The function registers with the server
 * to receive notifications.
 */
wiced_bt_gatt_status_t wiced_bt_hrc_start(uint16_t conn_id)
{
    wiced_bt_gatt_status_t status;

    // verify that CCCD has been discovered
    if (wiced_bt_hrc_cb.connection.heart_rate_measure_cccd_hdl == 0 )
    {
        return WICED_BT_GATT_NOT_FOUND;
    }
    if (wiced_bt_hrc_cb.connection.state != HRC_STATE_IDLE)
    {
        return WICED_BT_GATT_BUSY;
    }

    // Register for notifications
    status = wiced_bt_util_set_gatt_client_config_descriptor(conn_id, wiced_bt_hrc_cb.connection.heart_rate_measure_cccd_hdl, GATT_CLIENT_CONFIG_NOTIFICATION);
    if (status == WICED_BT_GATT_SUCCESS)
        wiced_bt_hrc_cb.connection.state = HRC_STATE_START;

    return status;
}

/*
 * Command from the main app to stop the client. The function un-registers with the server
 * to not to receive notifications.
 */
wiced_bt_gatt_status_t wiced_bt_hrc_stop(uint16_t conn_id)
{
    wiced_bt_gatt_status_t status;

    if (wiced_bt_hrc_cb.connection.heart_rate_measure_cccd_hdl == 0 )
    {
        return WICED_BT_GATT_NOT_FOUND;
    }

    if (wiced_bt_hrc_cb.connection.state != HRC_STATE_IDLE)
    {
        return WICED_BT_GATT_BUSY;
    }

    status = wiced_bt_util_set_gatt_client_config_descriptor(conn_id, wiced_bt_hrc_cb.connection.heart_rate_measure_cccd_hdl, GATT_CLIENT_CONFIG_NONE);
    if (status == WICED_BT_GATT_SUCCESS)
        wiced_bt_hrc_cb.connection.state = HRC_STATE_STOP;

    return status;
}

wiced_bt_gatt_status_t wiced_bt_hrc_reset_energy_expended( uint16_t conn_id )
{
    wiced_bt_gatt_value_t write;

    if (wiced_bt_hrc_cb.connection.control_point_char_val_hdl == 0 )
    {
        return WICED_BT_GATT_NOT_FOUND;
    }

    write.handle = wiced_bt_hrc_cb.connection.control_point_char_val_hdl;
    write.len = 1;
    write.offset = 0;
    write.auth_req = GATT_AUTH_REQ_NONE;
    write.value[0] = 1;
    return (wiced_bt_gatt_send_write(conn_id, GATT_WRITE, &write));
}

/*
 * While application performs GATT discovery it shall pass discovery results for 
 * for the handles that belong to HRC service, to this function.
 * Process discovery results from the stack.  We are looking for 3 characteristics
 * notification source, data source, and control point.  First 2 have client
 * configuration descriptors (CCCD).
 */
void wiced_bt_hrc_discovery_result(wiced_bt_gatt_discovery_result_t *p_data)
{
    HRC_LIB_TRACE("[%s]\n", __FUNCTION__);

    uint16_t HEART_RATE_MEASURE_UUID            = UUID_CHARACTERISTIC_HEART_RATE_MEASUREMENT;
    uint16_t HEART_RATE_SENSOR_LOCATION_UUID    = UUID_CHARACTERISTIC_HEART_RATE_SENSOR_LOCATION;
    uint16_t HEART_RATE_CONTROL_POINT_UUID      = UUID_CHARACTERISTIC_HEART_RATE_CONTROL_POINT;

    if (p_data->discovery_type == GATT_DISCOVER_CHARACTERISTICS)
    {
        // Result for characteristic discovery.  Save appropriate handle based on the UUID.
        wiced_bt_gatt_char_declaration_t *p_char = &p_data->discovery_data.characteristic_declaration;
        if (p_char->char_uuid.len == 2)
        {
            if (memcmp(&p_char->char_uuid.uu, &HEART_RATE_MEASURE_UUID, 2) == 0)
            {
                wiced_bt_hrc_cb.connection.heart_rate_measure_char_hdl = p_char->handle;
                wiced_bt_hrc_cb.connection.heart_rate_measure_char_val_hdl  = p_char->val_handle;
                HRC_LIB_TRACE("heart rate measure hdl:%04x-%04x\n", wiced_bt_hrc_cb.connection.heart_rate_measure_char_hdl, wiced_bt_hrc_cb.connection.heart_rate_measure_char_val_hdl);
            }
            else if (memcmp(&p_char->char_uuid.uu, &HEART_RATE_SENSOR_LOCATION_UUID, 2) == 0)
            {
                wiced_bt_hrc_cb.connection.sensor_location_char_hdl = p_char->handle;
                wiced_bt_hrc_cb.connection.sensor_location_char_val_hdl  = p_char->val_handle;
                HRC_LIB_TRACE("sensor location hdl:%04x-%04x\n", wiced_bt_hrc_cb.connection.sensor_location_char_hdl, wiced_bt_hrc_cb.connection.sensor_location_char_val_hdl);
            }
            else if (memcmp(&p_char->char_uuid.uu, &HEART_RATE_CONTROL_POINT_UUID, 2) == 0)
            {
                wiced_bt_hrc_cb.connection.control_point_char_hdl = p_char->handle;
                wiced_bt_hrc_cb.connection.control_point_char_val_hdl  = p_char->val_handle;
                HRC_LIB_TRACE("control point hdl:%04x-%04x\n", wiced_bt_hrc_cb.connection.control_point_char_hdl, wiced_bt_hrc_cb.connection.control_point_char_val_hdl);
            }
        }
    }
    else if ((p_data->discovery_type == GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS) &&
             (p_data->discovery_data.char_descr_info.type.len == 2) &&
             (p_data->discovery_data.char_descr_info.type.uu.uuid16 == UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION))
    {
        wiced_bt_hrc_cb.connection.heart_rate_measure_cccd_hdl = p_data->discovery_data.char_descr_info.handle;
        HRC_LIB_TRACE("heart measure cccd hdl:%04x\n", wiced_bt_hrc_cb.connection.heart_rate_measure_cccd_hdl);
    }
}

/*
 * While application performs GATT discovery it shall pass discovery complete callbacks
 * for the HRC service to the HRC Library. This function initiates the next discovery
 * request or write request to configure the HRC service on the iOS device.
 */
void wiced_bt_hrc_discovery_complete(wiced_bt_gatt_discovery_complete_t *p_data)
{
    uint16_t end_handle;
    wiced_bt_hrc_event_data_t event_data;

    HRC_LIB_TRACE("[%s] state:%d\n", __FUNCTION__, wiced_bt_hrc_cb.connection.state);

    if (p_data->disc_type == GATT_DISCOVER_CHARACTERISTICS)
    {
        // done with mandatory Heart Rate Service characteristics, start reading descriptor handles
        // make sure that all characteristics are present
        if ((wiced_bt_hrc_cb.connection.heart_rate_measure_char_hdl == 0) ||
            (wiced_bt_hrc_cb.connection.heart_rate_measure_char_val_hdl == 0))
        {
            // something is very wrong
            HRC_LIB_TRACE("[%s] failed\n", __FUNCTION__);
            wiced_bt_hrc_connection_reset();
            event_data.discovery.conn_id = p_data->conn_id;
            event_data.discovery.status = WICED_BT_GATT_NOT_FOUND;
            wiced_bt_hrc_cb.p_callback(WICED_BT_HRC_EVENT_DISCOVERY, &event_data);
            return;
        }

        // search for descriptor from the characteristic characteristic until the end of the
        // service or until the start of the next characteristic.
        //Sensor location is optional characteristic and control point is conditional.characteristic
        end_handle = wiced_bt_hrc_cb.connection.heart_rate_service_end_handle;

        if (wiced_bt_hrc_cb.connection.sensor_location_char_hdl && wiced_bt_hrc_cb.connection.control_point_char_hdl)
        {
            if (wiced_bt_hrc_cb.connection.control_point_char_hdl > wiced_bt_hrc_cb.connection.heart_rate_measure_char_hdl)
                end_handle = wiced_bt_hrc_cb.connection.control_point_char_hdl - 1;
            if ((wiced_bt_hrc_cb.connection.sensor_location_char_hdl > wiced_bt_hrc_cb.connection.heart_rate_measure_char_hdl) && (wiced_bt_hrc_cb.connection.sensor_location_char_hdl < end_handle))
                end_handle = wiced_bt_hrc_cb.connection.sensor_location_char_hdl - 1;
        }
        else if (wiced_bt_hrc_cb.connection.sensor_location_char_hdl || wiced_bt_hrc_cb.connection.control_point_char_hdl)
        {
            end_handle = (wiced_bt_hrc_cb.connection.sensor_location_char_hdl !=0) ? wiced_bt_hrc_cb.connection.sensor_location_char_hdl : wiced_bt_hrc_cb.connection.control_point_char_hdl;
            if (end_handle > wiced_bt_hrc_cb.connection.heart_rate_measure_char_hdl)
                end_handle = end_handle-1;
        }

        //There is only one descriptor in Heart Rate Service
        wiced_bt_util_send_gatt_discover(p_data->conn_id, GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                         wiced_bt_hrc_cb.connection.heart_rate_measure_char_hdl + 1, end_handle);
    }
    else if (p_data->disc_type == GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS)
    {
        if (wiced_bt_hrc_cb.connection.heart_rate_measure_cccd_hdl == 0)
        {
            HRC_LIB_TRACE("[%s] fail. heart rate cccd not found \n", __FUNCTION__);
            wiced_bt_hrc_connection_reset();
            event_data.discovery.status = WICED_BT_GATT_NOT_FOUND;
            wiced_bt_hrc_cb.p_callback(WICED_BT_HRC_EVENT_DISCOVERY, &event_data);
        }
        else
        {
            event_data.discovery.status = WICED_BT_GATT_SUCCESS;
        }

        wiced_bt_hrc_cb.connection.state = HRC_STATE_IDLE;
        event_data.discovery.conn_id = p_data->conn_id;
        wiced_bt_hrc_cb.p_callback(WICED_BT_HRC_EVENT_DISCOVERY, &event_data);
    }
}

/*
 * Process write response from the stack.
 * Application passes it here if handle belongs to our service.
 */
void wiced_bt_hrc_gatt_op_complete(wiced_bt_gatt_operation_complete_t *p_data)
{
    uint8_t     state   = wiced_bt_hrc_cb.connection.state;
    wiced_bt_hrc_cb.connection.state    = HRC_STATE_IDLE;
    wiced_bt_hrc_event_data_t event_data;

    HRC_LIB_TRACE("[%s] state:%02x rc:%d\n", __FUNCTION__, state, p_data->status);

    // if we were writing 1 to notification source, still need to write 1 to data source
    switch (p_data->op)
    {
    case GATTC_OPTYPE_WRITE:
        if ( p_data->response_data.att_value.handle == wiced_bt_hrc_cb.connection.control_point_char_val_hdl )
        {
            event_data.reset_energy_expended.conn_id = p_data->conn_id;
            event_data.reset_energy_expended.status = p_data->status;
            wiced_bt_hrc_cb.p_callback(WICED_BT_HRC_EVENT_RESET_ENERGY_EXPENDED, &event_data);
        }
        else
        {
            if ( state == HRC_STATE_START)
            {
                event_data.start.conn_id = p_data->conn_id;
                event_data.start.status = p_data->status;
                wiced_bt_hrc_cb.p_callback(WICED_BT_HRC_EVENT_START, &event_data);
            }
            else if ( state == HRC_STATE_STOP )
            {
                event_data.stop.conn_id = p_data->conn_id;
                event_data.stop.status = p_data->status;
                wiced_bt_hrc_cb.p_callback(WICED_BT_HRC_EVENT_STOP, &event_data);
            }
        }
        break;

    // if we were writing 1 to data source, done with initialization
    case GATTC_OPTYPE_NOTIFICATION:
        hrc_lib_process_heart_rate_notification(p_data);
        break;
    }
}

/*
 * The function parse the recived heart rate notification and provide heart rate data to application
 */
void hrc_lib_process_heart_rate_notification(wiced_bt_gatt_operation_complete_t *p_data)
{
    uint8_t flags;
    wiced_bt_hrc_event_data_t event_data;

    if ( p_data->response_data.att_value.len && p_data->response_data.att_value.p_data )
    {
        uint8_t *p = p_data->response_data.att_value.p_data;
        uint8_t *p1 = p;
        uint16_t len = p_data->response_data.att_value.len;

        STREAM_TO_UINT8(flags, p);
        len--;

        memset(&event_data, 0, sizeof(event_data));

        event_data.notification_data.conn_id = p_data->conn_id;

        if ((flags & HRC_HEART_RATE_VALUE_UINT16_FORMAT) && (len>=2) )
        {
            STREAM_TO_UINT16(event_data.notification_data.heart_rate, p);
            len -= 2;
        }
        else if (len>=1)
        {
            STREAM_TO_UINT8(event_data.notification_data.heart_rate, p);
            len--;
        }

        if ((flags & HRC_ENERGY_EXPENDED_FIELD_PRESENT) && (len>=2) )
        {
            STREAM_TO_UINT16(event_data.notification_data.energy_expended, p);
            len -=2;
            event_data.notification_data.energy_expended_present = TRUE;
        }

        //TODO: Enable below RR Interval logic once we support at Server side
#if 0
    {
        uint16_t    rr_int[9]; /* with GATT MTU = 23, maximum 9 RR intervals possible */
        event_data.notification_data.num_of_rr_intervals = ((p_data->response_data.len - (p - p1))/sizeof(uint16_t));
        memcpy(rr_int, p, , event_data.notification_data.num_of_rr_intervals*sizeof(uint16_t));
        event_data.notification_data.rr_intervals_data = rr_int;
    }
#endif

        wiced_bt_hrc_cb.p_callback(WICED_BT_HRC_EVENT_NOTIFICATION_DATA, &event_data);
    }
}

/*
 * wiced_bt_hrc_connection_reset
 * This function reset the parameters of a connection
 */
static void wiced_bt_hrc_connection_reset(void)
{
    memset(&wiced_bt_hrc_cb.connection, 0, sizeof(wiced_bt_hrc_cb.connection));
}


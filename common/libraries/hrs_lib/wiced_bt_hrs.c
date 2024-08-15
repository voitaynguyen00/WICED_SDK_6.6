
#include "wiced_bt_hrp.h"
#include "wiced_bt_hrs.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_types.h"
#include "wiced_bt_trace.h"
#include "string.h"

/*
 * Definitions
 */
#define HRS_STATE_DISCONNECTED           0
#define HRS_STATE_CONNECTED              1

#define HRS_HEART_RATE_VALUE_UINT8_FORMAT       0
#define HRS_HEART_RATE_VALUE_UINT16_FORMAT      (0x1 << 0)
#define HRS_SENSOR_CONTACT_DETECTED             (0x1 << 1)
#define HRS_SENSOR_CONTACT_FEATURE_SUPPORTED    (0x1 << 2)
#define HRS_ENERGY_EXPENDED_FIELD_PRESENT       (0x1 << 3)
#define HRS_ENERGY_RR_INTERVAL_FIELD_PRESENT    (0x1 << 4)

#ifdef WICED_BT_TRACE_ENABLE
#define     HRS_LIB_TRACE                          WICED_BT_TRACE
#else
#define     HRS_LIB_TRACE(...)
#endif

/*
 * Structures
 */
typedef struct
{
    uint16_t                    conn_id;               /* connection identifier */
    uint16_t                    heart_rate_measurement_cccd; /* Heart Rate Notification client cfg desc */
    uint8_t                     sensor_location;
} wiced_bt_hrs_connection_t;

typedef struct
{
    wiced_bt_hrs_connection_t   connection;
    wiced_bt_hrs_event_cback_t  *p_app_event_cback;
    wiced_bt_hrs_handles_t      gatt_handles;           /* Heart Rate Service GATT Handles */
} hrs_lib_cb_t;

/*
 * Gobal Variables
 */
static hrs_lib_cb_t hrs_lib_cb;

/*
 * Local Function
 */
static void wiced_bt_hrs_reset_connection(void);

/*
 * wiced_bt_hrs_init
 */
wiced_result_t wiced_bt_hrs_init(wiced_bt_hrs_event_cback_t *p_app_event_cb, wiced_bt_hrs_handles_t *p_gatt_handles)
{
    if ((p_app_event_cb == NULL)             ||
        (p_gatt_handles == NULL)             ||
        (p_gatt_handles->configuration == 0) ||
        (p_gatt_handles->control == 0)       ||
        (p_gatt_handles->location == 0)      ||
        (p_gatt_handles->value == 0))
    {
        WICED_BT_TRACE("Wrong GATT Handle\n");
        return WICED_BT_BADARG;
    }

    memset(&hrs_lib_cb, 0, sizeof(hrs_lib_cb));

    hrs_lib_cb.p_app_event_cback = p_app_event_cb;

    /* Save the HRS GATT Handles */
    memcpy(&hrs_lib_cb.gatt_handles, p_gatt_handles, sizeof(hrs_lib_cb.gatt_handles));

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_hrs_connection_up
 */
void wiced_bt_hrs_connection_up(uint16_t conn_id)
{
    hrs_lib_cb.connection.conn_id          = conn_id;
    hrs_lib_cb.connection.sensor_location  = WICED_BT_HEART_RATE_SENSOR_LOCATION_CHEST;
}

void wiced_bt_hrs_connection_down(uint16_t conn_id)
{
    wiced_bt_hrs_reset_connection();
}

wiced_bt_gatt_status_t wiced_bt_hrs_process_client_read_req(uint16_t conn_id, wiced_bt_gatt_read_t *p_read)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;

    if (p_read->handle == hrs_lib_cb.gatt_handles.location)
    {
        *(p_read->p_val_len) = 1;
        *p_read->p_val = hrs_lib_cb.connection.sensor_location;
    }
    else if (p_read->handle == hrs_lib_cb.gatt_handles.configuration)
    {
        *(p_read->p_val_len) = 2;
        memcpy(p_read->p_val, &hrs_lib_cb.connection.heart_rate_measurement_cccd, 2);
    }
    else
    {
        status = WICED_BT_GATT_READ_NOT_PERMIT;
    }

    return status;
}

wiced_bt_gatt_status_t wiced_bt_hrs_process_client_write_req(uint16_t conn_id, wiced_bt_gatt_write_t *p_write)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INVALID_ATTR_LEN;
    wiced_bt_hrs_event_data_t event_data;

    if (p_write)
    {
        if (p_write->handle == hrs_lib_cb.gatt_handles.configuration)
        {
            if (p_write->val_len == 2 && p_write->p_val)
            {
                hrs_lib_cb.connection.heart_rate_measurement_cccd = p_write->p_val[0] | ( p_write->p_val[1] << 8 );
                if (hrs_lib_cb.connection.heart_rate_measurement_cccd & GATT_CLIENT_CONFIG_NOTIFICATION)
                {
                    event_data.notification_enabled.conn_id = conn_id;
                    hrs_lib_cb.p_app_event_cback(WICED_BT_HRS_EVENT_HEART_RATE_NOTIFICATIONS_ENABLED, &event_data);
                }
                else
                {
                    event_data.notification_disabled.conn_id = conn_id;
                    hrs_lib_cb.p_app_event_cback(WICED_BT_HRS_EVENT_HEART_RATE_NOTIFICATIONS_DISABLED, &event_data);
                }
                status = WICED_BT_GATT_SUCCESS;
            }
        }
        else if (p_write->handle == hrs_lib_cb.gatt_handles.control)
        {
            if (p_write->val_len == 1 && p_write->p_val)
            {
                status = WICED_BT_GATT_SUCCESS;
                if ( *p_write->p_val == 1 )
                {
                    event_data.reset_energy_expended.conn_id = conn_id;
                    hrs_lib_cb.p_app_event_cback(WICED_BT_HRS_RESET_ENERGY_EXPENDED_VALUE, &event_data);
                }
                else
                    status = WICED_BT_HRP_CONTROL_POINT_WRITE_UNSUPPORTED_VALUE;
            }
        }
        else
        {
            status = WICED_BT_GATT_WRITE_NOT_PERMIT;
        }
    }

    return status;
}

wiced_bt_gatt_status_t wiced_bt_hrs_send_heart_rate(uint16_t conn_id, wiced_bt_hrs_notification_data_t *heart_rate_data)
{
    wiced_bt_gatt_status_t status;
    uint8_t     heart_rate_notification[20];// 3 bytes for GATT notification header
    uint8_t     *p_n = heart_rate_notification;
    uint8_t     *p_flag = p_n;

    if (!(hrs_lib_cb.connection.heart_rate_measurement_cccd & GATT_CLIENT_CONFIG_NOTIFICATION))
    {
        return WICED_BT_GATT_REQ_NOT_SUPPORTED;
    }

    memset(heart_rate_notification, 0, sizeof(heart_rate_notification));

    *p_n++ = HRS_SENSOR_CONTACT_DETECTED | HRS_SENSOR_CONTACT_FEATURE_SUPPORTED ;

    if (heart_rate_data->heart_rate <= 255)
    {
        *p_flag = *p_flag | HRS_HEART_RATE_VALUE_UINT8_FORMAT;
        *p_n++ = heart_rate_data->heart_rate;
    }
    else
    {
        *p_flag = *p_flag | HRS_HEART_RATE_VALUE_UINT16_FORMAT;
        *p_n++ =  heart_rate_data->heart_rate & 0xff;
        *p_n++ =  (heart_rate_data->heart_rate >> 8) & 0xff;
    }

    if (heart_rate_data->energy_expended_present)
    {
        *p_flag = *p_flag | HRS_ENERGY_EXPENDED_FIELD_PRESENT;
        *p_n++ = heart_rate_data->energy_expended & 0xff;
        *p_n++ = (heart_rate_data->energy_expended >> 8) & 0xff;
    }

    status = wiced_bt_gatt_send_notification( conn_id, hrs_lib_cb.gatt_handles.value,
            (uint16_t)(p_n - heart_rate_notification), heart_rate_notification );
    if (status)
    {
        HRS_LIB_TRACE("Heart rate notification failure %d, \n", status);
    }

    return status;
}

void wiced_bt_hrs_set_previous_connection_client_notification_configuration(uint16_t conn_id, wiced_bool_t notifications_enabled)
{
    if (notifications_enabled)
    {
        hrs_lib_cb.connection.heart_rate_measurement_cccd = 1;
    }
}

/*
 * wiced_bt_hrs_reset_connection
 */
static void wiced_bt_hrs_reset_connection(void)
{
    memset(&hrs_lib_cb.connection, 0, sizeof(hrs_lib_cb.connection));
}

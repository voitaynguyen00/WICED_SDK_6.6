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

#include "ble_event_map.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_gatt.h"

typedef struct event_map {
    uint32_t val;
    char *shortName;
    char *descr;
} event_map_t;

event_map_t BTM_events[] = {
    {BTM_ENABLED_EVT,"BTM_ENABLED_EVT","< Bluetooth controller and host stack enabled. Event data: wiced_bt_dev_enabled_t "}  ,

    {BTM_DISABLED_EVT,"BTM_DISABLED_EVT","< Bluetooth controller and host stack disabled. Event data: NULL "},
    {BTM_POWER_MANAGEMENT_STATUS_EVT,"BTM_POWER_MANAGEMENT_STATUS_EVT","< Power management status change. Event data: wiced_bt_power_mgmt_notification_t"},
    {BTM_PIN_REQUEST_EVT, " BTM_PIN_REQUEST_EVT","< PIN request (used only with legacy devices). Event data: #wiced_bt_dev_name_and_class_t"},
    {BTM_USER_CONFIRMATION_REQUEST_EVT,"BTM_USER_CONFIRMATION_REQUEST_EVT","< received USER_CONFIRMATION_REQUEST event (respond using #wiced_bt_dev_confirm_req_reply). Event data: #wiced_bt_dev_user_cfm_req_t"},
    {BTM_PASSKEY_NOTIFICATION_EVT,"BTM_PASSKEY_NOTIFICATION_EVT","< received USER_PASSKEY_NOTIFY event. Event data: #wiced_bt_dev_user_key_notif_t"},

    {BTM_PASSKEY_REQUEST_EVT,"BTM_PASSKEY_REQUEST_EVT","< received USER_PASSKEY_REQUEST event (respond using #wiced_bt_dev_pass_key_req_reply). Event data: #wiced_bt_dev_user_key_req_t"},
    {BTM_KEYPRESS_NOTIFICATION_EVT,"BTM_KEYPRESS_NOTIFICATION_EVT","< received KEYPRESS_NOTIFY event. Event data: #wiced_bt_dev_user_keypress_t "},
    {BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT,"BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT","< Requesting IO capabilities for BR/EDR pairing. Event data: #wiced_bt_dev_bredr_io_caps_req_t"},
    {BTM_PAIRING_IO_CAPABILITIES_BR_EDR_RESPONSE_EVT,"BTM_PAIRING_IO_CAPABILITIES_BR_EDR_RESPONSE_EVT","< Received IO capabilities response for BR/EDR pairing. Event data: #wiced_bt_dev_bredr_io_caps_rsp_t"},
    {BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT,"BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT","< Requesting IO capabilities for BLE pairing. Event data: #wiced_bt_dev_ble_io_caps_req_t" },

    {BTM_PAIRING_COMPLETE_EVT, "BTM_PAIRING_COMPLETE_EVT","< received SIMPLE_PAIRING_COMPLETE event. Event data: #wiced_bt_dev_pairing_cplt_t "},

    {BTM_ENCRYPTION_STATUS_EVT, "BTM_ENCRYPTION_STATUS_EVT","< Encryption status change. Event data: #wiced_bt_dev_encryption_status_t "},
    {BTM_SECURITY_REQUEST_EVT, "BTM_SECURITY_REQUEST_EVT","< Security request (respond using #wiced_bt_ble_security_grant). Event data: #wiced_bt_dev_security_request_t "},
    {BTM_SECURITY_FAILED_EVT, "BTM_SECURITY_FAILED_EVT","< Security procedure/authentication failed. Event data: #wiced_bt_dev_security_failed_t "},

    {BTM_SECURITY_ABORTED_EVT, "BTM_SECURITY_ABORTED_EVT", "< Security procedure aborted locally, or unexpected link drop. Event data: #wiced_bt_dev_name_and_class_t "},

    {BTM_READ_LOCAL_OOB_DATA_COMPLETE_EVT, "BTM_READ_LOCAL_OOB_DATA_COMPLETE_EVT","< Result of reading local OOB data (wiced_bt_dev_read_local_oob_data). Event data: #wiced_bt_dev_local_oob_t "},

    {BTM_REMOTE_OOB_DATA_REQUEST_EVT, "BTM_READ_LOCAL_OOB_DATA_COMPLETE_EVT","< OOB data from remote device (respond using #wiced_bt_dev_remote_oob_data_reply). Event data: #wiced_bt_dev_remote_oob_t "},

    {BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT,"BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT","< Updated remote device link keys (store device_link_keys to  NV memory). Event data: #wiced_bt_device_link_keys_t "},
    {BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT,"BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT","< Request for stored remote device link keys (restore device_link_keys from NV memory). If successful, return WICED_BT_SUCCESS. Event data: #wiced_bt_device_link_keys_t "},

    {BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT,"BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT","< Update local identity key (stored local_identity_keys NV memory). Event data: #wiced_bt_local_identity_keys_t "},
    {BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT,"BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT","< Request local identity key (get local_identity_keys from NV memory). If successful, return WICED_BT_SUCCESS. Event data: #wiced_bt_local_identity_keys_t "},
    {BTM_BLE_SCAN_STATE_CHANGED_EVT,"BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT","< BLE scan state change. Event data: #wiced_bt_ble_scan_type_t "},
    {BTM_BLE_ADVERT_STATE_CHANGED_EVT, "BTM_BLE_ADVERT_STATE_CHANGED_EVT","< BLE advertisement state change. Event data: #wiced_bt_ble_advert_mode_t "},

    {BTM_SMP_REMOTE_OOB_DATA_REQUEST_EVT,"BTM_SMP_REMOTE_OOB_DATA_REQUEST_EVT","< SMP remote oob data request. Reply using wiced_bt_smp_oob_data_reply. Event data: wiced_bt_smp_remote_oob_req_t  "},
    {BTM_SMP_SC_REMOTE_OOB_DATA_REQUEST_EVT, "BTM_SMP_SC_REMOTE_OOB_DATA_REQUEST_EVT","< LE secure connection remote oob data request. Reply using wiced_bt_smp_sc_oob_reply. Event data: #wiced_bt_smp_sc_remote_oob_req_t "},
    {BTM_SMP_SC_LOCAL_OOB_DATA_NOTIFICATION_EVT,"BTM_SMP_SC_LOCAL_OOB_DATA_NOTIFICATION_EVT","< LE secure connection local OOB data (wiced_bt_smp_create_local_sc_oob_data). Event data: #wiced_bt_smp_sc_local_oob_t"},
    {BTM_SCO_CONNECTED_EVT,"BTM_SCO_CONNECTED_EVT","< SCO connected event. Event data: #wiced_bt_sco_connected_t "},
    {BTM_SCO_DISCONNECTED_EVT, "BTM_SCO_DISCONNECTED_EVT","< SCO disconnected event. Event data: #wiced_bt_sco_disconnected_t "},
    {BTM_SCO_CONNECTION_REQUEST_EVT,"BTM_SCO_CONNECTION_REQUEST_EVT","< SCO connection request event. Event data: #wiced_bt_sco_connection_request_t"},
    {BTM_SCO_CONNECTION_CHANGE_EVT, "BTM_SCO_CONNECTION_CHANGE_EVT","< SCO connection change event. Event data: #wiced_bt_sco_connection_change_t "},
    {BTM_LPM_STATE_LOW_POWER,   "BTM_LPM_STATE_LOW_POWER","< BT device wake has been deasserted "}
    };



char *getEventEnum(event_map_t events[], uint32_t event,int num_events)
{
    for(int i=0;i<num_events;i++)
    {
        if(events[i].val == event)
            return events[i].shortName;
    }
    return "Event Unknown";
}

char*getEventEnumBTM(uint32_t event)
{

    return getEventEnum(BTM_events,event,sizeof(BTM_events)/sizeof(event_map_t));
}



event_map_t GATT_events[] = {
    {GATT_CONNECTION_STATUS_EVT,"GATT_CONNECTION_STATUS_EVT","< GATT connection status change. Event data: #wiced_bt_gatt_connection_status_t"},
    {GATT_OPERATION_CPLT_EVT,"GATT_OPERATION_CPLT_EVT","< GATT operation complete. Event data: #wiced_bt_gatt_event_data_t"},
    {GATT_DISCOVERY_RESULT_EVT,"GATT_DISCOVERY_RESULT_EVT","< GATT attribute discovery result. Event data: #wiced_bt_gatt_discovery_result_t"},
    {GATT_DISCOVERY_CPLT_EVT,"GATT_DISCOVERY_CPLT_EVT","< GATT attribute discovery complete. Event data: #wiced_bt_gatt_event_data_t"},
    {GATT_ATTRIBUTE_REQUEST_EVT, "GATT_ATTRIBUTE_REQUEST_EVT" , "< GATT attribute request (from remote client). Event data: #wiced_bt_gatt_attribute_request_t"},

};

char*getEventEnumGATT(uint32_t event)
{
    return getEventEnum(GATT_events,event,sizeof(GATT_events)/sizeof(event_map_t));
}


event_map_t GATTS_REQ_events[] = {
    {GATTS_REQ_TYPE_READ , "GATTS_REQ_TYPE_READ","< Attribute read notification (attribute value internally read from GATT database) "},
    {GATTS_REQ_TYPE_WRITE,"GATTS_REQ_TYPE_WRITE","< Attribute write notification (attribute value internally written to GATT database) "},
    {GATTS_REQ_TYPE_PREP_WRITE,"GATTS_REQ_TYPE_PREP_WRITE","< Attribute Prepare Write Notification (Suspending write request before triggering actual execute write ) "},
    {GATTS_REQ_TYPE_WRITE_EXEC,"GATTS_REQ_TYPE_WRITE_EXEC","< Execute write request "},
    {GATTS_REQ_TYPE_MTU,"GATTS_REQ_TYPE_MTU","< MTU exchange information "},
    {GATTS_REQ_TYPE_CONF,"GATTS_REQ_TYPE_CONF","Value confirmation"},
};


char*getEventEnumGATTS_REQ(uint32_t event)
{
    return getEventEnum(GATTS_REQ_events,event,sizeof(GATTS_REQ_events)/sizeof(event_map_t));
}

event_map_t BTM_BLE_ADVERT_events[] = {

    {BTM_BLE_ADVERT_OFF,"BTM_BLE_ADVERT_OFF","< Stop advertising"} ,
    {BTM_BLE_ADVERT_DIRECTED_HIGH , "BTM_BLE_ADVERT_DIRECTED_HIGH" , "Directed avertisement (high duty cycle)" } ,
    {BTM_BLE_ADVERT_DIRECTED_LOW,"BTM_BLE_ADVERT_DIRECTED_LOW","< Directed avertisement (low duty cycle)" },
    {BTM_BLE_ADVERT_UNDIRECTED_HIGH,"BTM_BLE_ADVERT_UNDIRECTED_HIGH","< Undirected avertisement (high duty cycle)" },
    {BTM_BLE_ADVERT_UNDIRECTED_LOW,"BTM_BLE_ADVERT_UNDIRECTED_LOW","< Undirected avertisement (low duty cycle)" },
    {BTM_BLE_ADVERT_NONCONN_HIGH,"BTM_BLE_ADVERT_NONCONN_HIGH","< Non-connectable avertisement (high duty cycle)" },
    {BTM_BLE_ADVERT_NONCONN_LOW,"BTM_BLE_ADVERT_NONCONN_LOW","< Non-connectable avertisement (low duty cycle)" },
    {BTM_BLE_ADVERT_DISCOVERABLE_HIGH,"BTM_BLE_ADVERT_DISCOVERABLE_HIGH","< discoverable avertisement (high duty cycle)"},
    {BTM_BLE_ADVERT_DISCOVERABLE_LOW,"BTM_BLE_ADVERT_DISCOVERABLE_LOW","< discoverable avertisement (low duty cycle)"},
};

char*getEventEnumBTM_BLE_ADVERT(uint32_t event)
{
    return getEventEnum(BTM_BLE_ADVERT_events,event,sizeof(BTM_BLE_ADVERT_events)/sizeof(event_map_t));
}

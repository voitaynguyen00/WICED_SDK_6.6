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
 * This file shows how to create a device supporting a vendor specific model.
 */
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_mesh_models.h"
#include "wiced_bt_trace.h"
#include "wiced_timer.h"
#include "wiced_bt_mesh_app.h"

#ifdef HCI_CONTROL
#include "wiced_transport.h"
#include "hci_control_api.h"
#endif
#include "mesh_vendor_server.h"

#include "wiced_bt_cfg.h"
extern wiced_bt_cfg_settings_t wiced_bt_cfg_settings;

/******************************************************
 *          Constants
 ******************************************************/
#define MESH_PID                0x3025
#define MESH_VID                0x0002
#define MESH_FWID               0x3025000101010001
#define MESH_CACHE_REPLAY_SIZE  0x0008

/******************************************************
 *          Structures
 ******************************************************/

/******************************************************
 *          Function Prototypes
 ******************************************************/
static void mesh_app_init(wiced_bool_t is_provisioned);
static uint32_t mesh_app_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length);
static void mesh_vendor_server_send_status(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);
static void mesh_vendor_server_process_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);
static void mesh_vendor_server_process_data(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

#ifdef HCI_CONTROL
static void mesh_vendor_hci_event_send_data(wiced_bt_mesh_hci_event_t *p_hci_event, uint8_t *p_data, uint16_t data_len);
#endif

/******************************************************
 *          Variables Definitions
 ******************************************************/
uint8_t mesh_mfr_name[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MANUFACTURER_NAME] = { 'C', 'y', 'p', 'r', 'e', 's', 's', 0 };
uint8_t mesh_model_num[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MODEL_NUMBER]     = { '1', '2', '3', '4', 0, 0, 0, 0 };
uint8_t mesh_system_id[8]                                                  = { 0xbb, 0xb8, 0xa1, 0x80, 0x5f, 0x9f, 0x91, 0x71 };

wiced_bool_t mesh_vendor_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);
uint16_t     mesh_vendor_server_scene_store_handler(uint8_t element_idx, uint8_t *p_buffer, uint16_t buffer_len);
uint16_t     mesh_vendor_server_scene_recall_handler(uint8_t element_idx, uint8_t *p_buffer, uint16_t buffer_len, uint32_t transition_time, uint32_t delay);

wiced_bt_mesh_core_config_model_t   mesh_element1_models[] =
{
    WICED_BT_MESH_DEVICE,
    { MESH_VENDOR_COMPANY_ID, MESH_VENDOR_MODEL_ID, mesh_vendor_server_message_handler, mesh_vendor_server_scene_store_handler, mesh_vendor_server_scene_recall_handler },
};
#define MESH_APP_NUM_MODELS  (sizeof(mesh_element1_models) / sizeof(wiced_bt_mesh_core_config_model_t))

#define MESH_VENDOR_SERVER_ELEMENT_INDEX   0

wiced_bt_mesh_core_config_element_t mesh_elements[] =
{
    {
        .location = MESH_ELEM_LOC_MAIN,                                 // location description as defined in the GATT Bluetooth Namespace Descriptors section of the Bluetooth SIG Assigned Numbers
        .default_transition_time = MESH_DEFAULT_TRANSITION_TIME_IN_MS,  // Default transition time for models of the element in milliseconds
        .onpowerup_state = WICED_BT_MESH_ON_POWER_UP_STATE_RESTORE,     // Default element behavior on power up
        .default_level = 0,                                             // Default value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .range_min = 1,                                                 // Minimum value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .range_max = 0xffff,                                            // Maximum value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .move_rollover = 0,                                             // If true when level gets to range_max during move operation, it switches to min, otherwise move stops.
        .properties_num = 0,                                            // Number of properties in the array models
        .properties = NULL,                                             // Array of properties in the element.
        .sensors_num = 0,                                               // Number of sensors in the sensor array
        .sensors = NULL,                                                // Array of sensors of that element
        .models_num = MESH_APP_NUM_MODELS,                              // Number of models in the array models
        .models = mesh_element1_models,                                 // Array of models located in that element. Model data is defined by structure wiced_bt_mesh_core_config_model_t
    },
};

wiced_bt_mesh_core_config_t  mesh_config =
{
    .company_id         = MESH_COMPANY_ID_CYPRESS,                  // Company identifier assigned by the Bluetooth SIG
    .product_id         = MESH_PID,                                 // Vendor-assigned product identifier
    .vendor_id          = MESH_VID,                                 // Vendor-assigned product version identifier
    .firmware_id        = MESH_FWID,                                // Vendor-assigned firmware version identifier
    .replay_cache_size  = MESH_CACHE_REPLAY_SIZE,                   // Number of replay protection entries, i.e. maximum number of mesh devices that can send application messages to this device.
#if LOW_POWER_NODE
    .features           = WICED_BT_MESH_CORE_FEATURE_BIT_LOW_POWER, // A bit field indicating the device features. In Low Power mode no Relay, no Proxy and no Friend
    .friend_cfg         =                                           // Empty Configuration of the Friend Feature
    {
        .receive_window = 0,                                        // Receive Window value in milliseconds supported by the Friend node.
        .cache_buf_len  = 0,                                        // Length of the buffer for the cache
        .max_lpn_num    = 0                                         // Max number of Low Power Nodes with established friendship. Must be > 0 if Friend feature is supported. 
    },
    .low_power          =                                           // Configuration of the Low Power Feature
    {
        .rssi_factor           = 2,                                 // contribution of the RSSI measured by the Friend node used in Friend Offer Delay calculations.
        .receive_window_factor = 2,                                 // contribution of the supported Receive Window used in Friend Offer Delay calculations.
        .min_cache_size_log    = 3,                                 // minimum number of messages that the Friend node can store in its Friend Cache.
        .receive_delay         = 100,                               // Receive delay in 1 ms units to be requested by the Low Power node.
        .poll_timeout          = 36000                              // Poll timeout in 100ms units to be requested by the Low Power node.
    },
#else
    .features           = WICED_BT_MESH_CORE_FEATURE_BIT_FRIEND | WICED_BT_MESH_CORE_FEATURE_BIT_RELAY | WICED_BT_MESH_CORE_FEATURE_BIT_GATT_PROXY_SERVER,   // In Friend mode support friend, relay
    .friend_cfg         =                                           // Configuration of the Friend Feature(Receive Window in Ms, messages cache)
    {
        .receive_window        = 200,
        .cache_buf_len         = 300,                               // Length of the buffer for the cache
        .max_lpn_num           = 4                                  // Max number of Low Power Nodes with established friendship. Must be > 0 if Friend feature is supported. 
    },
    .low_power          =                                           // Configuration of the Low Power Feature
    {
        .rssi_factor           = 0,                                 // contribution of the RSSI measured by the Friend node used in Friend Offer Delay calculations.
        .receive_window_factor = 0,                                 // contribution of the supported Receive Window used in Friend Offer Delay calculations.
        .min_cache_size_log    = 0,                                 // minimum number of messages that the Friend node can store in its Friend Cache.
        .receive_delay         = 0,                                 // Receive delay in 1 ms units to be requested by the Low Power node.
        .poll_timeout          = 0                                  // Poll timeout in 100ms units to be requested by the Low Power node.
    },
#endif
    .gatt_client_only          = WICED_FALSE,                       // Can connect to mesh over GATT or ADV
    .elements_num  = (uint8_t)(sizeof(mesh_elements) / sizeof(mesh_elements[0])),   // number of elements on this device
    .elements      = mesh_elements                                  // Array of elements for this device
};

/*
 * Mesh application library will call into application functions if provided by the application.
 */
wiced_bt_mesh_app_func_table_t wiced_bt_mesh_app_func_table =
{
    mesh_app_init,          // application initialization
    NULL,                   // Default SDK platform button processing
    NULL,                   // GATT connection status
    NULL,                   // attention processing
    NULL,                   // notify period set
    mesh_app_proc_rx_cmd,   // WICED HCI command
    NULL,                   // LPN sleep
    NULL                    // factory reset
};

/******************************************************
 *               Function Definitions
 ******************************************************/
void mesh_app_init(wiced_bool_t is_provisioned)
{
    WICED_BT_TRACE("app_init provisioned:%d\n", is_provisioned);

    wiced_bt_cfg_settings.device_name = (uint8_t *)"Vendor Server";
    wiced_bt_cfg_settings.gatt_cfg.appearance = APPEARANCE_GENERIC_TAG;
    // Adv Data is fixed. Spec allows to put URI, Name, Appearance and Tx Power in the Scan Response Data.
    if (!is_provisioned)
    {
        wiced_bt_ble_advert_elem_t  adv_elem[3];
        uint8_t                     buf[2];
        uint8_t                     num_elem = 0;
        adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
        adv_elem[num_elem].len = (uint16_t)strlen((const char*)wiced_bt_cfg_settings.device_name);
        adv_elem[num_elem].p_data = wiced_bt_cfg_settings.device_name;
        num_elem++;

        adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_APPEARANCE;
        adv_elem[num_elem].len = 2;
        buf[0] = (uint8_t)wiced_bt_cfg_settings.gatt_cfg.appearance;
        buf[1] = (uint8_t)(wiced_bt_cfg_settings.gatt_cfg.appearance >> 8);
        adv_elem[num_elem].p_data = buf;
        num_elem++;

        wiced_bt_mesh_set_raw_scan_response_data(num_elem, adv_elem);
    }
}

/*
 * This function is called when core receives a valid message for the define Vendor
 * Model (MESH_VENDOR_COMPANY_ID/MESH_VENDOR_MODEL_ID) combination.  The function shall return TRUE if it
 * was able to process the message, and FALSE if the message is unknown.  In the latter case the core
 * will call other registered models.
 */
wiced_bool_t mesh_vendor_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len)
{
    WICED_BT_TRACE("mesh_vendor_server_message_handler: opcode:%x model_id:%x\n", p_event->opcode, p_event->model_id);

    // 0xffff model_id means request to check if that opcode belongs to that model
    if (p_event->model_id == 0xffff)
    {
        switch (p_event->opcode)
        {
        case MESH_VENDOR_OPCODE_GET:
        case MESH_VENDOR_OPCODE_SET:
        case MESH_VENDOR_OPCODE_STATUS:
            break;
        default:
            return WICED_FALSE;
        }
        return WICED_TRUE;
    }

    switch(p_event->opcode)
    {
    // this is a special case.  The provisioner configured this device to periodically publish
    // information.  Process is the same way as the GET command
    case WICED_BT_MESH_OPCODE_UNKNOWN:
    case MESH_VENDOR_OPCODE_GET:
        mesh_vendor_server_process_get(p_event, p_data, data_len);
        break;

    case MESH_VENDOR_OPCODE_SET:
        p_event->reply = WICED_FALSE;
        mesh_vendor_server_process_data(p_event, p_data, data_len);
        break;

    case MESH_VENDOR_OPCODE_SET_UNACKED:
        p_event->reply = WICED_FALSE;
        mesh_vendor_server_process_data(p_event, p_data, data_len);
        break;

    case MESH_VENDOR_OPCODE_STATUS:
        p_event->reply = WICED_FALSE;
        mesh_vendor_server_process_data(p_event, p_data, data_len);
        break;

    default:
        wiced_bt_mesh_release_event(p_event);
        return WICED_FALSE;
    }
    return WICED_TRUE;
}

/*
 * Scene Store Handler.  If the model need to be a part of a scene, store the data in the provided buffer.
 */
uint16_t mesh_vendor_server_scene_store_handler(uint8_t element_idx, uint8_t *p_buffer, uint16_t buffer_len)
{
    // return number of the bytes stored in the buffer.
    return 0;
}

/*
 * Scene Store Handler.  If the model need to be a part of a scene, restore the data from the provided buffer.
 */
uint16_t mesh_vendor_server_scene_recall_handler(uint8_t element_idx, uint8_t *p_buffer, uint16_t buffer_len, uint32_t transition_time, uint32_t delay)
{
    // return number of the bytes read from the buffer.
    return 0;
}

void mesh_vendor_server_process_data(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len)
{
#if defined HCI_CONTROL
    wiced_bt_mesh_hci_event_t *p_hci_event;
#endif
    WICED_BT_TRACE("vs process data from:%04x reply:%04x len:%d\n", p_event->src, p_event->reply, data_len);

    // Because the same app publishes and subscribes the same model, it will receive messages that it
    //sent out.
    if (p_event->src == wiced_bt_mesh_core_get_local_addr())
        return;

#if defined HCI_CONTROL
    if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
        mesh_vendor_hci_event_send_data(p_hci_event, p_data, data_len);
#endif
    if (p_event->reply)
    {
        // return the data that we received in the command.  Real app can send anything it wants.
        mesh_vendor_server_send_status(wiced_bt_mesh_create_reply_event(p_event), p_data, data_len);
    }
    else
    {
        wiced_bt_mesh_release_event(p_event);
    }
}

void mesh_vendor_server_process_get(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len)
{
    mesh_vendor_server_send_status(wiced_bt_mesh_create_reply_event(p_event), (uint8_t *)"Hello", 6);
}

/*
 * In 2 chip solutions MCU can send commands to set vendor specific status.
 */
uint32_t mesh_app_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_event_t *p_event;
    uint16_t dst         = p_data[0] + (p_data[1] << 8);
    uint16_t app_key_idx = p_data[2] + (p_data[3] << 8);
    uint8_t  element_idx = p_data[4];

    WICED_BT_TRACE("[%s] cmd_opcode 0x%02x\n", __FUNCTION__, opcode);

    if (opcode != HCI_CONTROL_MESH_COMMAND_VENDOR_DATA)
        return WICED_FALSE;

    p_event = wiced_bt_mesh_create_event(element_idx, MESH_VENDOR_COMPANY_ID, MESH_VENDOR_MODEL_ID, dst, app_key_idx);
    if (p_event == NULL)
    {
        WICED_BT_TRACE("app_proc_rx_cmd: no mem\n");
        return WICED_TRUE;
    }
    mesh_vendor_server_send_status(p_event, p_data + 6, length - 6);
    return WICED_TRUE;
}

/*
 * Send Vendor Data status message to the Client
 */
void mesh_vendor_server_send_status(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len)
{
    p_event->opcode = MESH_VENDOR_OPCODE_STATUS;
    wiced_bt_mesh_core_send(p_event, p_data, data_len, NULL);
}

#ifdef HCI_CONTROL
/*
 * Send Vendor Data received from the mesh over transport
 */
void mesh_vendor_hci_event_send_data(wiced_bt_mesh_hci_event_t *p_hci_event, uint8_t *p_data, uint16_t data_len)
{
    uint8_t *p = p_hci_event->data;

    ARRAY_TO_STREAM(p, p_data, data_len);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_VENDOR_SET, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

#endif

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
* Mesh Main implementation.
*/

#define _CRT_RAND_S  

#include <windows.h> 
#include <stdio.h> 
#include <stdlib.h>
#include <string.h> 
#include <stdlib.h>
#include "wiced_bt_mesh_provision.h"
#include "wiced_bt_mesh_cfg.h"
#include "wiced_bt_mesh_models.h"
#include "wiced_bt_mesh_core.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"

#define MESH_VENDOR_COMPANY_ID          0x131   // Cypress Company ID
#define MESH_VENDOR_MODEL_ID            1       // ToDo.  This need to be modified

// This sample shows simple use of vendor get/set/status messages.  Vendor model
// can define any opcodes it wants.
#define MESH_VENDOR_OPCODE_GET          1       // Command to Get data
#define MESH_VENDOR_OPCODE_SET          2       // Command to Set data ack is required
#define MESH_VENDOR_OPCODE_SET_UNACKED  3       // Command to Set data no ack is required
#define MESH_VENDOR_OPCODE_STATUS       4       // Response from the server, or unsolicited data change.

static void mesh_app_init(wiced_bool_t is_provisioned);
static void mesh_provision_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);
static void mesh_config_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);
static void mesh_control_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);
static void mesh_sensor_client_callback(uint8_t element_idx, uint16_t addr, uint16_t event, void *p_data);

static void mesh_core_state_changed(wiced_bt_mesh_core_state_type_t type, wiced_bt_mesh_core_state_t *p_state);
extern void mesh_provision_process_event(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);
extern void mesh_sensor_process_event(uint16_t addr, uint16_t event, void *p_data);

const char* szRegKey = "Software\\Cypress\\Mesh\\MeshClient";
static UINT32 read_reg(const char* name, UINT8* value, UINT16 len, DWORD *p_res)
{
    HKEY    hKey;
    DWORD   dwLen = len;
    *p_res = RegCreateKeyExA(HKEY_CURRENT_USER, szRegKey, 0, NULL, 0, KEY_QUERY_VALUE, NULL, &hKey, NULL);
    if (*p_res == ERROR_SUCCESS)
    {
        *p_res = RegQueryValueExA(hKey, name, 0, NULL, value, &dwLen);
        RegCloseKey(hKey);
    }
    if (*p_res != ERROR_SUCCESS)
        dwLen = 0;
    return dwLen;
}

static UINT32 write_reg(const char* name, UINT8* value, UINT16 len, DWORD *p_res)
{
    HKEY    hKey;
    *p_res = RegCreateKeyExA(HKEY_CURRENT_USER, szRegKey, 0, NULL, 0, KEY_SET_VALUE, NULL, &hKey, NULL);
    if (*p_res == ERROR_SUCCESS)
    {
        *p_res = RegSetValueExA(hKey, name, 0, REG_BINARY, value, len);
        RegCloseKey(hKey);
    }
    if (*p_res != ERROR_SUCCESS)
        len = 0;
    return len;
}

/*
* Application provided function to read/write information from/into NVRAM
*/
static uint32_t mesh_nvram_access(wiced_bool_t write, int inx, uint8_t* value, uint16_t len, wiced_result_t *p_result)
{
    *p_result = ERROR_SUCCESS;
    if (write)
        return len;
    else
        return 0;
}

typedef wiced_bool_t(*wiced_model_message_handler_t)(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

static wiced_bt_mesh_core_received_msg_handler_t get_msg_handler_callback(uint16_t company_id, uint16_t opcode, uint16_t *p_model_id, wiced_bool_t *p_dont_save_rpl);
static void                     mesh_start_stop_scan_callback(wiced_bool_t start, wiced_bool_t is_active);
void                            proxy_gatt_send_cb(uint32_t conn_id, uint32_t ref_data, const uint8_t *packet, uint32_t packet_len);
static uint32_t                 mesh_nvram_access(wiced_bool_t write, int inx, uint8_t* node_info, uint16_t len, wiced_result_t *p_result);
static wiced_bool_t             vendor_data_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

#define MESH_PID                0x3006
#define MESH_VID                0x0002
#define MESH_CACHE_REPLAY_SIZE  200

wiced_bt_mesh_core_config_model_t   mesh_element1_models[] =
{
    WICED_BT_MESH_DEVICE,
    WICED_BT_MESH_MODEL_CONFIG_CLIENT,
    WICED_BT_MESH_MODEL_HEALTH_CLIENT,
    WICED_BT_MESH_MODEL_PROPERTY_CLIENT,
    WICED_BT_MESH_MODEL_REMOTE_PROVISION_SERVER,
    WICED_BT_MESH_MODEL_REMOTE_PROVISION_CLIENT,
    WICED_BT_MESH_MODEL_DEFAULT_TRANSITION_TIME_CLIENT,
    WICED_BT_MESH_MODEL_ONOFF_CLIENT,
    WICED_BT_MESH_MODEL_LEVEL_CLIENT,
    WICED_BT_MESH_MODEL_LIGHT_LIGHTNESS_CLIENT,
    WICED_BT_MESH_MODEL_LIGHT_CTL_CLIENT,
    WICED_BT_MESH_MODEL_LIGHT_HSL_CLIENT,
    WICED_BT_MESH_MODEL_SENSOR_CLIENT,
    WICED_BT_MESH_MODEL_FW_DISTRIBUTION_CLIENT,
    WICED_BT_MESH_MODEL_FW_DISTRIBUTION_SERVER, 
#ifdef MESH_VENDOR_MODEL_ID
    { MESH_VENDOR_COMPANY_ID, MESH_VENDOR_MODEL_ID, vendor_data_handler, NULL, NULL },
#endif
};
#define MESH_APP_NUM_MODELS  (sizeof(mesh_element1_models) / sizeof(wiced_bt_mesh_core_config_model_t))

#define MESH_PROVISIONER_CLIENT_ELEMENT_INDEX   0

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
    .replay_cache_size  = MESH_CACHE_REPLAY_SIZE,                   // Number of replay protection entries, i.e. maximum number of mesh devices that can send application messages to this device.
    .features                  = 0,                                 // 
    .friend_cfg         =                                           // Empty Configuration of the Friend Feature
    {
        .receive_window        = 0,                                 // Receive Window value in milliseconds supported by the Friend node.
        .cache_buf_len  = 0,                                        // Length of the buffer for the cache
        .max_lpn_num    = 0                                         // Max number of Low Power Nodes with established friendship. Must be > 0 if Friend feature is supported. 
    },
    .low_power          =                                           // Configuration of the Low Power Feature
    {
        .rssi_factor           = 0,                                 // contribution of the RSSI measured by the Friend node used in Friend Offer Delay calculations.
        .receive_window_factor = 0,                                 // contribution of the supported Receive Window used in Friend Offer Delay calculations.
        .min_cache_size_log    = 0,                                 // minimum number of messages that the Friend node can store in its Friend Cache.
        .receive_delay         = 0,                                 // Receive delay in 1 ms units to be requested by the Low Power node.
        .poll_timeout          = 0                                  // Poll timeout in 100ms units to be requested by the Low Power node.
    },
    .gatt_client_only          = WICED_TRUE,                        // Can connect to mesh over GATT or ADV
    .elements_num  = (uint8_t)(sizeof(mesh_elements) / sizeof(mesh_elements[0])),   // number of elements on this device
    .elements      = mesh_elements                                  // Array of elements for this device
};


void mesh_application_init(void)
{
    static int core_initialized = 0;

    extern uint8_t wiced_bt_mesh_model_trace_enabled;
    wiced_bt_mesh_model_trace_enabled = WICED_TRUE;

    extern void wiced_bt_mesh_core_set_trace_level(uint32_t fids_mask, uint8_t level);
    wiced_bt_mesh_core_set_trace_level(0xffffffff, 4);      //(ALL, TRACE_DEBUG)
    wiced_bt_mesh_core_set_trace_level(0x8, 3);             //(CORE_AES_CCM_C, TRACE_INFO)

    if (!core_initialized)
    {
        // activate IV Update test mode to remove the 96-hour limit
        //wiced_bt_core_iv_update_test_mode = WICED_TRUE;

        core_initialized = 1;
        wiced_bt_mesh_core_init_t   init = { 0 };
#ifdef MESH_SUPPORT_PB_GATT
        mesh_config.features |= WICED_BT_MESH_CORE_FEATURE_BIT_PB_GATT;
#endif
        mesh_config.directed_forward.wanted_rssi = -127;
        wiced_bt_mesh_core_net_key_max_num = 4;
        wiced_bt_mesh_core_app_key_max_num = 8;

        init.p_config_data = &mesh_config;
        init.callback = get_msg_handler_callback;
        init.scan_callback = mesh_start_stop_scan_callback;
        init.proxy_send_callback = proxy_gatt_send_cb;
        init.nvram_access_callback = mesh_nvram_access;
        init.state_changed_cb = mesh_core_state_changed;
        wiced_bt_mesh_core_init(&init);
        wiced_bt_mesh_remote_provisioning_server_init();
        wiced_bt_mesh_core_start();
    }
    mesh_app_init(WICED_TRUE);
}

void mesh_application_deinit(void)
{
    mesh_app_init(WICED_FALSE);
}

/*
* Application implements that function to handle received messages. Call each library that this device needs to support.
*/
wiced_bt_mesh_core_received_msg_handler_t get_msg_handler_callback(uint16_t company_id, uint16_t opcode, uint16_t *p_model_id, wiced_bool_t *p_dont_save_rpl)
{
    wiced_bt_mesh_core_received_msg_handler_t p_message_handler = NULL;
    uint8_t                                   idx_elem, idx_model;
    wiced_bt_mesh_event_t                     temp_event;
    uint16_t                                  model_id;

    WICED_BT_TRACE("company_id:%x opcode:%x\n", company_id, opcode);
    if (company_id == MESH_COMPANY_ID_UNUSED)
    {
        p_message_handler = wiced_bt_mesh_proxy_client_message_handler;
    }
    else
    {
        temp_event.company_id = company_id;
        temp_event.opcode = opcode;
        temp_event.model_id = 0xffff;   //it is sign of special mode for model to just return model_id without message handling
                                        // model changes it to any other value if it wants do disable RPL saving for its messages

        for (idx_elem = 0; idx_elem < mesh_config.elements_num; idx_elem++)
        {
            for (idx_model = 0; idx_model < mesh_config.elements[idx_elem].models_num; idx_model++)
            {
                if (company_id != mesh_config.elements[idx_elem].models[idx_model].company_id)
                    continue;
                p_message_handler = (wiced_bt_mesh_core_received_msg_handler_t)mesh_config.elements[idx_elem].models[idx_model].p_message_handler;
                if (p_message_handler == NULL)
                    continue;
                if (!p_message_handler(&temp_event, NULL, 0))
                    continue;
                model_id = mesh_config.elements[idx_elem].models[idx_model].model_id;
                if (p_model_id)
                    *p_model_id = model_id;
                // Check if model wants to disable RPL saving for its messages
                if (temp_event.model_id != 0xffff && p_dont_save_rpl != NULL)
                    *p_dont_save_rpl = WICED_TRUE;
                break;
            }
            if (idx_model < mesh_config.elements[idx_elem].models_num)
                break;
        }
        if (idx_elem >= mesh_config.elements_num)
            p_message_handler = NULL;
    }
    if (!p_message_handler)
        WICED_BT_TRACE("ignored\n");
    return p_message_handler;
}

/*
 * Application implements that function to start/stop scanning as requested by the core
 */
void mesh_start_stop_scan_callback(wiced_bool_t start, wiced_bool_t is_scan_active)
{
    extern void wiced_bt_ble_set_scan_mode(uint8_t is_active);

    static wiced_bool_t started = WICED_FALSE;
    static wiced_bool_t active = WICED_FALSE;
    uint16_t local_addr = wiced_bt_mesh_core_get_local_addr();
    WICED_BT_TRACE("scan callback: start:%d active:%d\n", start, is_scan_active);

    if ((started == start) && (active == is_scan_active))
        return;

    // check if the request is to stop the scan, or scan type is different
    if ((started && !start) || (start && (active != is_scan_active)))
    {
        WICED_BT_TRACE("scan callback stop active:%d\n", active);
        wiced_bt_ble_observe(0, 0, NULL);

        started = WICED_FALSE;
        if (!start)
            return;
    }

    started = WICED_TRUE;
    active = is_scan_active;
    WICED_BT_TRACE("scan callback start active:%d\n", active);
    wiced_bt_ble_set_scan_mode(is_scan_active);
    wiced_bt_ble_observe(start, 0, NULL);
}

void mesh_app_init(wiced_bool_t is_provisioned)
{
    extern uint8_t wiced_bt_mesh_model_trace_enabled;
    wiced_bt_mesh_model_trace_enabled = WICED_TRUE;

    wiced_bt_mesh_provision_client_init(mesh_provision_message_handler, is_provisioned);
    wiced_bt_mesh_client_init(mesh_provision_message_handler, is_provisioned);

    wiced_bt_mesh_config_client_init(mesh_config_message_handler, is_provisioned);
    wiced_bt_mesh_health_client_init(mesh_config_message_handler, is_provisioned);
    wiced_bt_mesh_proxy_client_init(mesh_config_message_handler, is_provisioned);
    wiced_bt_mesh_model_fw_provider_init();
    wiced_bt_mesh_model_fw_distribution_server_init();

    wiced_bt_mesh_model_property_client_init(0, mesh_control_message_handler, is_provisioned);
    wiced_bt_mesh_model_onoff_client_init(0, mesh_control_message_handler, is_provisioned);
    wiced_bt_mesh_model_power_onoff_client_init(0, mesh_control_message_handler, is_provisioned);
    wiced_bt_mesh_model_level_client_init(0, mesh_control_message_handler, is_provisioned);
    wiced_bt_mesh_model_light_lightness_client_init(0, mesh_control_message_handler, is_provisioned);
    wiced_bt_mesh_model_light_ctl_client_init(0, mesh_control_message_handler, is_provisioned);
    wiced_bt_mesh_model_light_hsl_client_init(0, mesh_control_message_handler, is_provisioned);
    wiced_bt_mesh_model_default_transition_time_client_init(0, mesh_control_message_handler, is_provisioned);
    wiced_bt_mesh_model_sensor_client_init(0, mesh_sensor_client_callback, is_provisioned);
}

void mesh_provision_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data)
{
    WICED_BT_TRACE("provision message:%d\n", event);
    mesh_provision_process_event(event, p_event, p_data);
}

void mesh_config_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data)
{
    WICED_BT_TRACE("config message:%d\n", event);
    mesh_provision_process_event(event, p_event, p_data);
}

void mesh_control_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data)
{
    WICED_BT_TRACE("control message:%d\n", event);
    mesh_provision_process_event(event, p_event, p_data);
}

static void mesh_sensor_client_callback(uint8_t element_idx, uint16_t addr, uint16_t event, void *p_data)
{
    WICED_BT_TRACE("sensor message:%d\n", event);
    mesh_sensor_process_event(addr, event, p_data);
}


void mesh_core_state_changed(wiced_bt_mesh_core_state_type_t type, wiced_bt_mesh_core_state_t *p_state)
{
    if (type == WICED_BT_MESH_CORE_STATE_TYPE_SEQ)
        mesh_provision_process_event(WICED_BT_MESH_SEQ_CHANGED, NULL, &p_state->seq);
    else if(type == WICED_BT_MESH_CORE_STATE_IV)
        mesh_provision_process_event(WICED_BT_MESH_IV_CHANGED, NULL, &p_state->iv);
}


wiced_bool_t vendor_data_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len)
{
    char buf[100] = { 0 };
    
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

    WICED_BT_TRACE("Vendor Data Opcode:%d\n", p_event->opcode);

    for (int i = 0; i < data_len && i < 33; i++)
        sprintf(&buf[strlen(buf)], "%02x", p_data[i]);

    WICED_BT_TRACE("%s\n", buf);
    wiced_bt_mesh_release_event(p_event);

    return WICED_TRUE;
}

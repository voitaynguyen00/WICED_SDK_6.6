/*
* Copyright 2017, Cypress Semiconductor Corporation or a subsidiary of Cypress Semiconductor 
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
 * This file shows how to create a device which implements mesh provisioner client.
 * The main purpose of the app is to process messages coming from the MCU and call Mesh Core
 * Library to perform functionality.
 */


#include "wiced.h"
#include "mesh_main.h"
#include <wiced_bt_mesh_cfg.h>
#include <wiced_bt_mesh_models.h>
#include <wiced_bt_mesh_provision.h>
#include <hci_control_api.h>
#include "trace.h"
#include <wiced_bt_ble.h>

#include <malloc.h>
#include <wiced_bt_mesh_core.h>
static void mesh_app_init(wiced_bool_t is_provisioned);
static void mesh_provision_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);
static void mesh_config_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);
static void mesh_control_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);
static void mesh_sensor_message_handler(uint8_t element_idx, uint16_t addr, uint16_t event, void *p_data);
static void mesh_core_state_changed(wiced_bt_mesh_core_state_type_t type, wiced_bt_mesh_core_state_t *p_state);
extern void mesh_provision_process_event(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);
extern void mesh_sensor_process_event(uint16_t addr, uint16_t event, void *p_data);

extern void proxy_gatt_send_cb(uint32_t conn_id, uint32_t ref_data, const uint8_t *packet, uint32_t packet_len);
static uint32_t mesh_nvram_access(wiced_bool_t write, int inx, uint8_t* value, uint16_t len, wiced_result_t *p_result);
typedef wiced_bool_t (*wiced_model_message_handler_t)(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);
static wiced_bt_mesh_core_received_msg_handler_t get_msg_handler_callback(uint16_t company_id, uint16_t opcode, uint16_t *p_model_id, wiced_bool_t *p_dont_save_rpl);
static void mesh_start_stop_scan_callback(wiced_bool_t start, wiced_bool_t is_active);
wiced_bool_t vendor_data_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

// defines Friendship Mode
#define MESH_FRIENDSHIP_NONE  0
#define MESH_FRIENDSHIP_FRND  1
#define MESH_FRIENDSHIP_LPN   2

/******************************************************
 *          Constants
 ******************************************************/
#define MESH_PID                0x3016
#define MESH_VID                0x0002
#define MESH_FWID               0x3016000101010000
#define MESH_CACHE_REPLAY_SIZE  200
#define APPEARANCE_GENERIC_TAG  512
#define MESH_VENDOR_COMPANY_ID          0x131   // Cypress Company ID
#define MESH_VENDOR_MODEL_ID            1       // ToDo.  This need to be modified

#define SENSOR_PROPERTY_ID_SIZE                     (2)
#define PROPERTY_ID_AMBIENT_LUX_LEVEL_ON            (0x2B)
#define PROPERTY_ID_MOTION_SENSED                   (0x42)
#define PROPERTY_ID_TOTAL_LIGHT_EXPOSURE_TIME       (0x6F)
#define PROPERTY_ID_PRESENCE_DETECTED               (0x4D)
#define PROPERTY_ID_PRESENT_AMBIENT_LIGHT_LEVEL     (0x4E)
#define PROPERTY_ID_PRESENT_AMBIENT_TEMPERATURE     (0x4F)
#define SETTING_PROPERTY_ID                         (0x2001)
// This sample shows simple use of vendor get/set/status messages.  Vendor model
// can define any opcodes it wants.
#define MESH_VENDOR_OPCODE_GET          1       // Command to Get data
#define MESH_VENDOR_OPCODE_SET          2       // Command to Set data ack is required
#define MESH_VENDOR_OPCODE_SET_UNACKED  3       // Command to Set data no ack is required
#define MESH_VENDOR_OPCODE_STATUS       4       // Response from the server, or unsolicited data change.

wiced_result_t mesh_transport_send_data( uint16_t opcode, uint8_t* p_data, uint16_t length );
void mesh_provisioner_client_message_handler(uint16_t event, void *p_data);
extern void mesh_provision_process_event(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

wiced_result_t provision_gatt_send(uint16_t conn_id, const uint8_t *packet, uint32_t packet_len);
//wiced_bool_t mesh_vendor_server_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);
//uint16_t     mesh_vendor_server_scene_store_handler(uint8_t element_idx, uint8_t *p_buffer, uint16_t buffer_len);
//uint16_t     mesh_vendor_server_scene_recall_handler(uint8_t element_idx, uint8_t *p_buffer, uint16_t buffer_len, uint32_t transition_time, uint32_t delay);
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

wiced_bt_mesh_core_config_property_t mesh_element1_properties[] =
        {
                { PROPERTY_ID_AMBIENT_LUX_LEVEL_ON,            0, 0, 3, NULL},
                { PROPERTY_ID_MOTION_SENSED,                   0, 0, 1, NULL},
                { PROPERTY_ID_PRESENCE_DETECTED,               0, 0, 1, NULL},
                { PROPERTY_ID_PRESENT_AMBIENT_LIGHT_LEVEL,     0, 0, 3, NULL},
                { WICED_BT_MESH_PROPERTY_TOTAL_DEVICE_RUNTIME, 0, 0, 3, NULL},
                { SETTING_PROPERTY_ID,                         0, 0, 2, NULL},
                { WICED_BT_MESH_PROPERTY_PRESENT_AMBIENT_TEMPERATURE,  0, 0, 1, NULL},
        };
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
        .properties_num = 7,                                            // Number of properties in the array models
        .properties = mesh_element1_properties,                                             // Array of properties in the element.
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
    .firmware_id        = MESH_FWID,
    .replay_cache_size  = MESH_CACHE_REPLAY_SIZE,                   // Number of replay protection entries, i.e. maximum number of mesh devices that can send application messages to this device.
    .features                  = 0,                                 // 
    .friend_cfg         =                                           // Empty Configuration of the Friend Feature
    {
        .receive_window        = 0,                                 // Receive Window value in milliseconds supported by the Friend node.
        .cache_buf_len         = 0                                  // Length of the buffer for the cache
    },
    .low_power          =                                           // Configuration of the Low Power Feature
    {
        .rssi_factor           = 0,                                 // contribution of the RSSI measured by the Friend node used in Friend Offer Delay calculations.
        .receive_window_factor = 0,                                 // contribution of the supported Receive Window used in Friend Offer Delay calculations.
        .min_cache_size_log    = 0,                                 // minimum number of messages that the Friend node can store in its Friend Cache.
        .receive_delay         = 0,                                 // Receive delay in 1 ms units to be requested by the Low Power node.
        .poll_timeout          = 0                                  // Poll timeout in 100ms unite to bt requested by the Low Power node.
    },
    .gatt_client_only          = WICED_TRUE,                        // Can connect to mesh over GATT or ADV
    .elements_num  = (uint8_t)(sizeof(mesh_elements) / sizeof(mesh_elements[0])),   // number of elements on this device
    .elements      = mesh_elements                                  // Array of elements for this device
};

/******************************************************
 *          Structures
 ******************************************************/

/******************************************************
 *          Function Prototypes
 ******************************************************/
extern wiced_bool_t mesh_gatt_client_provision_connect(wiced_bt_mesh_provision_connect_data_t *p_connect, wiced_bool_t use_pb_gatt);
extern void mesh_gatt_client_scan_unprovisioned_devices(uint8_t start);
extern wiced_bool_t mesh_gatt_client_proxy_connect(wiced_bt_mesh_proxy_connect_data_t *p_connect);
//extern wiced_bool_t mesh_gatt_client_provision_disconnect(wiced_bt_mesh_provision_disconnect_data_t *p_disconnect);
extern void mesh_gatt_client_search_proxy(uint8_t start);
extern void mesh_core_state_changed(wiced_bt_mesh_core_state_type_t type, wiced_bt_mesh_core_state_t *p_state);

static void mesh_config_client_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);

extern wiced_result_t wiced_send_gatt_packet( uint16_t opcode, uint8_t* p_data, uint16_t length );
wiced_bool_t wiced_bt_mesh_config_client_message_handler(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint16_t data_len);

char* filePath;
/******************************************************
 *               Function Definitions
 ******************************************************/
void mesh_application_init(void)
{
    static int core_initialized = 0;
    WICED_BT_TRACE("mesh_application_init enter");
    if(!core_initialized)
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

#if WICED_BT_MODELS_VERBOSE == 1
    WICED_BT_TRACE("company_id:%x opcode:%s\n", company_id, mesh_opcode_string(opcode));
#else
    WICED_BT_TRACE("company_id:%x opcode:%x\n", company_id, opcode);
#endif
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

#if 0
void SendWicedCommand(UINT16 opcode, LPBYTE p_data, DWORD len)
{
    WICED_BT_TRACE("SendWicedCommand\n");
    uint8_t* data = (uint8_t*)malloc(len);

    if (data)
        memcpy(data, p_data, len);

//    mesh_app_proc_rx_cmd(opcode, data, len);

    free(data);
}

uint8_t wiced_hci_send(uint16_t opcode, uint8_t *p_buffer, uint16_t length)
{
    SendWicedCommand(opcode, p_buffer, length);
    return TRUE;
}
#endif 

/*
 * Application implements that function to start/stop scanning as requested by the core
 */
void mesh_start_stop_scan_callback(wiced_bool_t start, wiced_bool_t is_scan_active)
{
    wiced_bt_ble_observe(start, 0, NULL);
}

void mesh_app_init(wiced_bool_t is_provisioned)
{
    WICED_BT_TRACE("mesh_app_init\n");
    extern uint8_t wiced_bt_mesh_model_trace_enabled;
    wiced_bt_mesh_model_trace_enabled = WICED_TRUE;

    wiced_bt_mesh_provision_client_init(mesh_provision_message_handler, is_provisioned);
    wiced_bt_mesh_client_init(mesh_provision_message_handler, is_provisioned);

    wiced_bt_mesh_config_client_init(mesh_config_message_handler, is_provisioned);
    wiced_bt_mesh_health_client_init(mesh_config_message_handler, is_provisioned);
    wiced_bt_mesh_proxy_client_init(mesh_config_message_handler, is_provisioned);
    WICED_BT_TRACE("wiced_bt_mesh_model_fw_provider_init\n");
    wiced_bt_mesh_model_fw_provider_init();
    WICED_BT_TRACE("wiced_bt_mesh_model_fw_distribution_server_init\n");
    wiced_bt_mesh_model_fw_distribution_server_init();

    wiced_bt_mesh_model_property_client_init(0, mesh_control_message_handler, is_provisioned);
    wiced_bt_mesh_model_onoff_client_init(0, mesh_control_message_handler, is_provisioned);
    wiced_bt_mesh_model_level_client_init(0, mesh_control_message_handler, is_provisioned);
    wiced_bt_mesh_model_light_lightness_client_init(0, mesh_control_message_handler, is_provisioned);
    wiced_bt_mesh_model_light_ctl_client_init(0, mesh_control_message_handler, is_provisioned);
    wiced_bt_mesh_model_light_hsl_client_init(0, mesh_control_message_handler, is_provisioned);
    wiced_bt_mesh_model_default_transition_time_client_init(0, mesh_control_message_handler, is_provisioned);
    wiced_bt_mesh_model_sensor_client_init(0, mesh_sensor_message_handler, is_provisioned);
}

void mesh_provision_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data)
{
    WICED_BT_TRACE("provision message:%d\n", event);
    mesh_provision_process_event(event, p_event, p_data);
}

void mesh_config_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data)
{
    Log("config message:%d\n", event);
    mesh_provision_process_event(event, p_event, p_data);
}

void mesh_control_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data)
{
    Log("control message:%d\n", event);
    mesh_provision_process_event(event, p_event, p_data);
}

void mesh_sensor_message_handler(uint8_t element_idx, uint16_t addr, uint16_t event, void *p_data)
{
    Log("sensor message:%d\n", event);
    mesh_sensor_process_event(addr, event, p_data);
}

#if 0
/*
 * Send provisioner provisioning end event over transport
 */
void mesh_provisioner_hci_event_provision_link_status_send(wiced_bt_mesh_provision_link_status_data_t *p_data)
{
    Log("provision connection status conn_id:%d addr:%x connected:%d\n", p_data->status);
    mesh_provision_process_event(WICED_BT_MESH_PROVISION_LINK_STATUS, NULL, (wiced_bt_mesh_provision_link_status_data_t *)p_data);
}
#endif

uint8_t mesh_provisioner_process_proxy_connected(uint8_t* p_data, uint16_t length) {
    int conn_id ;
    int mtu;
    STREAM_TO_UINT32(conn_id, p_data);
    STREAM_TO_UINT16(mtu, p_data);

    wiced_bt_mesh_core_connection_status(conn_id, TRUE,0,mtu);
    return 0;
}

#if 0
wiced_bool_t wiced_bt_mesh_proxy_disconnect(wiced_bt_mesh_provision_disconnect_data_t *p_disconnect)
{
//    return mesh_gatt_client_proxy_disconnect(&p_disconnect)  ? HCI_CONTROL_MESH_STATUS_SUCCESS : HCI_CONTROL_MESH_STATUS_ERROR;
}
#endif

uint32_t mesh_app_on_received_provision_gatt_pkt(uint8_t *p_data, uint32_t length)
{
    // packet has arrived from JAVA
    Log("\n mesh_app_on_received_provision_gatt_pkt\n");
    Logn(p_data,length);
    //TODO send proper connection id from JAVA
    wiced_bt_mesh_core_provision_gatt_packet(WICED_TRUE, 1, p_data, length);
    return 0;
}

static uint16_t write_reg(char name[80], uint8_t *value, uint16_t len, uint32_t *pInt) {
    return len;
}

static uint16_t read_reg(char name[80], uint8_t *value, uint16_t len, uint32_t *pInt) {
    return  len;
}

static uint32_t mesh_nvram_access(wiced_bool_t write, int inx, uint8_t* value, uint16_t len, wiced_result_t *p_result)
{
    char        name[80];
    uint32_t    res;
    //Log(name, sizeof(name), "NVRAM_%d", NVRAM_ID_MESH_START + inx);
    Log(name, sizeof(name), "NVRAM_%d", inx);
    if (!write)
        len = (uint16_t)read_reg(name, value, len, &res);
    else
        len = (uint16_t)write_reg(name, value, len, &res);
    if (p_result)
        //*p_result = res == ERROR_SUCCESS ? WICED_BT_SUCCESS : WICED_BT_ERROR;
        *p_result = WICED_BT_SUCCESS;
    return len;
}

void setDfuFilePath(char* filepath)
{
    Log("setting filepath");
    filePath = malloc(strlen(filepath));
    memcpy(filePath,filepath, strlen(filepath));
}

uint32_t GetDfuImageSize()
{
    uint32_t file_size;

    FILE *fPatch;

    if (filePath == NULL)
        return 0;

    if (fPatch = fopen(filePath, "rb"))
        return 0;

    // Load OTA FW file into memory
    fseek(fPatch, 0, SEEK_END);
    file_size = (int)ftell(fPatch);
    fclose(fPatch);
    return file_size;
}

void GetDfuImageChunk(uint8_t *p_data, uint32_t offset, uint16_t data_len)
{

    FILE *fPatch;
    if (fPatch = fopen(filePath, "rb"))
        return;

    // Load OTA FW file into memory
    fseek(fPatch, offset, SEEK_SET);
    fread(p_data, 1, data_len, fPatch);
    fclose(fPatch);
}

uint32_t wiced_bt_get_fw_image_size(uint8_t partition)
{
     return GetDfuImageSize();
}

void wiced_bt_get_fw_image_chunk(uint8_t partition, uint32_t offset, uint8_t *p_data, uint16_t data_len)
{
    return GetDfuImageChunk(p_data, offset, data_len);
}

wiced_bool_t wiced_bt_get_upgrade_fw_info(uint16_t *company_id, uint8_t *fw_id_len, uint8_t *fw_id)
{
    return WICED_FALSE;
}

wiced_bool_t wiced_ota_fw_upgrade_get_new_fw_info(uint16_t *company_id, uint8_t *fw_id_len, uint8_t *fw_id)
{
    return WICED_FALSE;
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
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
 * This file shows how to create a device which publishes onoff level.
 */
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_mesh_models.h"
#include "wiced_bt_trace.h"
#include "wiced_timer.h"
#include "wiced_bt_mesh_app.h"
#include "wiced_hal_gpio.h"

#ifdef HCI_CONTROL
#include "wiced_transport.h"
#include "hci_control_api.h"
#endif

#include "wiced_bt_cfg.h"
extern wiced_bt_cfg_settings_t wiced_bt_cfg_settings;

// Needed to pass some PTS tests which require vendor model
//#define MESH_VENDOR_TST_COMPANY_ID  0x131
//#define MESH_VENDOR_TST_MODEL_ID    1

#define MESH_ONOFF_TICK_IN_MS       100

/******************************************************
 *          Constants
 ******************************************************/
#define MESH_PID                0x3016
#define MESH_VID                0x0002
#define MESH_FWID               0x3016000101010001
#define MESH_CACHE_REPLAY_SIZE  0x0008

/******************************************************
 *          Structures
 ******************************************************/
typedef struct
{
#define MESH_TRANSITION_STATE_IDLE         0
#define MESH_TRANSITION_STATE_DELAY        1
#define MESH_TRANSITION_STATE_TRANSITION   2
    uint8_t              transition_state;
    uint8_t              present_state;
    uint8_t              target_state;
    uint32_t             transition_start_time;
    uint32_t             transition_remaining_time;
    uint32_t             transition_time;
    uint16_t             delay;
    wiced_timer_t        timer;
} mesh_onoff_server_t;

/******************************************************
 *          Function Prototypes
 ******************************************************/
static void     mesh_app_init(wiced_bool_t is_provisioned);
static uint32_t mesh_app_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length);
static void mesh_onoff_server_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data);
static void mesh_onoff_server_status_changed(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length);
static void mesh_onoff_server_send_status(wiced_bt_mesh_event_t *p_event);
static void mesh_onoff_process_set(wiced_bt_mesh_onoff_set_data_t *p_data);
static void mesh_onoff_app_timer_callback(uint32_t arg);

#ifdef HCI_CONTROL
static void mesh_onoff_hci_event_send_set(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_onoff_set_data_t *p_data);
#endif

/******************************************************
 *          Variables Definitions
 ******************************************************/
// Application state
mesh_onoff_server_t app_state;

uint8_t mesh_mfr_name[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MANUFACTURER_NAME] = { 'C', 'y', 'p', 'r', 'e', 's', 's', 0 };
uint8_t mesh_model_num[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MODEL_NUMBER]     = { '1', '2', '3', '4', 0, 0, 0, 0 };
uint8_t mesh_system_id[8]                                                  = { 0xbb, 0xb8, 0xa1, 0x80, 0x5f, 0x9f, 0x91, 0x71 };

#undef WICED_BT_MESH_DEVICE

#define WICED_BT_MESH_DEVICE \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_CONFIG_SRV, NULL, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_HEALTH_SRV, NULL, NULL, NULL }, \
    { MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_DIRECTED_FORWARDING_SRV, NULL, NULL, NULL }

wiced_bt_mesh_core_config_model_t   mesh_element1_models[] =
{
    WICED_BT_MESH_DEVICE,
    WICED_BT_MESH_MODEL_FW_DISTRIBUTION_SERVER,
    WICED_BT_MESH_MODEL_FW_UPDATE_SERVER,
    WICED_BT_MESH_MODEL_ONOFF_SERVER,
#ifdef MESH_VENDOR_TST_MODEL_ID
    { MESH_VENDOR_TST_COMPANY_ID, MESH_VENDOR_TST_MODEL_ID, NULL, NULL, NULL },
#endif
};
#if NUM_ONOFF_SERVERS > 1
wiced_bt_mesh_core_config_model_t   mesh_element2_models[] =
{
    WICED_BT_MESH_MODEL_ONOFF_SERVER,
};
#endif
#if NUM_ONOFF_SERVERS > 2
wiced_bt_mesh_core_config_model_t   mesh_element3_models[] =
{
    WICED_BT_MESH_MODEL_ONOFF_SERVER,
};
#endif
#if NUM_ONOFF_SERVERS > 3
wiced_bt_mesh_core_config_model_t   mesh_element4_models[] =
{
    WICED_BT_MESH_MODEL_ONOFF_SERVER,
};
#endif

#define MESH_ONOFF_SERVER_ELEMENT_INDEX   0

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
        .models_num = (sizeof(mesh_element1_models) / sizeof(wiced_bt_mesh_core_config_model_t)),    // Number of models in the array models
        .models = mesh_element1_models,                                 // Array of models located in that element. Model data is defined by structure wiced_bt_mesh_core_config_model_t
    },
#if NUM_ONOFF_SERVERS > 1
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
        .models_num = (sizeof(mesh_element2_models) / sizeof(wiced_bt_mesh_core_config_model_t)),                              // Number of models in the array models
        .models = mesh_element2_models,                                 // Array of models located in that element. Model data is defined by structure wiced_bt_mesh_core_config_model_t
    },
#endif
#if NUM_ONOFF_SERVERS > 2
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
        .models_num = (sizeof(mesh_element3_models) / sizeof(wiced_bt_mesh_core_config_model_t)),                              // Number of models in the array models
        .models = mesh_element3_models,                                 // Array of models located in that element. Model data is defined by structure wiced_bt_mesh_core_config_model_t
    },
#endif
#if NUM_ONOFF_SERVERS > 3
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
        .models_num = (sizeof(mesh_element4_models) / sizeof(wiced_bt_mesh_core_config_model_t)),                              // Number of models in the array models
        .models = mesh_element4_models,                                 // Array of models located in that element. Model data is defined by structure wiced_bt_mesh_core_config_model_t
    },
#endif
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
        .poll_timeout          = 200                                // Poll timeout in 100ms units to be requested by the Low Power node.
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
    .directed_forward   =
    {
        .ptreq_delay_timer_sec = 0,                                 // Value of Path Tracing Delay timer. Path originator start path tracing with that delay after path is established. Default value is 0 - no path tracing.
        .flags                 = 0,                                 // Any combinations of WICED_BT_MESH_CORE_CONFIG_DIRECTED_FORWARDING_FLAG_XXX
        .wanted_rssi           = 0                                  // The minimum RSSI that the Directed Forwarding node requires at its antenna. 0 means use default value -127 
    },
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
    // uncomment out next line for PTS tests  when the node is originator
    wiced_bt_mesh_core_directed_forwarding_test_flags = WICED_BT_MESH_CORE_DIRECTED_FORWARDING_TEST_FLAGS_DISABLE_DISCOVERY_INIT;
#if 0
    extern uint8_t wiced_bt_mesh_model_trace_enabled;
    wiced_bt_mesh_model_trace_enabled = WICED_TRUE;

    // enable core trace
    extern void wiced_bt_mesh_core_set_trace_level(uint32_t fids_mask, uint8_t level);
    wiced_bt_mesh_core_set_trace_level(0xffffffff, 4);      //(ALL, TRACE_DEBUG)
    wiced_bt_mesh_core_set_trace_level(0x4, 3);             //(CORE_AES_CCM_C, TRACE_INFO)
#endif

    wiced_bt_cfg_settings.device_name = (uint8_t *)"OnOff Server";
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

    memset (&app_state, 0, sizeof(app_state));
    app_state.transition_state = MESH_TRANSITION_STATE_IDLE;
    wiced_init_timer(&app_state.timer, &mesh_onoff_app_timer_callback, 0, WICED_MILLI_SECONDS_TIMER);

#if REMOTE_PROVISIONG_SERVER_SUPPORTED
    wiced_bt_mesh_remote_provisioning_server_init();
#endif

    wiced_bt_mesh_model_onoff_server_init(MESH_ONOFF_SERVER_ELEMENT_INDEX, mesh_onoff_server_message_handler, is_provisioned);

    wiced_bt_mesh_model_fw_update_server_init(MESH_ONOFF_SERVER_ELEMENT_INDEX, NULL, is_provisioned);
    wiced_bt_mesh_model_fw_distribution_server_init();

#if NUM_ONOFF_SERVERS > 1
    wiced_bt_mesh_model_onoff_server_init(1, mesh_onoff_server_message_handler, is_provisioned);
#endif
#if NUM_ONOFF_SERVERS > 2
    wiced_bt_mesh_model_onoff_server_init(2, mesh_onoff_server_message_handler, is_provisioned);
#endif
#if NUM_ONOFF_SERVERS > 3
    wiced_bt_mesh_model_onoff_server_init(3, mesh_onoff_server_message_handler, is_provisioned);
#endif
}

// Uncomment next line for PTS testing
wiced_bool_t directed_forwarding_discovery_init(uint16_t src, uint16_t dst, uint16_t dpo, uint8_t bearer_idx, uint8_t net_key_idx);
void mesh_application_factory_reset(void);

/*
 * Process HW interrupts. 
 * Return TRUE if processed, if function returns false, the common logic in mesh_app lib performs factory reset on a button push.
 *
 * To build app for DF PTS tests do following steps:
 * 1. Use the special build of the core library with DF support:
 *    Copy attached mesh_core_lib-d.a to the C:\Users\<user>\Documents\WICED-Studio-6.2\20719-B1_Bluetooth\WICED\libraries overwriting existing one.
 * 2. Copy DF test app to the SDK:
 *    Copy attached folder mesh_direct_forward_test with two files inside to the C:\Users\<user>\Documents\WICED-Studio-6.2\common\apps\snip\mesh
 * 3. Configure to use hardcoded node-UUID and BD-ADDRESS instead of random:
 *    Open file makefile.mk in the C:\Users\<user>\Documents\WICED-Studio-6.2\common\libraries\mesh_app_lib and uncomment following line:
 * C_FLAGS += -DPTS
 * 4. Make sure you selected platform 20719-B1_Bluetooth in the menu "WICED Platform"
 * 5. Create Make Target for DF test app with hardcoded node BD-ADDRESS and COM port:
 *    Right click in the Make Target window (right window) and select New and paste this string into Target Name and press OK.
 *    snip.mesh.mesh_direct_forward_test-CYW920719Q40EVB_01 BT_DEVICE_ADDRESS=207350000005 UART=COM3 download
 *    When you attach device to USB it creates two COM ports. For Example, COM3 and COM4. Then UART field in the above Make Target should be COM3,
 *    and you can use COM4 to collect device log in case you want to send it to us for issue investigation.
 * 6. Build/download app:
 *    Double click on Make Target created on step 5
 * 7. Friend feature is always denable on the device
 * 8. We can’t just initiate PTREQ - spec doesn’t have such procedure. But per spec “When a Path Originator creates a Forwarding Table entry or the node may start a Path Tracing Delay timer”.
 *    To enable that set non-0 value (for example 10 seconds) to ptreq_delay_timer_sec:
 *    Open file mesh_direct_forward_test.c in the the C:\Users\<user>\Documents\WICED-Studio-6.2\common\apps\snip\mesh\mesh_direct_forward_test
 *    Find the place of the member ptreq_delay_timer_sec initialization and set 10
 *         .ptreq_delay_timer_sec = 10,                                // Value of Path Tracing Delay timer. Path originator start path tracing with that delay after path is established. Default value is 0 - no path tracing.
 *
 * To reset device keeping it provisioned state press Reset button SW2 - middle button
 * For DF PTS tests actions do N short button press (shorter than 1 sec) and one long button press (longer than 1 sec).
 * Use button SW3 - right most button.
 * Where N can be:
 * 0 - (meaning don't do short press) initialize discovery for unicast PD (1)
 * 1 - initialize discovery for group PD (0xc110)
 * 2 - initialize discovery for virtual PD (0x8110)
 * 3 - factory reset - puts node to un-provisioned state
 */
wiced_bool_t mesh_app_interrupt_handler(void* user_data, uint8_t pin)
{
// make it if 0 for PTS testing
#if 0
    return WICED_FALSE;
#else
    static uint8_t  cnt = 0;
    static uint64_t press_time = 0;
    uint32_t value = wiced_hal_gpio_get_pin_input_status(pin);
    WICED_BT_TRACE("mesh_app_interrupt_handler: pin:%d value:%d cnt:%d\n", pin, value, cnt);
    // on press just remember current time
    if (value == 0)
        press_time = wiced_bt_mesh_core_get_tick_count();
    else
    {
        // on release measure pressed time.
        press_time = wiced_bt_mesh_core_get_tick_count() - press_time;
        WICED_BT_TRACE(" press_time:%d cnt:%d\n", (uint32_t)press_time, cnt);
        // in case of short press (< 1 second) just increment press counter
        if (press_time < 1000)
            cnt++;
        else
        {
            WICED_BT_TRACE(" run case %d\n", cnt);
            // in case of long press do action depending on press counter
            switch (cnt)
            {
            case 0:
                //PTS test 1: node is originator: unicast PD 
                directed_forwarding_discovery_init(2, 1, 0, 0, 0);
                break;
            case 1:
                // PTS test 2:node is originatore: group PD 
                directed_forwarding_discovery_init(2, 0xc110, 0, 0, 0);
                break;
            case 2:
                // PTS test 2:node is originatore: virtual PD 
                directed_forwarding_discovery_init(2, 0x8110, 0, 0, 0);
                break;
            case 3:
                // Factory reset
                mesh_application_factory_reset();
                break;
            }
            // reset press counter
            cnt = 0;
        }
    }
#endif
    return WICED_TRUE;
}

/*
 * Process event received from the OnOff Client.
 */
void mesh_onoff_server_message_handler(uint16_t event, wiced_bt_mesh_event_t *p_event, void *p_data)
{
#if defined HCI_CONTROL
    wiced_bt_mesh_hci_event_t *p_hci_event;
#endif
    WICED_BT_TRACE("onoff srv msg:%d\n", event);

    switch (event)
    {
    case WICED_BT_MESH_ONOFF_GET:
        mesh_onoff_server_send_status(wiced_bt_mesh_create_reply_event(p_event));
        break;

    case WICED_BT_MESH_ONOFF_SET:
#if defined HCI_CONTROL
        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
            mesh_onoff_hci_event_send_set(p_hci_event, (wiced_bt_mesh_onoff_set_data_t *)p_data);
#endif
        mesh_onoff_process_set((wiced_bt_mesh_onoff_set_data_t *)p_data);
        if (p_event->reply)
        {
            mesh_onoff_server_send_status(wiced_bt_mesh_create_reply_event(p_event));
        }
        else
        {
            wiced_bt_mesh_release_event(p_event);
        }
        break;

    default:
        WICED_BT_TRACE("unknown\n");
    }
}

/*
 * In 2 chip solutions MCU can send commands to change onoff state.
 */
uint32_t mesh_app_proc_rx_cmd(uint16_t opcode, uint8_t *p_data, uint32_t length)
{
    wiced_bt_mesh_event_t *p_event;

    WICED_BT_TRACE("[%s] cmd_opcode 0x%02x\n", __FUNCTION__, opcode);

    switch (opcode)
    {
#ifdef HCI_CONTROL
    case HCI_CONTROL_MESH_COMMAND_ONOFF_SET:
        p_event = wiced_bt_mesh_create_event_from_wiced_hci(opcode, MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_ONOFF_SRV, &p_data, &length);
        if (p_event == NULL)
        {
            WICED_BT_TRACE("bad hdr\n");
            return WICED_TRUE;
        }
        mesh_onoff_server_status_changed(p_event, p_data, length);
        break;
#endif
    default:
        WICED_BT_TRACE("unknown\n");
        return WICED_FALSE;
    }
    return WICED_TRUE;
}

/*
 * Send OnOff Status event
 */
void mesh_onoff_server_status_changed(wiced_bt_mesh_event_t *p_event, uint8_t *p_data, uint32_t length)
{
    STREAM_TO_UINT8(app_state.present_state, p_data);
    STREAM_TO_UINT8(app_state.target_state, p_data);
    STREAM_TO_UINT32(app_state.transition_remaining_time, p_data);

    mesh_onoff_server_send_status(p_event);
}

/*
 * Sample implementation of the command from the on/off client to set the state
 */
void mesh_onoff_process_set(wiced_bt_mesh_onoff_set_data_t *p_set)
{
    wiced_result_t res;

    WICED_BT_TRACE("onoff srv set state:%d onoff:%d trans time:%d delay:%d\n", app_state.transition_state, p_set->onoff, p_set->transition_time, p_set->delay);

    // whenever we receive set, the previous transition is cancelled.
    app_state.transition_state  = MESH_TRANSITION_STATE_IDLE;
    wiced_stop_timer(&app_state.timer);

    // If transition time is 0, change the state immediately to the target state.
    if (p_set->transition_time == 0)
    {
        app_state.target_state              = p_set->onoff;
        app_state.present_state             = p_set->onoff;
        app_state.transition_remaining_time = 0;
    }
    else
    {
        app_state.target_state          = p_set->onoff;
        app_state.transition_time       = p_set->transition_time;
        app_state.delay                 = p_set->delay;
        app_state.transition_start_time = wiced_bt_mesh_core_get_tick_count();

        if (p_set->delay != 0)
        {
            app_state.transition_state  = MESH_TRANSITION_STATE_DELAY;
            res = wiced_start_timer(&app_state.timer, app_state.delay);
        }
        else
        {
            app_state.transition_state          = MESH_TRANSITION_STATE_TRANSITION;
            app_state.transition_remaining_time = app_state.transition_time;
            res = wiced_start_timer(&app_state.timer, MESH_ONOFF_TICK_IN_MS);
        }
        WICED_BT_TRACE("next state:%d start_timer_result:%d\n", app_state.transition_state, res);
    }
}

/*
 * Transition timeout
 */
void mesh_onoff_app_timer_callback(uint32_t arg)
{
    WICED_BT_TRACE("OnOff timer state:%d remain:%d\n", app_state.transition_state, app_state.transition_remaining_time);

    // if it was a delay before actual transition, start transition now
    if (app_state.transition_state == MESH_TRANSITION_STATE_DELAY)
    {
        app_state.transition_state          = MESH_TRANSITION_STATE_TRANSITION;
        app_state.transition_remaining_time = app_state.transition_time;

        // if transition is to On state, the present should be On as soon as we started transition.
        if (app_state.target_state == 1)
        {
            app_state.present_state = 1;
        }
        if (app_state.transition_remaining_time == 0)
        {
            mesh_onoff_server_send_status(NULL);
        }
        else
        {
            wiced_start_timer(&app_state.timer, MESH_ONOFF_TICK_IN_MS);
        }
        return;
    }
    // check if we are done
    if (app_state.transition_remaining_time < MESH_ONOFF_TICK_IN_MS)
    {
        app_state.transition_state          = MESH_TRANSITION_STATE_IDLE;
        app_state.transition_remaining_time = 0;
        app_state.present_state             = app_state.target_state;

        mesh_onoff_server_send_status(NULL);
        return;
    }
    // still in the transition, adjust remaining time and start timer for another tick
    app_state.transition_remaining_time -= MESH_ONOFF_TICK_IN_MS;
    wiced_start_timer(&app_state.timer, MESH_ONOFF_TICK_IN_MS);
}

/*
 * Send On/Off status message to the On/Off Client
 */
void mesh_onoff_server_send_status(wiced_bt_mesh_event_t *p_event)
{
    wiced_bt_mesh_onoff_status_data_t event;

    event.present_onoff  = app_state.present_state;
    event.target_onoff   = app_state.target_state;
    event.remaining_time = (app_state.transition_state == MESH_TRANSITION_STATE_DELAY) ?
        app_state.transition_time + app_state.delay - (wiced_bt_mesh_core_get_tick_count() - app_state.transition_start_time) :
        app_state.transition_remaining_time;

    if (p_event == NULL)
    {
        p_event = wiced_bt_mesh_create_event(0, MESH_COMPANY_ID_BT_SIG, WICED_BT_MESH_CORE_MODEL_ID_GENERIC_ONOFF_SRV, 0, 0);
        if (p_event == NULL)
        {
            WICED_BT_TRACE("mesh onoff status: no pub\n");
            return;
        }
    }
    wiced_bt_mesh_model_onoff_server_send_status(p_event, &event);
}

#ifdef HCI_CONTROL
/*
 * Send OnOff Set event over transport
 */
void mesh_onoff_hci_event_send_set(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_onoff_set_data_t *p_data)
{
    uint8_t *p = p_hci_event->data;

    UINT8_TO_STREAM(p, p_data->onoff);
    UINT32_TO_STREAM(p, p_data->transition_time);
    UINT16_TO_STREAM(p, p_data->delay);

    mesh_transport_send_data(HCI_CONTROL_MESH_EVENT_ONOFF_SET, (uint8_t *)p_hci_event, (uint16_t)(p - (uint8_t *)p_hci_event));
}

#endif

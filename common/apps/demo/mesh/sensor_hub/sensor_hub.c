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
 * This demo application shows a  implementation of a sensor hub.
 * The app is based on the snip/mesh/mesh_sensor_server sample which
 * implements BLE Mesh Sensor Server model.
 *
 * Features demonstrated
 *  - Temperature measurement using the on board Thermistor on the EVK
 *  - Ambient light measurement using the on board sensor
 *  - Configuring and receiving interrupts from the PIR motion sensor
 *  - Publishing motion data over BLE mesh
 *
 *
 * To demonstrate the app, walk through the following steps.
 * 1. Build and download the application (to the WICED board)
 * 2. Use Android MeshController and provision the sensor hub
 * 3. After successful provisioning, user can use the Android MeshController/Mesh Client to configure the below parameters of the sensor
 *    a> configure sensor to publish the sensor data to a group(all-nodes, all-relays).
 *    b> configure publish period : publish period defines how often the user wants the sensor to publish the data.
 *    c> set cadence of the sensor :
 *       set minimum interval in which sensor data has to be published.
 *       set the range in which the fast cadence has to be observed.
 *       set the fast cadence period (how fast the data has to be published with respect to publish period).
 *       set the unit in which if the values change the data should be published and trigger type (Native or percentage).
 *           example : publish data if the data changes by 2 units/10%
 */
#include "wiced_bt_uuid.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_mesh_models.h"
#include "wiced_bt_trace.h"
#include "wiced_timer.h"
#include "wiced_bt_mesh_app.h"
#include "wiced_thermistor.h"
#include "wiced_hal_nvram.h"
#include "wiced_sleep.h"
#include "e93196.h"
#include "max_44009.h"

#include "wiced_bt_cfg.h"
extern wiced_bt_cfg_settings_t wiced_bt_cfg_settings;

/******************************************************
 *          Constants
 ******************************************************/
#define MESH_PID                0x3122
#define MESH_VID                0x0002
#define MESH_FWID               0x3122000101010001
#define MESH_CACHE_REPLAY_SIZE  0x0008

#define NUM_ELEMENTS                2
#define NUM_SENSORS                 3

#define PRESENCE_SENSOR_INDEX       0
#define LIGHT_SENSOR_INDEX          1
#define TEMPERATURE_SENSOR_INDEX    2

// The onboard thermistor hardware has a positive and negative tolerance of 1%
#define MESH_SENSOR_HUB_POSITIVE_TOLERANCE      CONVERT_TOLERANCE_PERCENTAGE_TO_MESH(1)
#define MESH_SENSOR_HUB_NEGATIVE_TOLERANCE      CONVERT_TOLERANCE_PERCENTAGE_TO_MESH(1)

#define MESH_SENSOR_HUB_SAMPLING_FUNCTION       WICED_BT_MESH_SENSOR_SAMPLING_FUNCTION_UNKNOWN
#define MESH_SENSOR_HUB_MEASUREMENT_PERIOD      WICED_BT_MESH_SENSOR_VAL_UNKNOWN
#define MESH_SENSOR_HUB_UPDATE_INTERVAL         WICED_BT_MESH_SENSOR_VAL_UNKNOWN

#define MESH_SENSOR_HUB_CADENCE_NVRAM_ID        WICED_NVRAM_VSID_START
#define MESH_SENSOR_HUB_PRESENCE_NVRAM_ID       (MESH_SENSOR_HUB_CADENCE_NVRAM_ID + PRESENCE_SENSOR_INDEX)
#define MESH_SENSOR_HUB_LIGHT_NVRAM_ID          (MESH_SENSOR_HUB_CADENCE_NVRAM_ID + LIGHT_SENSOR_INDEX)
#define MESH_SENSOR_HUB_TEMPERATURE_NVRAM_ID    (MESH_SENSOR_HUB_CADENCE_NVRAM_ID + TEMPERATURE_SENSOR_INDEX)

// After presence is detected, interrupts are disabled for 5 seconds
#define MESH_PRESENCE_DETECTED_BLIND_TIME               5
/******************************************************
 *          Structures
 ******************************************************/
uint8_t cfg_data[2] = { MAX44009_CFG_REGISTER, 0x40 };
uint8_t irq_set[2] = { MAX44009_INTERRUPT_ENABLE, 0x01 };
uint8_t low_threshold_set[2] = { MAX44009_LOW_THRESHOLD, 0x03 }; //set low threadhold to 2.16
uint8_t threshold_timer_set[2] = { MAX44009_THRESHOLD_TIMER, 0x00 };

MAX44009_USR_SET max44009_usr_set = { .scl_pin = WICED_P27,
        .sda_pin = WICED_P32, .irq_pin = WICED_P29, .cfg_reg = cfg_data,
        .interrupt_enable = irq_set, .low_threashHold = low_threshold_set,
        .threadHold_timer = threshold_timer_set
};

e93196_usr_cfg_t e93196_usr_cfg =
{
    .doci_pin           = WICED_P05,                                /* Interrupt/Data output Clock input configure pin          */
    .serin_pin          = WICED_P17,                                /* Serial Input configure pin                               */
    .e93196_init_reg    =
    {
        .sensitivity    = 0x10,                                     /* [24:17]sensitivity,   [Register Value] * 6.5uV           */
        .blind_time     = MESH_PRESENCE_DETECTED_BLIND_TIME * 2,    /* [16:13]blind time,    [Register Value] * 0.5s, max is 8s */
        .pulse_cnt      = 0x01,                                     /* [12:11]pulse count                                       */
        .window_time    = 0x01,                                     /* [10:9]window time                                        */
        .move_dete_en   = 0x01,                                     /* [8]move detect enable                                    */
        .int_src        = 0x00,                                     /* [7]irq source                                            */
        .adc_filter     = 0x01,                                     /* [6:5]ADC filter                                          */
        .power_en       = 0x00,                                     /* [4]power enable                                          */
        .self_test_en   = 0x00,                                     /* [3]selftest                                              */
        .capa           = 0x00,                                     /* [2]selftest capacity                                     */
        .test_mode      = 0x00,                                     /* [1:0]reserved                                            */
    }
};

/******************************************************
 *          Function Prototypes
 ******************************************************/
static void         mesh_app_init(wiced_bool_t is_provisioned);
static wiced_bool_t mesh_app_notify_period_set(uint8_t element_idx, uint16_t company_id, uint16_t model_id, uint32_t period);
static void         mesh_app_lpn_sleep(uint32_t timeout);
static void         mesh_app_factory_reset(void);
static void         mesh_sensor_server_restart_timer(wiced_bt_mesh_core_config_sensor_t *p_sensor);
static void         mesh_sensor_server_report_handler(uint16_t event, uint8_t element_idx, void *p_get_data, void *p_ref_data);
static void         mesh_sensor_server_config_change_handler(uint8_t element_idx, uint16_t event, uint16_t property_id, uint16_t setting_property_id);
static void         mesh_sensor_server_send_status(wiced_bt_mesh_event_t *p_event, uint16_t property_id);
static void         mesh_sensor_server_process_cadence_changed(uint8_t element_idx, uint16_t property_id);
static void         mesh_sensor_server_process_setting_changed(uint8_t element_idx, uint16_t property_id, uint16_t setting_property_id);
static int8_t       mesh_sensor_get_temperature_8(void);
static void         mesh_sensor_publish_timer_callback(TIMER_PARAM_TYPE arg);
static void         mesh_sensor_server_enter_hid_off(uint32_t timeout_ms);
static uint32_t     mesh_sensor_get_current_ambient_light(void);
static int32_t      mesh_sensor_get_current_value(void);
static void         max44009IntProc(void* data, uint8_t port_pin);
static void         e93196_int_proc(void* data, uint8_t port_pin);
static void         mesh_sensor_presence_detected_timer_callback(TIMER_PARAM_TYPE arg);

#ifdef HCI_CONTROL
static void         mesh_sensor_hci_event_send_cadence_set(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_sensor_cadence_set_data_t *p_set);
static void         mesh_sensor_hci_event_send_setting_set(wiced_bt_mesh_hci_event_t *p_hci_event, wiced_bt_mesh_sensor_setting_set_data_t *p_set);
#endif

/******************************************************
 *          Variables Definitions
 ******************************************************/
uint8_t mesh_mfr_name[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MANUFACTURER_NAME]          = { 'C', 'y', 'p', 'r', 'e', 's', 's', 0 };
uint8_t mesh_model_num[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MODEL_NUMBER]              = { '1', '2', '3', '4', 0, 0, 0, 0 };
uint8_t mesh_prop_fw_version[WICED_BT_MESH_PROPERTY_LEN_DEVICE_FIRMWARE_REVISION] =   { '0', '6', '.', '0', '2', '.', '0', '5' }; // this is overwritten during init
uint8_t mesh_system_id[8]                                                           = { 0xbb, 0xb8, 0xa1, 0x80, 0x5f, 0x9f, 0x91, 0x71 };

// We define optional setting for the motion sensor, the Motion Threshold. Default is 80%.
uint8_t         mesh_motion_sensor_threshold_val = 0x50;

// Present Ambient Temperature property uses Temperature 8 format, i.e. 0.5 degree Celsius.
uint32_t        mesh_sensor_current_value[NUM_SENSORS] = { 0 };
uint32_t        mesh_sensor_sent_value[NUM_SENSORS] = { 0 };
uint32_t        mesh_sensor_sent_time[NUM_ELEMENTS];                    // time stamp when temperature was published
uint32_t        mesh_sensor_publish_period[NUM_ELEMENTS] = { 0 };       // publish period in msec
uint32_t        mesh_sensor_fast_publish_period[NUM_ELEMENTS] = { 0 };  // publish period in msec when values are outside of limit
uint32_t        mesh_sensor_sleep_timeout[NUM_ELEMENTS] = { 0 };        // timeout value in msec that is currently running
wiced_timer_t   mesh_sensor_cadence_timer[NUM_ELEMENTS];

wiced_bool_t  presence_detected = WICED_FALSE;
wiced_timer_t mesh_sensor_presence_detected_timer;

uint8_t mesh_SENSOR_HUB_setting0_val[] = { 0x01, 0x00, 0x00 };

wiced_bt_mesh_core_config_model_t mesh_element1_models[] =
{
    WICED_BT_MESH_DEVICE,
    WICED_BT_MESH_MODEL_SENSOR_SERVER,
};
#define MESH_APP_NUM_MODELS  (sizeof(mesh_element1_models) / sizeof(wiced_bt_mesh_core_config_model_t))

wiced_bt_mesh_core_config_model_t mesh_element2_models[] =
{
    WICED_BT_MESH_MODEL_SENSOR_SERVER,
};

wiced_bt_mesh_sensor_config_setting_t sensor_settings[] =
{
    {
        .setting_property_id = WICED_BT_MESH_PROPERTY_TOTAL_DEVICE_RUNTIME,
        .access              = WICED_BT_MESH_SENSOR_SETTING_READABLE_AND_WRITABLE,
        .value_len           = 3,
        .val                 = mesh_SENSOR_HUB_setting0_val
    },
};

wiced_bt_mesh_sensor_config_setting_t sensor_settings_motion_sensor[] =
{
    {
        .setting_property_id = WICED_BT_MESH_PROPERTY_MOTION_THRESHOLD,
        .access              = WICED_BT_MESH_SENSOR_SETTING_READABLE_AND_WRITABLE,
        .value_len           = WICED_BT_MESH_PROPERTY_LEN_MOTION_THRESHOLD,
        .val                 = &mesh_motion_sensor_threshold_val
    },
};


// First element will contain Presence and Ambient light sensors. That way the
// device will publish data in the same message.  The second element will contain the
// temperature sensor. It can be configured and read separately.
#define MESH_SENSOR_HUB_PRESENCE_ELEMENT_INDEX      0
#define MESH_SENSOR_HUB_LIGHT_ELEMENT_INDEX         0
#define MESH_SENSOR_HUB_TEMPERATURE_ELEMENT_INDEX   1

#define MESH_SENSOR_HUB_PRESENCE_SENSOR_INDEX       0
#define MESH_SENSOR_HUB_LIGHT_SENSOR_INDEX          1
#define MESH_SENSOR_HUB_TEMPERATURE_SENSOR_INDEX    0

wiced_bt_mesh_core_config_sensor_t mesh_element1_sensors[] =
{
    {
        .property_id    = WICED_BT_MESH_PROPERTY_PRESENCE_DETECTED,
        .prop_value_len = WICED_BT_MESH_PROPERTY_LEN_PRESENCE_DETECTED,
        .descriptor =
        {
            .positive_tolerance = CONVERT_TOLERANCE_PERCENTAGE_TO_MESH(1),
            .negative_tolerance = CONVERT_TOLERANCE_PERCENTAGE_TO_MESH(1),
            .sampling_function  = WICED_BT_MESH_SENSOR_SAMPLING_FUNCTION_UNKNOWN,
            .measurement_period = WICED_BT_MESH_SENSOR_VAL_UNKNOWN,
            .update_interval    = WICED_BT_MESH_SENSOR_VAL_UNKNOWN,
         },
        .data = (uint8_t*)&mesh_sensor_sent_value[0],
        .cadence =
        {
            // Value 0 indicates that cadence does not change depending on the measurements
            .fast_cadence_period_divisor = 32,          // Recommended publish period is 320sec, 32 will make fast period 10sec
            .trigger_type_percentage     = WICED_FALSE, // The Property is Bool, does not make sense to use percentage
            .trigger_delta_down          = 0,           // This will not cause message when presence changes from 1 to 0
            .trigger_delta_up            = 1,           // This will cause immediate message when presence changes from 0 to 1
            .min_interval                = (1 << 10),   // Not used
            .fast_cadence_low            = 1,           // When low is larger than high, the fast cadence is on if the value
            .fast_cadence_high           = 0,           // is more or equal cadence_low or less then cadence_high. This is what we need.
        },
        .num_series     = 0,
        .series_columns = NULL,
        .num_settings   = 0,
        .settings       = NULL,
    },
    {
        .property_id = WICED_BT_MESH_PROPERTY_PRESENT_AMBIENT_LIGHT_LEVEL,
        .prop_value_len = WICED_BT_MESH_PROPERTY_LEN_PRESENT_AMBIENT_LIGHT_LEVEL,
        .descriptor =
        {
            .positive_tolerance = CONVERT_TOLERANCE_PERCENTAGE_TO_MESH(1),
            .negative_tolerance = CONVERT_TOLERANCE_PERCENTAGE_TO_MESH(1),
            .sampling_function  = WICED_BT_MESH_SENSOR_SAMPLING_FUNCTION_UNKNOWN,
            .measurement_period = WICED_BT_MESH_SENSOR_VAL_UNKNOWN,
            .update_interval    = WICED_BT_MESH_SENSOR_VAL_UNKNOWN,
        },
        .data = (uint8_t*)&mesh_sensor_current_value[1],
        .cadence =
        {
            // Value 1 indicates that cadence does not change depending on the measurements
            .fast_cadence_period_divisor = 1,
            .trigger_type_percentage     = WICED_FALSE,
            .trigger_delta_down          = 0,
            .trigger_delta_up            = 0,
            .min_interval                = (1 << 0x0C),
            .fast_cadence_low            = 0,
            .fast_cadence_high           = 0,
        },
        .num_series     = 0,
        .series_columns = NULL,
        .num_settings   = 0,
        .settings       = NULL,
    },
};

wiced_bt_mesh_core_config_sensor_t mesh_element2_sensors[] =
{
    {
        .property_id = WICED_BT_MESH_PROPERTY_PRESENT_AMBIENT_TEMPERATURE,
        .prop_value_len = WICED_BT_MESH_PROPERTY_LEN_PRESENT_AMBIENT_TEMPERATURE,
        .descriptor =
        {
            .positive_tolerance = MESH_SENSOR_HUB_POSITIVE_TOLERANCE,
            .negative_tolerance = MESH_SENSOR_HUB_NEGATIVE_TOLERANCE,
            .sampling_function  = MESH_SENSOR_HUB_SAMPLING_FUNCTION,
            .measurement_period = MESH_SENSOR_HUB_MEASUREMENT_PERIOD,
            .update_interval    = MESH_SENSOR_HUB_UPDATE_INTERVAL,
        },
        .data = (uint8_t*)&mesh_sensor_current_value[2],
        .cadence =
        {
            // Value 1 indicates that cadence does not change depending on the measurements
            .fast_cadence_period_divisor = 1,
            .trigger_type_percentage     = WICED_FALSE,
            .trigger_delta_down          = 0,
            .trigger_delta_up            = 0,
            .min_interval                = (1 << 0x0C),
            .fast_cadence_low            = 0,
            .fast_cadence_high           = 0,
        },
        .num_series     = 0,
        .series_columns = NULL,
        .num_settings   = 0,
        .settings       = NULL,
    },
};

wiced_bt_mesh_core_config_element_t mesh_elements[] =
{
    {
        .location = MESH_ELEM_LOC_MAIN,                                  // location description as defined in the GATT Bluetooth Namespace Descriptors section of the Bluetooth SIG Assigned Numbers
        .default_transition_time = MESH_DEFAULT_TRANSITION_TIME_IN_MS,   // Default transition time for models of the element in milliseconds
        .onpowerup_state = WICED_BT_MESH_ON_POWER_UP_STATE_RESTORE,      // Default element behavior on power up
        .default_level = 0,                                              // Default value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .range_min = 1,                                                  // Minimum value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .range_max = 0xffff,                                             // Maximum value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .move_rollover = 0,                                              // If true when level gets to range_max during move operation, it switches to min, otherwise move stops.
        .properties_num = 0,                                             // Number of properties in the array models
        .properties = NULL,                                              // Array of properties in the element.
        .sensors_num = sizeof(mesh_element1_sensors) / sizeof(wiced_bt_mesh_core_config_sensor_t),    // Number of sensors in the array sensors
        .sensors = mesh_element1_sensors,                                // Array of properties in the element.
        .models_num = (sizeof(mesh_element1_models) / sizeof(wiced_bt_mesh_core_config_model_t)),  // Number of models in the array models
        .models = mesh_element1_models,                                  // Array of models located in that element. Model data is defined by structure wiced_bt_mesh_core_config_model_t
    },
    {
        .location = MESH_ELEM_LOC_MAIN,                                  // location description as defined in the GATT Bluetooth Namespace Descriptors section of the Bluetooth SIG Assigned Numbers
        .default_transition_time = MESH_DEFAULT_TRANSITION_TIME_IN_MS,   // Default transition time for models of the element in milliseconds
        .onpowerup_state = WICED_BT_MESH_ON_POWER_UP_STATE_RESTORE,      // Default element behavior on power up
        .default_level = 0,                                              // Default value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .range_min = 1,                                                  // Minimum value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .range_max = 0xffff,                                             // Maximum value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .move_rollover = 0,                                              // If true when level gets to range_max during move operation, it switches to min, otherwise move stops.
        .properties_num = 0,                                             // Number of properties in the array models
        .properties = NULL,                                              // Array of properties in the element.
        .sensors_num = sizeof(mesh_element2_sensors) / sizeof(wiced_bt_mesh_core_config_sensor_t),    // Number of sensors in the array sensors
        .sensors = mesh_element2_sensors,                                // Array of properties in the element.
        .models_num = (sizeof(mesh_element2_models) / sizeof(wiced_bt_mesh_core_config_model_t)),  // Number of models in the array models
        .models = mesh_element2_models,                                  // Array of models located in that element. Model data is defined by structure wiced_bt_mesh_core_config_model_t
    },
};

wiced_bt_mesh_core_config_t  mesh_config =
{
    .company_id         = MESH_COMPANY_ID_CYPRESS,                  // Company identifier assigned by the Bluetooth SIG
    .product_id         = MESH_PID,                                 // Vendor-assigned product identifier
    .vendor_id          = MESH_VID,                                 // Vendor-assigned product version identifier
    .firmware_id        = MESH_FWID,                                // Vendor-assigned firmware version identifier
    .replay_cache_size  = MESH_CACHE_REPLAY_SIZE,                   // Number of replay protection entries, i.e. maximum number of mesh devices that can send application messages to this device.
#if defined(LOW_POWER_NODE) && (LOW_POWER_NODE == 1)
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
        .cache_buf_len         = 300,                                // Length of the buffer for the cache
        .max_lpn_num           = 4                                         // Max number of Low Power Nodes with established friendship. Must be > 0 if Friend feature is supported. 
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
    mesh_app_init,                  // application initialization
    NULL,                           // Default SDK platform button processing
    NULL,                           // GATT connection status
    NULL,                           // attention processing
    mesh_app_notify_period_set,     // notify period set
    NULL,                           // WICED HCI command
    mesh_app_lpn_sleep,             // LPN sleep
    mesh_app_factory_reset,         // factory reset
};

 /******************************************************
 *               Function Definitions
 ******************************************************/
void mesh_app_init(wiced_bool_t is_provisioned)
{
    WICED_BT_TRACE("mesh_app_init\n");
#if 1
    extern uint8_t wiced_bt_mesh_model_trace_enabled;
    wiced_bt_mesh_model_trace_enabled = WICED_TRUE;
#endif
    uint32_t        cur_time = wiced_bt_mesh_core_get_tick_count();
    wiced_result_t  result;
    wiced_bt_mesh_core_config_sensor_t *p_sensor_presence_detected = &mesh_config.elements[0].sensors[0];
    wiced_bt_mesh_core_config_sensor_t *p_sensor_ambient_light = &mesh_config.elements[0].sensors[1];
    wiced_bt_mesh_core_config_sensor_t *p_sensor_temp_sensor = &mesh_config.elements[1].sensors[0];
    wiced_bt_mesh_sensor_config_cadence_t temp_cadence;

    wiced_bt_cfg_settings.device_name = (uint8_t *)"Sensor hub";
    wiced_bt_cfg_settings.gatt_cfg.appearance = APPEARANCE_SENSOR_MOTION;

    mesh_prop_fw_version[0] = 0x30 + (WICED_SDK_MAJOR_VER / 10);
    mesh_prop_fw_version[1] = 0x30 + (WICED_SDK_MAJOR_VER % 10);
    mesh_prop_fw_version[2] = 0x30 + (WICED_SDK_MINOR_VER / 10);
    mesh_prop_fw_version[3] = 0x30 + (WICED_SDK_MINOR_VER % 10);
    mesh_prop_fw_version[4] = 0x30 + (WICED_SDK_REV_NUMBER / 10);
    mesh_prop_fw_version[5] = 0x30 + (WICED_SDK_REV_NUMBER % 10);
    mesh_prop_fw_version[6] = 0x30 + (WICED_SDK_BUILD_NUMBER / 10);
    mesh_prop_fw_version[7] = 0x30 + (WICED_SDK_BUILD_NUMBER % 10);

    // Adv Data is fixed. Spec allows to put URI, Name, Appearance and Tx Power in the Scan Response Data.
    if (!is_provisioned)
    {
        wiced_bt_ble_advert_elem_t  adv_elem[3];
        uint8_t                     buf[2];
        uint8_t                     num_elem = 0;

        adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
        adv_elem[num_elem].len         = (uint16_t)strlen((const char*)wiced_bt_cfg_settings.device_name);
        adv_elem[num_elem].p_data      = wiced_bt_cfg_settings.device_name;
        num_elem++;

        adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_APPEARANCE;
        adv_elem[num_elem].len         = 2;
        buf[0]                         = (uint8_t)wiced_bt_cfg_settings.gatt_cfg.appearance;
        buf[1]                         = (uint8_t)(wiced_bt_cfg_settings.gatt_cfg.appearance >> 8);
        adv_elem[num_elem].p_data      = buf;
        num_elem++;

        wiced_bt_mesh_set_raw_scan_response_data(num_elem, adv_elem);
    }

    if (!is_provisioned)
        return;

    //initilize thermistor
    thermistor_init();
    //initialize ambient light
    max44009_init(&max44009_usr_set, max44009IntProc, NULL);
    //initialize motion sensor
    e93196_init(&e93196_usr_cfg, e93196_int_proc, NULL);

    // initialize the cadence timer.  Need a timer for each element because each sensor model can be
    // configured for different publication period.

    //Element 0 timer
    wiced_init_timer(&mesh_sensor_cadence_timer[0], &mesh_sensor_publish_timer_callback, (TIMER_PARAM_TYPE)p_sensor_presence_detected, WICED_MILLI_SECONDS_TIMER);
    //Element 1 timer
    wiced_init_timer(&mesh_sensor_cadence_timer[1], &mesh_sensor_publish_timer_callback, (TIMER_PARAM_TYPE)p_sensor_temp_sensor, WICED_MILLI_SECONDS_TIMER);

    //presence detected timer
    wiced_init_timer(&mesh_sensor_presence_detected_timer, mesh_sensor_presence_detected_timer_callback, (TIMER_PARAM_TYPE)p_sensor_presence_detected, WICED_SECONDS_TIMER);

    if (wiced_hal_read_nvram(MESH_SENSOR_HUB_PRESENCE_NVRAM_ID, sizeof(wiced_bt_mesh_sensor_config_cadence_t), (uint8_t*)(&temp_cadence), &result) == sizeof(wiced_bt_mesh_sensor_config_cadence_t))
    {
        WICED_BT_TRACE("Sensor hub read presence nvram\n");
        memcpy(&p_sensor_presence_detected->cadence, &temp_cadence, sizeof(wiced_bt_mesh_sensor_config_cadence_t));
    }

    if (wiced_hal_read_nvram(MESH_SENSOR_HUB_LIGHT_NVRAM_ID, sizeof(wiced_bt_mesh_sensor_config_cadence_t), (uint8_t*)(&temp_cadence), &result) == sizeof(wiced_bt_mesh_sensor_config_cadence_t))
    {
        WICED_BT_TRACE("Sensor hub read light nvram\n");
        memcpy(&p_sensor_ambient_light->cadence, &temp_cadence, sizeof(wiced_bt_mesh_sensor_config_cadence_t));
    }

    if (wiced_hal_read_nvram(MESH_SENSOR_HUB_TEMPERATURE_NVRAM_ID, sizeof(wiced_bt_mesh_sensor_config_cadence_t), (uint8_t*)(&temp_cadence), &result) == sizeof(wiced_bt_mesh_sensor_config_cadence_t))
    {
        WICED_BT_TRACE("Sensor hub read temp nvram \n");
        memcpy(&p_sensor_temp_sensor->cadence, &temp_cadence, sizeof(wiced_bt_mesh_sensor_config_cadence_t));
    }

    wiced_bt_mesh_model_sensor_server_init(0, mesh_sensor_server_report_handler, mesh_sensor_server_config_change_handler, is_provisioned);
    wiced_bt_mesh_model_sensor_server_init(1, mesh_sensor_server_report_handler, mesh_sensor_server_config_change_handler, is_provisioned);

    // When we are coming out of HID OFF need to send data
    if (is_provisioned)
    {
        mesh_sensor_current_value[TEMPERATURE_SENSOR_INDEX]  = mesh_sensor_get_temperature_8();
        mesh_sensor_sent_time[TEMPERATURE_SENSOR_INDEX]      = cur_time;

        mesh_sensor_current_value[LIGHT_SENSOR_INDEX]        = mesh_sensor_get_current_ambient_light();
        mesh_sensor_sent_time[LIGHT_SENSOR_INDEX]            = cur_time;

        WICED_BT_TRACE("Pub value:%d time:%d\n", mesh_sensor_current_value[TEMPERATURE_SENSOR_INDEX], mesh_sensor_sent_time[TEMPERATURE_SENSOR_INDEX]);
        wiced_bt_mesh_model_sensor_server_data(1, WICED_BT_MESH_PROPERTY_PRESENT_AMBIENT_TEMPERATURE, NULL);
        wiced_bt_mesh_model_sensor_server_data(0, WICED_BT_MESH_PROPERTY_PRESENT_AMBIENT_LIGHT_LEVEL, NULL);
        wiced_bt_mesh_model_sensor_server_data(0, WICED_BT_MESH_PROPERTY_PRESENCE_DETECTED, NULL);
    }
}

/*
 * New publication period is set. If it is for the sensor model, this application should take care of it.
 * The period may need to be adjusted based on the divisor.
 */
wiced_bool_t mesh_app_notify_period_set(uint8_t element_idx, uint16_t company_id, uint16_t model_id, uint32_t period)
{

    if (((element_idx != 0) && (element_idx != 1)) || (company_id != MESH_COMPANY_ID_BT_SIG) || (model_id != WICED_BT_MESH_CORE_MODEL_ID_SENSOR_SRV))
    {
        return WICED_FALSE;
    }
    mesh_sensor_publish_period[element_idx] = period;
    WICED_BT_TRACE("Sensor data send period:%dms element_idx:%d\n", mesh_sensor_publish_period[element_idx], element_idx);

    mesh_sensor_server_restart_timer(&(mesh_config.elements[element_idx].sensors[0]));

    return WICED_TRUE;
}

/*
 * Application is notified that core enters LPN mode.
 */
void mesh_app_lpn_sleep(uint32_t timeout_ms)
{
#if defined(LOW_POWER_NODE) && (LOW_POWER_NODE == 1)
    if (wiced_sleep_enter_hid_off(timeout_ms, 0, 0) != WICED_SUCCESS)
    {
        WICED_BT_TRACE("Entering HID-Off failed\n\r");
    }
#endif
}

/*
 * Application is notified that factory reset is executed.
 */
void mesh_app_factory_reset(void)
{
    int i;
    for(i = 0; i < NUM_SENSORS; i++)
    {
        wiced_hal_delete_nvram(MESH_SENSOR_HUB_CADENCE_NVRAM_ID + i, NULL);
    }
}

/*
 * Start periodic timer depending on the publication period, fast cadence divisor and minimum interval
 */
void mesh_sensor_server_restart_timer(wiced_bt_mesh_core_config_sensor_t* p_sensor)
{
    int element_index = 0;
    int index;
    uint32_t timeout = 0;

    switch(p_sensor->property_id)
    {
        case WICED_BT_MESH_PROPERTY_PRESENT_AMBIENT_LIGHT_LEVEL:
            element_index = MESH_SENSOR_HUB_LIGHT_ELEMENT_INDEX;
            break;
        case WICED_BT_MESH_PROPERTY_PRESENT_AMBIENT_TEMPERATURE:
            element_index = MESH_SENSOR_HUB_TEMPERATURE_ELEMENT_INDEX;
            break;
        case WICED_BT_MESH_PROPERTY_PRESENCE_DETECTED:
            element_index = MESH_SENSOR_HUB_PRESENCE_ELEMENT_INDEX;
            break;
    }

    for (index = 0; index < mesh_config.elements[element_index].sensors_num; index++)
    {
        wiced_bt_mesh_core_config_sensor_t* p_temp_sensor = &mesh_config.elements[element_index].sensors[index];
        // If there are no specific cadence settings, publish every publish period.
        timeout = mesh_sensor_publish_period[element_index];

        wiced_stop_timer(&mesh_sensor_cadence_timer[element_index]);
        if (mesh_sensor_publish_period[element_index] == 0)
        {
            // The thermistor is not interrupt driven.  If client configured sensor to send notification when
            // the value changes, we will need to check periodically if the condition has been satisfied.
            // The cadence.min_interval can be used because we do not need to send data more often than that.
            if ((p_temp_sensor->cadence.min_interval != 0) &&
                ((p_temp_sensor->cadence.trigger_delta_up != 0) || (p_temp_sensor->cadence.trigger_delta_down != 0)))
            {
                timeout = p_temp_sensor->cadence.min_interval;
            }
            else
            {
                WICED_BT_TRACE("sensor restart timer period:%d\n", mesh_sensor_publish_period);
                return;
            }
        }
        else
        {
            // If fast cadence period divisor is set, we need to check temperature more
            // often than publication period.  Publish if measurement is in specified range
            if (p_temp_sensor->cadence.fast_cadence_period_divisor > 1)
            {
                mesh_sensor_fast_publish_period[element_index] = mesh_sensor_publish_period[element_index] / p_temp_sensor->cadence.fast_cadence_period_divisor;
                timeout = mesh_sensor_fast_publish_period[element_index];
            }
            else
            {
                mesh_sensor_fast_publish_period[element_index] = 0;
            }
            // The thermistor is not interrupt driven.  If client configured sensor to send notification when
            // the value changes, we may need to check value more often not to miss the trigger.
            // The cadence.min_interval can be used because we do not need to send data more often than that.
            if ((p_temp_sensor->cadence.min_interval != 0) && (p_temp_sensor->cadence.min_interval < timeout) &&
                ((p_temp_sensor->cadence.trigger_delta_up != 0) || (p_temp_sensor->cadence.trigger_delta_down != 0)))
            {
                timeout = p_temp_sensor->cadence.min_interval;
            }
        }
    }

    WICED_BT_TRACE("sensor restart timer:%d index:%d\n", timeout, element_index);
    wiced_start_timer(&mesh_sensor_cadence_timer[element_index], timeout);
}

/*
 * Helper function to read temperature from the thermistor and convert temperature in celsius
 * to Temperature 8 format.  Unit is degree Celsius with a resolution of 0.5. Minimum: -64.0 Maximum: 63.5.
 */
int8_t mesh_sensor_get_temperature_8(void)
{
    int16_t temp_celsius_100 = thermistor_read();

    if (temp_celsius_100 < -6400)
    {
        return 0x80;
    }
    else if (temp_celsius_100 >= 6350)
    {
        return 0x7F;
    }
    else
    {
        return (int8_t)(temp_celsius_100 / 50);
    }
}

void e93196_int_proc(void* data, uint8_t port_pin)
{
    WICED_BT_TRACE("presence detected TRUE\n");
    e93196_int_clean(port_pin);

    // We disable interrupts for MESH_PRESENCE_DETECTED_BLIND_TIME.  If interrupt does not happen within
    // MESH_PRESENCE_DETECTED_BLIND_TIME + 1, we assume that there is no presence anymore
    wiced_start_timer(&mesh_sensor_presence_detected_timer, 2 * MESH_PRESENCE_DETECTED_BLIND_TIME);

    if (!presence_detected)
    {
        presence_detected = WICED_TRUE;
        mesh_sensor_publish_timer_callback((TIMER_PARAM_TYPE)&mesh_config.elements[MESH_SENSOR_HUB_PRESENCE_ELEMENT_INDEX].sensors[PRESENCE_SENSOR_INDEX]);
    }
}

int32_t mesh_sensor_get_current_value(void)
{
    return presence_detected;
     e93196_read_reg_t read_data;
     e93196_reg_read (&read_data);
     return read_data.pir_out;
}

void max44009IntProc(void* data, uint8_t port_pin) {
    WICED_BT_TRACE( "93196 Internal \n" );
    max44009IntClean();
}

/*
 * Helper function to read ambient light from the sensor and convert light to illuminance format
 * Unit is lux with a resolution of 0.01 Minimum: 0 Maximum: 167772.14.
 * TODO current app considers it as 0.1 change it to 0.01
 */
uint32_t mesh_sensor_get_current_ambient_light(void) {
    uint32_t luminance = 0;
    luminance = max44009_read_ambient_light();
    WICED_BT_TRACE("mesh_sensor_get_current_ambient_light : lumiance:%d.%d\r\n",luminance/100, luminance%100);
    return luminance;
}

/*
 * Process the configuration changes set by the Sensor Client.
 */
//void mesh_sensor_server_config_change_handler(uint8_t element_idx, uint16_t event, void *p_data)
void mesh_sensor_server_config_change_handler(uint8_t element_idx, uint16_t event, uint16_t property_id, uint16_t setting_property_id)
{
#if defined HCI_CONTROL
    wiced_bt_mesh_hci_event_t *p_hci_event;
#endif
    WICED_BT_TRACE("mesh_sensor_server_config_change_handler msg: %d\n", event);

    switch (event)
    {

    case WICED_BT_MESH_SENSOR_CADENCE_SET:
#if defined HCI_CONTROL
//        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
//            mesh_sensor_hci_event_send_cadence_set(p_hci_event, (wiced_bt_mesh_sensor_cadence_set_data_t *)p_data);
#endif
        mesh_sensor_server_process_cadence_changed(element_idx, property_id);
        break;

    case WICED_BT_MESH_SENSOR_SETTING_SET:
#if defined HCI_CONTROL
//        if ((p_hci_event = wiced_bt_mesh_create_hci_event(p_event)) != NULL)
//            mesh_sensor_hci_event_send_setting_set(p_hci_event, (wiced_bt_mesh_sensor_setting_set_data_t *)p_data);
#endif
        mesh_sensor_server_process_setting_changed(element_idx, property_id, setting_property_id);
        break;
    }
}

void mesh_sensor_presence_detected_timer_callback(TIMER_PARAM_TYPE arg)
{
    WICED_BT_TRACE("presence detected FALSE\n");
    presence_detected = WICED_FALSE;
    mesh_sensor_publish_timer_callback((TIMER_PARAM_TYPE)&mesh_config.elements[MESH_SENSOR_HUB_PRESENCE_ELEMENT_INDEX].sensors[PRESENCE_SENSOR_INDEX]);
}

/*
 * Process get request from Sensor Client and respond with sensor data
 */
void mesh_sensor_server_report_handler(uint16_t event, uint8_t element_idx, void *p_get, void *p_ref_data)
{
    wiced_bt_mesh_sensor_get_t *p_sensor_get = (wiced_bt_mesh_sensor_get_t *)p_get;
    WICED_BT_TRACE("mesh_sensor_server_report_handler msg: %d propertyid: %04x\n", event, p_sensor_get->property_id);

    switch (event)
    {
    case WICED_BT_MESH_SENSOR_GET:
        {
            // measure the temperature and update it to mesh_config
            switch(p_sensor_get->property_id)
            {
                case WICED_BT_MESH_PROPERTY_PRESENCE_DETECTED:
                    mesh_sensor_current_value[PRESENCE_SENSOR_INDEX] = mesh_sensor_get_current_value();
                    mesh_sensor_sent_value[PRESENCE_SENSOR_INDEX]  = mesh_sensor_current_value[PRESENCE_SENSOR_INDEX];
                    break;
                case WICED_BT_MESH_PROPERTY_PRESENT_AMBIENT_LIGHT_LEVEL:
                    mesh_sensor_current_value[LIGHT_SENSOR_INDEX] = mesh_sensor_get_current_ambient_light();
                    break;
                case WICED_BT_MESH_PROPERTY_PRESENT_AMBIENT_TEMPERATURE:
                    mesh_sensor_current_value[TEMPERATURE_SENSOR_INDEX] = mesh_sensor_get_temperature_8();
            }
        }

        // tell mesh models library that data is ready to be shipped out, the library will get data from mesh_config
        wiced_bt_mesh_model_sensor_server_data(element_idx, p_sensor_get->property_id, p_ref_data);
        break;

    case WICED_BT_MESH_SENSOR_COLUMN_GET:
        break;

    case WICED_BT_MESH_SENSOR_SERIES_GET:
        break;

    default:
        WICED_BT_TRACE("unknown\n");
        break;
    }
}

/*
 * Process cadence change
 */
void mesh_sensor_server_process_cadence_changed(uint8_t element_idx, uint16_t property_id)
{
    wiced_bt_mesh_core_config_sensor_t *p_sensor = NULL;
    uint8_t written_byte = 0;
    wiced_result_t status;
    int i, index = 0;

    for (i = 0; i < mesh_config.elements[element_idx].sensors_num; i++)
    {
        if (mesh_config.elements[element_idx].sensors[i].property_id == property_id)
        {
            p_sensor = &mesh_config.elements[element_idx].sensors[i];
            break;
        }
    }

    //no such sensor present
    if (p_sensor == NULL)
        return;

    WICED_BT_TRACE("cadence changed property id:%04x\n", property_id);
    WICED_BT_TRACE("Fast cadence period divisor:%d\n", p_sensor->cadence.fast_cadence_period_divisor);
    WICED_BT_TRACE("Is trigger type percent:%d\n", p_sensor->cadence.trigger_type_percentage);
    WICED_BT_TRACE("Trigger delta up:%d\n", p_sensor->cadence.trigger_delta_up);
    WICED_BT_TRACE("Trigger delta down:%d\n", p_sensor->cadence.trigger_delta_down);
    WICED_BT_TRACE("Min Interval:%d\n", p_sensor->cadence.min_interval);
    WICED_BT_TRACE("Fast cadence low:%d\n", p_sensor->cadence.fast_cadence_low);
    WICED_BT_TRACE("Fast cadence high:%d\n", p_sensor->cadence.fast_cadence_high);


    switch(p_sensor->property_id)
    {
        case WICED_BT_MESH_PROPERTY_PRESENT_AMBIENT_LIGHT_LEVEL:
            index = LIGHT_SENSOR_INDEX;
            break;
        case WICED_BT_MESH_PROPERTY_PRESENT_AMBIENT_TEMPERATURE:
            index = TEMPERATURE_SENSOR_INDEX;
            break;
        case WICED_BT_MESH_PROPERTY_PRESENCE_DETECTED:
            index = PRESENCE_SENSOR_INDEX;
            break;
    }
    /* save cadence to NVRAM */
    //written_byte = wiced_hal_write_nvram(MESH_SENSOR_HUB_CADENCE_NVRAM_ID, sizeof(wiced_bt_mesh_sensor_config_cadence_t), (uint8_t*)(&p_sensor->cadence), &status);
    written_byte = wiced_hal_write_nvram(MESH_SENSOR_HUB_CADENCE_NVRAM_ID + index, sizeof(wiced_bt_mesh_sensor_config_cadence_t), (uint8_t*)(&p_sensor->cadence), &status);
    WICED_BT_TRACE("NVRAM write: %d\n", written_byte);

    mesh_sensor_server_restart_timer(p_sensor);
}

/*
 * Publication timer callback.  Need to send data if publish period expired, or
 * if value has changed more than specified in the triggers, or if value is in range
 * of fast cadence values and fast cadence interval expired.
 */
void mesh_sensor_publish_timer_callback(TIMER_PARAM_TYPE arg)
{

    wiced_bt_mesh_event_t *p_event;
    wiced_bt_mesh_core_config_sensor_t *p_sensor = (wiced_bt_mesh_core_config_sensor_t *)arg;
    wiced_bool_t pub_needed = WICED_FALSE;
    uint32_t cur_time = wiced_bt_mesh_core_get_tick_count();
    int element_index = 0;
    int index = 0;
    int property_id = p_sensor->property_id;

    mesh_sensor_current_value[LIGHT_SENSOR_INDEX] = mesh_sensor_get_current_ambient_light();
    mesh_sensor_current_value[TEMPERATURE_SENSOR_INDEX] = mesh_sensor_get_temperature_8();
    mesh_sensor_current_value[PRESENCE_SENSOR_INDEX] = mesh_sensor_get_current_value();

    switch(p_sensor->property_id)
    {
        case WICED_BT_MESH_PROPERTY_PRESENT_AMBIENT_TEMPERATURE:
            element_index = MESH_SENSOR_HUB_TEMPERATURE_ELEMENT_INDEX;
            break;
        case WICED_BT_MESH_PROPERTY_PRESENCE_DETECTED:
            element_index = MESH_SENSOR_HUB_PRESENCE_ELEMENT_INDEX;
            break;
    }


    for (index = 0; index < mesh_config.elements[element_index].sensors_num; index++)
    {
        if (element_index == 1)
            index = 1;

        wiced_bt_mesh_core_config_sensor_t *p_temp_sensor = &mesh_config.elements[element_index].sensors[index];

        WICED_BT_TRACE("mesh_sensor_publish_timer_callback sensor index : %d\n", element_index + index);

        if ((p_temp_sensor->cadence.min_interval != 0) && ((cur_time - mesh_sensor_sent_time[element_index]) < p_temp_sensor->cadence.min_interval))
        {
            WICED_BT_TRACE("time since last pub:%d interval:%d\n", cur_time - mesh_sensor_sent_time[element_index], p_temp_sensor->cadence.min_interval);
            wiced_start_timer(&mesh_sensor_cadence_timer[element_index], p_temp_sensor->cadence.min_interval - cur_time + mesh_sensor_sent_time[element_index]);
        }
        else
        {
            // check if publication timer expired
            if ((mesh_sensor_publish_period != 0) && (cur_time - mesh_sensor_sent_time[element_index] >= mesh_sensor_publish_period[element_index]))
            {
                WICED_BT_TRACE("Pub needed period\n");
                pub_needed = WICED_TRUE;
                property_id = 0;
            }
            // still need to send if publication timer has not expired, but triggers are configured, and value
            // changed too much
            if (!pub_needed && ((p_temp_sensor->cadence.trigger_delta_up != 0) || (p_sensor->cadence.trigger_delta_down != 0)))
            {
                if (!p_temp_sensor->cadence.trigger_type_percentage)
                {
                    WICED_BT_TRACE("Native cur value:%d sent:%d delta:%d/%d\n",
                            mesh_sensor_current_value[element_index + index], mesh_sensor_sent_value[element_index + index], p_temp_sensor->cadence.trigger_delta_up, p_sensor->cadence.trigger_delta_down);

                    if (((p_temp_sensor->cadence.trigger_delta_up != 0)   && (mesh_sensor_current_value[element_index + index] >= (mesh_sensor_sent_value[element_index + index] + p_temp_sensor->cadence.trigger_delta_up))) ||
                        ((p_temp_sensor->cadence.trigger_delta_down != 0) && (mesh_sensor_current_value[element_index + index] <= (mesh_sensor_sent_value[element_index + index] - p_temp_sensor->cadence.trigger_delta_down))))
                    {
                        WICED_BT_TRACE("Pub needed native value\n");
                        pub_needed = WICED_TRUE;
                    }
                }
                else
                {
                    // need to calculate percentage of the increase or decrease.  The deltas are in 0.01%.
                    if ((p_temp_sensor->cadence.trigger_delta_up != 0) && (mesh_sensor_current_value[element_index + index] > mesh_sensor_sent_value[element_index + index]))
                    {
                        WICED_BT_TRACE("Delta up:%d\n", ((uint32_t)(mesh_sensor_current_value[element_index + index] - mesh_sensor_sent_value[element_index + index]) * 10000 / mesh_sensor_current_value[element_index + index]));
                        if (((uint32_t)(mesh_sensor_current_value[element_index + index] - mesh_sensor_sent_value[element_index + index]) * 10000 / mesh_sensor_current_value[element_index + index]) > p_temp_sensor->cadence.trigger_delta_up)
                        {
                            WICED_BT_TRACE("Pub needed percent delta up:%d\n", ((mesh_sensor_current_value[element_index + index] - mesh_sensor_sent_value[element_index + index]) * 10000 / mesh_sensor_current_value[element_index + index]));
                            pub_needed = WICED_TRUE;
                        }
                    }
                    else if ((p_temp_sensor->cadence.trigger_delta_down != 0) && (mesh_sensor_current_value[element_index + index] < mesh_sensor_sent_time[element_index + index]))
                    {
                        WICED_BT_TRACE("Delta down:%d\n", ((uint32_t)(mesh_sensor_sent_value[element_index + index] - mesh_sensor_current_value[element_index + index]) * 10000 / mesh_sensor_current_value[element_index + index]));
                        if (((uint32_t)(mesh_sensor_sent_value[element_index + index] - mesh_sensor_current_value[element_index + index]) * 10000 / mesh_sensor_current_value[element_index + index]) > p_temp_sensor->cadence.trigger_delta_down)
                        {
                            WICED_BT_TRACE("Pub needed percent delta down:%d\n", ((mesh_sensor_sent_value[element_index + index] - mesh_sensor_current_value[element_index + index]) * 10000 / mesh_sensor_current_value[element_index + index]));
                            pub_needed = WICED_TRUE;
                        }
                    }
                }
            }
            // may still need to send if fast publication is configured
            if (!pub_needed && (mesh_sensor_fast_publish_period != 0))
            {
                // check if fast publish period expired
                if (cur_time - mesh_sensor_sent_time[element_index] >= mesh_sensor_fast_publish_period[element_index])
                {
                    // if cadence high is more than cadence low, to publish, the value should be in range
                    if (p_temp_sensor->cadence.fast_cadence_high >= p_temp_sensor->cadence.fast_cadence_low)
                    {
                        if ((mesh_sensor_current_value[element_index + index] >= p_temp_sensor->cadence.fast_cadence_low) &&
                            (mesh_sensor_current_value[element_index + index] <= p_temp_sensor->cadence.fast_cadence_high))
                        {
                            WICED_BT_TRACE("Pub needed in range\n");
                            pub_needed = WICED_TRUE;
                        }
                    }
                    else if (p_temp_sensor->cadence.fast_cadence_high < p_temp_sensor->cadence.fast_cadence_low)
                    {
                        if ((mesh_sensor_current_value[element_index + index] > p_temp_sensor->cadence.fast_cadence_low) ||
                            (mesh_sensor_current_value[element_index + index] < p_temp_sensor->cadence.fast_cadence_high))
                        {
                            WICED_BT_TRACE("Pub needed out of range\n");
                            pub_needed = WICED_TRUE;
                        }
                    }
                }
            }
            /*
            if (!pub_needed)
            {
               if (((p_sensor->cadence.trigger_delta_up == 0) && (mesh_sensor_current_value[TEMPERATURE_SENSOR_INDEX] > mesh_sensor_sent_time[TEMPERATURE_SENSOR_INDEX])) ||
                   ((p_sensor->cadence.trigger_delta_down == 0) && (mesh_sensor_current_value[TEMPERATURE_SENSOR_INDEX] < mesh_sensor_sent_time[TEMPERATURE_SENSOR_INDEX])))
                {
                   WICED_BT_TRACE("Pub needed new value no deltas\n");
                   pub_needed = WICED_TRUE;
                }
            }
            */
            if (pub_needed)
            {
                mesh_sensor_sent_value[element_index + index]  = mesh_sensor_current_value[element_index + index];
                mesh_sensor_sent_time[element_index]   = cur_time;

                WICED_BT_TRACE("mesh_sensor_publish_timer_callback sensor index publishing...: %d\n", element_index + index);

                WICED_BT_TRACE("Pub value:%d time:%d property_id:%d\n", mesh_sensor_sent_value[element_index + index], mesh_sensor_sent_time[element_index], property_id);
                wiced_bt_mesh_model_sensor_server_data(element_index, 0, NULL);

            }
            mesh_sensor_server_restart_timer(p_sensor);
        }
    }
}

/*
 * Process setting change
 */
void mesh_sensor_server_process_setting_changed(uint8_t element_idx, uint16_t property_id, uint16_t setting_property_id)
{
    WICED_BT_TRACE("settings changed  property id of sensor = %x , sensor prop id = %x \n", property_id, setting_property_id);
}


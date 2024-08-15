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
* Apple Media Service (AMS) snippet application
*
* The AMS snippet application shows how to initialize and use WICED BT AMS
* library.  Device works in the peripheral mode accepting connection from
* an iOS phone.
*
* When connected is established the application performs GATT discovery of the
* connected device.  If the AMS service is found on the device, the application
* initializes the AMS library and passes discovery events to the library for
* processing.  Similarly notifications received from the phone are passed to
* the library which implements the protocol and sends back functional
* information.
*
* The 'ams' application links with "ams_lib" library. See library source code under
* <WICED Platform>\libraries\ams_lib folder.
* Not all media attributes are enabled by default. To enable these attributes,
* enable the compile flags in \libraries\ams_lib\ams_client_lib.c
* #define AMS_SUPPORT_PLAYER_NAME 1     // Player name
* #define AMS_SUPPORT_TRACK_POSITION 1  // Track position
* #define AMS_SUPPORT_PLAYLIST_INFO 1   // Track index and track count
*
* There are 2 ways to test this application.  One is to use the application button
* and monitor traces (define TEST_USE_BUTTON below).  Another is to control the app
* from a ClientControl application (define TEST_HCI_CONTROL below).
*
* Features demonstrated
*  - Initialize and use WICED BT AMS library
*
* To demonstrate the app, work through the following steps.
* 1. Plug the WICED eval board into your computer
* 2. Build and download the application (to the WICED board)
* 3. Start tracing to monitor the activity (see quick start guide for details)
* 4. Pair with a client (iOS device)
* 5. Play music on the phone and verify that notifications are received in the app
* 6. Push button on the tag to toggle play/stop
*/

#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_gatt_util.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_uuid.h"
#if ( defined(CYW20706A2) || defined(CYW20719B1) || defined(CYW20719B0) || defined(CYW20721B1) || defined(CYW20735B0) || defined(CYW43012C0) )
#include "wiced_bt_app_common.h"
#endif
#include "wiced_bt_trace.h"
#include "wiced_hal_nvram.h"
#include "wiced_result.h"
#include "hci_control_api.h"
#include "wiced_platform.h"
#include "gatt_server_db.h"
#include "wiced_bt_ams.h"
#include "wiced_bt_stack.h"
#include "wiced_memory.h"
#include "string.h"
#include "wiced_transport.h"


#define TEST_USE_BUTTON     1
#define TEST_HCI_CONTROL    1

#ifdef  TEST_HCI_CONTROL
#include "hci_control_api.h"
#include "wiced_bt_avrc_defs.h"
#endif

#if defined TEST_HCI_CONTROL || defined WICED_BT_TRACE_ENABLE
#include "wiced_transport.h"
#endif

// Following flags can be change to 1 or 0 to enable or
// disable additional features
#define AMS_ADDITIONAL_TRACE            1   // Set to one to print additional traces to the debug output
#define AMS_SUPPORT_PLAYLIST_INFO       1   // Set to 1 to register to receive number of tracks/track number
#define AMS_SUPPORT_PLAYER_NAME         1   // Set to 1 to register to receive player name
#define AMS_SUPPORT_TRACK_POSITION      1   // Set to 1 to register to receive track length/track position

#define HCI_TRACE_OVER_TRANSPORT        1

#ifdef WICED_BT_TRACE_ENABLE
#define AMS_ENTITY_ID_MAX               3
static char *EntityId[] =
{
    "Player",
    "Queue",
    "Track",
    "Unknown"
};

#define AMS_PLAYER_ATTRIBUTE_ID_MAX     3
static char *PlayerAttributeId[] =
{
    "PlayerAttributeIDName",
    "PlayerAttributeIDPlaybackInfo",
    "PlayerAttributeIDVolume",
    "Unknown"
};

#define AMS_QUEUE_ATTRIBUTE_ID_MAX      4
static char *QueueAttributeId[] =
{
    "QueueAttributeIDIndex",
    "QueueAttributeIDCount",
    "QueueAttributeIDShuffleMode",
    "QueueAttributeIDRepeatMode",
    "Unknown"
};

#define AMS_TRACK_ATTRIBUTE_ID_MAX      4
static char *TrackAttributeId[] =
{
    "TrackAttributeIDArtist",
    "TrackAttributeIDAlbum",
    "TrackAttributeIDTitle",
    "TrackAttributeIDDuration",
    "Unknown"
};
#endif

#ifdef  TEST_HCI_CONTROL
// Following table translates from AMS play status to AVRC status
uint8_t ams_client_to_hci_playback_status[] = {AVRC_PLAYSTATE_PAUSED, AVRC_PLAYSTATE_PLAYING, AVRC_PLAYSTATE_REV_SEEK, AVRC_PLAYSTATE_FWD_SEEK};

// Following table translates from AMS shuffle mode to AVRC shuffle mode
uint8_t ams_client_to_hci_shuffle_mode[]    = {AVRC_PLAYER_VAL_OFF, AVRC_PLAYER_VAL_ON, AVRC_PLAYER_VAL_ALL_SHUFFLE};

// Following table translates from AMS repeat mode to AVRC repeat mode
uint8_t ams_client_to_hci_repeat_mode[]     = {AVRC_PLAYER_VAL_OFF, AVRC_PLAYER_VAL_ON, AVRC_PLAYER_VAL_ALL_REPEAT};
#endif

extern const wiced_bt_cfg_settings_t wiced_app_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_app_cfg_buf_pools[];

/******************************************************
 *                     Structures
 ******************************************************/

/******************************************************
 *               Function Prototypes
 ******************************************************/
#ifdef TEST_HCI_CONTROL
static void                   hci_control_transport_status( wiced_transport_type_t type );
static void                   hci_control_send_device_started_evt( void );
#endif
static wiced_result_t         ams_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
static wiced_bt_gatt_status_t ams_gatts_callback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data);
static void                   ams_connection_up(wiced_bt_gatt_connection_status_t *p_conn_status);
static void                   ams_connection_down(wiced_bt_gatt_connection_status_t *p_conn_status);
static void                   ams_process_pairing_complete(wiced_bt_dev_pairing_cplt_t *p_pairing_complete);
static void                   ams_process_encryption_changed(wiced_bt_dev_encryption_status_t *p_encryption_changed);
static wiced_bt_gatt_status_t ams_gatt_operation_complete(wiced_bt_gatt_operation_complete_t *p_data);
static wiced_bt_gatt_status_t ams_gatt_discovery_result(wiced_bt_gatt_discovery_result_t *p_data);
static wiced_bt_gatt_status_t ams_gatt_discovery_complete(wiced_bt_gatt_discovery_complete_t *p_data);
static wiced_bt_gatt_status_t ams_gatts_req_callback(wiced_bt_gatt_attribute_request_t *p_data);
static wiced_bt_gatt_status_t ams_gatts_req_read_handler(uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data);
static wiced_bt_gatt_status_t ams_gatts_req_write_handler(uint16_t conn_id, wiced_bt_gatt_write_t * p_data);
static wiced_bt_gatt_status_t ams_gatts_req_write_exec_handler(uint16_t conn_id, wiced_bt_gatt_exec_flag_t exec_falg);
static wiced_bt_gatt_status_t ams_gatts_req_mtu_handler(uint16_t conn_id, uint16_t mtu);
static wiced_bt_gatt_status_t ams_gatts_req_conf_handler(uint16_t conn_id, uint16_t handle);
static void                   ams_load_keys_to_addr_resolution_db(void);
static wiced_bool_t           ams_save_link_keys(wiced_bt_device_link_keys_t *p_keys);
static wiced_bool_t           ams_read_link_keys(wiced_bt_device_link_keys_t *p_keys);
static wiced_bool_t           ams_is_bonded(BD_ADDR bd_addr);
static void                   ams_set_advertisement_data(void);
static void                   ams_discovery_complete_callback(uint16_t conn_id, wiced_bool_t result);
static void                   ams_start_complete_callback(uint16_t conn_id, wiced_bool_t result);
static void                   ams_stop_complete_callback(uint16_t conn_id, wiced_bool_t result);
static void                   ams_notification_callback(uint16_t conn_id, uint8_t entity_id, uint8_t attrib_id, uint8_t flags, uint8_t *attribute, uint8_t attribute_len);
static void                   ams_process_write_rsp(wiced_bt_gatt_operation_complete_t *p_data);
static void                   ams_notification_handler(wiced_bt_gatt_operation_complete_t *p_data);

#if TEST_USE_BUTTON
#include "wiced_hal_gpio.h"

#define BUTTON_GPIO           WICED_P30
static void                   ams_interrupt_handler(void *data, uint8_t port_pin);
#endif

#ifdef TEST_HCI_CONTROL
static void                   ams_client_process_playback_info(uint8_t *p_info, int len);
static void                   ams_client_process_volume_info(uint8_t *p_info, int len);
static void                   ams_client_process_queue_index(uint8_t *p_info, int len);
static void                   ams_client_process_shuffle_mode(uint8_t *p_info, int len);
static int                    ams_client_process_get_track_duration_len(uint8_t *p_info, int len);
static wiced_result_t         ams_client_send_track_info(uint8_t info_id, uint8_t *attribute, uint8_t attribute_len);
static void                   ams_client_process_repeat_mode(uint8_t *p_info, int len);
static wiced_result_t         ams_client_send_player_name_change(uint8_t *p_name, uint8_t len);
static wiced_result_t         ams_client_send_play_status_change(uint8_t playback_status);
static wiced_result_t         ams_client_send_play_position_change(uint32_t elapsed_time);
static wiced_result_t         ams_client_send_volume_level_change(uint8_t volume_level);
static wiced_result_t         ams_client_send_setting_change(uint8_t setting_id, uint8_t mode);
#endif

/******************************************************
 *               Variables Definitions
 ******************************************************/

#pragma pack(1)
// host information for NVRAM
typedef PACKED struct
{
    // BD address of the bonded host
    BD_ADDR  bdaddr;
}  HOSTINFO;

typedef struct
{
    uint16_t                    conn_id;

#define AMS_DISCOVERY_STATE_SERVICE 0
#define AMS_DISCOVERY_STATE_AMS     1
#define AMS_DISCOVERY_STATE_DONE    2
    uint8_t                     discovery_state;

    uint8_t                     started;

    // Current value of the client configuration descriptor for characteristic 'Report'
    uint16_t                    ams_s_handle;
    uint16_t                    ams_e_handle;

    BD_ADDR                     remote_addr;   //address of currently connected client
    wiced_bt_ble_address_type_t addr_type;
} ams_app_state_t;

#pragma pack()

// NVRAM save area
HOSTINFO ams_hostinfo;

ams_app_state_t ams_app_state;

// Registration structure to be passed to the library
wiced_bt_ams_reg_t ams_client_reg =
{
    .p_discovery_complete_callback = ams_discovery_complete_callback,
    .p_start_complete_callback	   = ams_start_complete_callback,
    .p_stop_complete_callback      = ams_stop_complete_callback,
};

#if defined WICED_BT_TRACE_ENABLE || defined HCI_TRACE_OVER_TRANSPORT || defined TEST_HCI_CONTROL

#define TRANS_UART_BUFFER_SIZE          1024
#define AMS_TRANS_MAX_BUFFERS           2

#ifdef TEST_HCI_CONTROL
static uint32_t  ams_proc_rx_hci_cmd(uint8_t *p_data, uint32_t length);
#endif

const wiced_transport_cfg_t transport_cfg =
{
    .type = WICED_TRANSPORT_UART,
    .cfg =
    {
        .uart_cfg =
        {
            .mode = WICED_TRANSPORT_UART_HCI_MODE,
            .baud_rate =  HCI_UART_DEFAULT_BAUD
        },
    },
    .rx_buff_pool_cfg =
    {
        .buffer_size  = TRANS_UART_BUFFER_SIZE,
        .buffer_count = 1
    },
#ifdef TEST_HCI_CONTROL
    .p_status_handler = hci_control_transport_status,
    .p_data_handler = ams_proc_rx_hci_cmd,
#else
    .p_status_handler = NULL,
    .p_data_handler = NULL,
#endif
    .p_tx_complete_cback = NULL
};

wiced_transport_buffer_pool_t*  host_trans_pool;
static void ams_trace_callback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data);
#endif

/******************************************************
 *               Function Definitions
 ******************************************************/

/*
 * Entry point to the application. Set device configuration and start BT
 * stack initialization.  The actual application initialization will happen
 * when stack reports that BT device is ready
 */
#ifndef CYW20735B0
APPLICATION_START()
#else
void application_start( void )
#endif
{
    wiced_result_t result;

#if defined WICED_BT_TRACE_ENABLE || defined HCI_TRACE_OVER_TRANSPORT
    wiced_transport_init(&transport_cfg);

    // create special pool for sending data to the MCU
    host_trans_pool = wiced_transport_create_buffer_pool(TRANS_UART_BUFFER_SIZE, AMS_TRANS_MAX_BUFFERS);

    // Set the debug uart as WICED_ROUTE_DEBUG_NONE to get rid of prints
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_NONE);

    // Set to PUART to see traces on peripheral uart(puart)
    // wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
    // wiced_hal_puart_select_uart_pads( WICED_PUART_RXD, WICED_PUART_TXD, 0, 0);

    // Set to HCI to see traces on HCI uart - default if no call to wiced_set_debug_uart()
    // wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_HCI_UART );

    // Use WICED_ROUTE_DEBUG_TO_WICED_UART to send formatted debug strings over the WICED
    // HCI debug interface to be parsed by ClientControl/BtSpy.
    // Note: WICED HCI must be configured to use this - see wiced_trasnport_init(), must 
    // be called with wiced_transport_cfg_t.wiced_tranport_data_handler_t callback present
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#endif
    WICED_BT_TRACE("AMS APP START\n");

    wiced_bt_ams_client_initialize (&ams_client_reg);

    memset(&ams_hostinfo, 0, sizeof(ams_hostinfo));
    memset(&ams_app_state, 0, sizeof(ams_app_state));

    /* Initialize Stack and Register Management Callback */
    wiced_bt_stack_init(ams_management_callback, &wiced_app_cfg_settings, wiced_app_cfg_buf_pools);
}

void ams_control_handle_get_version()
{
    uint8_t   tx_buf[20];
    uint8_t   cmd = 0;
    uint32_t  chip = CHIP;

    tx_buf[cmd++] = WICED_SDK_MAJOR_VER;
    tx_buf[cmd++] = WICED_SDK_MINOR_VER;
    tx_buf[cmd++] = WICED_SDK_REV_NUMBER;
    tx_buf[cmd++] = WICED_SDK_BUILD_NUMBER & 0xFF;
    tx_buf[cmd++] = (WICED_SDK_BUILD_NUMBER>>8) & 0xFF;
    tx_buf[cmd++] = chip & 0xFF;
    tx_buf[cmd++] = (chip>>8) & 0xFF;
    tx_buf[cmd++] = (chip>>24) & 0xFF;
    tx_buf[cmd++] = POWER_CLASS;

    /* Send MCU app the supported features */
    tx_buf[cmd++] = HCI_CONTROL_GROUP_GATT;
    tx_buf[cmd++] = HCI_CONTROL_GROUP_AMS;

    wiced_transport_send_data(HCI_CONTROL_MISC_EVENT_VERSION, tx_buf, cmd);
}



/*
 * AMS application initialization is executed after BT stack initialization is completed.
 */
void ams_application_init(void)
{
    wiced_bt_gatt_status_t gatt_status;
    wiced_result_t         result;
#if !defined(CYW20735B1) && !defined(CYW20819A1)
    /* Initialize wiced app */
    wiced_bt_app_init();
#endif

#if TEST_USE_BUTTON
    /* Configure the button available on the platform */
#if ( defined(CYW20719B1) || defined(CYW20721B1) || defined(CYW20735B1) || CYW20819A1)
    wiced_platform_register_button_callback( WICED_PLATFORM_BUTTON_1, ams_interrupt_handler, NULL, GPIO_EN_INT_RISING_EDGE);
#elif defined(CYW20706A2)
    wiced_hal_gpio_configure_pin( WICED_GPIO_BUTTON, WICED_GPIO_BUTTON_SETTINGS( GPIO_EN_INT_RISING_EDGE ), WICED_GPIO_BUTTON_DEFAULT_STATE );
    wiced_hal_gpio_register_pin_for_interrupt(WICED_GPIO_BUTTON, ams_interrupt_handler, NULL);
#else
    wiced_hal_gpio_register_pin_for_interrupt(WICED_GPIO_PIN_BUTTON, ams_interrupt_handler, NULL);
    wiced_hal_gpio_configure_pin(WICED_GPIO_PIN_BUTTON, WICED_GPIO_BUTTON_SETTINGS, GPIO_PIN_OUTPUT_LOW);
#endif
#endif

    /* Register with stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(ams_gatts_callback);

    WICED_BT_TRACE("wiced_bt_gatt_register: %d\n", gatt_status);

    /*  Tell stack to use our GATT databse */
    gatt_status =  wiced_bt_gatt_db_init(gatt_server_db, gatt_server_db_len);

    WICED_BT_TRACE("wiced_bt_gatt_db_init %d\n", gatt_status);

#ifdef WICED_BT_TRACE_ENABLE
    /* Register callback for receiving hci traces */
    wiced_bt_dev_register_hci_trace(ams_trace_callback);
#endif

    /* Load the address resolution DB with the keys stored in the NVRAM */
    ams_load_keys_to_addr_resolution_db();

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);

    /* Set the advertising params and make the device discoverable */
    ams_set_advertisement_data();

    result =  wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL);
    WICED_BT_TRACE("wiced_bt_start_advertisements %d\n", result);
}

/*
 * Callback for various GATT events.  As this application performs only as a GATT server, some of the events are omitted.
 */
wiced_bt_gatt_status_t ams_gatts_callback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    WICED_BT_TRACE("GATT callback event %d\n", event);

    switch(event)
    {
    case GATT_CONNECTION_STATUS_EVT:
        if (p_data->connection_status.connected)
        {
            ams_connection_up(&p_data->connection_status);
        }
        else
        {
            ams_connection_down(&p_data->connection_status);
        }
        break;

    case GATT_OPERATION_CPLT_EVT:
        result = ams_gatt_operation_complete(&p_data->operation_complete);
        break;

    case GATT_DISCOVERY_RESULT_EVT:
        result = ams_gatt_discovery_result(&p_data->discovery_result);
        break;

    case GATT_DISCOVERY_CPLT_EVT:
        result = ams_gatt_discovery_complete(&p_data->discovery_complete);
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        result = ams_gatts_req_callback(&p_data->attribute_request);
        break;

    default:
        break;
    }

    return result;
}

/*
 * AMS Client link management callback
 */
wiced_result_t ams_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t                    result = WICED_BT_SUCCESS;
    wiced_bt_dev_encryption_status_t *p_status;
    wiced_bt_dev_ble_pairing_info_t  *p_info;
    wiced_bt_ble_advert_mode_t       *p_mode;
    wiced_bt_device_link_keys_t       paired_device_link_keys_request;
    uint8_t                           bytes_written, bytes_read;
    wiced_bt_local_identity_keys_t*   p_keys;

    WICED_BT_TRACE("ams_management_cback: 0x%x\n", event);

    switch(event)
    {
    /* Bluetooth  stack enabled */
    case BTM_ENABLED_EVT:
        ams_application_init();
        break;

    case BTM_DISABLED_EVT:
        break;

    case BTM_USER_CONFIRMATION_REQUEST_EVT:
        WICED_BT_TRACE("Numeric_value: %d \n", p_event_data->user_confirmation_request.numeric_value);
        wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS , p_event_data->user_confirmation_request.bd_addr);
        break;

    case BTM_PASSKEY_NOTIFICATION_EVT:
        WICED_BT_TRACE("PassKey Notification. BDA %B, Key %d \n", p_event_data->user_passkey_notification.bd_addr, p_event_data->user_passkey_notification.passkey );
        wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS, p_event_data->user_passkey_notification.bd_addr );
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
        p_event_data->pairing_io_capabilities_ble_request.local_io_cap  = BTM_IO_CAPABILITIES_NONE;
        p_event_data->pairing_io_capabilities_ble_request.oob_data      = BTM_OOB_NONE;
        p_event_data->pairing_io_capabilities_ble_request.auth_req      = BTM_LE_AUTH_REQ_SC_MITM_BOND;
        p_event_data->pairing_io_capabilities_ble_request.max_key_size  = 0x10;
        p_event_data->pairing_io_capabilities_ble_request.init_keys     = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
        p_event_data->pairing_io_capabilities_ble_request.resp_keys     = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
        break;

    case BTM_PAIRING_COMPLETE_EVT:
        WICED_BT_TRACE("Pairing Complete: %d\n", p_event_data->pairing_complete.pairing_complete_info.ble.reason);
        ams_process_pairing_complete(&p_event_data->pairing_complete);
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
        WICED_BT_TRACE("Link Keys Update\n");
        ams_save_link_keys(&p_event_data->paired_device_link_keys_update);
        break;

     case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
         WICED_BT_TRACE("Link Keys Request\n");
        if (ams_read_link_keys(&p_event_data->paired_device_link_keys_request))
        {
            result = WICED_BT_SUCCESS;
            WICED_BT_TRACE("Key retrieval success\n");
        }
        else
        {
            result = WICED_BT_ERROR;
            WICED_BT_TRACE("Key retrieval failure\n");
        }
        break;

    case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
        /* save keys to NVRAM */
        p_keys = &p_event_data->local_identity_keys_update;
        wiced_hal_write_nvram (WICED_NVRAM_VSID_START + 1, sizeof(wiced_bt_local_identity_keys_t), (uint8_t*)p_keys, &result);
        WICED_BT_TRACE("local keys save to NVRAM result: %d \n", result);
        break;

    case  BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
        /* read keys from NVRAM */
        p_keys = &p_event_data->local_identity_keys_request;
        wiced_hal_read_nvram(WICED_NVRAM_VSID_START + 1, sizeof(wiced_bt_local_identity_keys_t), (uint8_t*)p_keys, &result);
        WICED_BT_TRACE("local keys read from NVRAM result: %d \n",  result);
        break;

    case BTM_ENCRYPTION_STATUS_EVT:
        WICED_BT_TRACE("Encryption Status Event: bd (%B) res %d", p_event_data->encryption_status.bd_addr, p_event_data->encryption_status.result);
        ams_process_encryption_changed(&p_event_data->encryption_status);
        break;

    case BTM_SECURITY_REQUEST_EVT:
        WICED_BT_TRACE("Security Req\n");
        wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr, WICED_BT_SUCCESS);
        break;

    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
        p_mode = &p_event_data->ble_advert_state_changed;
        WICED_BT_TRACE("Advertisement State Change: %d\n", *p_mode);
        if ((*p_mode == BTM_BLE_ADVERT_OFF) && (ams_app_state.conn_id == 0))
        {
            result =  wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL);
            WICED_BT_TRACE("wiced_bt_start_advertisements %d\n", result);
        }
        break;

    default:
        break;
    }
    return result;
}

/*
 * This function will be called when a connection is established
 */
void ams_connection_up(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    wiced_bt_gatt_status_t  status;

    WICED_BT_TRACE("%s\n", __FUNCTION__);

    ams_app_state.conn_id = p_conn_status->conn_id;

    // save address of the connected device.
    memcpy(ams_app_state.remote_addr, p_conn_status->bd_addr, sizeof(ams_app_state.remote_addr));
    ams_app_state.addr_type = p_conn_status->addr_type;

    // need to notify AMS library that the connection is up
    wiced_bt_ams_client_connection_up(p_conn_status);

    /* Initialize WICED BT AMS library Start discovery */
    ams_app_state.discovery_state = AMS_DISCOVERY_STATE_SERVICE;
    ams_app_state.started         = WICED_FALSE;
    ams_app_state.ams_s_handle    = 0;
    ams_app_state.ams_e_handle    = 0;

    wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);

    // iOS shows AMS service only to paired devices
    if (ams_is_bonded(p_conn_status->bd_addr))
    {
        // perform primary service search
        status = wiced_bt_util_send_gatt_discover(ams_app_state.conn_id, GATT_DISCOVER_SERVICES_ALL, UUID_ATTRIBUTE_PRIMARY_SERVICE, 1, 0xffff);
        WICED_BT_TRACE("start discover status:%d\n", status);
    }
}

/*
 * This function will be called when connection goes down
 */
void ams_connection_down(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    WICED_BT_TRACE("%s\n", __FUNCTION__);

    ams_app_state.conn_id         = 0;
    ams_app_state.discovery_state = AMS_DISCOVERY_STATE_SERVICE;
    ams_app_state.ams_s_handle    = 0;
    ams_app_state.ams_e_handle    = 0;

    memset(&ams_hostinfo, 0, sizeof(ams_hostinfo));

    // tell library that connection is down
    wiced_bt_ams_client_connection_down(p_conn_status);

    // restart advertisiments so that iOS can reconnect at will
    wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL);
}

/*
 * Process pairing complete event from the stack
 */
void ams_process_pairing_complete(wiced_bt_dev_pairing_cplt_t *p_pairing_complete)
{
    wiced_bt_gatt_status_t  status;

    if (p_pairing_complete->pairing_complete_info.ble.reason == 0)
    {
    	// If we are doing pairing after discovery is completed, just do the start
    	if (ams_app_state.discovery_state == AMS_DISCOVERY_STATE_DONE)
    	{
            // This version automatically starts AMS, i.e. registers to receive notifications
            wiced_bt_ams_client_start(ams_app_state.conn_id);
    	}
    	else
        {
            // perform primary service search
            status = wiced_bt_util_send_gatt_discover(ams_app_state.conn_id, GATT_DISCOVER_SERVICES_ALL, UUID_ATTRIBUTE_PRIMARY_SERVICE, 1, 0xffff);
            WICED_BT_TRACE("start discover status:%d\n", status);
        }
    }
}

/*
 * Process encryption changed event from the stack
 */
void ams_process_encryption_changed(wiced_bt_dev_encryption_status_t *p_encryption_changed)
{
    wiced_bt_gatt_status_t  status;

    if (p_encryption_changed->result == 0)
    {
        wiced_result_t              result;
        wiced_bt_device_link_keys_t keys;

        wiced_hal_read_nvram(WICED_NVRAM_VSID_START, sizeof(keys), (uint8_t *)&keys, &result);
        if ((result == 0) &&
            (memcmp(p_encryption_changed->bd_addr, keys.bd_addr, BD_ADDR_LEN) == 0))
        {
            // perform primary service search
            status = wiced_bt_util_send_gatt_discover(ams_app_state.conn_id, GATT_DISCOVER_SERVICES_ALL, UUID_ATTRIBUTE_PRIMARY_SERVICE, 1, 0xffff);
            WICED_BT_TRACE("start discover status:%d\n", status);
        }
    }
}
/*
 * GATT operation started by the client has been completed
 */
wiced_bt_gatt_status_t ams_gatt_operation_complete(wiced_bt_gatt_operation_complete_t *p_data)
{
    switch (p_data->op)
    {
    case GATTC_OPTYPE_WRITE:
        ams_process_write_rsp(p_data);
        break;

    case GATTC_OPTYPE_CONFIG:
        WICED_BT_TRACE("peer mtu:%d\n", p_data->response_data.mtu);
        break;

    case GATTC_OPTYPE_NOTIFICATION:
        ams_notification_handler(p_data);
        break;

    case GATTC_OPTYPE_READ:
    case GATTC_OPTYPE_INDICATION:
        WICED_BT_TRACE("This app does not support op:%d\n", p_data->op);
        break;
    }
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process GATT request from the peer. Even our main goal is to be a GATT client for the
 * AMS service, we need to support mandatory GATT procedures.
 */
wiced_bt_gatt_status_t ams_gatts_req_callback(wiced_bt_gatt_attribute_request_t *p_data)
{
    wiced_result_t result = WICED_BT_GATT_INVALID_PDU;

    WICED_BT_TRACE("ams_gatts_req_cb. conn %d, type %d\n", p_data->conn_id, p_data->request_type);

    switch (p_data->request_type)
    {
    case GATTS_REQ_TYPE_READ:
        result = ams_gatts_req_read_handler(p_data->conn_id, &(p_data->data.read_req));
        break;

    case GATTS_REQ_TYPE_WRITE:
        result = ams_gatts_req_write_handler(p_data->conn_id, &(p_data->data.write_req));
        break;

    case GATTS_REQ_TYPE_WRITE_EXEC:
        result = ams_gatts_req_write_exec_handler(p_data->conn_id, p_data->data.exec_write);
        break;

    case GATTS_REQ_TYPE_MTU:
        result = ams_gatts_req_mtu_handler(p_data->conn_id, p_data->data.mtu);
        break;

    case GATTS_REQ_TYPE_CONF:
        result = ams_gatts_req_conf_handler(p_data->conn_id, p_data->data.handle);
        break;

   default:
        break;
    }

    return result;
}

/*
 * Process Read request or command from peer device
 */
wiced_bt_gatt_status_t ams_gatts_req_read_handler(uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data)
{
    int          i, attr_len_to_copy;

    // Check for a matching handle entry
    for (i = 0; i < ams_gatt_db_ext_attr_tbl_size; i++)
    {
        if (ams_gatt_db_ext_attr_tbl[i].handle == p_read_data->handle)
        {
            break;
        }
    }
    if (i == ams_gatt_db_ext_attr_tbl_size)
    {
        WICED_BT_TRACE("read_hndlr attr not found hdl:%x\n", p_read_data->handle);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    attr_len_to_copy = ams_gatt_db_ext_attr_tbl[i].cur_len;

    WICED_BT_TRACE("read_hndlr conn_id:%d hdl:%x offset:%d len:%d\n", conn_id, p_read_data->handle, p_read_data->offset, attr_len_to_copy);

    if (p_read_data->offset >= ams_gatt_db_ext_attr_tbl[i].cur_len)
    {
        attr_len_to_copy = 0;
    }

    if (attr_len_to_copy != 0)
    {
        uint8_t *from;
        int      to_copy = attr_len_to_copy - p_read_data->offset;


        if (to_copy > *p_read_data->p_val_len)
        {
            to_copy = *p_read_data->p_val_len;
        }

        from = ams_gatt_db_ext_attr_tbl[i].p_data + p_read_data->offset;
        *p_read_data->p_val_len = to_copy;

        memcpy(p_read_data->p_val, from, to_copy);
    }
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process write request or write command from peer device
 */
wiced_bt_gatt_status_t ams_gatts_req_write_handler(uint16_t conn_id, wiced_bt_gatt_write_t * p_data)
{
    WICED_BT_TRACE("Ignored write_handler: conn_id:%d hdl:0x%x prep:%d offset:%d len:%d\n ", conn_id, p_data->handle, p_data->is_prep, p_data->offset, p_data->val_len);
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Write Execute Procedure
 */
wiced_bt_gatt_status_t ams_gatts_req_write_exec_handler(uint16_t conn_id, wiced_bt_gatt_exec_flag_t exec_falg)
{
    WICED_BT_TRACE("write exec: flag:%d\n", exec_falg);
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process MTU request from the peer
 */
wiced_bt_gatt_status_t ams_gatts_req_mtu_handler(uint16_t conn_id, uint16_t mtu)
{
    WICED_BT_TRACE("req_mtu: %d\n", mtu);
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process indication confirm from the peer.
 */
wiced_bt_gatt_status_t ams_gatts_req_conf_handler(uint16_t conn_id, uint16_t handle)
{
    WICED_BT_TRACE("indication_cfm, conn %d hdl %d\n", conn_id, handle);
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Set Advertising Data
 */
void ams_set_advertisement_data(void)
{
    wiced_bt_ble_advert_elem_t adv_elem[3];
    uint8_t num_elem = 0;
    uint8_t flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t power = 0;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = sizeof(uint8_t);
    adv_elem[num_elem].p_data       = &flag;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = strlen((char *)wiced_app_cfg_settings.device_name);
    adv_elem[num_elem].p_data       = (uint8_t*)wiced_app_cfg_settings.device_name;
    num_elem++;

    wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);
}

/*
 * Process discovery results from the stack
 */
wiced_bt_gatt_status_t ams_gatt_discovery_result(wiced_bt_gatt_discovery_result_t *p_data)
{
    WICED_BT_TRACE("[%s] conn %d type %d state 0x%02x\n", __FUNCTION__, p_data->conn_id, p_data->discovery_type, ams_app_state.discovery_state);

    switch (ams_app_state.discovery_state)
    {
    case AMS_DISCOVERY_STATE_AMS:
        wiced_bt_ams_client_discovery_result(p_data);
        break;

    default:
        if (p_data->discovery_type  == GATT_DISCOVER_SERVICES_ALL)
        {
            if (p_data->discovery_data.group_value.service_type.len == 16)
            {
                WICED_BT_TRACE("%04x e:%04x uuid\n", p_data->discovery_data.group_value.s_handle, p_data->discovery_data.group_value.e_handle);
                if (memcmp(p_data->discovery_data.group_value.service_type.uu.uuid128, AMS_SERVICE, 16) == 0)
                {
                    WICED_BT_TRACE("AMS Service found s:%04x e:%04x\n",
                            p_data->discovery_data.group_value.s_handle,
                            p_data->discovery_data.group_value.e_handle);
                    ams_app_state.ams_s_handle = p_data->discovery_data.group_value.s_handle;
                    ams_app_state.ams_e_handle = p_data->discovery_data.group_value.e_handle;
                }
            }
        }
        else
        {
            WICED_BT_TRACE("!!!! invalid op:%d\n", p_data->discovery_type);
        }
    }
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process discovery complete from the stack
 */
wiced_bt_gatt_status_t ams_gatt_discovery_complete(wiced_bt_gatt_discovery_complete_t *p_data)
{
    wiced_result_t result;

    WICED_BT_TRACE("[%s] conn %d type %d state %d\n", __FUNCTION__, p_data->conn_id, p_data->disc_type, ams_app_state.discovery_state);

    switch (ams_app_state.discovery_state)
    {
    case AMS_DISCOVERY_STATE_AMS:
        wiced_bt_ams_client_discovery_complete(p_data);
        break;

    default:
        if (p_data->disc_type == GATT_DISCOVER_SERVICES_ALL)
        {
            WICED_BT_TRACE("AMS:%04x-%04x\n", ams_app_state.ams_s_handle, ams_app_state.ams_e_handle);

            /* If AMS Service found tell WICED BT AMS library to start its discovery */
            if ((ams_app_state.ams_s_handle != 0) && (ams_app_state.ams_e_handle != 0))
            {
                ams_app_state.discovery_state = AMS_DISCOVERY_STATE_AMS;
                if (wiced_bt_ams_client_discover(ams_app_state.conn_id, ams_app_state.ams_s_handle, ams_app_state.ams_e_handle))
                    break;
            }
        }
        else
        {
            WICED_BT_TRACE("!!!! invalid op:%d\n", p_data->disc_type);
        }
    }
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Pass write response to appropriate client based on the attribute handle
 */
void ams_process_write_rsp(wiced_bt_gatt_operation_complete_t *p_data)
{
    WICED_BT_TRACE("write response handle:%04x\n", p_data->response_data.handle);

    // Verify that write response is for our service
    if ((p_data->response_data.handle >= ams_app_state.ams_s_handle) &&
        (p_data->response_data.handle <= ams_app_state.ams_e_handle))
    {
        wiced_bt_ams_client_write_rsp(p_data);
    }
}

/*
 * Pass notification to appropriate client based on the attribute handle
 */
void ams_notification_handler(wiced_bt_gatt_operation_complete_t *p_data)
{
    wiced_bt_ams_event_t ams_event;

    WICED_BT_TRACE("notification handle:%04x\n", p_data->response_data.att_value.handle);

    // Verify that notification is for our service
    if ((p_data->response_data.att_value.handle >= ams_app_state.ams_s_handle) &&
        (p_data->response_data.att_value.handle < ams_app_state.ams_e_handle))
    {
        if (wiced_bt_ams_client_process_notification(p_data, &ams_event))
        {
#ifdef    WICED_BT_TRACE_ENABLE
            uint8_t *p_attribute_name = (uint8_t *)PlayerAttributeId[AMS_ENTITY_ID_PLAYER];

            if (ams_event.entity_id == AMS_ENTITY_ID_PLAYER)
                p_attribute_name = (uint8_t *) ((ams_event.attribute_id < AMS_PLAYER_ATTRIBUTE_ID_MAX) ?
                        PlayerAttributeId[ams_event.attribute_id] : PlayerAttributeId[AMS_ENTITY_ID_PLAYER]);
            else if (ams_event.entity_id == AMS_ENTITY_ID_QUEUE)
                p_attribute_name = (uint8_t *) ((ams_event.attribute_id < AMS_QUEUE_ATTRIBUTE_ID_MAX) ?
                        QueueAttributeId[ams_event.attribute_id] : QueueAttributeId[AMS_ENTITY_ID_QUEUE]);
            else if (ams_event.entity_id == AMS_ENTITY_ID_TRACK)
                p_attribute_name = (uint8_t *) ((ams_event.attribute_id < AMS_TRACK_ATTRIBUTE_ID_MAX) ?
                        TrackAttributeId[ams_event.attribute_id] : TrackAttributeId[AMS_ENTITY_ID_TRACK]);

            // buffer that we received data in should have some safe area at the end.  should be ok for debugging.
            ams_event.attribute_str[ams_event.attribute_len] = 0;

            WICED_BT_TRACE ("AMS Entity ID:%s Attribute:%s Flags:%04x Value:%s",
                    (ams_event.entity_id < AMS_ENTITY_ID_MAX) ?
                            EntityId[ams_event.entity_id] : EntityId[AMS_ENTITY_ID_MAX],
                    p_attribute_name, ams_event.flags, ams_event.attribute_str);
        }
#endif
#ifdef TEST_HCI_CONTROL
        // if we are testing with a Client Control, control received event to WICED HCI AVRC
        switch(ams_event.entity_id)
        {
        case AMS_ENTITY_ID_PLAYER:
            switch(ams_event.attribute_id)
            {
    #if AMS_SUPPORT_PLAYER_NAME
            case AMS_PLAYER_ATTRIBUTE_ID_NAME:
                ams_client_send_player_name_change(ams_event.attribute_str, ams_event.attribute_len);
                break;
    #endif
            case AMS_PLAYER_ATTRIBUTE_ID_PLAYBACK_INFO:
                ams_client_process_playback_info(ams_event.attribute_str, ams_event.attribute_len);
                break;
            case AMS_PLAYER_ATTRIBUTE_ID_VOLUME:
                ams_client_process_volume_info(ams_event.attribute_str, ams_event.attribute_len);
                break;
            default:
                break;
            }
            break;

        case AMS_ENTITY_ID_QUEUE:
            switch(ams_event.attribute_id)
            {
    #if AMS_SUPPORT_PLAYLIST_INFO
            case AMS_QUEUE_ATTRIBUTE_ID_INDEX:
                ams_client_process_queue_index(ams_event.attribute_str, ams_event.attribute_len);
                break;
            case AMS_QUEUE_ATTRIBUTE_ID_COUNT:
                ams_client_send_track_info(AVRC_MEDIA_ATTR_ID_NUM_TRACKS, ams_event.attribute_str,
                        ams_event.attribute_len);
                break;
    #endif
            case AMS_QUEUE_ATTRIBUTE_ID_SHUFFLE_MODE:
                ams_client_process_shuffle_mode(ams_event.attribute_str, ams_event.attribute_len);
                break;
            case AMS_QUEUE_ATTRIBUTE_ID_REPEAT_MODE:
                ams_client_process_repeat_mode(ams_event.attribute_str, ams_event.attribute_len);
                break;
            default:
                break;
            }
            break;

        case AMS_ENTITY_ID_TRACK:
            switch(ams_event.attribute_id)
            {
            case AMS_TRACK_ATTRIBUTE_ID_ARTIST:
                ams_client_send_track_info(AVRC_MEDIA_ATTR_ID_ARTIST, ams_event.attribute_str, ams_event.attribute_len);
                break;
            case AMS_TRACK_ATTRIBUTE_ID_ALBUM:
                ams_client_send_track_info(AVRC_MEDIA_ATTR_ID_ALBUM, ams_event.attribute_str, ams_event.attribute_len);
                break;
            case AMS_TRACK_ATTRIBUTE_ID_TITLE:
                ams_client_send_track_info(AVRC_MEDIA_ATTR_ID_TITLE, ams_event.attribute_str, ams_event.attribute_len);
                break;
    #if AMS_SUPPORT_TRACK_POSITION
            case AMS_TRACK_ATTRIBUTE_ID_DURATION:
                ams_client_send_track_info(AVRC_MEDIA_ATTR_ID_PLAYING_TIME, ams_event.attribute_str,
                        ams_client_process_get_track_duration_len(ams_event.attribute_str,
                                ams_event.attribute_len));
                break;
    #endif
            default:
                break;
            }
        }
#endif
    }
}

/*
 * AMS server discovery complete
 */
void ams_discovery_complete_callback(uint16_t conn_id, wiced_bool_t result)
{
    WICED_BT_TRACE("[%s] result:%d\n", __FUNCTION__, result);
#ifdef TEST_HCI_CONTROL
    wiced_transport_send_data(HCI_CONTROL_AMS_EVENT_SERVICE_FOUND, (uint8_t*)&result, 1);
#endif

    if (result)
    {
        ams_app_state.discovery_state = AMS_DISCOVERY_STATE_DONE;

        // This version automatically starts AMS, i.e. registers to receive notifications
        wiced_bt_ams_client_start(ams_app_state.conn_id);
    }
}

/*
 * AMS server start complete
 */
void ams_start_complete_callback(uint16_t conn_id, wiced_bool_t result)
{
    WICED_BT_TRACE("[%s] result:%d\n", __FUNCTION__, result);

#ifdef TEST_HCI_CONTROL
    wiced_transport_send_data(HCI_CONTROL_AMS_EVENT_CONNECTED, (uint8_t*)&result, 1);
#endif
    if (result)
    {
        ams_app_state.started = WICED_TRUE;
    }
}

/*
 * AMS server stop complete
 */
void ams_stop_complete_callback(uint16_t conn_id, wiced_bool_t result)
{
    WICED_BT_TRACE("[%s] result:%d\n", __FUNCTION__, result);
#ifdef TEST_HCI_CONTROL
    wiced_transport_send_data(HCI_CONTROL_AMS_EVENT_DISCONNECTED, (uint8_t*)&result, 1);
#endif
    ams_app_state.started = WICED_FALSE;
}

#ifdef TEST_HCI_CONTROL
/*
 * Handle received command over UART. Please refer to the WICED HCI Control
 * Protocol for details on the protocol.  The function converts from the WICED
 * HCI remote control commands to the commands expected by the AMS.
 */
uint32_t  ams_proc_rx_hci_cmd(uint8_t *p_buffer, uint32_t length)
{
    uint16_t                opcode;
    uint8_t*                p_data = p_buffer;
    uint16_t                payload_len;
    uint8_t                 status = HCI_CONTROL_STATUS_SUCCESS;
    wiced_bt_gatt_status_t  gatt_status = WICED_BT_GATT_SUCCESS;
    uint8_t                 ams_opcode;

    // WICED_BT_TRACE("hci_control_proc_rx_cmd:%d\n", length);

    if (!p_data)
    {
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    //Expected minimum 4 byte as the wiced header
    if (length < 4)
    {
        WICED_BT_TRACE("invalid params\n");
        wiced_transport_free_buffer(p_data);
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }
    STREAM_TO_UINT16(opcode, p_data);       // Get OpCode
    STREAM_TO_UINT16(payload_len, p_data);  // Gen Payload Length

    WICED_BT_TRACE("cmd_opcode 0x%02x payload_len %d \n", opcode, payload_len);

    if (opcode == HCI_CONTROL_MISC_COMMAND_GET_VERSION)
    {
        WICED_BT_TRACE("HCI_CONTROL_MISC_COMMAND_GET_VERSION\n");
        ams_control_handle_get_version();
    }
    if (ams_app_state.conn_id == 0)
    {
        WICED_BT_TRACE("no connection\n");
        status = HCI_CONTROL_STATUS_NOT_CONNECTED;
    }
    else
    {
        if (((opcode >> 8) & 0xff) == HCI_CONTROL_GROUP_AMS)
        {
            switch (opcode)
            {
            case HCI_CONTROL_AMS_COMMAND_CONNECT:
                status = (wiced_bt_ams_client_start(ams_app_state.conn_id) ? HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED);
                break;

            case HCI_CONTROL_AMS_COMMAND_DISCONNECT:
                status = (wiced_bt_ams_client_stop(ams_app_state.conn_id) ? HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED);
                break;

            default:
                WICED_BT_TRACE("cmd_opcode 0x%02x ignored\n", opcode);
                status = HCI_CONTROL_STATUS_UNKNOWN_COMMAND;
                break;
            }
            wiced_transport_send_data(HCI_CONTROL_AMS_EVENT_COMMAND_STATUS, &status, 1);
        }
        else if (((opcode >> 8) & 0xff) == HCI_CONTROL_GROUP_AVRC_CONTROLLER)
        {
            switch (opcode)
            {
            case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_PLAY:
                gatt_status = wiced_bt_ams_send_remote_command(ams_app_state.conn_id, AMS_REMOTE_COMMAND_ID_PLAY);
                break;

            case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_PAUSE:
                gatt_status = wiced_bt_ams_send_remote_command(ams_app_state.conn_id, AMS_REMOTE_COMMAND_ID_PAUSE);
                break;

            case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_NEXT_TRACK:
                gatt_status = wiced_bt_ams_send_remote_command(ams_app_state.conn_id, AMS_REMOTE_COMMAND_ID_NEXT_TRACK);
                break;

            case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_PREVIOUS_TRACK:
                gatt_status = wiced_bt_ams_send_remote_command(ams_app_state.conn_id, AMS_REMOTE_COMMAND_ID_PREVIOUS_TRACK);
                break;

            case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_VOLUME_UP:
                gatt_status = wiced_bt_ams_send_remote_command(ams_app_state.conn_id, AMS_REMOTE_COMMAND_ID_VOLUME_UP);
                break;

            case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_VOLUME_DOWN:
                gatt_status = wiced_bt_ams_send_remote_command(ams_app_state.conn_id, AMS_REMOTE_COMMAND_ID_VOLUME_DOWN);
                break;

            case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_SET_REPEAT_MODE:
                gatt_status = wiced_bt_ams_send_remote_command(ams_app_state.conn_id, AMS_REMOTE_COMMAND_ID_ADVANCED_REPEAT_MODE);
                break;

            case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_SET_SHUFFLE_MODE:
                gatt_status = wiced_bt_ams_send_remote_command(ams_app_state.conn_id, AMS_REMOTE_COMMAND_ID_ADVANCED_SHUFFLE_MODE);
                break;

            case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_BEGIN_FAST_FORWARD:
                gatt_status = wiced_bt_ams_send_remote_command(ams_app_state.conn_id, AMS_REMOTE_COMMAND_ID_SKIP_FORWARD);
                break;

            case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_BEGIN_REWIND:
                gatt_status = wiced_bt_ams_send_remote_command(ams_app_state.conn_id, AMS_REMOTE_COMMAND_ID_SKIP_BACKWARD);
                break;

            case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_END_FAST_FORWARD:
            case HCI_CONTROL_AVRC_CONTROLLER_COMMAND_END_REWIND:
            	// this messages are not needed for AMS which just skips 10 sec forward or backward
            	// on the button push. Silently ignore.
            	break;

            default:
                WICED_BT_TRACE("cmd_opcode 0x%02x ignored\n", opcode);
                status = HCI_CONTROL_STATUS_UNKNOWN_COMMAND;
                break;
            }
            if ((status == HCI_CONTROL_STATUS_SUCCESS) && (gatt_status != WICED_BT_GATT_SUCCESS))
            {
                status = HCI_CONTROL_STATUS_FAILED;
            }
            wiced_transport_send_data(HCI_CONTROL_AVRC_CONTROLLER_EVENT_COMMAND_STATUS, &status, 1);
        }
        else
        {
            WICED_BT_TRACE("ignored group:0x%02x\n", (opcode >> 8) & 0xff);
        }
    }

    // Freeing the buffer in which data is received
    wiced_transport_free_buffer(p_buffer);
    return HCI_CONTROL_STATUS_SUCCESS;
}
#endif

/*
 * Read keys from the NVRAM and update address resolution database
 */
void ams_load_keys_to_addr_resolution_db(void)
{
    uint8_t                     bytes_read;
    wiced_result_t              result;
    wiced_bt_device_link_keys_t keys;

    bytes_read = wiced_hal_read_nvram(WICED_NVRAM_VSID_START, sizeof(keys), (uint8_t *)&keys, &result);

    WICED_BT_TRACE("[%s] read status %d bytes read %d \n", __FUNCTION__, result, bytes_read);

    // if failed to read NVRAM, there is nothing saved at that location
    if (result == WICED_SUCCESS)
    {
#ifdef CYW20706A2
        result = wiced_bt_dev_add_device_to_address_resolution_db(&keys, keys.key_data.ble_addr_type);
#else
        result = wiced_bt_dev_add_device_to_address_resolution_db(&keys);
#endif
    }
}

/*
 * This function is called to save keys generated as a result of pairing or keys update
 */
wiced_bool_t ams_save_link_keys(wiced_bt_device_link_keys_t *p_keys)
{
    uint8_t          bytes_written;
    wiced_result_t   result;

    bytes_written = wiced_hal_write_nvram(WICED_NVRAM_VSID_START, sizeof(wiced_bt_device_link_keys_t), (uint8_t *)p_keys, &result);
    WICED_BT_TRACE("Saved %d bytes at id:%d %d\n", bytes_written, WICED_NVRAM_VSID_START);
    return (bytes_written == sizeof (wiced_bt_device_link_keys_t));
}

/*
 * This function is called to read keys for specific bdaddr
 */
wiced_bool_t ams_read_link_keys(wiced_bt_device_link_keys_t *p_keys)
{
    uint8_t         bytes_read;
    wiced_result_t  result;

    bytes_read = wiced_hal_read_nvram(WICED_NVRAM_VSID_START, sizeof(wiced_bt_device_link_keys_t), (uint8_t *)p_keys, &result);
    WICED_BT_TRACE("read %d bytes at id:%d %d\n", bytes_read, WICED_NVRAM_VSID_START);
    return (bytes_read == sizeof (wiced_bt_device_link_keys_t));
}

/*
 * Return TRUE if the link key for specified BDADDR is saved in the NVRAM
 */
wiced_bool_t ams_is_bonded(BD_ADDR bd_addr)
{
    wiced_bt_device_link_keys_t keys;

    return ((ams_read_link_keys(&keys)) &&
            (memcmp(keys.bd_addr, bd_addr, BD_ADDR_LEN) == 0));
}

#ifdef HCI_TRACE_OVER_TRANSPORT
/*
 *  Pass protocol traces up over the transport
 */
void ams_trace_callback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data)
{
    wiced_transport_send_hci_trace(host_trans_pool, type, length, p_data);
}
#endif

#ifdef TEST_HCI_CONTROL
/*
 * Process playback information from the iOS device
 */
void ams_client_process_playback_info(uint8_t *p_info, int len)
{
    uint8_t playback_status = p_info[0] - '0';
    uint32_t elapsed_time = 0;
    int i;

    WICED_BT_TRACE("playback info len:%d status:%d\n", len, playback_status);

    // Playback info concatenation of three comma-separated values
    if ((len < 2) || (playback_status > AMS_PLAYBACK_STATUS_MAX) || (p_info[1] != ','))
    {
        WICED_BT_TRACE("failed\n");
        return;
    }
//    if (playback_status != ams_client.playback_status)
    {
//        ams_client.playback_status = playback_status;
        ams_client_send_play_status_change(ams_client_to_hci_playback_status[playback_status]);
    }
#if AMS_SUPPORT_TRACK_POSITION
    p_info += 2;
    len -= 2;

    // second value is PlaybackRate: a string that represents the floating point value of the playback rate.  Skip it.
    while ((len != 0) && (*p_info != ','))
    {
        p_info++;
        len--;
    }
    if (len == 0)
        return;
    p_info++;
    while ((len != 0) && (*p_info != '.'))
    {
        elapsed_time = (elapsed_time * 10) + (*p_info - '0');
        p_info++;
        len--;
    }
    ams_client_send_play_position_change(elapsed_time);
#endif
}

/*
 * Process volume change notification from the iOS device
 */
void ams_client_process_volume_info(uint8_t *p_info, int len)
{
    uint8_t volume_level = 0;

    WICED_BT_TRACE("volume info len:%d\n", len);

    // A string that represents the floating point value of the volume, ranging from 0 (silent) to 1 (full volume).
    if ((len < 2) || (p_info[0] != '0') || (p_info[1] != '.'))
    {
        WICED_BT_TRACE("failed\n");
        return;
    }
    volume_level = (p_info[2] - '0') * 10;
    if (len > 3)
        volume_level += (p_info[3] - '0');

    ams_client_send_volume_level_change(volume_level);
}


#if AMS_SUPPORT_PLAYLIST_INFO
/*
 * ams_client_u32toa
 * Convert an uint32 to a string
 */
void ams_client_u32toa(char * p_buffer, int buffer_len, uint32_t value)
{
    int divisor = 1000000000;
    int digit;
    char * p = p_buffer;

    memset(p_buffer, 0, buffer_len);

    /* Search for the first significant (not null) dozen */
    while( divisor && ((value / divisor) == 0))
    {
        divisor /= 10;
    }

    if (divisor == 0)
    {
        *p = '0';
        return;
    }

    do
    {
        digit = value / divisor;
        *p++ = (char)(digit + '0');
        value -= digit * divisor;
        divisor /= 10;
    } while (divisor > 0);
}

/*
 * Process queue index change notification from the iOS device.
 * A string containing the integer value of the queue index, zero-based.
 * AVRC track number is 1 based
 * Convert the received string (number) to an uint32, add one and convert it
 * back to a string
 */
void ams_client_process_queue_index(uint8_t *p_info, int len)
{
    uint32_t track_number = 0;
    int     i;
    uint8_t queue_idx_str[10]; /* uint32 requires 9 digits and '\0' */

    for (i = 0; i < len; i++)
    {
        track_number = (track_number * 10) + (p_info[i] - '0');
    }

    // AVRC track number is 1 based
    track_number += 1;

    ams_client_u32toa((char *)queue_idx_str, sizeof(queue_idx_str), track_number);

    ams_client_send_track_info(AVRC_MEDIA_ATTR_ID_TRACK_NUM,
            queue_idx_str, strlen((char *)queue_idx_str));
}
#endif

/*
 * Process shuffle mode change notification from the iOS device
 */
void ams_client_process_shuffle_mode(uint8_t *p_info, int len)
{
    uint8_t  shuffle_mode = p_info[0] - '0';

    if ((len < 1) || (shuffle_mode > AMS_SHUFFLE_MODE_MAX))
    {
        WICED_BT_TRACE("failed\n");
        return;
    }
    ams_client_send_setting_change(AVRC_PLAYER_SETTING_SHUFFLE, ams_client_to_hci_shuffle_mode[shuffle_mode]);
}

/*
 * Process shuffle mode change notification from the iOS device
 */
void ams_client_process_repeat_mode(uint8_t *p_info, int len)
{
    uint8_t  repeat_mode = p_info[0] - '0';

    if ((len < 1) || (repeat_mode > AMS_REPEAT_MODE_MAX))
    {
        WICED_BT_TRACE("failed\n");
        return;
    }
    ams_client_send_setting_change(AVRC_PLAYER_SETTING_REPEAT, ams_client_to_hci_repeat_mode[repeat_mode]);
}

/*
 *  send playback status to the MCU
 */
wiced_result_t ams_client_send_play_status_change(uint8_t playback_status)
{
    uint8_t event_data[3];

    WICED_BT_TRACE("[%s]:%d\n", __FUNCTION__, playback_status);

    event_data[0] = 0;                    // handle
    event_data[1] = 0;
    event_data[2] = playback_status;      // play status

    return wiced_transport_send_data(HCI_CONTROL_AVRC_CONTROLLER_EVENT_PLAY_STATUS, event_data, 3);
}

/*
 *  send playback position to the MCU
 */
#if AMS_SUPPORT_TRACK_POSITION
wiced_result_t ams_client_send_play_position_change(uint32_t elapsed_time)
{
    uint8_t event_data[6];

    WICED_BT_TRACE("[%s]:%d\n", __FUNCTION__, elapsed_time);

    event_data[0] = 0;                    // handle
    event_data[1] = 0;
    event_data[2] = elapsed_time & 0xff;  // play position
    event_data[3] = (elapsed_time >> 8)  & 0xff;
    event_data[4] = (elapsed_time >> 16) & 0xff;
    event_data[5] = (elapsed_time >> 24) & 0xff;

    return wiced_transport_send_data(HCI_CONTROL_AVRC_CONTROLLER_EVENT_PLAY_POSITION, event_data, 6);
}
#endif

/*
 *  send volume level change to the MCU
 */
wiced_result_t ams_client_send_setting_change(uint8_t setting_id, uint8_t mode)
{
    uint8_t event_data[5];

    WICED_BT_TRACE("[%s]:%d %d\n", __FUNCTION__, setting_id, mode);

    event_data[0] = 0;                  // handle
    event_data[1] = 0;
    event_data[2] = 1;                  // number of settings
    event_data[3] = setting_id;
    event_data[4] = mode;

    return wiced_transport_send_data(HCI_CONTROL_AVRC_CONTROLLER_EVENT_SETTING_CHANGE, event_data, 5);
}

/*
 *  send volume level change to the MCU
 */
wiced_result_t ams_client_send_volume_level_change(uint8_t volume_level)
{
    uint8_t event_data[3];

    WICED_BT_TRACE("[%s]:%d\n", __FUNCTION__, volume_level);

    event_data[0] = 0;                 // handle
    event_data[1] = 0;
    event_data[2] = volume_level;      // volume level status

    return wiced_transport_send_data(HCI_CONTROL_AVRC_TARGET_EVENT_VOLUME_LEVEL, event_data, 3);
}

/*
 *  send player name to the MCU
 */
#if AMS_SUPPORT_PLAYER_NAME
wiced_result_t ams_client_send_player_name_change(uint8_t *p_name, uint8_t len)
{
    uint8_t event_data[60];

    event_data[0] = 0;                 // handle
    event_data[1] = 0;

    if (len > sizeof(event_data) - 2)
        len = sizeof(event_data) - 2;

    memcpy(&event_data[2], p_name, len);

    return wiced_transport_send_data(HCI_CONTROL_AVRC_CONTROLLER_EVENT_PLAYER_CHANGE, event_data, len + 2);
}
#endif

/*
 * Process track duration notification from the iOS device
 * A string containing the floating point value of the total duration of the track in seconds.
 * WICED HCI AVRC is uint32 in seconds
 */
#if AMS_SUPPORT_TRACK_POSITION
int ams_client_process_get_track_duration_len(uint8_t *p_info, int len)
{
    int duration_len = 0;

    /* AMS sends the track duration using float (sec.ms) */
    /* Let's ignore the milli-sec part (starting from the '.') */
    while ((len != 0) && (*p_info++ != '.'))
    {
        duration_len++;
    }
    return duration_len;
}
#endif

/*
 * send track information change to the MCU
 */
wiced_result_t ams_client_send_track_info(uint8_t info_id, uint8_t *attribute, uint8_t attribute_len)
{
    wiced_result_t  rc = WICED_BT_ERROR;
    uint8_t         *p_event_data;
    uint16_t        msg_size = sizeof(uint16_t) + /* handle*/
                               sizeof(uint8_t)  + /* status */
                               sizeof(uint8_t)  + /* element type ID */
                               sizeof(uint16_t) + /* element string length */
                               attribute_len;

    p_event_data = (uint8_t *)wiced_bt_get_buffer(msg_size);
    if (p_event_data != NULL)
    {
        p_event_data[0] = 0;        // handle
        p_event_data[1] = 0;
        p_event_data[2] = 0;        // status
        p_event_data[3] = info_id;
        p_event_data[4] = attribute_len & 0xff;
        p_event_data[5] = (attribute_len >> 8) & 0xff;
        memcpy(&p_event_data[6], attribute, attribute_len);

        rc = wiced_transport_send_data(HCI_CONTROL_AVRC_CONTROLLER_EVENT_CURRENT_TRACK_INFO, p_event_data, msg_size);
        wiced_bt_free_buffer(p_event_data);
    }
    return rc;
}
#endif

#ifdef TEST_USE_BUTTON
/*
 * Test function to process button push.
 * If application is not connected, start high duty advertisements
 * If connected and discovery complete, tell ams library to connect to AMS server
 * If connected and started, send remote control commands
 */
void ams_interrupt_handler(void *data, uint8_t port_pin)
{
    wiced_result_t result;
	wiced_bt_gatt_status_t  gatt_status = WICED_BT_GATT_SUCCESS;

    WICED_BT_TRACE("gpio_interrupt_handler pin: %d\n", port_pin);

     /* Get the status of interrupt on P# */
    if (wiced_hal_gpio_get_pin_interrupt_status(BUTTON_GPIO))
    {
        /* Clear the GPIO interrupt */
        wiced_hal_gpio_clear_pin_interrupt_status(BUTTON_GPIO);
    }

    if (ams_app_state.conn_id == 0)
    {
        result =  wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
        WICED_BT_TRACE("wiced_bt_start_advertisements %d\n", result);
    }
    else if (ams_app_state.ams_s_handle == 0)
    {
        wiced_bt_util_send_gatt_discover(ams_app_state.conn_id, GATT_DISCOVER_SERVICES_ALL, UUID_ATTRIBUTE_PRIMARY_SERVICE, 1, 0xffff);
    }
    else if (!ams_app_state.started)
    {
        wiced_bt_ams_client_start(ams_app_state.conn_id);
    }
    else
    {
        static int cur_inx = 0;
        uint16_t rc_command[] =
        {
            AMS_REMOTE_COMMAND_ID_PLAY,
            AMS_REMOTE_COMMAND_ID_PAUSE,
            AMS_REMOTE_COMMAND_ID_TOGGLE_PLAY_PAUSE,
            AMS_REMOTE_COMMAND_ID_NEXT_TRACK,
            AMS_REMOTE_COMMAND_ID_PREVIOUS_TRACK,
            AMS_REMOTE_COMMAND_ID_VOLUME_UP,
            AMS_REMOTE_COMMAND_ID_VOLUME_DOWN,
            AMS_REMOTE_COMMAND_ID_ADVANCED_REPEAT_MODE,
            AMS_REMOTE_COMMAND_ID_ADVANCED_SHUFFLE_MODE,
            AMS_REMOTE_COMMAND_ID_SKIP_FORWARD,
            AMS_REMOTE_COMMAND_ID_SKIP_BACKWARD
        };
        gatt_status = wiced_bt_ams_send_remote_command(ams_app_state.conn_id, rc_command[cur_inx++]);
        if(gatt_status != WICED_BT_GATT_SUCCESS)
        {
            WICED_BT_TRACE("wiced_bt_ams_send_remote_command failed %d\n", gatt_status);
        }
        if (cur_inx == sizeof(rc_command) / sizeof(rc_command[0]))
            cur_inx = 0;
    }
}
#endif

#ifdef TEST_HCI_CONTROL
static void hci_control_transport_status( wiced_transport_type_t type )
{
    WICED_BT_TRACE( " hci_control_transport_status %x \n", type );
    hci_control_send_device_started_evt();
}
void hci_control_send_device_started_evt( void )
{
    wiced_transport_send_data( HCI_CONTROL_EVENT_DEVICE_STARTED, NULL, 0 );
}
#endif


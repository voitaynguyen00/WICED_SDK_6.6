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
* Apple Notification Center Service (ANCS) snippet application
*
* The ANCS snippet application shows how to initialize and use WICED BT ANCS
* library.  Device works in the peripheral mode accepting connection from
* an iOS phone.
*
* When connected is established the application performs GATT discovery of the
* connected device.  If the ANCS service is found on the device, the application
* initializes the ANCS library and passes discovery events to the library for
* processing.  Similarly notifications received from the phone are passed to
* the library which implements the protocol and sends back functional
* information.
*
* To test this snippet app use ClientControl application -
* \apps\host\ClientControl\<OS>\ClientControl
*
* Features demonstrated
*  - Initialize and use WICED BT ANCS library
*
* To demonstrate the app, work through the following steps.
* 1. Plug the WICED eval board into your computer
* 2. Build and download the application (to the WICED board)
* 3. Start tracing to monitor the activity (see Kit Guide for details)
* 4. Pair with a client (iOS device)
* 5. Send SMS/incoming call to the phone and verify traces
* 6. Use ClientControl app to send positive/negative actions to the phone
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
#include "wiced_bt_ancs.h"
#include "wiced_bt_stack.h"
#include "wiced_memory.h"
#include "string.h"
#include "wiced_transport.h"

#define HCI_TRACE_OVER_TRANSPORT    1   // Send trace messages over WICED HCI interface
#define TEST_HCI_CONTROL            1   // Use WICED HCI interface for test purposes

#if defined WICED_BT_TRACE_ENABLE || defined TEST_HCI_CONTROL || defined HCI_TRACE_OVER_TRANSPORT
#include "wiced_transport.h"
#endif

/******************************************************
 *                      Constants
 ******************************************************/

#ifdef WICED_BT_TRACE_ENABLE
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
static wiced_result_t         ancs_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
static wiced_bt_gatt_status_t ancs_gatts_callback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data);
static void                   ancs_connection_up(wiced_bt_gatt_connection_status_t *p_conn_status);
static void                   ancs_connection_down(wiced_bt_gatt_connection_status_t *p_conn_status);
static void                   ancs_process_pairing_complete(wiced_bt_dev_pairing_cplt_t *pairing_complete);
static wiced_bt_gatt_status_t ancs_gatt_operation_complete(wiced_bt_gatt_operation_complete_t *p_data);
static wiced_bt_gatt_status_t ancs_gatt_discovery_result(wiced_bt_gatt_discovery_result_t *p_data);
static wiced_bt_gatt_status_t ancs_gatt_discovery_complete(wiced_bt_gatt_discovery_complete_t *p_data);
static wiced_bt_gatt_status_t ancs_gatts_req_callback(wiced_bt_gatt_attribute_request_t *p_data);
static wiced_bt_gatt_status_t ancs_gatts_req_read_handler(uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data);
static wiced_bt_gatt_status_t ancs_gatts_req_write_handler(uint16_t conn_id, wiced_bt_gatt_write_t * p_data);
static wiced_bt_gatt_status_t ancs_gatts_req_write_exec_handler(uint16_t conn_id, wiced_bt_gatt_exec_flag_t exec_falg);
static wiced_bt_gatt_status_t ancs_gatts_req_mtu_handler(uint16_t conn_id, uint16_t mtu);
static wiced_bt_gatt_status_t ancs_gatts_req_conf_handler(uint16_t conn_id, uint16_t handle);
static void                   ancs_load_keys_to_addr_resolution_db(void);
static wiced_bool_t           ancs_save_link_keys(wiced_bt_device_link_keys_t *p_keys);
static wiced_bool_t           ancs_read_link_keys(wiced_bt_device_link_keys_t *p_keys);
static wiced_bool_t           ancs_is_bonded(BD_ADDR bd_addr);
static void                   ancs_set_advertisement_data(void);
static void                   ancs_discovery_complete_callback(uint16_t conn_id, wiced_bool_t result);
static void                   ancs_start_complete_callback(uint16_t conn_id, wiced_bt_gatt_status_t result);
static void                   ancs_stop_complete_callback(uint16_t conn_id, wiced_bt_gatt_status_t result);
static void                   ancs_notification_callback(uint16_t conn_id, ancs_event_t *p_ancs_event);
static void                   ancs_process_write_rsp(wiced_bt_gatt_operation_complete_t *p_data);
static void                   ancs_notification_handler(wiced_bt_gatt_operation_complete_t *p_data);

static char                   *utl_strcpy(char *p_dst, char *p_src);

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

#define ANCS_DISCOVERY_STATE_SERVICE    0
#define ANCS_DISCOVERY_STATE_ANCS       1
    uint8_t                     discovery_state;

    uint8_t                     started;

    // Current value of the client configuration descriptor for characteristic 'Report'
    uint16_t                    ancs_s_handle;
    uint16_t                    ancs_e_handle;

    BD_ADDR                     remote_addr;   //address of currently connected client
    wiced_bt_ble_address_type_t addr_type;
} ancs_app_state_t;

#pragma pack()

// NVRAM save area
HOSTINFO ancs_hostinfo;

ancs_app_state_t ancs_app_state;

// Registration structure to be passed to the library
wiced_bt_ancs_reg_t ancs_client_reg =
{
    .p_discovery_complete_callback = ancs_discovery_complete_callback,
    .p_start_complete_callback	   = ancs_start_complete_callback,
    .p_stop_complete_callback      = ancs_stop_complete_callback,
    .p_notification_callback       = ancs_notification_callback,
};

#if defined WICED_BT_TRACE_ENABLE || defined HCI_TRACE_OVER_TRANSPORT || defined TEST_HCI_CONTROL

#define TRANS_UART_BUFFER_SIZE          1024
#define ANCS_TRANS_MAX_BUFFERS          2

#ifdef TEST_HCI_CONTROL
static uint32_t  ancs_proc_rx_hci_cmd(uint8_t *p_data, uint32_t length);
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
    .p_data_handler = ancs_proc_rx_hci_cmd,
#else
    .p_status_handler = NULL,
    .p_data_handler = NULL,
#endif
    .p_tx_complete_cback = NULL
};

wiced_transport_buffer_pool_t*  host_trans_pool;
static void ancs_trace_callback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data);
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
    host_trans_pool = wiced_transport_create_buffer_pool(TRANS_UART_BUFFER_SIZE, ANCS_TRANS_MAX_BUFFERS);

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
    WICED_BT_TRACE("ANCS APP START\n");

    wiced_bt_ancs_client_initialize (&ancs_client_reg);

    memset(&ancs_hostinfo, 0, sizeof(ancs_hostinfo));
    memset(&ancs_app_state, 0, sizeof(ancs_app_state));

    /* Initialize Stack and Register Management Callback */
    wiced_bt_stack_init(ancs_management_callback, &wiced_app_cfg_settings, wiced_app_cfg_buf_pools);
}

/*
 * ANCS application initialization is executed after BT stack initialization is completed.
 */
void ancs_application_init(void)
{
    wiced_bt_gatt_status_t gatt_status;
    wiced_result_t         result;
#if !defined(CYW20735B1) && !defined(CYW20819A1)
    /* Initialize wiced app */
    wiced_bt_app_init();
#endif
    /* Register with stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(ancs_gatts_callback);

    WICED_BT_TRACE("wiced_bt_gatt_register: %d\n", gatt_status);

    /*  Tell stack to use our GATT databse */
    gatt_status =  wiced_bt_gatt_db_init(gatt_server_db, gatt_server_db_len);

    WICED_BT_TRACE("wiced_bt_gatt_db_init %d\n", gatt_status);

#ifdef WICED_BT_TRACE_ENABLE
    /* Register callback for receiving hci traces */
    wiced_bt_dev_register_hci_trace(ancs_trace_callback);
#endif

    /* Load the address resolution DB with the keys stored in the NVRAM */
    ancs_load_keys_to_addr_resolution_db();

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);

    /* Set the advertising params and make the device discoverable */
    ancs_set_advertisement_data();

    result =  wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL);
    WICED_BT_TRACE("wiced_bt_start_advertisements %d\n", result);
    UNUSED_VARIABLE(result);
    UNUSED_VARIABLE(gatt_status);
}

void ansc_control_handle_get_version()
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
    tx_buf[cmd++] = HCI_CONTROL_GROUP_ANCS;

    wiced_transport_send_data(HCI_CONTROL_MISC_EVENT_VERSION, tx_buf, cmd);
}



/*
 * Callback for various GATT events.  As this application performs only as a GATT server, some of the events are omitted.
 */
wiced_bt_gatt_status_t ancs_gatts_callback(wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    switch(event)
    {
    case GATT_CONNECTION_STATUS_EVT:
        if (p_data->connection_status.connected)
        {
            ancs_connection_up(&p_data->connection_status);
        }
        else
        {
            ancs_connection_down(&p_data->connection_status);
        }
        break;

    case GATT_OPERATION_CPLT_EVT:
        result = ancs_gatt_operation_complete(&p_data->operation_complete);
        break;

    case GATT_DISCOVERY_RESULT_EVT:
        result = ancs_gatt_discovery_result(&p_data->discovery_result);
        break;

    case GATT_DISCOVERY_CPLT_EVT:
        result = ancs_gatt_discovery_complete(&p_data->discovery_complete);
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        result = ancs_gatts_req_callback(&p_data->attribute_request);
        break;

    default:
        break;
    }

    return result;
}

/*
 * ANCS Client link management callback
 */
wiced_result_t ancs_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t                    result = WICED_BT_SUCCESS;
    wiced_bt_dev_encryption_status_t *p_status;
    wiced_bt_dev_ble_pairing_info_t  *p_info;
    wiced_bt_ble_advert_mode_t       *p_mode;
    wiced_bt_device_link_keys_t       paired_device_link_keys_request;
    uint8_t                           bytes_written, bytes_read;
    wiced_bt_local_identity_keys_t*   p_keys;

    WICED_BT_TRACE("ancs_management_cback:%d\n", event);

    switch(event)
    {
    /* Bluetooth  stack enabled */
    case BTM_ENABLED_EVT:
        ancs_application_init();
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
        ancs_process_pairing_complete(&p_event_data->pairing_complete);
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
        WICED_BT_TRACE("Link Keys Update\n");
        ancs_save_link_keys(&p_event_data->paired_device_link_keys_update);
        break;

     case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
         WICED_BT_TRACE("Link Keys Request\n");
        if (ancs_read_link_keys(&p_event_data->paired_device_link_keys_request))
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
        WICED_BT_TRACE("Encryption Status Event: bd (%B) res %d\n", p_event_data->encryption_status.bd_addr, p_event_data->encryption_status.result);
        break;

    case BTM_SECURITY_REQUEST_EVT:
        WICED_BT_TRACE("Security Req\n");
        wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr, WICED_BT_SUCCESS);
        break;

    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
        p_mode = &p_event_data->ble_advert_state_changed;
        WICED_BT_TRACE("Advertisement State Change: %d\n", *p_mode);
        if ((*p_mode == BTM_BLE_ADVERT_OFF) && (ancs_app_state.conn_id == 0))
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
void ancs_connection_up(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    wiced_bt_gatt_status_t  status;

    WICED_BT_TRACE("%s\n", __FUNCTION__);

    ancs_app_state.conn_id = p_conn_status->conn_id;

    // save address of the connected device.
    memcpy(ancs_app_state.remote_addr, p_conn_status->bd_addr, sizeof(ancs_app_state.remote_addr));
    ancs_app_state.addr_type = p_conn_status->addr_type;

    // need to notify ANCS library that the connection is up
    wiced_bt_ancs_client_connection_up(p_conn_status);

    /* Initialize WICED BT ACSS library Start discovery */
    ancs_app_state.discovery_state = ANCS_DISCOVERY_STATE_SERVICE;
    ancs_app_state.started         = WICED_FALSE;
    ancs_app_state.ancs_s_handle = 0;
    ancs_app_state.ancs_e_handle = 0;

    wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);

    // perform primary service search
    status = wiced_bt_util_send_gatt_discover(ancs_app_state.conn_id, GATT_DISCOVER_SERVICES_ALL, UUID_ATTRIBUTE_PRIMARY_SERVICE, 1, 0xffff);
    WICED_BT_TRACE("start discover status:%d\n", status);
    UNUSED_VARIABLE(status);
}

/*
 * This function will be called when connection goes down
 */
void ancs_connection_down(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    WICED_BT_TRACE("%s\n", __FUNCTION__);

    ancs_app_state.conn_id         = 0;
    ancs_app_state.discovery_state = ANCS_DISCOVERY_STATE_SERVICE;
    ancs_app_state.ancs_s_handle   = 0;
    ancs_app_state.ancs_e_handle   = 0;

    memset(&ancs_hostinfo, 0, sizeof(ancs_hostinfo));

    // tell library that connection is down
    wiced_bt_ancs_client_connection_down(p_conn_status);

    // restart advertisements so that iOS can reconnect at will
    wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL);
}

/*
 * Process pairing complete event from the stack
 */
void ancs_process_pairing_complete(wiced_bt_dev_pairing_cplt_t *p_pairing_complete)
{
    if (p_pairing_complete->pairing_complete_info.ble.reason == 0)
    {
        // if we started bonding because we could not start client, do it now
        if (!ancs_app_state.started && (ancs_app_state.ancs_s_handle != 0) && (ancs_app_state.ancs_e_handle != 0))
        {
            wiced_bt_ancs_client_start(ancs_app_state.conn_id);
        }
    }
}

/*
 * GATT operation started by the client has been completed
 */
wiced_bt_gatt_status_t ancs_gatt_operation_complete(wiced_bt_gatt_operation_complete_t *p_data)
{
    switch (p_data->op)
    {
    case GATTC_OPTYPE_WRITE:
        ancs_process_write_rsp(p_data);
        break;

    case GATTC_OPTYPE_CONFIG:
        WICED_BT_TRACE("peer mtu:%d\n", p_data->response_data.mtu);
        break;

    case GATTC_OPTYPE_NOTIFICATION:
        ancs_notification_handler(p_data);
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
 * ANCS service, we need to support mandatory GATT procedures.
 */
wiced_bt_gatt_status_t ancs_gatts_req_callback(wiced_bt_gatt_attribute_request_t *p_data)
{
    wiced_result_t result = WICED_BT_GATT_INVALID_PDU;

    WICED_BT_TRACE("ancs_gatts_req_cb. conn %d, type %d\n", p_data->conn_id, p_data->request_type);

    switch (p_data->request_type)
    {
    case GATTS_REQ_TYPE_READ:
        result = ancs_gatts_req_read_handler(p_data->conn_id, &(p_data->data.read_req));
        break;

    case GATTS_REQ_TYPE_WRITE:
        result = ancs_gatts_req_write_handler(p_data->conn_id, &(p_data->data.write_req));
        break;

    case GATTS_REQ_TYPE_WRITE_EXEC:
        result = ancs_gatts_req_write_exec_handler(p_data->conn_id, p_data->data.exec_write);
        break;

    case GATTS_REQ_TYPE_MTU:
        result = ancs_gatts_req_mtu_handler(p_data->conn_id, p_data->data.mtu);
        break;

    case GATTS_REQ_TYPE_CONF:
        result = ancs_gatts_req_conf_handler(p_data->conn_id, p_data->data.handle);
        break;

   default:
        break;
    }

    return result;
}

/*
 * Process Read request or command from peer device
 */
wiced_bt_gatt_status_t ancs_gatts_req_read_handler(uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data)
{
    int          i, attr_len_to_copy;

    // Check for a matching handle entry
    for (i = 0; i < ancs_gatt_db_ext_attr_tbl_size; i++)
    {
        if (ancs_gatt_db_ext_attr_tbl[i].handle == p_read_data->handle)
        {
            break;
        }
    }
    if (i == ancs_gatt_db_ext_attr_tbl_size)
    {
        WICED_BT_TRACE("read_hndlr attr not found hdl:%x\n", p_read_data->handle);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    attr_len_to_copy = ancs_gatt_db_ext_attr_tbl[i].cur_len;

    WICED_BT_TRACE("read_hndlr conn_id:%d hdl:%x offset:%d len:%d\n", conn_id, p_read_data->handle, p_read_data->offset, attr_len_to_copy);

    if (p_read_data->offset >= ancs_gatt_db_ext_attr_tbl[i].cur_len)
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

        from = ancs_gatt_db_ext_attr_tbl[i].p_data + p_read_data->offset;
        *p_read_data->p_val_len = to_copy;

        memcpy(p_read_data->p_val, from, to_copy);
    }
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process write request or write command from peer device
 */
wiced_bt_gatt_status_t ancs_gatts_req_write_handler(uint16_t conn_id, wiced_bt_gatt_write_t * p_data)
{
    WICED_BT_TRACE("Ignored write_handler: conn_id:%d hdl:0x%x prep:%d offset:%d len:%d\n ", conn_id, p_data->handle, p_data->is_prep, p_data->offset, p_data->val_len);
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Write Execute Procedure
 */
wiced_bt_gatt_status_t ancs_gatts_req_write_exec_handler(uint16_t conn_id, wiced_bt_gatt_exec_flag_t exec_falg)
{
    WICED_BT_TRACE("write exec: flag:%d\n", exec_falg);
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process MTU request from the peer
 */
wiced_bt_gatt_status_t ancs_gatts_req_mtu_handler(uint16_t conn_id, uint16_t mtu)
{
    WICED_BT_TRACE("req_mtu: %d\n", mtu);
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process indication confirm from the peer.
 */
wiced_bt_gatt_status_t ancs_gatts_req_conf_handler(uint16_t conn_id, uint16_t handle)
{
    WICED_BT_TRACE("indication_cfm, conn %d hdl %d\n", conn_id, handle);
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Set Advertising Data
 */
void ancs_set_advertisement_data(void)
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
wiced_bt_gatt_status_t ancs_gatt_discovery_result(wiced_bt_gatt_discovery_result_t *p_data)
{
    WICED_BT_TRACE("[%s] conn %d type %d state 0x%02x\n", __FUNCTION__, p_data->conn_id, p_data->discovery_type, ancs_app_state.discovery_state);

    switch (ancs_app_state.discovery_state)
    {
    case ANCS_DISCOVERY_STATE_ANCS:
        wiced_bt_ancs_client_discovery_result(p_data);
        break;

    default:
        if (p_data->discovery_type  == GATT_DISCOVER_SERVICES_ALL)
        {
            if (p_data->discovery_data.group_value.service_type.len == 16)
            {
                WICED_BT_TRACE("%04x e:%04x uuid\n", p_data->discovery_data.group_value.s_handle, p_data->discovery_data.group_value.e_handle);
                if (memcmp(p_data->discovery_data.group_value.service_type.uu.uuid128, ANCS_SERVICE, 16) == 0)
                {
                    WICED_BT_TRACE("ANCS Service found s:%04x e:%04x\n",
                            p_data->discovery_data.group_value.s_handle,
                            p_data->discovery_data.group_value.e_handle);
                    ancs_app_state.ancs_s_handle = p_data->discovery_data.group_value.s_handle;
                    ancs_app_state.ancs_e_handle = p_data->discovery_data.group_value.e_handle;
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
wiced_bt_gatt_status_t ancs_gatt_discovery_complete(wiced_bt_gatt_discovery_complete_t *p_data)
{
    wiced_result_t result;

    WICED_BT_TRACE("[%s] conn %d type %d state %d\n", __FUNCTION__, p_data->conn_id, p_data->disc_type, ancs_app_state.discovery_state);

    switch (ancs_app_state.discovery_state)
    {
    case ANCS_DISCOVERY_STATE_ANCS:
        wiced_bt_ancs_client_discovery_complete(p_data);
        break;

    default:
        if (p_data->disc_type == GATT_DISCOVER_SERVICES_ALL)
        {
            WICED_BT_TRACE("ANCS:%04x-%04x\n", ancs_app_state.ancs_s_handle, ancs_app_state.ancs_e_handle);

            /* If ancs Service found tell WICED BT ancs library to start its discovery */
            if ((ancs_app_state.ancs_s_handle != 0) && (ancs_app_state.ancs_e_handle != 0))
            {
                ancs_app_state.discovery_state = ANCS_DISCOVERY_STATE_ANCS;
                if (wiced_bt_ancs_client_discover(ancs_app_state.conn_id, ancs_app_state.ancs_s_handle, ancs_app_state.ancs_e_handle))
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
void ancs_process_write_rsp(wiced_bt_gatt_operation_complete_t *p_data)
{
    WICED_BT_TRACE("write response handle:%04x\n", p_data->response_data.handle);

    // Verify that write response is for our service
    if ((p_data->response_data.handle >= ancs_app_state.ancs_s_handle) &&
        (p_data->response_data.handle <= ancs_app_state.ancs_e_handle))
    {
        wiced_bt_ancs_client_write_rsp(p_data);
    }
}

/*
 * Pass notification to appropriate client based on the attribute handle
 */
void ancs_notification_handler(wiced_bt_gatt_operation_complete_t *p_data)
{
    WICED_BT_TRACE("notification handle:%04x\n", p_data->response_data.att_value.handle);

    // Verify that notification is for ANCS service and if true pass to the library for processing
    if ((p_data->response_data.att_value.handle >= ancs_app_state.ancs_s_handle) &&
        (p_data->response_data.att_value.handle < ancs_app_state.ancs_e_handle))
    {
        wiced_bt_ancs_client_process_notification(p_data);
    }
}

/*
 * The library calls this function when it finishes receiving complete ANCS event
 */
void ancs_notification_callback(uint16_t conn_id, ancs_event_t *p_ancs_event)
{
#ifdef TEST_HCI_CONTROL
    // Allocating a buffer to send the trace
    uint8_t  *p_tx_buf = (uint8_t*)wiced_bt_get_buffer(sizeof (ancs_event_t));

    if (p_tx_buf)
    {
        int len;
        p_tx_buf[0] = p_ancs_event->notification_uid & 0xff;
        p_tx_buf[1] = (p_ancs_event->notification_uid >> 8) & 0xff;
        p_tx_buf[2] = (p_ancs_event->notification_uid >> 16) & 0xff;
        p_tx_buf[3] = (p_ancs_event->notification_uid >> 24) & 0xff;
        p_tx_buf[4] = p_ancs_event->command;
        p_tx_buf[5] = p_ancs_event->category;
        p_tx_buf[6] = p_ancs_event->flags;
        len = 7;
        utl_strcpy((char *)&p_tx_buf[len], (char *)p_ancs_event->title);
        len += strlen((const char *)p_ancs_event->title);
        p_tx_buf[len++] = 0;
        utl_strcpy((char *)&p_tx_buf[len], (char *)p_ancs_event->message);
        len += strlen((const char *)p_ancs_event->message);
        p_tx_buf[len++] = 0;
        utl_strcpy((char *)&p_tx_buf[len], (char *)p_ancs_event->positive_action_label);
        len += strlen((const char *)p_ancs_event->positive_action_label);
        p_tx_buf[len++] = 0;
        utl_strcpy((char *)&p_tx_buf[len], (char *)p_ancs_event->negative_action_label);
        len += strlen((const char *)p_ancs_event->negative_action_label);
        p_tx_buf[len++] = 0;
        wiced_transport_send_data(HCI_CONTROL_ANCS_EVENT_NOTIFICATION, p_tx_buf, len);
        wiced_bt_free_buffer(p_tx_buf);
    }
#endif

    WICED_BT_TRACE("ANCS notification UID:%d command:%d category:%d flags:%04x\n", p_ancs_event->notification_uid, p_ancs_event->command, p_ancs_event->category, p_ancs_event->flags);
    WICED_BT_TRACE("title:'%s' message:'%s' positive:'%s' negative:'%s'\n", p_ancs_event->title, p_ancs_event->message, p_ancs_event->positive_action_label, p_ancs_event->negative_action_label);

    wiced_bt_free_buffer(p_ancs_event);
}

/*
 * ANCS server discovery complete
 */
void ancs_discovery_complete_callback(uint16_t conn_id, wiced_bool_t result)
{
    WICED_BT_TRACE("[%s] result:%d\n", __FUNCTION__, result);
#ifdef TEST_HCI_CONTROL
    wiced_transport_send_data(HCI_CONTROL_ANCS_EVENT_SERVICE_FOUND, (uint8_t *)&result, 1);
#endif

    // This app automatically starts the client
    wiced_bt_ancs_client_start(ancs_app_state.conn_id);
}

/*
 * ANCS server start complete
 */
void ancs_start_complete_callback(uint16_t conn_id, wiced_bt_gatt_status_t result)
{
    wiced_result_t rc;

    WICED_BT_TRACE("[%s] result:%d\n", __FUNCTION__, result);
#ifdef TEST_HCI_CONTROL
    wiced_transport_send_data(HCI_CONTROL_ANCS_EVENT_CONNECTED, &result, 1);
#endif
    // Special case when we try to register for notification and we are not paired yet
    if (result == WICED_BT_GATT_INSUF_AUTHENTICATION)
    {
        rc = wiced_bt_dev_sec_bond(ancs_app_state.remote_addr, ancs_app_state.addr_type, BT_TRANSPORT_LE, 0, NULL);
        WICED_BT_TRACE("start bond result:%d\n", rc);
        return;
    }
    if (result == WICED_BT_GATT_SUCCESS)
    {
        ancs_app_state.started = WICED_TRUE;
    }
    UNUSED_VARIABLE(rc);
}

/*
 * ANCS server stop complete
 */
void ancs_stop_complete_callback(uint16_t conn_id, wiced_bt_gatt_status_t result)
{
    WICED_BT_TRACE("[%s] result:%d\n", __FUNCTION__, result);
#ifdef TEST_HCI_CONTROL
    wiced_transport_send_data(HCI_CONTROL_ANCS_EVENT_DISCONNECTED, &result, 1);
#endif
    ancs_app_state.started = WICED_FALSE;
}

#ifdef TEST_HCI_CONTROL
/*
 * Handle received command over UART. Please refer to the WICED HCI Control
 * Protocol for details on the protocol.  The function converts from the WICED
 * HCI remote control commands to the commands expected by the AMS.
 */
uint32_t  ancs_proc_rx_hci_cmd(uint8_t *p_buffer, uint32_t length)
{
    uint16_t                opcode;
    uint8_t*                p_data = p_buffer;
    uint16_t                payload_len;
    uint8_t                 status = HCI_CONTROL_STATUS_SUCCESS;
    wiced_bt_gatt_status_t  gatt_status = WICED_BT_GATT_SUCCESS;

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
        ansc_control_handle_get_version();
    }

    if (ancs_app_state.conn_id == 0)
    {
        WICED_BT_TRACE("no connection\n");
        status = HCI_CONTROL_STATUS_NOT_CONNECTED;
    }
    else
    {
        switch (opcode)
        {
        case HCI_CONTROL_AMS_COMMAND_CONNECT:
            gatt_status = (wiced_bt_ancs_client_start(ancs_app_state.conn_id) ? HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED);
            break;

        case HCI_CONTROL_AMS_COMMAND_DISCONNECT:
            gatt_status = (wiced_bt_ancs_client_stop(ancs_app_state.conn_id) ? HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED);
            break;

        case HCI_CONTROL_ANCS_COMMAND_ACTION:
            gatt_status = wiced_bt_ancs_perform_action(ancs_app_state.conn_id, p_data[0] + (p_data[1] << 8) + (p_data[2] << 16) + (p_data[3] << 24), p_data[4]);
            break;

        default:
            WICED_BT_TRACE("ignored\n", opcode);
            status = HCI_CONTROL_STATUS_UNKNOWN_COMMAND;
            break;
        }
    }
    if (gatt_status != WICED_BT_GATT_SUCCESS)
    {
        status = HCI_CONTROL_STATUS_FAILED;
    }
    wiced_transport_send_data(HCI_CONTROL_ANCS_EVENT_COMMAND_STATUS, &status, 1);

    // Freeing the buffer in which data is received
    wiced_transport_free_buffer(p_buffer);
    UNUSED_VARIABLE(payload_len);
    return HCI_CONTROL_STATUS_SUCCESS;
}
#endif

/*
 * Read keys from the NVRAM and update address resolution database
 */
void ancs_load_keys_to_addr_resolution_db(void)
{
    uint8_t                     bytes_read;
    wiced_result_t              result;
    wiced_bt_device_link_keys_t keys;

    bytes_read = wiced_hal_read_nvram(WICED_NVRAM_VSID_START, sizeof(keys), (uint8_t *)&keys, &result);

    WICED_BT_TRACE(" [%s] read status %d bytes read %d \n", __FUNCTION__, result, bytes_read);

    // if failed to read NVRAM, there is nothing saved at that location
    if (result == WICED_SUCCESS)
    {
#ifdef CYW20706A2
        result = wiced_bt_dev_add_device_to_address_resolution_db(&keys, keys.key_data.ble_addr_type);
#else
        result = wiced_bt_dev_add_device_to_address_resolution_db(&keys);
#endif
    }
    UNUSED_VARIABLE(bytes_read);
}

/*
 * This function is called to save keys generated as a result of pairing or keys update
 */
wiced_bool_t ancs_save_link_keys(wiced_bt_device_link_keys_t *p_keys)
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
wiced_bool_t ancs_read_link_keys(wiced_bt_device_link_keys_t *p_keys)
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
wiced_bool_t ancs_is_bonded(BD_ADDR bd_addr)
{
    wiced_bt_device_link_keys_t keys;

    return ((ancs_read_link_keys(&keys)) &&
            (memcmp(keys.bd_addr, bd_addr, BD_ADDR_LEN) == 0));
}

#ifdef HCI_TRACE_OVER_TRANSPORT
/*
 *  Pass protocol traces up over the transport
 */
void ancs_trace_callback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data)
{
    wiced_transport_send_hci_trace(host_trans_pool, type, length, p_data);
}
#endif

/*
 * This utility copies a character string to another
 */
char *utl_strcpy(char *p_dst, char *p_src)
{
    register char *pd = p_dst;
    register char *ps = p_src;

    while (*ps)
        *pd++ = *ps++;

    *pd++ = 0;

    return (p_dst);
}

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

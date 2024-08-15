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
 * GATT DB Sample Application for 207XX devices.
 *
 * This file implements 207XX embedded application controlled over UART. 
 * This app shows how GATT database can be created and configured via MCU at run time.
 *
 * The sample ClientControl application is provided to show sample
 * MCU implementation.
 *
 * Features demonstrated
 *  - Creating and configuring GATT database at runtime
 *  - Handling of the UART WICED protocol
 *
 *
 * Application Instructions
 *  -  Download the app to your WICED board
 *  -  Start the client control application and open the WICED HCI port
 *  -  From the GATT DB tab, enter the services, characteristics, descriptors
 *  -  Enter advertisement data and click start advert button. 
 *  -  From a peer app such as LightBlue on smart phone, query the services and characteristics
 *
 */
#include "wiced_bt_gatt.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_trace.h"
#include "wiced_platform.h"
#include "wiced_memory.h"
#include "wiced_timer.h"
#include "wiced_transport.h"
#include "sparcommon.h"

#include "hci_control.h"
#include "wiced_app_cfg.h"
#if ( defined(CYW20706A2) || defined(CYW20719B1) || defined(CYW20719B0) || defined(CYW20721B1) || defined(CYW20735B0) || defined(CYW43012C0) )
#include "wiced_bt_app_common.h"
#endif
#include "string.h"
#include "wiced_bt_stack.h"
#ifdef CYW43012C0
#include "wiced_hal_watchdog.h"
#else
#include "wiced_hal_wdog.h"
#endif

/*****************************************************************************
**  Constants
*****************************************************************************/
#define KEY_INFO_POOL_BUFFER_SIZE   145  // Size of the buffer used for holding the peer device key info
#define KEY_INFO_POOL_BUFFER_COUNT  10   // Correspond's to the number of peer devices

#define WICED_PIN_CODE_LEN          4
#define MAX_DEV_NAME_LEN            32

#define DEFAULT_DEV_NAME            "gatt_db"


/*****************************************************************************
**  Structures
*****************************************************************************/
typedef struct
{
    void    *p_next;
    uint16_t nvram_id;
    uint8_t  chunk_len;
    uint8_t  data[1];
} hci_control_nvram_chunk_t;

/******************************************************
 *               Function Declarations
 ******************************************************/
static void     hci_control_transport_status(wiced_transport_type_t type);
static void     hci_control_handle_reset_cmd(void);
static void     hci_control_handle_trace_enable(uint8_t *p_data);
static void     hci_control_device_handle_command(uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len);
static void     hci_control_handle_set_local_bda(uint8_t *p_bda);
static void     hci_control_handle_set_pairability (uint8_t pairing_allowed);
static void     hci_control_handle_read_local_bda(void);
static void     hci_control_handle_user_confirmation(uint8_t *p_bda, uint8_t accept_pairing);
static void     hci_control_handle_read_buffer_stats(void);
static void     hci_control_handle_set_local_name(uint8_t* p_data, uint32_t data_len);
static void     hci_control_send_device_started_evt(void);
static void     hci_control_send_device_error_evt(uint8_t fw_error_code, uint8_t app_error_code);
static void     hci_control_send_pairing_completed_evt(uint8_t status, wiced_bt_device_address_t bdaddr);
static void     hci_control_send_user_confirmation_request_evt(BD_ADDR bda, uint32_t numeric_value);
static void     hci_control_send_encryption_changed_evt(uint8_t encrypted, wiced_bt_device_address_t bdaddr);
static void     hci_control_misc_handle_command(uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len);
static void     hci_control_misc_handle_get_version(void);
static uint32_t hci_control_proc_rx_cmd(uint8_t *p_data, uint32_t length);
static void     hci_control_transport_tx_cplt_cback(wiced_transport_buffer_pool_t* p_pool);
static void     hci_control_init(void);

static wiced_result_t hci_control_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);

/******************************************************
 *               Variables Definitions
 ******************************************************/

char gatt_db_device_name[MAX_DEV_NAME_LEN];

static const uint8_t pincode[WICED_PIN_CODE_LEN] = { 0x30, 0x30, 0x30, 0x30 };

hci_control_nvram_chunk_t *p_nvram_first = NULL;

/* Control block */
#if BTA_DYNAMIC_MEMORY == FALSE
hci_control_cb_t  hci_control_cb;
#endif

wiced_bt_buffer_pool_t* p_key_info_pool;  // Pool for storing the  key info


static const wiced_transport_cfg_t transport_cfg =
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
        .buffer_count = 2
    },
    .p_status_handler = hci_control_transport_status,
    .p_data_handler = hci_control_proc_rx_cmd,
    .p_tx_complete_cback = hci_control_transport_tx_cplt_cback
};

/******************************************************
 *               Function Implementations
 ******************************************************/
#ifndef CYW20735B0
APPLICATION_START()
#else
void application_start( void )
#endif
{
    hci_control_init();
}

/*
 *  hci_control_init
 */
void hci_control_init(void)
{
    int            name_len;
    wiced_result_t result;

    memset(&hci_control_cb, 0, sizeof(hci_control_cb));

    /* Set the initial default device name */
    name_len = strlen(DEFAULT_DEV_NAME);
    memcpy(gatt_db_device_name, DEFAULT_DEV_NAME, name_len);
    gatt_db_device_name[name_len] = 0;

    wiced_transport_init(&transport_cfg);

#ifdef WICED_BT_TRACE_ENABLE
    // Set the debug uart as WICED_ROUTE_DEBUG_NONE to get rid of prints
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_NONE);

    // Set to PUART to see traces on peripheral uart(puart)
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_PUART);
    //wiced_hal_puart_select_uart_pads(WICED_PUART_RXD, WICED_PUART_TXD, 0, 0);

    // Set to HCI to see traces on HCI uart - default if no call to wiced_set_debug_uart()
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_HCI_UART);

    // Use WICED_ROUTE_DEBUG_TO_WICED_UART to send formatted debug strings over the WICED
    // HCI debug interface to be parsed by ClientControl/BtSpy
    // Note: WICED HCI must be configured to use this - see wiced_trasnport_init(), must 
    // be called with wiced_transport_cfg_t.wiced_tranport_data_handler_t callback present
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#endif

    WICED_BT_TRACE("%s: APP START: dev_name = %s\n", __func__, gatt_db_device_name);

    wiced_bt_stack_init(hci_control_management_callback, &wiced_bt_cfg_settings, wiced_app_cfg_buf_pools);
}

/*
 * This callback function is called when the MCU opens the Wiced UART
 */
void hci_control_transport_status(wiced_transport_type_t type)
{
    WICED_BT_TRACE("%s: type = %d\n", __func__, type);

    // Tell Host that App is started
    hci_control_send_device_started_evt();
}

/*
 *  Process all HCI packet from
 */
void hci_control_hci_packet_cback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data)
{
    // send the trace
    wiced_transport_send_hci_trace(NULL, type, length, p_data);
}

/*
 *  Management callback receives various notifications from the stack
 */
wiced_result_t hci_control_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t                     result = WICED_BT_SUCCESS;
    wiced_bt_dev_status_t              dev_status;
    wiced_bt_dev_ble_pairing_info_t    *p_pairing_info;
    wiced_bt_dev_encryption_status_t   *p_encryption_status;
    int                                bytes_written, bytes_read;
    int                                nvram_id;
    wiced_bt_power_mgmt_notification_t *p_power_mgmt_notification;
    wiced_bt_dev_pairing_cplt_t        *p_pairing_cmpl;
    uint8_t                            pairing_result;

#ifdef TRACE_VERBOSE
    static const char* evt[] =
    {
        "ENABLED",
        "DISABLED",
        "POWER_MANAGEMENT_STATUS",
        "PIN_REQUEST",
        "USER_CONFIRMATION_REQUEST",
        "PASSKEY_NOTIFICATION",
        "PASSKEY_REQUEST",
        "KEYPRESS_NOTIFICATION",
        "PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST",
        "PAIRING_IO_CAPABILITIES_BR_EDR_RESPONSE",
        "PAIRING_IO_CAPABILITIES_BLE_REQUEST",
        "PAIRING_COMPLETE",
        "ENCRYPTION_STATUS",
        "SECURITY_REQUEST",
        "SECURITY_FAILED",
        "SECURITY_ABORTED",
        "READ_LOCAL_OOB_DATA_COMPLETE",
        "REMOTE_OOB_DATA_REQUEST",
        "PAIRED_DEVICE_LINK_KEYS_UPDATE",
        "PAIRED_DEVICE_LINK_KEYS_REQUEST",
        "LOCAL_IDENTITY_KEYS_UPDATE",
        "LOCAL_IDENTITY_KEYS_REQUEST",
        "BLE_SCAN_STATE_CHANGED",
        "BLE_ADVERT_STATE_CHANGED",
        "SMP_REMOTE_OOB_DATA_REQUEST",
        "SMP_SC_REMOTE_OOB_DATA_REQUEST",
        "SMP_SC_LOCAL_OOB_DATA_NOTIFICATION",
        "SCO_CONNECTED",
        "SCO_DISCONNECTED",
        "SCO_CONNECTION_REQUEST",
        "SCO_CONNECTION_CHANGE",
        "BLE_CONNECTION_PARAM_UPDATE"
    };

    WICED_BT_TRACE("%s: event[%d] = %s\n", __func__, event, evt[event]);
#else
    WICED_BT_TRACE("%s: event:%d\n", __func__, event);
#endif
    switch (event)
    {
    case BTM_ENABLED_EVT:  /* Bluetooth  stack enabled */
#if !defined(CYW20735B1) && !defined(CYW20819A1)
        wiced_bt_app_init();
#endif
        /* Disable pairing on startup */
        wiced_bt_set_pairable_mode(0, 0);

        hci_control_le_enable();

        // Disable while streaming audio over the uart.
        wiced_bt_dev_register_hci_trace(hci_control_hci_packet_cback);

        // Creating a buffer pool for holding the peer devices's key info
        p_key_info_pool = (wiced_bt_buffer_pool_t*) wiced_bt_create_pool(KEY_INFO_POOL_BUFFER_SIZE,
                    KEY_INFO_POOL_BUFFER_COUNT);
        if (p_key_info_pool == NULL)
        {
            WICED_BT_TRACE("Err: wiced_bt_create_pool failed\n");
        }

        // Tell Host that App is started
        hci_control_send_device_started_evt();
        break;

    case BTM_DISABLED_EVT:
        hci_control_send_device_error_evt(0, 0);
        break;

    case BTM_PIN_REQUEST_EVT:
        WICED_BT_TRACE("remote address = %B\n", p_event_data->pin_request.bd_addr);
        wiced_bt_dev_pin_code_reply(*p_event_data->pin_request.bd_addr, WICED_BT_SUCCESS, WICED_PIN_CODE_LEN, (uint8_t*)pincode);
        break;

    case BTM_USER_CONFIRMATION_REQUEST_EVT:
        // If this is just works pairing, accept. Otherwise send event to the MCU to confirm the same value.
        if (p_event_data->user_confirmation_request.just_works)
        {
            wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr);
        }
        else
        {
            hci_control_send_user_confirmation_request_evt(p_event_data->user_confirmation_request.bd_addr, p_event_data->user_confirmation_request.numeric_value);
        }
        break;

    case BTM_PASSKEY_NOTIFICATION_EVT:
        WICED_BT_TRACE("PassKey Notification. BDA %B, Key %d\n", p_event_data->user_passkey_notification.bd_addr, p_event_data->user_passkey_notification.passkey);
        hci_control_send_user_confirmation_request_evt(p_event_data->user_passkey_notification.bd_addr, p_event_data->user_passkey_notification.passkey);
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:
        /* Use the default security for BR/EDR*/
        WICED_BT_TRACE("BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT bda %B\n", p_event_data->pairing_io_capabilities_br_edr_request.bd_addr);
        p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap = BTM_IO_CAPABILITIES_DISPLAY_AND_YES_NO_INPUT;
        p_event_data->pairing_io_capabilities_br_edr_request.auth_req     = BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_NO;
        p_event_data->pairing_io_capabilities_br_edr_request.oob_data     = WICED_FALSE;
        p_event_data->pairing_io_capabilities_br_edr_request.auth_req     = BTM_AUTH_ALL_PROFILES_NO;
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
        /* Use the default security for BLE */
        WICED_BT_TRACE("BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT bda %B\n",
                    p_event_data->pairing_io_capabilities_ble_request.bd_addr);
        p_event_data->pairing_io_capabilities_ble_request.local_io_cap  = BTM_IO_CAPABILITIES_DISPLAY_AND_YES_NO_INPUT;
        p_event_data->pairing_io_capabilities_ble_request.oob_data      = BTM_OOB_NONE;
        p_event_data->pairing_io_capabilities_ble_request.auth_req      = BTM_LE_AUTH_REQ_SC_MITM_BOND;
        p_event_data->pairing_io_capabilities_ble_request.max_key_size  = 16;
        p_event_data->pairing_io_capabilities_ble_request.init_keys     = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
        p_event_data->pairing_io_capabilities_ble_request.resp_keys     = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
        break;

    case BTM_PAIRING_COMPLETE_EVT:
        p_pairing_cmpl = &p_event_data->pairing_complete;
        if (p_pairing_cmpl->transport == BT_TRANSPORT_BR_EDR)
        {
            pairing_result = p_pairing_cmpl->pairing_complete_info.br_edr.status;
        }
        else
        {
            pairing_result = p_pairing_cmpl->pairing_complete_info.ble.reason;
        }
        WICED_BT_TRACE("Pairing Result: %d\n", pairing_result);
        hci_control_send_pairing_completed_evt(pairing_result, p_event_data->pairing_complete.bd_addr);
        break;

    case BTM_ENCRYPTION_STATUS_EVT:
        p_encryption_status = &p_event_data->encryption_status;
        WICED_BT_TRACE("Encryption Status:(%B) res:%d\n", p_encryption_status->bd_addr, p_encryption_status->result);
        hci_control_send_encryption_changed_evt(p_encryption_status->result, p_encryption_status->bd_addr);
        break;

    case BTM_SECURITY_REQUEST_EVT:
        WICED_BT_TRACE("Security Request Event, Pairing allowed %d\n", hci_control_cb.pairing_allowed);
        if (hci_control_cb.pairing_allowed)
        {
            wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr, WICED_BT_SUCCESS);
        }
        else
        {
            // Pairing not allowed, return error
            result = WICED_BT_ERROR;
        }
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
        /* Check if we already have information saved for this bd_addr */
        if ((nvram_id = hci_control_find_nvram_id(p_event_data->paired_device_link_keys_update.bd_addr, BD_ADDR_LEN)) == 0)
        {
            // This is the first time, allocate id for the new memory chunk
            nvram_id = hci_control_alloc_nvram_id();
            WICED_BT_TRACE("Allocated NVRAM ID:%d\n", nvram_id);
        }
        bytes_written = hci_control_write_nvram(nvram_id, sizeof(wiced_bt_device_link_keys_t), &p_event_data->paired_device_link_keys_update, WICED_FALSE);

        WICED_BT_TRACE("NVRAM write:id:%d bytes:%d dev: [%B]\n", nvram_id, bytes_written, p_event_data->paired_device_link_keys_update.bd_addr);
        break;

    case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        /* read existing key from the NVRAM  */
        WICED_BT_TRACE("\t\tfind device %B\n", p_event_data->paired_device_link_keys_request.bd_addr);

        if ((nvram_id = hci_control_find_nvram_id(p_event_data->paired_device_link_keys_request.bd_addr, BD_ADDR_LEN)) != 0)
        {
            bytes_read = hci_control_read_nvram(nvram_id, &p_event_data->paired_device_link_keys_request, sizeof(wiced_bt_device_link_keys_t));

            result = WICED_BT_SUCCESS;
            WICED_BT_TRACE("Read:nvram_id:%d bytes:%d\n", nvram_id, bytes_read);
        }
        else
        {
            result = WICED_BT_ERROR;
            WICED_BT_TRACE("Key retrieval failure\n");
        }
        break;

    case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
        /* Request to store newly generated local identity keys to NVRAM */
        /* (sample app does not store keys to NVRAM) */
        break;

    case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
        /*
         * Request to restore local identity keys from NVRAM
         * (requested during Bluetooth start up)
         * */
        /* (sample app does not store keys to NVRAM)
         * New local identity keys will be generated
         * */
        result = WICED_BT_NO_RESOURCES;
        break;

    case BTM_BLE_SCAN_STATE_CHANGED_EVT:
        hci_control_le_scan_state_changed(p_event_data->ble_scan_state_changed);
        break;

    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
        hci_control_le_advert_state_changed(p_event_data->ble_advert_state_changed);
        break;

    case BTM_POWER_MANAGEMENT_STATUS_EVT:
        p_power_mgmt_notification = &p_event_data->power_mgmt_notification;
        WICED_BT_TRACE("Power mgmt status event: bd (%B) status:%d hci_status:%d\n",
                       p_power_mgmt_notification->bd_addr,
                       p_power_mgmt_notification->status,
                       p_power_mgmt_notification->hci_status);
        break;

    default:
        result = WICED_BT_USE_DEFAULT_SECURITY;
#ifdef TRACE_VERBOSE
        WICED_BT_TRACE("%s: skip event:0x%x = %s\n", __func__, event, evt[event]);
#else
        WICED_BT_TRACE("%s: skip event:0x%x\n", __func__, event);
#endif
    }
    return result;
}

#ifdef TRACE_VERBOSE
static char* group_str(uint16_t opcode)
{
    static char* str[] =
    {
        "GROUP_DEVICE",
        "GROUP_LE",
        "GROUP_GATT",
        "GROUP_HF",
        "GROUP_SPP",
        "GROUP_AUDIO",
        "GROUP_HIDD",
        "GROUP_AVRC_TARGET",
        "GROUP_TEST",
        "GROUP_ANCS",
        "GROUP_ALERT",
        "GROUP_IAP2",
        "GROUP_AG",
        "GROUP_LN",
        "GROUP_BSG",
        "GROUP_AVRC_CONTROLLER",
        "GROUP_AMS",
        "GROUP_HIDH",
        "GROUP_AUDIO_SINK",
        "GROUP_PBC",
        "GROUP_MESH",
        "GROUP_BATT_CLIENT",
        "GROUP_FINDME_LOCATOR",
        "GROUP_DEMO",
        "GROUP_ZB_FIRST",
        "GROUP_ZB_LAST",
        "GROUP_MISC"
    };

    opcode = (opcode >> 8) & 0xFF;

    if (opcode <= 0x19)
    {
        return str[opcode];
    }
    else if (opcode == 0x80)
    {
        return str[0x20];
    }
    else if (opcode == 0x9F)
    {
        return str[0x21];
    }
    else if (opcode == 0xFF)
    {
        return str[0x22];
    }
    return NULL;
}

static char* cmd_str(uint16_t cmd_opcode)
{
    uint16_t cmd_group = (cmd_opcode >> 8) & 0xFF;

    static char* device_str[] =
    {
        "DEVICE_COMMAND_RESET",
        "DEVICE_COMMAND_TRACE_ENABLE",
        "DEVICE_COMMAND_SET_LOCAL_BDA",
        "DEVICE_COMMAND_SET_BAUD_RATE",
        "DEVICE_COMMAND_PUSH_NVRAM_DATA",
        "DEVICE_COMMAND_DELETE_NVRAM_DATA",
        "DEVICE_COMMAND_INQUIRY",
        "DEVICE_COMMAND_SET_VISIBILITY",
        "DEVICE_COMMAND_SET_PAIRING_MODE",
        "DEVICE_COMMAND_UNBOND",
        "DEVICE_COMMAND_USER_CONFIRMATION",
        "DEVICE_COMMAND_ENABLE_COEX",
        "DEVICE_COMMAND_DISABLE_COEX",
        "DEVICE_COMMAND_SET_BATTERY_LEVEL",
        "DEVICE_COMMAND_READ_LOCAL_BDA",
        "DEVICE_COMMAND_BOND",
        "DEVICE_COMMAND_READ_BUFF_STATS",
        "DEVICE_COMMAND_SET_LOCAL_NAME"
    };

    static char* le_str[] =
    {
        "LE_COMMAND_SCAN",
        "LE_COMMAND_ADVERTISE",
        "LE_COMMAND_CONNECT",
        "LE_COMMAND_CANCEL_CONNECT",
        "LE_COMMAND_DISCONNECT",
        "LE_RE_PAIR",
        "LE_COMMAND_GET_IDENTITY_ADDRESS",
        "LE_COMMAND_SET_CHANNEL_CLASSIFICATION",
        "LE_COMMAND_SET_CONN_PARAMS",
        "LE_COMMAND_SET_RAW_ADVERTISE_DATA",
        "LE_COMMAND_SECURITY_GRANT",
    };

    static char* gatt_str[] =
    {
        "GATT_COMMAND_DISCOVER_SERVICES",
        "GATT_COMMAND_DISCOVER_CHARACTERISTICS",
        "GATT_COMMAND_DISCOVER_DESCRIPTORS",
        "GATT_COMMAND_READ_REQUEST",
        "GATT_COMMAND_READ_RESPONSE",
        "GATT_COMMAND_WRITE_COMMAND",
        "GATT_COMMAND_WRITE_REQUEST",
        "GATT_COMMAND_WRITE_RESPONSE",
        "GATT_COMMAND_NOTIFY",
        "GATT_COMMAND_INDICATE",
        "GATT_COMMAND_INDICATE_CONFIRM",
        "GATT_COMMAND_REGISTER",
        "GATT_COMMAND_DB_INIT",
        "GATT_COMMAND_DB_PRIMARY_SERVICE_ADD",
        "GATT_COMMAND_DB_SECONDARY_SERVICE_ADD",
        "GATT_COMMAND_DB_INCLUDED_SERVICE_ADD",
        "GATT_COMMAND_DB_CHARACTERISTIC_ADD",
        "GATT_COMMAND_DB_DESCRIPTOR_ADD"
    };

    static char* misc_str[] =
    {
        "MISC_COMMAND_PING",
        "MISC_COMMAND_GET_VERSION"
    };

    if (cmd_group == HCI_CONTROL_GROUP_DEVICE)
    {
        return device_str[(cmd_opcode & 0xFF) - 1];
    }
    else if (cmd_group == HCI_CONTROL_GROUP_LE)
    {
        return le_str[(cmd_opcode & 0xFF) - 1];
    }
    else if (cmd_group == HCI_CONTROL_GROUP_GATT)
    {
        return gatt_str[(cmd_opcode & 0xFF) - 1];
    }
    else if (cmd_group == HCI_CONTROL_GROUP_MISC)
    {
        return misc_str[(cmd_opcode & 0xFF) - 1];
    }
    return NULL;
}
#endif

/*
 * Handle received command over UART. Please refer to the WICED Smart Ready
 * Software User Manual (WICED-Smart-Ready-SWUM100-R) for details on the
 * HCI UART control protocol.
*/
uint32_t hci_control_proc_rx_cmd(uint8_t *p_buffer, uint32_t length)
{
    uint16_t opcode;
    uint16_t payload_len;
    uint8_t *p_data = p_buffer;
    uint8_t  buffer_processed = WICED_TRUE;

    if (!p_buffer)
    {
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    //Expected minimum 4 byte as the wiced header
    if (length < 4)
    {
        WICED_BT_TRACE("invalid params\n");
        wiced_transport_free_buffer(p_buffer);
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    STREAM_TO_UINT16(opcode, p_data);       // Get OpCode
    STREAM_TO_UINT16(payload_len, p_data);  // Gen Payload Length
#ifdef TRACE_VERBOSE
    WICED_BT_TRACE("%s: opcode[0x%04x] = %s / %s, len = %d\n",
                   __func__, opcode, group_str(opcode), cmd_str(opcode), payload_len);
#else
    WICED_BT_TRACE("%s: opcode:0x%04x len:%d\n", __func__, opcode, payload_len);
#endif
    switch ((opcode >> 8) & 0xff)
    {
    case HCI_CONTROL_GROUP_DEVICE:
        hci_control_device_handle_command(opcode, p_data, payload_len);
        break;

    case HCI_CONTROL_GROUP_LE:
    case HCI_CONTROL_GROUP_GATT:
        hci_control_le_handle_command(opcode, p_data, payload_len);
        break;

    case HCI_CONTROL_GROUP_MISC:
        hci_control_misc_handle_command(opcode, p_data, payload_len);
        break;

    default:
#if TRACE_VERBOSE
        WICED_BT_TRACE("%s: skipped opcode[0x%04x] = %s, len = %d\n",
                       __func__, opcode, group_str(opcode), payload_len);
#else
        WICED_BT_TRACE("%s: skipped opcode:0x%04x len:%d\n", __func__, opcode, payload_len);
#endif
        break;
    }
    if (buffer_processed)
    {
        // Freeing the buffer in which data is received
        wiced_transport_free_buffer(p_buffer);
    }

    return HCI_CONTROL_STATUS_SUCCESS;
}

/*
 *  Handle the device group commands received by MCU
 */
void hci_control_device_handle_command(uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len)
{
    uint8_t bytes_written;
    switch(cmd_opcode)
    {
    case HCI_CONTROL_COMMAND_RESET:
        hci_control_handle_reset_cmd();
        break;

    case HCI_CONTROL_COMMAND_TRACE_ENABLE:
        hci_control_handle_trace_enable(p_data);
        break;

    case HCI_CONTROL_COMMAND_SET_LOCAL_BDA:
        hci_control_handle_set_local_bda(p_data);
        break;

    case HCI_CONTROL_COMMAND_PUSH_NVRAM_DATA:
        bytes_written = hci_control_write_nvram(p_data[0] | (p_data[1] << 8), data_len - 2, &p_data[2], WICED_TRUE);
        WICED_BT_TRACE("NVRAM write: %d dev: [%B]\n", bytes_written , &p_data[2]);
        break;

    case HCI_CONTROL_COMMAND_DELETE_NVRAM_DATA:
        hci_control_delete_nvram(p_data[0] | (p_data[1] << 8), WICED_TRUE);
        WICED_BT_TRACE("NVRAM delete: %d\n", p_data[0] | (p_data[1] << 8));
        break;

    case  HCI_CONTROL_COMMAND_SET_PAIRING_MODE:
        hci_control_handle_set_pairability(p_data[0]);
        break;

    case HCI_CONTROL_COMMAND_READ_LOCAL_BDA:
        hci_control_handle_read_local_bda();
        break;

    case HCI_CONTROL_COMMAND_USER_CONFIRMATION:
        hci_control_handle_user_confirmation(p_data, p_data[6]);
        break;

    case HCI_CONTROL_COMMAND_READ_BUFF_STATS:
        hci_control_handle_read_buffer_stats();
        break;

    case HCI_CONTROL_COMMAND_SET_LOCAL_NAME:
        hci_control_handle_set_local_name(p_data, data_len);
        break;

    default:
        WICED_BT_TRACE("??? Unknown command code\n");
        break;
    }
}

/*
 * handle reset command from UART
 */
void hci_control_handle_reset_cmd(void)
{
    hci_control_send_command_status_evt(HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS);

    // trig watchdog now.
    wiced_hal_wdog_reset_system();
}

/*
 * handle command from UART to configure traces
 */
void hci_control_handle_trace_enable(uint8_t *p_data)
{
    uint8_t hci_trace_enable = *p_data++;
    wiced_debug_uart_types_t route_debug = (wiced_debug_uart_types_t)*p_data;

    WICED_BT_TRACE("HCI Traces:%d DebugRoute:%d\n", hci_trace_enable, route_debug);

    if (hci_trace_enable)
    {
        /* Register callback for receiving hci traces */
        // Disable while streaming audio over the uart.
        wiced_bt_dev_register_hci_trace(hci_control_hci_packet_cback);
    }
    else
    {
        wiced_bt_dev_register_hci_trace(NULL);
    }
    WICED_BT_TRACE("%s: route debug...\n", __func__);

    /* If using SpyLite, keep the line below.
     * If using PUART only, disable the line below.
     */
    wiced_set_debug_uart(route_debug);

    hci_control_send_command_status_evt(HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS);
}

/*
 * handle command to set local Bluetooth device address
 */
void hci_control_handle_set_local_bda(uint8_t *p_bda)
{
    BD_ADDR bd_addr;

    STREAM_TO_BDADDR(bd_addr,p_bda);
    WICED_BT_TRACE("%s: bd_addr = %B\n", __func__, bd_addr);

    wiced_bt_set_local_bdaddr(bd_addr, BLE_ADDR_PUBLIC);

    hci_control_send_command_status_evt(HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS);
}

/*
 *  Handle read buffer statistics
 */
void hci_control_handle_read_buffer_stats(void)
{
    uint16_t                     i;
    wiced_result_t               result;
    wiced_bt_buffer_statistics_t buff_stats[wiced_bt_get_number_of_buffer_pools()];

    result = wiced_bt_get_buffer_usage(buff_stats, sizeof(buff_stats));

    if (result == WICED_BT_SUCCESS)
    {
        // Print out the stats to trace
        WICED_BT_TRACE("Buffer usage statistics:\n");
        for (i = 0; i < wiced_bt_get_number_of_buffer_pools(); i++) {
            WICED_BT_TRACE("pool_id:%d size:%d curr_cnt:%d max_cnt:%d total:%d\n",
                           buff_stats[i].pool_id, buff_stats[i].pool_size,
                           buff_stats[i].current_allocated_count, buff_stats[i].max_allocated_count,
                           buff_stats[i].total_count);
        }

        // Return the stats via WICED-HCI
        wiced_transport_send_data(HCI_CONTROL_EVENT_READ_BUFFER_STATS, (uint8_t*)&buff_stats, sizeof(buff_stats));
    }
    else
    {
        hci_control_send_command_status_evt(HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_FAILED);
    }
}

/*
 * handle command to set local Bluetooth device name
 */
void hci_control_handle_set_local_name(uint8_t* p_data, uint32_t data_len)
{
    if (data_len >= MAX_DEV_NAME_LEN)
    {
        data_len = MAX_DEV_NAME_LEN - 1;
    }
    memcpy(gatt_db_device_name, p_data, data_len);
    gatt_db_device_name[data_len] = 0;

    WICED_BT_TRACE("%s: dev_name = %s\n", __func__, gatt_db_device_name);

#ifdef CYW20706A2
    wiced_bt_dev_set_local_name(gatt_db_device_name);
#endif

    hci_control_send_command_status_evt(HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS);
}

/*
 *  Handle Set Pairability command received over UART
 */
void hci_control_handle_set_pairability(uint8_t pairing_allowed)
{
    uint8_t                   status = HCI_CONTROL_STATUS_SUCCESS;
    hci_control_nvram_chunk_t *p1 = NULL;

    if (hci_control_cb.pairing_allowed != pairing_allowed)
    {
        if (pairing_allowed)
        {
            // Check if key buffer pool has buffer available. If not, cannot enable pairing until nvram entries are deleted
#ifndef CYW20735B0
            if (wiced_bt_get_buffer_count(p_key_info_pool) != 0)
#else
            if ( ( p1 = ( hci_control_nvram_chunk_t * )wiced_bt_get_buffer_from_pool( p_key_info_pool ) ) == NULL)
#endif
            {
                WICED_BT_TRACE("Err: No more memory for Pairing\n");
                pairing_allowed = 0; //The key buffer pool is full therefore we cannot allow pairing to be enabled
                status = HCI_CONTROL_STATUS_OUT_OF_MEMORY;
            }
            else
            {
                wiced_bt_free_buffer(p1);
            }
        }

        hci_control_cb.pairing_allowed = pairing_allowed;
        wiced_bt_set_pairable_mode(hci_control_cb.pairing_allowed, 0);
        WICED_BT_TRACE(" Set the pairing allowed to %d \n", hci_control_cb.pairing_allowed);
    }

    hci_control_send_command_status_evt(HCI_CONTROL_EVENT_COMMAND_STATUS, status);
}

/*
 *  Handle Get Local BDA command received over UART
 */
void hci_control_handle_read_local_bda(void)
{
    wiced_bt_device_address_t bda = { 0 };

    wiced_bt_dev_read_local_addr(bda);
    WICED_BT_TRACE("Local BT address: [%B]\n", bda);

    wiced_transport_send_data(HCI_CONTROL_EVENT_READ_LOCAL_BDA, (uint8_t*)bda , 6);
}

/*
 *  Handle User Confirmation received over UART
 */
void hci_control_handle_user_confirmation(uint8_t *p_bda, uint8_t accept_pairing)
{
    wiced_bt_device_address_t bd_addr;

    STREAM_TO_BDADDR(bd_addr,p_bda);
    wiced_bt_dev_confirm_req_reply(accept_pairing == WICED_TRUE ? WICED_BT_SUCCESS : WICED_BT_ERROR, bd_addr);

    hci_control_send_command_status_evt(HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS);
}

/*
 *  Send Device Started event through UART
 */
void hci_control_send_device_started_evt(void)
{
    wiced_transport_send_data(HCI_CONTROL_EVENT_DEVICE_STARTED, NULL, 0);

    WICED_BT_TRACE("maxLinks:%d maxChannels:%d maxpsm:%d rfcom max links%d, rfcom max ports:%d\n",
                   wiced_bt_cfg_settings.l2cap_application.max_links,
                   wiced_bt_cfg_settings.l2cap_application.max_channels,
                   wiced_bt_cfg_settings.l2cap_application.max_psm,
                   wiced_bt_cfg_settings.rfcomm_cfg.max_links,
                   wiced_bt_cfg_settings.rfcomm_cfg.max_ports);
}

/*
 *  Send Device Error event through UART
 */
void hci_control_send_device_error_evt(uint8_t fw_error_code, uint8_t app_error_code)
{
    uint8_t event_data[] = { 0, 0 };

    event_data[0] = app_error_code;
    event_data[1] = fw_error_code;

    WICED_BT_TRACE("[hci_control_send_device_error_evt] app_error_code=0x%02x fw_error_code=0x%02x\n", event_data[0], event_data[1]);

    wiced_transport_send_data(HCI_CONTROL_EVENT_DEVICE_ERROR, event_data, 2);
}

/*
 * transfer command status event to UART
 */
void hci_control_send_command_status_evt(uint16_t code, uint8_t status)
{
    wiced_transport_send_data(code, &status, 1);
}

/*
 *  Send Pairing Completed event through UART
 */
void hci_control_send_pairing_completed_evt(uint8_t status , wiced_bt_device_address_t bdaddr)
{
    int       i;
    const int cmd_size = BD_ADDR_LEN + sizeof(uint8_t);
    uint8_t   event_data[cmd_size];
    int       cmd_bytes = 0;

    event_data[cmd_bytes++] = status;

    for (i = 0 ; i < BD_ADDR_LEN; i++)
    {
        event_data[cmd_bytes++] = bdaddr[BD_ADDR_LEN - 1 - i];
    }
    WICED_BT_TRACE("pairing complete evt: %B as %B status %d\n", bdaddr, &event_data[1], status);

    wiced_transport_send_data(HCI_CONTROL_EVENT_PAIRING_COMPLETE, event_data, cmd_bytes);
}

/*
 *  Send User Confirmation Request event through UART
 */
void hci_control_send_user_confirmation_request_evt(BD_ADDR bda, uint32_t numeric_value)
{
    uint8_t buf[10];
    uint8_t *p = &buf[6];
    memcpy(buf, bda, BD_ADDR_LEN);
    *p++ = numeric_value & 0xff;
    *p++ = (numeric_value >> 8) & 0xff;
    *p++ = (numeric_value >> 16) & 0xff;
    *p++ = (numeric_value >> 24) & 0xff;
    wiced_transport_send_data(HCI_CONTROL_EVENT_USER_CONFIRMATION, buf, 10);
}

/*
 *  Send Encryption Changed event through UART
 */
void hci_control_send_encryption_changed_evt(uint8_t encrypted, wiced_bt_device_address_t bdaddr)
{
    int       i;
    const int cmd_size = BD_ADDR_LEN + sizeof(uint8_t);
    uint8_t   event_data[cmd_size];
    int       cmd_bytes = 0;

    event_data[cmd_bytes++] = encrypted;
    for (i = 0 ; i < BD_ADDR_LEN; i++)
    {
        event_data[cmd_bytes++] = bdaddr[BD_ADDR_LEN - 1 - i];
    }

    wiced_transport_send_data(HCI_CONTROL_EVENT_ENCRYPTION_CHANGED, event_data, cmd_bytes);
}


/*
 * Read NVRAM actually finds the memory chunk in the RAM
 */
int hci_control_read_nvram(int nvram_id, void *p_data, int data_len)
{
    hci_control_nvram_chunk_t *p1;
    int                        data_read = 0;

    /* Go through the linked list of chunks */
    for (p1 = p_nvram_first; p1 != NULL; p1 = p1->p_next)
    {
        if (p1->nvram_id == nvram_id)
        {
            data_read = (data_len < p1->chunk_len) ? data_len : p1->chunk_len;
            memcpy(p_data, p1->data, data_read);
            break;
        }
    }
    return (data_read);
}

/*
 * Write NVRAM function is called to store information in the RAM.  This can be called when
 * stack requires persistent storage, for example to save link keys.  In this case
 * data is also formatted and send to the host for real NVRAM storage.  The same function is
 * called when host pushes NVRAM chunks during the startup.  Parameter from_host in this
 * case is set to FALSE indicating that data does not need to be forwarded.
 */
int hci_control_write_nvram(int nvram_id, int data_len, void *p_data, wiced_bool_t from_host)
{
    uint8_t                     tx_buf[257];
    uint8_t                     *p = tx_buf;
    hci_control_nvram_chunk_t   *p1;
    wiced_result_t              result;
    wiced_bt_device_link_keys_t *p_keys;

    /* First check if this ID is being reused and release the memory chunk */
    hci_control_delete_nvram(nvram_id, WICED_FALSE);

    /* Allocating a buffer from the pool created for storing the peer info */
    if ((p1 = (hci_control_nvram_chunk_t*)wiced_bt_get_buffer_from_pool(p_key_info_pool)) == NULL)
    {
        WICED_BT_TRACE("Failed to alloc:%d\n", data_len);
        return 0;
    }

    if (wiced_bt_get_buffer_size(p1) < (sizeof(hci_control_nvram_chunk_t) + data_len - 1))
    {
        WICED_BT_TRACE("Insufficient buffer size, Buff Size %d, Len %d  \n",
                       wiced_bt_get_buffer_size(p1),
                       (sizeof(hci_control_nvram_chunk_t) + data_len - 1));
        wiced_bt_free_buffer(p1);
        return 0;
    }

    p1->p_next    = p_nvram_first;
    p1->nvram_id  = nvram_id;
    p1->chunk_len = data_len;
    memcpy(p1->data, p_data, data_len);

    p_nvram_first = p1;

    p_keys = (wiced_bt_device_link_keys_t*)p_data;
#ifdef CYW20706A2
    result = wiced_bt_dev_add_device_to_address_resolution_db(p_keys,
            p_keys->key_data.ble_addr_type);
#else
    result = wiced_bt_dev_add_device_to_address_resolution_db(p_keys);
#endif

    WICED_BT_TRACE("Updated Addr Resolution DB:%d\n", result);

    // If NVRAM chunk arrived from host, no need to send it back, otherwise send over transport
    if (!from_host)
    {
        *p++ = nvram_id & 0xff;
        *p++ = (nvram_id >> 8) & 0xff;
        memcpy(p, p_data, data_len);

        wiced_transport_send_data(HCI_CONTROL_EVENT_NVRAM_DATA, tx_buf, (int)(data_len + 2));
    }
    else
    {
        hci_control_send_command_status_evt(HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS);
    }
    return data_len;
}

/*
 * Find nvram_id of the NVRAM chunk with first bytes matching specified byte array
 */
int hci_control_find_nvram_id(uint8_t *p_data, int len)
{
    hci_control_nvram_chunk_t *p1;

    /* Go through the linked list of chunks */
    for (p1 = p_nvram_first; p1 != NULL; p1 = (hci_control_nvram_chunk_t*)p1->p_next)
    {
        WICED_BT_TRACE("find %B %B len:%d\n", p1->data, p_data, len);
        if (memcmp(p1->data, p_data, len) == 0)
        {
            return (p1->nvram_id);
        }
    }
    return HCI_CONTROL_INVALID_NVRAM_ID;
}

/*
 * Delete NVRAM function is called when host deletes NVRAM chunk from the persistent storage.
 */
void hci_control_delete_nvram(int nvram_id, wiced_bool_t from_host)
{
    hci_control_nvram_chunk_t *p1;
    hci_control_nvram_chunk_t *p2;

    /* If Delete NVRAM data command arrived from host, send a Command Status response to ack command */
    if (from_host)
    {
        hci_control_send_command_status_evt(HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS);
    }

    if (p_nvram_first == NULL)
    {
        return;
    }

    /* Special case when need to remove the first chunk */
    if (p_nvram_first != NULL && p_nvram_first->nvram_id == nvram_id)
    {
        p1 = p_nvram_first;
        if (from_host && (wiced_bt_dev_delete_bonded_device (p1->data) == WICED_ERROR))
        {
            WICED_BT_TRACE("ERROR: while Unbonding device\n");
        }
        else
        {
            p_nvram_first = (hci_control_nvram_chunk_t *)p_nvram_first->p_next;
            wiced_bt_free_buffer(p1);
        }
        return;
    }

    /* Go through the linked list of chunks */
    for (p1 = p_nvram_first; p1 != NULL; p1 = (hci_control_nvram_chunk_t*)p1->p_next)
    {
        p2 = (hci_control_nvram_chunk_t*)p1->p_next;

        if (p2 != NULL && p2->nvram_id == nvram_id)
        {
            if (from_host && (wiced_bt_dev_delete_bonded_device(p2->data) == WICED_ERROR))
            {
                WICED_BT_TRACE("ERROR: while Unbonding device\n");
            }
            else
            {
                p1->p_next = p2->p_next;
                wiced_bt_free_buffer(p2);
            }
            return;
        }
    }
}

/*
 * Allocate nvram_id to save new NVRAM chunk
 */
int hci_control_alloc_nvram_id(void)
{
    hci_control_nvram_chunk_t *p1 = p_nvram_first;
    int                        nvram_id;
    uint8_t                    allocated_key_pool_count;

    /* Go through the linked list of chunks */
    WICED_BT_TRACE("hci_control_alloc_nvram_id\n");
    for (nvram_id = HCI_CONTROL_FIRST_VALID_NVRAM_ID; p1 != NULL; nvram_id++)
    {
        allocated_key_pool_count = 1;

        for (p1 = p_nvram_first; p1 != NULL; p1 = (hci_control_nvram_chunk_t*)p1->p_next)
        {
            /* If the key buffer pool is becoming full, we need notify the MCU and disable
             * pairing. The MCU need delete some nvram entries and enable pairing in order
             * to pair with more devices.
             */
            allocated_key_pool_count++;
            if (allocated_key_pool_count == KEY_INFO_POOL_BUFFER_COUNT && hci_control_cb.pairing_allowed)
            {
                // Send Max Number of Paired Devices Reached event message
                wiced_transport_send_data(HCI_CONTROL_EVENT_MAX_NUM_OF_PAIRED_DEVICES_REACHED, NULL, 0);

                hci_control_cb.pairing_allowed = WICED_FALSE;
                wiced_bt_set_pairable_mode(hci_control_cb.pairing_allowed, 0);
            }

            if (p1->nvram_id == nvram_id)
            {
                break;  /* this nvram_id is already used */
            }
        }
        if (p1 == NULL)
        {
            break;
        }
    }
    WICED_BT_TRACE("hci_control_alloc_nvram_id:%d\n", nvram_id);
    return nvram_id;
}

/*
 * This function is called when a Transport Buffer has been sent to the MCU
 */
void hci_control_transport_tx_cplt_cback(wiced_transport_buffer_pool_t* p_pool)
{
    // Do nothing
}

/*
 * Miscellaneous command handler function
 */
void hci_control_misc_handle_command(uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len)
{
    switch (cmd_opcode)
    {
    case HCI_CONTROL_MISC_COMMAND_PING:
        wiced_transport_send_data(HCI_CONTROL_MISC_EVENT_PING_REPLY, p_data, data_len);
        break;

    case HCI_CONTROL_MISC_COMMAND_GET_VERSION:
        hci_control_misc_handle_get_version();
        break;

    default:
        WICED_BT_TRACE("unknown miscellaneous command\n");
        break;
    }
}

/*
 * Handle the GET_VERSION command
 */
void hci_control_misc_handle_get_version(void)
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

    wiced_transport_send_data(HCI_CONTROL_MISC_EVENT_VERSION, tx_buf, cmd);
}

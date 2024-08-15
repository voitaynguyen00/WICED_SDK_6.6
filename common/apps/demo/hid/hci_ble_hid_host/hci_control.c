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
 * HID Host (using BLE) Sample Application for 2070X devices.
 *
 * This file implements 2070x embedded application controlled over UART.
 *
 * The sample Windows ClientControl application is provided to show sample
 * MCU implementation on Windows platform.
 *
 * Features demonstrated
 *  - WICED BLE HID host APIs
 *
 * On startup this demo:
 *  - Initializes the Bluetooth/BLE sub system
 *  - Receive NVRAM information from the host
 *  - Allow previously paired HID Devices to reconnect. Note that the MCU have to set the
 *    device 'Connectable".
 *
 * BLE HID Host
 *  - The BLE HID Host profile is used to connect to HID Devices (e.g mice/keyboards/remote control).
 *  - To search BLE HID Devices to connect, press the Start BLE Discovery button.
 *  - To Connect/Disconnect BLE HID Devices, press the Connect or Disconnect Button
 *  - The "Get Desc" button is typically used immediately after the Pairing (first connection
 *    initiated by the HID Host) to retrieve the HID Descriptor of the device.
 *  - The HID Descriptor of an HID Device describes the HID Reports supported by the HID Device.
 *    See "http://www.usb.org/developers/hidpage" page for more details.
 *  - The "HID Protocol" ComboBox can be used to change the HID Protocol to "Report Mode" (default)
 *    or to "Boot Mode" (generic & basic mode).
 *  - The Received Report are printed in the Log frame of the ClientControl app.
 *  - Note: The peer host information is saved in the Windows registry. This is the place to clean
 *     up, for example if information about the paired should be empty.
 *
 */
#include "sparcommon.h"
#include "wiced_bt_cfg.h"
#include "wiced_memory.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_gatt.h"

#include "wiced_timer.h"
#include "wiced_platform.h"
#include "wiced_transport.h"
#include "wiced_app_cfg.h"
#if ( defined(CYW20706A2) || defined(CYW20719B1) || defined(CYW20719B0) || defined(CYW20721B1) || defined(CYW20735B0) || defined(CYW43012C0) )
#include "wiced_bt_app_common.h"
#endif
#include "hci_control.h"

#include "hci_control_ble_hidh.h"
#include "wiced_bt_ble_hidh.h"
#include "wiced_bt_stack.h"
#include "wiced_hal_wdog.h"
#include "hci_control_le.h"
#include "string.h"

/*****************************************************************************
**  Constants
*****************************************************************************/
#define KEY_INFO_POOL_BUFFER_COUNT  3  // Correspond's to the number of peer devices

/*****************************************************************************
**  Structures
*****************************************************************************/
/* NVRAM Data */
typedef struct
{
    wiced_bt_device_link_keys_t     link_key;   /* This field must be the first of the structure) */
    wiced_bt_ble_hidh_gatt_cache_t  ble_hidh_gatt_cache;
} hci_control_nvram_data_t;

/* NVRAM Chunk */
typedef struct
{
    uint16_t nvram_id;
    uint8_t  chunk_len;
    hci_control_nvram_data_t data;
} hci_control_nvram_chunk_t;

/* The main application control block */
typedef struct
{
    hci_control_nvram_chunk_t *nvram_chunk[KEY_INFO_POOL_BUFFER_COUNT];
    uint8_t pairing_allowed;
} hci_control_cb_t;

/*
 * Local functions
 */
static uint32_t hci_control_proc_rx_cmd( uint8_t *p_data, uint32_t length );
void hci_control_send_command_status_evt( uint16_t code, uint8_t status );
static void hci_hid_control_transport_status( wiced_transport_type_t type );
void hci_control_misc_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len );
void hci_control_misc_handle_get_version( void );

int hci_control_nvram_alloc(int nvram_id);
void hci_control_nvram_delete(int nvram_id);
int hci_control_nvram_write(int nvram_id, void *p_data, int length);
int hci_control_nvram_read(int nvram_id, void *p_data, int length);
int hci_control_nvram_find_id(wiced_bt_device_address_t bdaddr);
int hci_control_nvram_write_link_keys(int nvram_id, wiced_bt_device_link_keys_t *p_link_keys);
int hci_control_nvram_read_link_keys(int nvram_id, wiced_bt_device_link_keys_t *p_link_keys);
int hci_control_nvram_write_ble_hidh_gatt_cache(wiced_bt_device_address_t bdaddr,
        wiced_bt_ble_hidh_gatt_cache_t *p_gatt_cache, wiced_bool_t send_to_host);
int hci_control_nvram_read_ble_hidh_gatt_cache( wiced_bt_device_address_t bdaddr,
        wiced_bt_ble_hidh_gatt_cache_t *p_gatt_cache);
void hci_control_nvram_send_to_host(int nvram_id);
void hci_control_nvram_host_push(int nvram_id, uint8_t *p_data, int length);
void hci_control_nvram_host_delete(int nvram_id);
void wiced_le_gatt_init(void);
typedef UINT8 tBTM_STATUS;
BTM_API extern tBTM_STATUS BTM_SetDataChannelPDULength(BD_ADDR bd_addr, UINT16 tx_pdu_length);
uint8_t wiced_bt_get_number_of_buffer_pools ( void );

/******************************************************
 *               Variables Definitions
 ******************************************************/
/* Main control block */
hci_control_cb_t  hci_control_cb;

wiced_bt_buffer_pool_t        *p_key_info_pool;  //Pool for storing the  key info
wiced_transport_buffer_pool_t *p_transport_pool;

const wiced_transport_cfg_t transport_cfg =
{
    WICED_TRANSPORT_UART,
    {{  WICED_TRANSPORT_UART_HCI_MODE, HCI_UART_DEFAULT_BAUD }},
    {  TRANS_UART_BUFFER_SIZE, 2 },
    hci_hid_control_transport_status,
    hci_control_proc_rx_cmd,
    NULL
};

/******************************************************
 *               Function Declarations
 ******************************************************/
static void     hci_control_handle_reset_cmd( void );
static void     hci_control_handle_trace_enable( uint8_t *p_data );
static void     hci_control_device_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len );
static void     hci_control_handle_set_local_bda( uint8_t *p_bda );
static void     hci_control_handle_set_pairability ( uint8_t pairing_allowed );
static void     hci_control_handle_user_confirmation( uint8_t *p_bda, uint8_t accept_pairing );
static void     hci_control_handle_read_buffer_stats( void );
static void     hci_control_send_device_started_evt( void );
static void     hci_control_send_device_error_evt( uint8_t fw_error_code, uint8_t app_error_code );
static void     hci_control_send_pairing_completed_evt( uint8_t status, wiced_bt_device_address_t bdaddr );
static void     hci_control_send_user_confirmation_request_evt( wiced_bt_device_address_t bda, uint32_t numeric_value );
static void     hci_control_send_keypress_notification_evt(wiced_bt_device_address_t bda, wiced_bt_dev_passkey_entry_type_t key);
static void     hci_control_send_encryption_changed_evt( uint8_t encrypted, wiced_bt_device_address_t bdaddr );
static wiced_result_t hci_control_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );

/******************************************************
 *               Function Definitions
 ******************************************************/

/*
 *  Application Start, ie, entry point to the application.
 */
APPLICATION_START( )
{
    wiced_result_t result;

    memset(&hci_control_cb, 0, sizeof(hci_control_cb));

    wiced_transport_init(&transport_cfg);

#ifdef WICED_BT_TRACE_ENABLE
    // Set the debug uart to enable the debug traces
    // Configure the Debug Traces to PUART by default (only way to see the debug traces during boot)
    // The MCU may send a command to change the Debug trace route later

    // WICED_ROUTE_DEBUG_TO_PUART  to send debug string over the PUART
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );

    // Set the debug uart as WICED_ROUTE_DEBUG_NONE to get rid of prints
    //wiced_set_debug_uart( WICED_ROUTE_DEBUG_NONE );
#endif

    wiced_bt_stack_init(hci_control_management_callback, &wiced_bt_cfg_settings,
            wiced_app_cfg_buf_pools);
}

/*
 *  Process all HCI packet from
 */
void hci_control_hci_packet_cback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data)
{
    // send the trace
    wiced_transport_send_hci_trace( NULL, type, length, p_data  );
}

/*
 *  Management callback receives various notifications from the stack
 */
wiced_result_t hci_control_management_callback( wiced_bt_management_evt_t event,
        wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t                     result = WICED_BT_SUCCESS;
    wiced_bt_dev_status_t              dev_status;
    wiced_bt_dev_ble_pairing_info_t   *p_pairing_info;
    wiced_bt_dev_encryption_status_t  *p_encryption_status;
    int                                bytes_written, bytes_read;
    int                                nvram_id;
    wiced_bt_power_mgmt_notification_t *p_power_mgmt_notification;
    wiced_bt_dev_pairing_cplt_t        *p_pairing_cmpl;
    uint8_t                             pairing_result;
    wiced_bool_t                       sdp_rv;
    wiced_bt_device_address_t          bda = { 0 };
    wiced_bt_device_address_t          null_bdaddr = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    wiced_bt_ble_connection_param_update_t *p_ble_conn_param_update;

    WICED_BT_TRACE( "hci_control_management_callback event:%d\n", event );

    switch( event )
    {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:
            WICED_BT_TRACE( "BTM_ENABLED_EVT\n" );
            /* initialize everything */
            memset( &hci_control_cb, 0, sizeof( hci_control_cb ) );

            /* create SDP records */
            sdp_rv = wiced_bt_sdp_db_init((uint8_t *)wiced_app_cfg_sdp_record,
                    wiced_app_cfg_sdp_record_get_size());
            if (sdp_rv != WICED_TRUE)
                WICED_BT_TRACE("Err: wiced_bt_sdp_db_init Failed\n");

            /* Register HCI Trace callback */
            wiced_bt_dev_register_hci_trace( hci_control_hci_packet_cback );

            /* Creating a buffer pool for holding the peer devices's info (Keys & BLE HIDH Gatt) */
            p_key_info_pool = wiced_bt_create_pool( sizeof(hci_control_nvram_chunk_t),
                    KEY_INFO_POOL_BUFFER_COUNT );
            if (p_key_info_pool == NULL)
                WICED_BT_TRACE("Err: wiced_bt_create_pool failed\n");

            p_transport_pool = wiced_transport_create_buffer_pool (
                    sizeof(uint16_t) + sizeof(hci_control_nvram_data_t), 1);
            if (p_transport_pool == NULL)
                WICED_BT_TRACE("Err: wiced_transport_create_buffer_pool failed\n");

            wiced_bt_dev_read_local_addr(bda);
            WICED_BT_TRACE("Local Bluetooth Address: [%B]\n", bda);

            hci_control_ble_hidh_init();
            wiced_le_gatt_init();

            hci_control_handle_set_pairability(1);

            /*
             * The application running on the 20719 will not start if the Wiced UART is already
             * opened.
             * If we reach this point, this means that the Wiced UART is not opened, so we cannot
             * send any event.
             * The p_status_handler callback of the wiced_transport_cfg_t will be called as soon
             * as the Host MCU will open the UART.
             * We will send the HCI_CONTROL_EVENT_DEVICE_STARTED message in this callback.
             */
            break;

        case BTM_DISABLED_EVT:
            WICED_BT_TRACE( "BTM_DISABLED_EVT\n" );
            hci_control_send_device_error_evt( p_event_data->disabled.reason, 0 );
            break;

        case BTM_PIN_REQUEST_EVT:
            WICED_BT_TRACE("Pin Code Request remote address=%B\n", p_event_data->pin_request.bd_addr);
            WICED_BT_TRACE_ARRAY(pincode, sizeof(pincode) , "Reply with default Pin Code ");
            wiced_bt_dev_pin_code_reply(*p_event_data->pin_request.bd_addr, WICED_BT_SUCCESS,
                    WICED_PIN_CODE_LEN, (uint8_t *)&pincode[0]);
            break;

        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            WICED_BT_TRACE( "BTM_USER_CONFIRMATION_REQUEST_EVT\n" );
            // If this is just works pairing, accept.
            // Otherwise send event to the MCU to confirm the same value.
            if (p_event_data->user_confirmation_request.just_works)
            {
                wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS,
                        p_event_data->user_confirmation_request.bd_addr );
            }
            else
            {
                hci_control_send_user_confirmation_request_evt(
                        p_event_data->user_confirmation_request.bd_addr,
                        p_event_data->user_confirmation_request.numeric_value );
            }
            break;

        case BTM_PASSKEY_NOTIFICATION_EVT:
            WICED_BT_TRACE("PassKey Notification. BDA %B, Key %d\n",
                    p_event_data->user_passkey_notification.bd_addr,
                    p_event_data->user_passkey_notification.passkey );
            hci_control_send_user_confirmation_request_evt(
                    p_event_data->user_passkey_notification.bd_addr,
                    p_event_data->user_passkey_notification.passkey );
            break;

        case BTM_KEYPRESS_NOTIFICATION_EVT:
            WICED_BT_TRACE("KeyPress Notification. BDA %B, Key %d\n",
                    p_event_data->user_keypress_notification.bd_addr,
                    p_event_data->user_keypress_notification.keypress_type );
            hci_control_send_keypress_notification_evt(
                    p_event_data->user_keypress_notification.bd_addr,
                    p_event_data->user_keypress_notification.keypress_type );
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            WICED_BT_TRACE( "BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT\n" );
#if 0
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap  = BTM_IO_CAPABILITIES_NONE;
#else
            /* Some HID Devices (such as Keyboard) Require MITM Protection */
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap  = BTM_IO_CAPABILITIES_DISPLAY_AND_YES_NO_INPUT;
#endif
            p_event_data->pairing_io_capabilities_ble_request.oob_data   = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req   = BTM_LE_AUTH_REQ_BOND | BTM_LE_AUTH_REQ_MITM;
            p_event_data->pairing_io_capabilities_ble_request.max_key_size  = 0x10;
            p_event_data->pairing_io_capabilities_ble_request.init_keys     = BTM_LE_KEY_PENC | BTM_LE_KEY_PID;
            p_event_data->pairing_io_capabilities_ble_request.resp_keys     = BTM_LE_KEY_PENC | BTM_LE_KEY_PID;
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            p_pairing_cmpl = &p_event_data->pairing_complete;
            if(p_pairing_cmpl->transport == BT_TRANSPORT_BR_EDR)
            {
                pairing_result = p_pairing_cmpl->pairing_complete_info.br_edr.status;
            }
            else
            {
                pairing_result = p_pairing_cmpl->pairing_complete_info.ble.reason;
            }
            WICED_BT_TRACE( "Pairing Result: %d\n", pairing_result );
            hci_control_send_pairing_completed_evt( pairing_result,
                    p_event_data->pairing_complete.bd_addr );
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            p_encryption_status = &p_event_data->encryption_status;
            WICED_BT_TRACE( "Encryption address:%B status:%d\n", p_encryption_status->bd_addr,
                    p_encryption_status->result );
            /* Tell BLE HIDH Profile that Encryption changed */
            wiced_bt_ble_hidh_encryption_changed(&p_event_data->encryption_status);

            hci_control_send_encryption_changed_evt( p_encryption_status->result,
                    p_encryption_status->bd_addr );
#if 1
            result = BTM_SetDataChannelPDULength(p_encryption_status->bd_addr, 251);
            if (result != 0)
                WICED_BT_TRACE("Err: BTM_SetDataChannelPDULength failed %d\n", result);
#endif
            break;

        case BTM_SECURITY_REQUEST_EVT:
            WICED_BT_TRACE( "Security Request Event, Pairing allowed %d\n",
                    hci_control_cb.pairing_allowed );
            if ( hci_control_cb.pairing_allowed )
            {
                wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr,
                        WICED_BT_SUCCESS );
            }
            else
            {
                // Pairing not allowed, return error
                result = WICED_BT_ERROR;
            }
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            WICED_BT_TRACE("BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT address:%B\n",
                    p_event_data->paired_device_link_keys_update.bd_addr);
            /* Workaround if the BT Stack sends a Null BdAddr (in case of Pairing Failure) */
            if (memcmp(p_event_data->paired_device_link_keys_update.bd_addr, null_bdaddr, BD_ADDR_LEN) != 0)
            {
                /* Check if we already have information saved for this bd_addr */
                nvram_id = hci_control_nvram_find_id(
                        p_event_data->paired_device_link_keys_update.bd_addr);
                if (nvram_id == HCI_CONTROL_INVALID_NVRAM_ID)
                {
                    // This is the first time, allocate id for the new nvram chunk
                    nvram_id = hci_control_nvram_alloc(HCI_CONTROL_INVALID_NVRAM_ID);
                }
                if (nvram_id == HCI_CONTROL_INVALID_NVRAM_ID)
                {
                    WICED_BT_TRACE( "No More NVRAM to save LinkKeys:%d\n", nvram_id );
                }
                else
                {
                    /* Write the link key in this NVRAM chunk */
                    hci_control_nvram_write_link_keys(nvram_id, &p_event_data->paired_device_link_keys_update);

                    /* Send the NVRAM to Host */
                    hci_control_nvram_send_to_host(nvram_id);
                }
            }
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            WICED_BT_TRACE("BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT addr:%B\n",
                    p_event_data->paired_device_link_keys_request.bd_addr);
            /* Search existing key from the NVRAM  */
            nvram_id = hci_control_nvram_find_id(
                    p_event_data->paired_device_link_keys_request.bd_addr);
            if (nvram_id != HCI_CONTROL_INVALID_NVRAM_ID)
            {
                bytes_read = hci_control_nvram_read_link_keys(nvram_id,
                        &p_event_data->paired_device_link_keys_request);
                if (bytes_read!=sizeof(hci_control_nvram_data_t))
                {
                    result = WICED_BT_ERROR;
                    WICED_BT_TRACE("Key read error\n");
                }
                else
                {
                    result = WICED_BT_SUCCESS;
                    WICED_BT_TRACE("Key retrieved\n");
                }
            }
            else
            {
                result = WICED_BT_ERROR;
                WICED_BT_TRACE("Key retrieval failure\n");
            }
            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            WICED_BT_TRACE("BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT\n");
            /* Request to store newly generated local identity keys to NVRAM */
            /* (sample app does not store keys to NVRAM) */
            break;

        case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            WICED_BT_TRACE("BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT\n");
            /*
             * Request to restore local identity keys from NVRAM
             * (requested during Bluetooth start up)
             * */
            /* (sample app does not store keys to NVRAM)
             * New local identity keys will be generated
             * */
            //result = WICED_BT_NO_RESOURCES;
            break;

        case BTM_POWER_MANAGEMENT_STATUS_EVT:
            p_power_mgmt_notification = &p_event_data->power_mgmt_notification;
            WICED_BT_TRACE( "Power mgmt status event: bd ( %B ) status:%d hci_status:%d\n",
                    p_power_mgmt_notification->bd_addr, p_power_mgmt_notification->status,
                    p_power_mgmt_notification->hci_status);
            break;

        case BTM_BLE_SCAN_STATE_CHANGED_EVT:
            hci_control_le_scan_state_changed( p_event_data->ble_scan_state_changed );
            break;

        case BTM_BLE_CONNECTION_PARAM_UPDATE:
            p_ble_conn_param_update = &p_event_data->ble_connection_param_update;
            WICED_BT_TRACE ("BTM BLE Connection Update event status:%d interval:%d latency:%d lsto:%d\n",
                                p_ble_conn_param_update->status,
                                p_ble_conn_param_update->conn_interval,
                                p_ble_conn_param_update->conn_latency,
                                p_ble_conn_param_update->supervision_timeout);
            break;

        default:
            WICED_BT_TRACE( "Unhandled management event:%d\n", event );
            result = WICED_BT_ERROR;
            break;
    }
    return result;
}

/*
 * hci_control_transport_status
 * This callback function is called when the MCU opens the Wiced UART
 */
static void hci_hid_control_transport_status( wiced_transport_type_t type )
{
    WICED_BT_TRACE( "hci_control_transport_status %x \n", type );

    // Tell Host that App is started
    hci_control_send_device_started_evt();
}

/*
 * Handle received command over UART. Please refer to the WICED Smart Ready
 * Software User Manual (WICED-Smart-Ready-SWUM100-R) for details on the
 * HCI UART control protocol.
*/
static uint32_t hci_control_proc_rx_cmd( uint8_t *p_buffer, uint32_t length )
{
    uint16_t opcode;
    uint16_t payload_len;
    uint8_t *p_data = p_buffer;

    if ( !p_buffer )
    {
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    //Expected minimum 4 byte as the wiced header
    if(( length < 4 ) || (p_data == NULL))
    {
        WICED_BT_TRACE("invalid params\n");
        wiced_transport_free_buffer( p_buffer );
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    STREAM_TO_UINT16(opcode, p_data);       // Get OpCode
    STREAM_TO_UINT16(payload_len, p_data);  // Gen Payload Length

    WICED_BT_TRACE("cmd_opcode 0x%02x\n", opcode);

    switch((opcode >> 8) & 0xff)
    {
    case HCI_CONTROL_GROUP_DEVICE:
        hci_control_device_handle_command( opcode, p_data, payload_len );
        break;

    case HCI_CONTROL_GROUP_LE:
        hci_control_le_handle_command( opcode, p_data, payload_len );
        break;

    case HCI_CONTROL_GROUP_HIDH:
        hci_control_ble_hidh_handle_command( opcode, p_data, payload_len );
        break;

    case HCI_CONTROL_GROUP_MISC:
        hci_control_misc_handle_command(opcode, p_data, payload_len);
        break;

    default:
        WICED_BT_TRACE( "unknown class code (opcode:%x)\n", opcode);
        break;
    }

    /* Freeing the buffer in which data is received */
    wiced_transport_free_buffer( p_buffer );

    return HCI_CONTROL_STATUS_SUCCESS;
}

/*
 * hci_control_device_handle_command
 * Handles the Device Group commands from MCU
 */
void hci_control_device_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len )
{
    uint8_t bytes_written;
    switch( cmd_opcode )
    {
    case HCI_CONTROL_COMMAND_RESET:
        hci_control_handle_reset_cmd( );
        break;

    case HCI_CONTROL_COMMAND_TRACE_ENABLE:
        hci_control_handle_trace_enable( p_data );
        break;

    case HCI_CONTROL_COMMAND_SET_LOCAL_BDA:
        hci_control_handle_set_local_bda( p_data );
        break;

    case HCI_CONTROL_COMMAND_PUSH_NVRAM_DATA:
        hci_control_nvram_host_push(p_data[0] | ( p_data[1] << 8 ), &p_data[2], data_len - 2);
        break;

    case HCI_CONTROL_COMMAND_DELETE_NVRAM_DATA:
        hci_control_nvram_host_delete(p_data[0] | ( p_data[1] << 8 ));
        WICED_BT_TRACE( "NVRAM delete: %d\n", p_data[0] | ( p_data[1] << 8 ) );
        break;

    case HCI_CONTROL_COMMAND_SET_PAIRING_MODE:
        hci_control_handle_set_pairability( p_data[0] );
        break;

    case HCI_CONTROL_COMMAND_USER_CONFIRMATION:
        hci_control_handle_user_confirmation( p_data, p_data[6] );
        break;

    case HCI_CONTROL_COMMAND_READ_BUFF_STATS:
        hci_control_handle_read_buffer_stats ();
        break;

    default:
        WICED_BT_TRACE("??? Unsupported command code:0x%04X\n");
        hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS,
                HCI_CONTROL_STATUS_UNKNOWN_COMMAND );
        break;
    }
}

/* Handle misc command group */
void hci_control_misc_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len )
{
    switch( cmd_opcode )
    {
    case HCI_CONTROL_MISC_COMMAND_GET_VERSION:
        hci_control_misc_handle_get_version();
        break;
    }
}


/* Handle get version command */
void hci_control_misc_handle_get_version( void )
{
    uint8_t   tx_buf[15];
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
    tx_buf[cmd++] = HCI_CONTROL_GROUP_HIDH;

    wiced_transport_send_data( HCI_CONTROL_MISC_EVENT_VERSION, tx_buf, cmd );
}

/*
 * handle reset command from UART
 */
void hci_control_handle_reset_cmd( void )
{
    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS,
            HCI_CONTROL_STATUS_SUCCESS );

    // trip watch dog now.
    wiced_hal_wdog_reset_system( );
}

/*
 * handle command from UART to configure traces
 */
void hci_control_handle_trace_enable( uint8_t *p_data )
{
    uint8_t hci_trace_enable = *p_data++;
    wiced_debug_uart_types_t route_debug = (wiced_debug_uart_types_t)*p_data;

    WICED_BT_TRACE("HCI Traces:%d DebugRoute:%d\n", hci_trace_enable, route_debug);

    if ( hci_trace_enable )
    {
        /* Register callback for receiving hci traces */
        // Disable while streaming audio over the uart.
        wiced_bt_dev_register_hci_trace( hci_control_hci_packet_cback );
    }
    else
    {
        wiced_bt_dev_register_hci_trace( NULL);
    }
    wiced_set_debug_uart( route_debug );

    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS,
            HCI_CONTROL_STATUS_SUCCESS );
}

/*
 * handle command to set local Bluetooth device address
 */
void hci_control_handle_set_local_bda( uint8_t *p_bda)
{
    wiced_bt_device_address_t bd_addr;

    STREAM_TO_BDADDR(bd_addr,p_bda);

    WICED_BT_TRACE( "Local BdAddr:%B\n", bd_addr);

    wiced_bt_set_local_bdaddr( bd_addr, BLE_ADDR_PUBLIC );

    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS,
            HCI_CONTROL_STATUS_SUCCESS );
}

/*
 *  Handle read buffer statistics
 */
void hci_control_handle_read_buffer_stats( void )
{
	uint8_t buff_pools = 0;
#ifdef WICEDX
#define BUFF_POOLS 5
	wiced_bt_buffer_statistics_t buff_stats[BUFF_POOLS];
	buff_pools = BUFF_POOLS;
#else    
	wiced_bt_buffer_statistics_t buff_stats[wiced_bt_get_number_of_buffer_pools()];
	buff_pools = wiced_bt_get_number_of_buffer_pools();
#endif
    wiced_result_t result;
    uint8_t i;

    result = wiced_bt_get_buffer_usage( buff_stats, sizeof( buff_stats ) );

    if( result == WICED_BT_SUCCESS )
    {
        // Print out the stats to trace
        WICED_BT_TRACE( "Buffer usage statistics:\n");

        for( i=0; i < buff_pools; i++) {
            WICED_BT_TRACE("pool_id:%d size:%d curr_cnt:%d max_cnt:%d total:%d\n",
                           buff_stats[i].pool_id, buff_stats[i].pool_size,
                           buff_stats[i].current_allocated_count,
                           buff_stats[i].max_allocated_count,
                           buff_stats[i].total_count);
        }

        // Return the stats via WICED-HCI
        wiced_transport_send_data( HCI_CONTROL_EVENT_READ_BUFFER_STATS,
                (uint8_t*)&buff_stats, sizeof( buff_stats ) );
    }
    else
    {
        hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS,
                HCI_CONTROL_STATUS_FAILED );
    }
}

/*
 *  Handle Inquiry result callback from teh stack, format and send event over UART
 */
void hci_control_inquiry_result_cback( wiced_bt_dev_inquiry_scan_result_t *p_inquiry_result,
        uint8_t *p_eir_data )
{
    int       i;
    uint8_t   len;
    uint8_t   tx_buf[300];
    uint16_t  code;
    uint8_t   *p = tx_buf;

    if ( p_inquiry_result == NULL )
    {
        code = HCI_CONTROL_EVENT_INQUIRY_COMPLETE;
        WICED_BT_TRACE( "inquiry complete\n");
    }
    else
    {
        code = HCI_CONTROL_EVENT_INQUIRY_RESULT;
        WICED_BT_TRACE( "inquiry result %B\n", p_inquiry_result->remote_bd_addr );
        for ( i = 0; i < 6; i++ )
            *p++ = p_inquiry_result->remote_bd_addr[5 - i];
        for ( i = 0; i < 3; i++ )
            *p++ = p_inquiry_result->dev_class[2 - i];
        *p++ = p_inquiry_result->rssi;

        // currently callback does not pass the data of the adv data, need to go through the data
        // zero len in the LTV means that there is no more data
        while ( ( p_eir_data != NULL ) && ( len = *p_eir_data ) != 0 )
        {
            // In the HCI event all parameters should fit into 255 bytes
            if ( p + len + 1 > tx_buf + 255 )
            {
                WICED_BT_TRACE("Bad data\n");
                break;
            }
            for ( i = 0; i < len + 1; i++ )
                *p++ = *p_eir_data++;
        }
    }
    wiced_transport_send_data( code, tx_buf, ( int )( p - tx_buf ) );
}

/*
 *  Handle Set Pairability command received over UART
 */
void hci_control_handle_set_pairability ( uint8_t pairing_allowed )
{
    uint8_t status = HCI_CONTROL_STATUS_SUCCESS;
    int nvram_id;

    if (hci_control_cb.pairing_allowed != pairing_allowed)
    {
        if (pairing_allowed)
        {
            nvram_id = hci_control_nvram_alloc(HCI_CONTROL_INVALID_NVRAM_ID);
            if (nvram_id == HCI_CONTROL_INVALID_NVRAM_ID)
            {
                pairing_allowed = 0; //The key buffer pool is full therefore we cannot allow pairing to be enabled
                status = HCI_CONTROL_STATUS_OUT_OF_MEMORY;
            }
            else
            {
                 hci_control_nvram_delete(nvram_id);
            }
        }

        hci_control_cb.pairing_allowed = pairing_allowed;
        wiced_bt_set_pairable_mode(hci_control_cb.pairing_allowed, 0);
        WICED_BT_TRACE( "Set the pairing allowed to %d\n", hci_control_cb.pairing_allowed );
    }

    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, status);
}

/*
 *  Handle User Confirmation received over UART
 */
void hci_control_handle_user_confirmation( uint8_t *p_bda, uint8_t accept_pairing )
{
    wiced_bt_device_address_t bd_addr;

    STREAM_TO_BDADDR(bd_addr,p_bda);
    wiced_bt_dev_confirm_req_reply( accept_pairing == WICED_TRUE ? WICED_BT_SUCCESS :
            WICED_BT_ERROR, bd_addr);

    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS,
            HCI_CONTROL_STATUS_SUCCESS );
}

/*
 *  Send Device Started event through UART
 */
void hci_control_send_device_started_evt( void )
{
    wiced_transport_send_data( HCI_CONTROL_EVENT_DEVICE_STARTED, NULL, 0 );

    WICED_BT_TRACE( "maxLinks:%d maxChannels:%d maxpsm:%d rfcom max links%d, rfcom max ports:%d\n",
            wiced_bt_cfg_settings.l2cap_application.max_links,
            wiced_bt_cfg_settings.l2cap_application.max_channels,
            wiced_bt_cfg_settings.l2cap_application.max_psm,
            wiced_bt_cfg_settings.rfcomm_cfg.max_links,
            wiced_bt_cfg_settings.rfcomm_cfg.max_ports );
}

/*
 *  Send Device Error event through UART
 */
void hci_control_send_device_error_evt( uint8_t fw_error_code, uint8_t app_error_code )
{
    uint8_t event_data[] = { 0, 0 };

    event_data[0] = app_error_code;
    event_data[1] = fw_error_code;

    WICED_BT_TRACE( "[hci_control_send_device_error_evt] app_error_code=0x%02x fw_error_code=0x%02x\n",
            event_data[0], event_data[1] );

    wiced_transport_send_data( HCI_CONTROL_EVENT_DEVICE_ERROR, event_data, 2 );
}

/*
* transfer command status event to UART
*/
void hci_control_send_command_status_evt( uint16_t code, uint8_t status )
{
    wiced_transport_send_data( code, &status, 1 );
}

/*
 *  Send Pairing Completed event through UART
 */
void hci_control_send_pairing_completed_evt( uint8_t status , wiced_bt_device_address_t bdaddr )
{
    int i;
	uint8_t event_data[BD_ADDR_LEN + sizeof(uint8_t)];
	int     cmd_bytes = 0;

    event_data[cmd_bytes++] = status;

    for ( i = 0 ; i < BD_ADDR_LEN; i++ )                     // bd address
        event_data[cmd_bytes++] = bdaddr[BD_ADDR_LEN - 1 - i];

    event_data[cmd_bytes++] = BT_DEVICE_TYPE_BLE;

    WICED_BT_TRACE( "pairing complete evt: %B as %B status %d\n", bdaddr, &event_data[1], status );

    wiced_transport_send_data( HCI_CONTROL_EVENT_PAIRING_COMPLETE, event_data, cmd_bytes );
}

/*
 *  Send User Confirmation Request event through UART
 */
void hci_control_send_user_confirmation_request_evt( BD_ADDR bda, uint32_t numeric_value )
{
    uint8_t buf[BD_ADDR_LEN + sizeof(uint32_t)];
    uint8_t *p = buf;

    BDADDR_TO_STREAM(p, bda);
    UINT32_TO_STREAM(p, numeric_value);
    wiced_transport_send_data( HCI_CONTROL_EVENT_USER_CONFIRMATION, buf, p - buf );
}

/*
 * hci_control_send_keypress_notification_evt
 */
void hci_control_send_keypress_notification_evt(wiced_bt_device_address_t bda, wiced_bt_dev_passkey_entry_type_t key)
{
    uint8_t buf[BD_ADDR_LEN + sizeof(uint8_t)];
    uint8_t *p = buf;

    BDADDR_TO_STREAM(p, bda);
    UINT8_TO_STREAM(p, key);
    wiced_transport_send_data( HCI_CONTROL_EVENT_KEYPRESS_NOTIFICATION, buf, p - buf );
}

/*
 *  Send Encryption Changed event through UART
 */
void hci_control_send_encryption_changed_evt(uint8_t encrypted , wiced_bt_device_address_t bdaddr)
{
    int i;
	uint8_t event_data[BD_ADDR_LEN + sizeof(uint8_t)];
	int     cmd_bytes = 0;

    event_data[cmd_bytes++] = encrypted;

    for ( i = 0 ; i < BD_ADDR_LEN; i++ )                     // bd address
        event_data[cmd_bytes++] = bdaddr[BD_ADDR_LEN - 1 - i];

    wiced_transport_send_data( HCI_CONTROL_EVENT_ENCRYPTION_CHANGED, event_data, cmd_bytes );
}

/*
 * Allocate nvram_id to save new NVRAM chunk
 */
int hci_control_nvram_alloc(int nvram_id)
{
    hci_control_nvram_chunk_t *p_chunk;
    int                         i;

    /* Try to allocate any NVRAM Id */
    if (nvram_id == HCI_CONTROL_INVALID_NVRAM_ID)
    {
        for (i = 0 ; i < KEY_INFO_POOL_BUFFER_COUNT ; i++)
        {
            if (hci_control_cb.nvram_chunk[i] == NULL)
            {
                /* Allocating a buffer from the pool created for storing the info */
                p_chunk = (hci_control_nvram_chunk_t *)wiced_bt_get_buffer_from_pool(p_key_info_pool);
                if (p_chunk == NULL)
                {
                    WICED_BT_TRACE( "hci_control_nvram_alloc alloc 1 Failed%d\n" );
                    return HCI_CONTROL_INVALID_NVRAM_ID;
                }
                /* Wipe the allocated buffer */
                memset(p_chunk, 0, wiced_bt_get_buffer_size(p_chunk));
                p_chunk->nvram_id = i + HCI_CONTROL_FIRST_VALID_NVRAM_ID;
                hci_control_cb.nvram_chunk[i] = p_chunk;
                return p_chunk->nvram_id;
            }
        }
        WICED_BT_TRACE( "hci_control_nvram_alloc no more nvram entry%d\n" );
        return HCI_CONTROL_INVALID_NVRAM_ID;
    }

    /* (re)Allocate a specific NVRAM Id */
    /* Check the nvram range */
    if ((nvram_id <= HCI_CONTROL_FIRST_VALID_NVRAM_ID) &&
         (nvram_id >= (HCI_CONTROL_FIRST_VALID_NVRAM_ID + KEY_INFO_POOL_BUFFER_COUNT)))
    {
        WICED_BT_TRACE( "hci_control_nvram_alloc wrong nvram_id:%d\n", nvram_id );
        return HCI_CONTROL_INVALID_NVRAM_ID;
    }

    i = nvram_id - HCI_CONTROL_FIRST_VALID_NVRAM_ID;
    if (hci_control_cb.nvram_chunk[i] != NULL)
    {
        WICED_BT_TRACE( "hci_control_nvram_alloc nvram_id:%d already allocated\n", nvram_id );
        return HCI_CONTROL_INVALID_NVRAM_ID;
    }

    /* Allocating a buffer from the pool created for storing the info */
    p_chunk = (hci_control_nvram_chunk_t *)wiced_bt_get_buffer_from_pool(p_key_info_pool);
    if (p_chunk == NULL)
    {
        WICED_BT_TRACE( "hci_control_nvram_alloc alloc 2 Failed%d\n" );
        return HCI_CONTROL_INVALID_NVRAM_ID;
    }

    p_chunk->nvram_id = i + HCI_CONTROL_FIRST_VALID_NVRAM_ID;
    hci_control_cb.nvram_chunk[i] = p_chunk;

    return p_chunk->nvram_id;
}

/*
 * find_nvram_chunk
 */
BOOL8 nvram_chunk_is_valid(int i)
{
    /* Check the nvram id range */
    if ( (i >= 0) && (i < KEY_INFO_POOL_BUFFER_COUNT) )
    {
        return hci_control_cb.nvram_chunk[i] != NULL;
    }
    return FALSE;
}


/*
 *  hci_control_nvram_delete
 */
void hci_control_nvram_delete(int nvram_id)
{
    int i = nvram_id - HCI_CONTROL_FIRST_VALID_NVRAM_ID;

    if ( nvram_chunk_is_valid(i) )
    {
        if (wiced_bt_dev_delete_bonded_device(hci_control_cb.nvram_chunk[i]->data.link_key.bd_addr) != WICED_BT_SUCCESS)
        {
            WICED_BT_TRACE( "wiced_bt_dev_delete_bonded_device failed: %B\n", hci_control_cb.nvram_chunk[i]->data.link_key.bd_addr );
        }

        wiced_bt_free_buffer(hci_control_cb.nvram_chunk[i]);
        hci_control_cb.nvram_chunk[i] = NULL;
    }
}


/*
 * Find nvram_id of the NVRAM chunk with first bytes matching specified bdaddr
 */
int hci_control_nvram_find_id(wiced_bt_device_address_t bdaddr)
{
    hci_control_nvram_chunk_t *p_nvram_chunk;
    int i;

    for (i = 0 ; i < KEY_INFO_POOL_BUFFER_COUNT ; i++)
    {
        if (hci_control_cb.nvram_chunk[i] != NULL)
        {
            p_nvram_chunk = hci_control_cb.nvram_chunk[i];
            if (memcmp(p_nvram_chunk->data.link_key.bd_addr, bdaddr, BD_ADDR_LEN) == 0)
            {
                return p_nvram_chunk->nvram_id;
            }
        }
    }
    return HCI_CONTROL_INVALID_NVRAM_ID;
}

/*
 * Write NVRAM function is called to store information in the RAM.  This can be called when
 * stack requires persistent storage, for example to save link keys.  In this case
 * data is also formatted and send to the host for real NVRAM storage.  The same function is
 * called when host pushes NVRAM chunks during the startup.  Parameter from_host in this
 * case is set to FALSE indicating that data does not need to be forwarded.
 */
int hci_control_nvram_write(int nvram_id, void *p_data, int length)
{
    hci_control_nvram_chunk_t *p_nvram_chunk;
    wiced_result_t result;
    int nb_bytes;
    int i;

    /* Check the nvram range */
    if ((nvram_id <= HCI_CONTROL_FIRST_VALID_NVRAM_ID) &&
         (nvram_id >= (HCI_CONTROL_FIRST_VALID_NVRAM_ID + KEY_INFO_POOL_BUFFER_COUNT)))
    {
        WICED_BT_TRACE( "hci_control_nvram_write wrong nvram_id:%d\n", nvram_id );
        return 0;
    }

    p_nvram_chunk = hci_control_cb.nvram_chunk[nvram_id - HCI_CONTROL_FIRST_VALID_NVRAM_ID];
    if (p_nvram_chunk == NULL)
    {
        WICED_BT_TRACE( "hci_control_nvram_write nvram_id:%d not allocated\n", nvram_id );
        return 0;
    }

    if ((length > sizeof(hci_control_nvram_data_t)) ||
        (sizeof(hci_control_nvram_chunk_t) > (wiced_bt_get_buffer_size(p_nvram_chunk))))
    {
        WICED_BT_TRACE( "hci_control_nvram_write too big l:%d d:%d c:%d b:%d\n",
                length, sizeof(hci_control_nvram_data_t),
                sizeof(hci_control_nvram_chunk_t),
                wiced_bt_get_buffer_size(p_nvram_chunk));
        return (0);
    }

    p_nvram_chunk->chunk_len = length;
    memcpy(&p_nvram_chunk->data, p_data, length);

    return length;
}

/*
 * Read NVRAM actually finds the memory chunk in the RAM
 */
int hci_control_nvram_read(int nvram_id, void *p_data, int length)
{
    hci_control_nvram_chunk_t *p_nvram_chunk;
    wiced_result_t result;
    int nb_bytes;
    int i;

    /* Check the nvram range */
    if ((nvram_id <= HCI_CONTROL_FIRST_VALID_NVRAM_ID) &&
         (nvram_id >= (HCI_CONTROL_FIRST_VALID_NVRAM_ID + KEY_INFO_POOL_BUFFER_COUNT)))
    {
        WICED_BT_TRACE( "hci_control_nvram_read wrong nvram_id:%d\n", nvram_id );
        return 0;
    }

    p_nvram_chunk = hci_control_cb.nvram_chunk[nvram_id - HCI_CONTROL_FIRST_VALID_NVRAM_ID];
    if (p_nvram_chunk == NULL)
    {
        WICED_BT_TRACE( "hci_control_nvram_read nvram_id:%d not allocated\n", nvram_id );
        return 0;
    }

    if ((length > sizeof(hci_control_nvram_data_t)) ||
        (sizeof(hci_control_nvram_chunk_t) > (wiced_bt_get_buffer_size(p_nvram_chunk))))
    {
        WICED_BT_TRACE( "hci_control_nvram_read too big l:%d d:%d c:%d b:%d\n",
                length, sizeof(hci_control_nvram_data_t),
                sizeof(hci_control_nvram_chunk_t),
                wiced_bt_get_buffer_size(p_nvram_chunk));
        return (0);
    }

    memcpy(p_data, &p_nvram_chunk->data, length);

    return length;
}


/*
 * hci_control_nvram_write_link_keys
 * Write the Link Keys of an NVRAM chunk
 */
int hci_control_nvram_write_link_keys(int nvram_id, wiced_bt_device_link_keys_t *p_link_keys)
{
    hci_control_nvram_data_t nvram_data;
    int nb_bytes;
    wiced_result_t result;

    /* Read the current NVRAM data */
    nb_bytes = hci_control_nvram_read(nvram_id, &nvram_data, sizeof(nvram_data));
    if (nb_bytes > 0)
    {
        /* If there was some data, remove this device from the Bonded devices */
        result = wiced_bt_dev_delete_bonded_device(nvram_data.link_key.bd_addr);
        if (result != WICED_BT_SUCCESS)
            WICED_BT_TRACE("Err: wiced_bt_dev_delete_bonded_device failed:%d\n", result);
    }
    /* Update the Link Keys part */
    memcpy(&nvram_data.link_key, p_link_keys, sizeof(nvram_data.link_key));

    result = wiced_bt_dev_add_device_to_address_resolution_db(p_link_keys);
    if (result != WICED_BT_SUCCESS)
        WICED_BT_TRACE("Err: wiced_bt_dev_add_device_to_address_resolution_db failed result:%d\n",
                result);

    /* Write back the NVRAM chunk */
    nb_bytes = hci_control_nvram_write(nvram_id, &nvram_data, sizeof(nvram_data));
    if (nb_bytes != sizeof(nvram_data))
        WICED_BT_TRACE("Err: hci_control_nvram_write_link_keys write failed result:%d\n", result);

    return nb_bytes;
}

/*
 * hci_control_nvram_read_link_keys
 * Write the Link Keys of an NVRAM chunk
 */
int hci_control_nvram_read_link_keys(int nvram_id, wiced_bt_device_link_keys_t *p_link_keys)
{
    hci_control_nvram_data_t nvram_data;
    int nb_bytes;
    wiced_result_t result;

    /* Read the current NVRAM data */
    nb_bytes = hci_control_nvram_read(nvram_id, &nvram_data, sizeof(nvram_data));
    if (nb_bytes == sizeof(nvram_data))
    {
        /* Copy the Link Keys part */
        memcpy(p_link_keys, &nvram_data.link_key, sizeof(nvram_data.link_key));
    }
    return nb_bytes;
}

/*
 * Write function is called to store the BLE GATT Cache information in the RAM.
 * This function search for the NVRAM Chunk matching the BdAddr and then update the GATT Cache
 * information.
 */
int hci_control_nvram_write_ble_hidh_gatt_cache(wiced_bt_device_address_t bdaddr,
        wiced_bt_ble_hidh_gatt_cache_t *p_gatt_cache, wiced_bool_t send_to_host)
{
    int nvram_id;
    int bytes_nb;
    hci_control_nvram_data_t nvram_data;

    WICED_BT_TRACE("hci_control_nvram_write_ble_hidh_gatt_cache address:%B\n", bdaddr);

    /* Check if this BdAddr exists in NVRAM */
    nvram_id = hci_control_nvram_find_id(bdaddr);
    if (nvram_id == HCI_CONTROL_INVALID_NVRAM_ID)
    {
        WICED_BT_TRACE("Err: hci_control_nvram_write_ble_hidh_gatt_cache address:%B unknown\n",
                bdaddr);
        return 0;
    }

    /* Read NVRAM info of this device */
    bytes_nb = hci_control_nvram_read(nvram_id, &nvram_data, sizeof(nvram_data));
    if (bytes_nb != sizeof(nvram_data))
    {
        WICED_BT_TRACE("Err: hci_control_nvram_write_ble_hidh_gatt_cache wrong read size:%d\n", bytes_nb);
        return 0;
    }

    /* Copy the GATT Cache information */
    memcpy(&nvram_data.ble_hidh_gatt_cache, p_gatt_cache,
            sizeof(nvram_data.ble_hidh_gatt_cache));

    /* Write it back */
    bytes_nb = hci_control_nvram_write(nvram_id, &nvram_data, sizeof(nvram_data));

    if (send_to_host)
        hci_control_nvram_send_to_host(nvram_id);

    return nvram_id;
}

/*
 * This Read function is called to retrieve the BLE GATT Cache information from the RAM.
 * This function search for the NVRAM Chunk matching the BdAddr and then returns the GATT Cache
 * information.
 */
int hci_control_nvram_read_ble_hidh_gatt_cache( wiced_bt_device_address_t bdaddr,
        wiced_bt_ble_hidh_gatt_cache_t *p_gatt_cache)
{
    int nvram_id;
    int bytes_nb;
    hci_control_nvram_data_t nvram_data;

    WICED_BT_TRACE("hci_control_nvram_read_ble_hidh_gatt_cache address:%B\n", bdaddr);

    /* Check if this BdAddr exists in NVRAM */
    nvram_id = hci_control_nvram_find_id(bdaddr);
    if (nvram_id == HCI_CONTROL_INVALID_NVRAM_ID)
    {
        WICED_BT_TRACE("Err: hci_control_nvram_read_ble_hidh_gatt_cache address:%B unknown\n", bdaddr);
        return -1;
    }

    /* Read NVRAM info of this device */
    bytes_nb = hci_control_nvram_read(nvram_id, &nvram_data, sizeof(nvram_data));
    if (bytes_nb != sizeof(nvram_data))
    {
        WICED_BT_TRACE("Err: hci_control_nvram_read_ble_hidh_gatt_cache wrong read size:%d\n", bytes_nb);
        return -1;
    }

    /* Retrieve the GATT Cache information */
    memcpy(p_gatt_cache, &nvram_data.ble_hidh_gatt_cache,
            sizeof(nvram_data.ble_hidh_gatt_cache));

    return sizeof(nvram_data.ble_hidh_gatt_cache);
}

/*
 * hci_control_nvram_send_to_host
 * Send an NVRAM chunk to Host
 */
void hci_control_nvram_send_to_host(int nvram_id)
{
    uint8_t *p_tx_buf;
    uint8_t *p;
    int nb_bytes;

    WICED_BT_TRACE("hci_control_nvram_send_to_host nvram_id:%d\n", nvram_id);
    /* The NVRAM data is too big to be sent using wiced_transport_send_data() */
    /* We need to use a specific transport buffer pool */
    p_tx_buf = wiced_transport_allocate_buffer(p_transport_pool);
    if (p_tx_buf == NULL)
    {
        WICED_BT_TRACE("Err: hci_control_nvram_send_to_host wiced_transport_allocate_buffer failed\n");
        return;
    }

    p = p_tx_buf;

    UINT16_TO_STREAM(p, nvram_id);

    /* Read the current NVRAM data */
    nb_bytes = hci_control_nvram_read(nvram_id, p, sizeof(hci_control_nvram_data_t));
    if (nb_bytes != sizeof(hci_control_nvram_data_t))
    {
        WICED_BT_TRACE("Err: hci_control_nvram_send_to_host hci_control_nvram_read failed\n",
                nb_bytes);
    }
    p += nb_bytes;

    /* Sent the buffer to the transport */
    wiced_transport_send_buffer(HCI_CONTROL_EVENT_NVRAM_DATA, p_tx_buf, p - p_tx_buf);
}

/*
 * hci_control_nvram_host_push
 * Host pushes NVRAM Chunk
 */
void hci_control_nvram_host_push(int nvram_id, uint8_t *p_data, int length)
{
    int bytes_nb;
    uint8_t status = HCI_CONTROL_STATUS_SUCCESS;
    hci_control_nvram_data_t *p_nvram_data = (hci_control_nvram_data_t *)p_data;

    if (length >= sizeof(wiced_bt_device_link_keys_t))
    {
        /* Alloc the NVRAM Id (delete if if it exists) */
        nvram_id = hci_control_nvram_alloc(nvram_id);
        if (nvram_id == HCI_CONTROL_INVALID_NVRAM_ID)
        {
            status = HCI_CONTROL_STATUS_OUT_OF_MEMORY;
        }
        else
        {
            /* Write the Link Keys */
            hci_control_nvram_write_link_keys(nvram_id, &p_nvram_data->link_key);

            if (length == sizeof(hci_control_nvram_data_t))
            {
                /* Write the GATT Cache */
                hci_control_nvram_write_ble_hidh_gatt_cache(p_nvram_data->link_key.bd_addr,
                        &p_nvram_data->ble_hidh_gatt_cache, WICED_FALSE);
            }
        }
    }
    else
    {
        status = HCI_CONTROL_STATUS_FAILED;
    }

    /* Send back the status to Host */
    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, status);
}

/*
 * hci_control_nvram_host_delete
 * Host delete an NVRAM Chunk
 */
void hci_control_nvram_host_delete(int nvram_id)
{
    int i = nvram_id - HCI_CONTROL_FIRST_VALID_NVRAM_ID;
    uint8_t status = HCI_CONTROL_STATUS_FAILED;

    /* Delete the NVRAM Id if it is valid */
    if ( nvram_chunk_is_valid(i) )
    {
        // remove it from HIDH database
        wiced_bt_ble_hidh_remove(hci_control_cb.nvram_chunk[i]->data.link_key.bd_addr);

        /* Delete it from NVRAM */
        hci_control_nvram_delete(nvram_id);

        status = HCI_CONTROL_STATUS_SUCCESS;
    }

    /* Send back the status to Host */
    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, status);
}


/*
 * hci_control_get_address_type
 * Used to get a BLE Address type. This function checks in the NVRAM, then ask LE
 */
wiced_bt_ble_address_type_t hci_control_get_address_type(wiced_bt_device_address_t bdaddr)
{
    int nvram_id;
    int bytes_read;
    wiced_bt_ble_address_type_t address_type = 0xFF;    /* Unknown for the moment */
    wiced_bt_device_link_keys_t device_link_key;

    /* Check if this BdAddr exists in NVRAM */
    nvram_id = hci_control_nvram_find_id(bdaddr);

    /* If this BdAddr  exists in NVRAM */
    if ( nvram_id != HCI_CONTROL_INVALID_NVRAM_ID)
    {
        /* Read NVRAM info of this device */
        bytes_read = hci_control_nvram_read( nvram_id, &device_link_key, sizeof( device_link_key ) );
        if (bytes_read!=sizeof( device_link_key ))
        {
            WICED_BT_TRACE( "NVRAM read error\n");
        }
        else
        {
            address_type = device_link_key.key_data.ble_addr_type;
            WICED_BT_TRACE( "NVRAM address:%B type:%d\n", bdaddr, address_type);
        }
    }
    else
    {
        address_type = hci_control_le_get_address_type(bdaddr);
        WICED_BT_TRACE( "LE address:%B type:%d\n", bdaddr, address_type);
    }
    return address_type;
}


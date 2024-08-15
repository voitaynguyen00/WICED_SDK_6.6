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
 * HID Host (using Classic Bluetooth) Sample Application for 2070X devices.
 *
 * This file implements 2070x embedded application controlled over UART.
 *
 * The sample Windows ClientControl application is provided to show sample
 * MCU implementation on Windows platform.
 *
 * Features demonstrated
 *  - WICED BT HID host APIs
 *
 * On startup this demo:
 *  - Initializes the Bluetooth sub system
 *  - Receive NVRAM information from the host
 *  - Allow previously paired HID Devices to reconnect. Note that the MCU have to set the
 *    device 'Connectable".
 *
 * BR/EDR
 * - To find BR/EDR devices: Click on "Start BR/EDR Discovery"
 * - The Connectable check boxes can be used to allow peer device to reconnect
 *
 * HID Host
 * - The HID Host profile is used to connect to HID Devices (e.g mice/keyboards/remote control).
 * - The "Get Desc" button is typically used immediately after the Pairing (first connection
 *   initiated by the HID Host) to retrieve the HID Descriptor of the device.
 * - The HID Descriptor of an HID Device describes the HID Reports supported by the HID Device.
 *   See "http://www.usb.org/developers/hidpage" page for more details.
 * - The "HID Protocol" ComboBox can be used to change the HID Protocol to "Report Mode" (default)
 *   or to "Boot Mode" (generic & basic mode).
 *
 */
#include "sparcommon.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_hidh.h"
#include "wiced_hal_wdog.h"
#include "wiced_memory.h"
#include "wiced_timer.h"
#include "hci_control.h"
#include "hci_control_hidh.h"

#include "wiced_platform.h"
#include "wiced_transport.h"

#include "wiced_app_cfg.h"


/*****************************************************************************
**  Constants
*****************************************************************************/
#define WICED_HS_EIR_BUF_MAX_SIZE   264
#define KEY_INFO_POOL_BUFFER_SIZE   145 //Size of the buffer used for holding the peer device key info
#define KEY_INFO_POOL_BUFFER_COUNT  3  //Correspond's to the number of peer devices

#define BCM920719                   20719

/*****************************************************************************
**  Structures
*****************************************************************************/
/* The main application control block */
typedef struct
{
    uint8_t pairing_allowed;
} hci_control_cb_t;

typedef struct
{
    void    *p_next;
    uint16_t nvram_id;
    uint8_t  chunk_len;
    uint8_t  data[1];
} hci_control_nvram_chunk_t;

/*
 * Local functions
 */
static void hci_control_transport_status( wiced_transport_type_t type );
static uint32_t hci_control_proc_rx_cmd( uint8_t *p_data, uint32_t length );
void   hci_control_send_command_status_evt( uint16_t code, uint8_t status );
int    hci_control_write_nvram( int nvram_id, int data_len, void *p_data, wiced_bool_t from_host );
int    hci_control_read_nvram( int nvram_id, void *p_data, int data_len );
int    hci_control_find_nvram_id( uint8_t *p_data, int len);
void   hci_control_delete_nvram( int nvram_id, wiced_bool_t from_host );
int    hci_control_alloc_nvram_id( void );

UINT8    wiced_bt_get_number_of_buffer_pools ( void );

/******************************************************
 *               Variables Definitions
 ******************************************************/
hci_control_nvram_chunk_t *p_nvram_first = NULL;

/* Main control block */
hci_control_cb_t  hci_control_cb;

wiced_bt_buffer_pool_t*        p_key_info_pool;  //Pool for storing the  key info

const wiced_transport_cfg_t transport_cfg =
{
    WICED_TRANSPORT_UART,
    {  WICED_TRANSPORT_UART_HCI_MODE, HCI_UART_DEFAULT_BAUD },
    {  TRANS_UART_BUFFER_SIZE, 2 },
    hci_control_transport_status,
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
static void     hci_control_inquiry( uint8_t enable );
static void     hci_control_handle_set_visibility( uint8_t connectability );
static void     hci_control_handle_set_pairability ( uint8_t pairing_allowed );
static void     hci_control_handle_user_confirmation( uint8_t *p_bda, uint8_t accept_pairing );
static void     hci_control_handle_read_buffer_stats( void );
static void     hci_control_send_device_started_evt( void );
static void     hci_control_send_device_error_evt( uint8_t fw_error_code, uint8_t app_error_code );
static void     hci_control_send_pairing_completed_evt( uint8_t status, wiced_bt_device_address_t bdaddr );
static void     hci_control_send_user_confirmation_request_evt( BD_ADDR bda, uint32_t numeric_value );
static void     hci_control_send_keypress_notification_evt(BD_ADDR bda, wiced_bt_dev_passkey_entry_type_t key);
static void     hci_control_send_encryption_changed_evt( uint8_t encrypted, wiced_bt_device_address_t bdaddr );
static void     hci_control_misc_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len );
static void     hci_control_misc_handle_get_version( void );
static wiced_result_t hci_control_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );

/******************************************************
 *               Function Definitions
 ******************************************************/

/*
 *  hci_control_init
 */
APPLICATION_START( )
{
    wiced_result_t result;

    memset( &hci_control_cb, 0, sizeof( hci_control_cb ) );

    hci_control_hidh_init();

    wiced_transport_init( &transport_cfg );

#ifdef WICED_BT_TRACE_ENABLE
    // Set the debug uart to enable the debug traces
    // Configure the Debug Traces to PUART by default (only way to see the debug traces during boot)
    // The MCU may send a command to change the Debug trace route later

    // WICED_ROUTE_DEBUG_TO_PUART  to send debug string over the PUART
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );

    // Set the debug uart as WICED_ROUTE_DEBUG_NONE to get rid of prints
    //wiced_set_debug_uart( WICED_ROUTE_DEBUG_NONE );
#endif

    wiced_bt_stack_init( hci_control_management_callback,
                         &wiced_bt_cfg_settings,
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

    WICED_BT_TRACE( "hci_control_management_callback 0x%02x\n", event );

    switch( event )
    {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:


            /* initialize everything */
            memset( &hci_control_cb, 0, sizeof( hci_control_cb ) );

            /* create SDP records */
            sdp_rv = wiced_bt_sdp_db_init((uint8_t *)wiced_app_cfg_sdp_record,
                    wiced_app_cfg_sdp_record_get_size());
            if (sdp_rv != WICED_TRUE)
                WICED_BT_TRACE("Err: wiced_bt_sdp_db_init Failed\n");

            // Disable while streaming audio over the uart.
            wiced_bt_dev_register_hci_trace( hci_control_hci_packet_cback );

            // Creating a buffer pool for holding the peer devices's key info
            p_key_info_pool = wiced_bt_create_pool( KEY_INFO_POOL_BUFFER_SIZE,
                    KEY_INFO_POOL_BUFFER_COUNT );
            if (p_key_info_pool == NULL)
                WICED_BT_TRACE("Err: wiced_bt_create_pool failed\n");

            hci_control_hidh_start();

            /*
             * The application running on the chip will not start if the Wiced UART is already
             * opened.
             * If we reach this point, this means that the Wiced UART is not opened, so we cannot
             * send any event.
             * The p_status_handler callback of the wiced_transport_cfg_t will be called as soon
             * as the Host MCU will open the UART.
             * We will send the HCI_CONTROL_EVENT_DEVICE_STARTED message in this callback.
             */
            break;

        case BTM_DISABLED_EVT:
            hci_control_send_device_error_evt( p_event_data->disabled.reason, 0 );
            break;

        case BTM_PIN_REQUEST_EVT:
            WICED_BT_TRACE("Pin Code Request remote address=%B\n", p_event_data->pin_request.bd_addr);
            WICED_BT_TRACE_ARRAY(pincode, sizeof(pincode) , "Reply with default Pin Code ");
            wiced_bt_dev_pin_code_reply(*p_event_data->pin_request.bd_addr, WICED_BT_SUCCESS,
                    WICED_PIN_CODE_LEN, (uint8_t *)&pincode[0]);
            break;

        case BTM_USER_CONFIRMATION_REQUEST_EVT:
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

        case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:
            /* Use the default security for BR/EDR*/
            WICED_BT_TRACE("BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT bda %B\n",
                    p_event_data->pairing_io_capabilities_br_edr_request.bd_addr);
            p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap = BTM_IO_CAPABILITIES_DISPLAY_AND_YES_NO_INPUT;
            p_event_data->pairing_io_capabilities_br_edr_request.auth_req     = BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_YES;
            p_event_data->pairing_io_capabilities_br_edr_request.oob_data     = WICED_FALSE;
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
            WICED_BT_TRACE( "Encryption Status:(%B) res:%d\n", p_encryption_status->bd_addr,
                    p_encryption_status->result );
            hci_control_send_encryption_changed_evt( p_encryption_status->result,
                    p_encryption_status->bd_addr );
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
            /* Check if we already have information saved for this bd_addr */
            if ( ( nvram_id = hci_control_find_nvram_id(
                    p_event_data->paired_device_link_keys_update.bd_addr, BD_ADDR_LEN ) ) == 0)
            {
                // This is the first time, allocate id for the new memory chunk
                nvram_id = hci_control_alloc_nvram_id( );
                WICED_BT_TRACE( "Allocated NVRAM ID:%d\n", nvram_id );
            }
            bytes_written = hci_control_write_nvram( nvram_id,
                    sizeof( wiced_bt_device_link_keys_t ),
                    &p_event_data->paired_device_link_keys_update, WICED_FALSE );

            WICED_BT_TRACE("NVRAM write:id:%d bytes:%d dev: [%B]\n", nvram_id, bytes_written,
                    p_event_data->paired_device_link_keys_update.bd_addr);
            break;

        case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            /* read existing key from the NVRAM  */

            WICED_BT_TRACE("find device %B\n", p_event_data->paired_device_link_keys_request.bd_addr);

            if ( ( nvram_id = hci_control_find_nvram_id(
                    p_event_data->paired_device_link_keys_request.bd_addr, BD_ADDR_LEN ) ) != 0)
            {
                 bytes_read = hci_control_read_nvram( nvram_id,
                         &p_event_data->paired_device_link_keys_request,
                         sizeof( wiced_bt_device_link_keys_t ) );

                 result = WICED_BT_SUCCESS;
                 WICED_BT_TRACE("Read:nvram_id:%d bytes:%d\n", nvram_id, bytes_read);
            }
            else
            {
                result = WICED_BT_ERROR;
                WICED_BT_TRACE("Key retrieval failure\n\n");
            }
            break;

        case BTM_POWER_MANAGEMENT_STATUS_EVT:
            p_power_mgmt_notification = &p_event_data->power_mgmt_notification;
            WICED_BT_TRACE( "Power mgmt status event: bd ( %B ) status:%d hci_status:%d\n",
                    p_power_mgmt_notification->bd_addr, p_power_mgmt_notification->status,
                    p_power_mgmt_notification->hci_status);
            break;

        default:
            WICED_BT_TRACE( "Unhandled management event:%d\n", event );
            result = WICED_BT_ERROR;
            break;
    }
    return result;
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

    WICED_BT_TRACE("cmd_opcode 0x%04x\n", opcode);

    switch((opcode >> 8) & 0xff)
    {
    case HCI_CONTROL_GROUP_DEVICE:
        hci_control_device_handle_command( opcode, p_data, payload_len );
        break;

    case HCI_CONTROL_GROUP_HIDH:
        hci_control_hidh_handle_command( opcode, p_data, payload_len );
        break;

    case HCI_CONTROL_GROUP_MISC:
        hci_control_misc_handle_command(opcode, p_data, payload_len);
        break;

    default:
        WICED_BT_TRACE( "unknown class code (opcode:%x)\n", opcode);
        break;
    }

    // Freeing the buffer in which data is received
    wiced_transport_free_buffer( p_buffer );

    return HCI_CONTROL_STATUS_SUCCESS;
}

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
        bytes_written = hci_control_write_nvram( p_data[0] | ( p_data[1] << 8 ), data_len - 2,&p_data[2], WICED_TRUE );
        WICED_BT_TRACE( "NVRAM write: %d dev: [%B]\n", bytes_written , &p_data[2]);
        break;

    case HCI_CONTROL_COMMAND_DELETE_NVRAM_DATA:
        hci_control_delete_nvram( p_data[0] | ( p_data[1] << 8 ), WICED_TRUE );
        WICED_BT_TRACE( "NVRAM delete: %d\n", p_data[0] | ( p_data[1] << 8 ) );
        break;

    case HCI_CONTROL_COMMAND_INQUIRY:
        hci_control_inquiry( *p_data );
        break;

    case HCI_CONTROL_COMMAND_SET_VISIBILITY:
        hci_control_handle_set_visibility( p_data[1] );
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
        WICED_BT_TRACE("??? Unknown command code\n");
        hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS,
                HCI_CONTROL_STATUS_UNKNOWN_COMMAND );
        break;
    }
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
    BD_ADDR bd_addr;

    STREAM_TO_BDADDR(bd_addr,p_bda);
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

    WICED_BT_TRACE( "inquiry_result_cback in\n");
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
    WICED_BT_TRACE( "inquiry_result_cback out\n");
    wiced_transport_send_data( code, tx_buf, ( int )( p - tx_buf ) );
}

/*
 *  Handle Inquiry command received over UART
 */
void hci_control_inquiry( uint8_t enable )
{
    wiced_result_t           result;
    wiced_bt_dev_inq_parms_t params;

    if ( enable )
    {
        memset( &params, 0, sizeof( params ) );
        params.mode             = BTM_GENERAL_INQUIRY;
        params.duration         = 5;
        params.filter_cond_type = BTM_CLR_INQUIRY_FILTER;

        WICED_BT_TRACE( "inquiry started:%d\n", result );
        result = wiced_bt_start_inquiry( &params, &hci_control_inquiry_result_cback );
    }
    else
    {
        WICED_BT_TRACE( "cancel inquiry:%d\n", result );
        result = wiced_bt_cancel_inquiry( );
    }
    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS,
            HCI_CONTROL_STATUS_SUCCESS );
}

/*
 *  Handle Set Visibility command received over UART
 */
void hci_control_handle_set_visibility( uint8_t connectability )
{
    wiced_bt_dev_set_connectability( ( connectability != 0 ) ? WICED_TRUE : WICED_FALSE ,
            wiced_bt_cfg_settings.br_edr_scan_cfg.page_scan_window,
            wiced_bt_cfg_settings.br_edr_scan_cfg.page_scan_interval);

    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS,
            HCI_CONTROL_STATUS_SUCCESS );
}

/*
 *  Handle Set Pairability command received over UART
 */
void hci_control_handle_set_pairability ( uint8_t pairing_allowed )
{
    uint8_t                   status = HCI_CONTROL_STATUS_SUCCESS;
    hci_control_nvram_chunk_t *p1;

    if ( hci_control_cb.pairing_allowed != pairing_allowed )
    {
        if ( pairing_allowed )
        {
            // Check if key buffer pool has buffer available. If not, cannot enable pairing until nvram entries are deleted
            if ( ( p1 = ( hci_control_nvram_chunk_t * )wiced_bt_get_buffer_from_pool( p_key_info_pool ) ) == NULL)
            {
                pairing_allowed = 0; //The key buffer pool is full therefore we cannot allow pairing to be enabled
                status = HCI_CONTROL_STATUS_OUT_OF_MEMORY;
            }
            else
            {
                wiced_bt_free_buffer( p1 );
            }
        }

        hci_control_cb.pairing_allowed = pairing_allowed;
        wiced_bt_set_pairable_mode( hci_control_cb.pairing_allowed, 0 );
        WICED_BT_TRACE( "Set the pairing allowed to %d\n", hci_control_cb.pairing_allowed );
    }

    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, status );
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
    uint8_t event_data[BD_ADDR_LEN + sizeof(uint8_t)+1];
    int     cmd_bytes = 0;

    event_data[cmd_bytes++] = status;

    for ( i = 0 ; i < BD_ADDR_LEN; i++ )                     // bd address
        event_data[cmd_bytes++] = bdaddr[BD_ADDR_LEN - 1 - i];

    event_data[cmd_bytes++] = BT_DEVICE_TYPE_BREDR;

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
void hci_control_send_keypress_notification_evt(BD_ADDR bda, wiced_bt_dev_passkey_entry_type_t key)
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
 * Write NVRAM function is called to store information in the RAM.  This can be called when
 * stack requires persistent storage, for example to save link keys.  In this case
 * data is also formatted and send to the host for real NVRAM storage.  The same function is
 * called when host pushes NVRAM chunks during the startup.  Parameter from_host in this
 * case is set to FALSE indicating that data does not need to be forwarded.
 */
int hci_control_write_nvram( int nvram_id, int data_len, void *p_data, wiced_bool_t from_host )
{
    uint8_t                    tx_buf[257];
    uint8_t                   *p = tx_buf;
    hci_control_nvram_chunk_t *p1;
    wiced_result_t            result;

    /* first check if this ID is being reused and release the memory chunk */
    hci_control_delete_nvram( nvram_id, WICED_FALSE );

    /* Allocating a buffer from the pool created for storing the peer info */
    if ( ( p1 = ( hci_control_nvram_chunk_t * )wiced_bt_get_buffer_from_pool( p_key_info_pool ) ) == NULL)
    {
        WICED_BT_TRACE( "Failed to alloc:%d\n", data_len );
        return ( 0 );
    }

    if ( wiced_bt_get_buffer_size( p1 ) < ( sizeof( hci_control_nvram_chunk_t ) + data_len - 1 ) )
    {
        WICED_BT_TRACE( "Insufficient buffer size, Buff Size %d, Len %d\n",
                        wiced_bt_get_buffer_size( p1 ),
                        ( sizeof( hci_control_nvram_chunk_t ) + data_len - 1 ) );
        wiced_bt_free_buffer( p1 );
        return ( 0 );
    }

    p1->p_next    = p_nvram_first;
    p1->nvram_id  = nvram_id;
    p1->chunk_len = data_len;
    memcpy( p1->data, p_data, data_len );

    p_nvram_first = p1;

    wiced_bt_device_link_keys_t * p_keys = ( wiced_bt_device_link_keys_t *) p_data;
    result = wiced_bt_dev_add_device_to_address_resolution_db( p_keys);

    WICED_BT_TRACE("Updated Addr Resolution DB:%d\n", result );

    // If NVRAM chunk arrived from host, no need to send it back, otherwise send over transport
    if (!from_host)
    {
        *p++ = nvram_id & 0xff;
        *p++ = (nvram_id >> 8) & 0xff;
        memcpy(p, p_data, data_len);

        wiced_transport_send_data( HCI_CONTROL_EVENT_NVRAM_DATA, tx_buf, ( int )( data_len + 2 ) );
    }
    else
    {
        hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS,
                HCI_CONTROL_STATUS_SUCCESS );
    }
    return (data_len);
}

/*
 * Find nvram_id of the NVRAM chunk with first bytes matching specified byte array
 */
int hci_control_find_nvram_id(uint8_t *p_data, int len)
{
    hci_control_nvram_chunk_t *p1;

    /* Go through the linked list of chunks */
    for (p1 = p_nvram_first; p1 != NULL; p1 = (hci_control_nvram_chunk_t *)p1->p_next)
    {
        WICED_BT_TRACE( "find %B %B len:%d\n", p1->data, p_data, len );
        if ( memcmp( p1->data, p_data, len ) == 0 )
        {
            return ( p1->nvram_id );
        }
    }
    return HCI_CONTROL_INVALID_NVRAM_ID;
}

/*
 * Delete NVRAM function is called when host deletes NVRAM chunk from the persistent storage.
 */
void hci_control_delete_nvram( int nvram_id, wiced_bool_t from_host )
{
    hci_control_nvram_chunk_t *p1, *p2;

    /* If Delete NVRAM data command arrived from host, send a Command Status response to ack command */
    if (from_host)
    {
        hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS,
                HCI_CONTROL_STATUS_SUCCESS );
    }

    if ( p_nvram_first == NULL )
        return;

    /* Special case when need to remove the first chunk */
    if ( ( p_nvram_first != NULL ) && ( p_nvram_first->nvram_id == nvram_id ) )
    {
        p1 = p_nvram_first;

        if ( from_host && ( wiced_bt_dev_delete_bonded_device (p1->data) == WICED_ERROR ) )
        {
            WICED_BT_TRACE("ERROR: while Unbonding device\n");
        }
        else
        {
        p_nvram_first = (hci_control_nvram_chunk_t *)p_nvram_first->p_next;
        wiced_bt_free_buffer( p1 );
        }
        return;
    }

    /* Go through the linked list of chunks */
    for ( p1 = p_nvram_first; p1 != NULL; p1 = (hci_control_nvram_chunk_t *)p1->p_next )
    {
        p2 = (hci_control_nvram_chunk_t *)p1->p_next;

        if ( ( p2 != NULL ) && ( p2->nvram_id == nvram_id ) )
        {
            if ( from_host && ( wiced_bt_dev_delete_bonded_device (p2->data) == WICED_ERROR ) )
            {
                WICED_BT_TRACE("ERROR: while Unbonding device\n");
            }
            else
            {
            p1->p_next = p2->p_next;
            wiced_bt_free_buffer( p2 );
            }
            return;
        }
    }
}

/*
 * Read NVRAM actually finds the memory chunk in the RAM
 */
int hci_control_read_nvram( int nvram_id, void *p_data, int data_len )
{
    hci_control_nvram_chunk_t *p1;
    int                        data_read = 0;

    /* Go through the linked list of chunks */
    for ( p1 = p_nvram_first; p1 != NULL; p1 = p1->p_next )
    {
        if ( p1->nvram_id == nvram_id )
        {
            data_read = ( data_len < p1->chunk_len ) ? data_len : p1->chunk_len;
            memcpy( p_data, p1->data, data_read );
            break;
        }
    }
    return ( data_read );
}

/*
 * Allocate nvram_id to save new NVRAM chunk
 */
int hci_control_alloc_nvram_id( void )
{
    hci_control_nvram_chunk_t *p1 = p_nvram_first;
    int                        nvram_id;
    uint8_t                    allocated_key_pool_count;

    /* Go through the linked list of chunks */
    WICED_BT_TRACE ("hci_control_alloc_nvram_id\n");
    for ( nvram_id = HCI_CONTROL_FIRST_VALID_NVRAM_ID; p1 != NULL; nvram_id++ )
    {
        allocated_key_pool_count = 1;

        for ( p1 = p_nvram_first; p1 != NULL; p1 = (hci_control_nvram_chunk_t *)p1->p_next )
        {
            /* If the key buffer pool is becoming full, we need to notify the mcu and disable Pairing.
             * The mcu will need to delete some nvram entries and enable pairing in order to
             * pair with more devices */
            allocated_key_pool_count++;
            if ( ( allocated_key_pool_count == KEY_INFO_POOL_BUFFER_COUNT ) &&
                 ( hci_control_cb.pairing_allowed ) )
            {
                // Send Max Number of Paired Devices Reached event message
                wiced_transport_send_data( HCI_CONTROL_EVENT_MAX_NUM_OF_PAIRED_DEVICES_REACHED,
                        NULL, 0 );

                hci_control_cb.pairing_allowed = WICED_FALSE;
                wiced_bt_set_pairable_mode( hci_control_cb.pairing_allowed, 0 );
            }

            if ( p1->nvram_id == nvram_id )
            {
                /* this nvram_id is already used */
                break;
            }
        }
        if ( p1 == NULL )
        {
            break;
        }
    }
    WICED_BT_TRACE ( "hci_control_alloc_nvram_id:%d\n", nvram_id );
    return ( nvram_id );
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
 * hci_control_transport_status
 * This callback function is called when the MCU opens the Wiced UART
 */
static void hci_control_transport_status( wiced_transport_type_t type )
{
    WICED_BT_TRACE( "hci_control_transport_status %x \n", type );

    // Tell Host that App is started
    hci_control_send_device_started_evt();
}


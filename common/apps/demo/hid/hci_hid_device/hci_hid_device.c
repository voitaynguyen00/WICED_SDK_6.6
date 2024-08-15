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
 * HID Device Sample Application for 2070X devices.
 *
 * This application demonstrates two chip HID implementation.  The standard
 * Bluetooth controller, core Bluetooth host stack and HID profile
 * implementation fully implemented on the 2070X device.  The role of the
 * second MCU is to implement application specific logic.  This sample
 * assume host MCU to handle NVRAM access.  At the startup the MCU should
 * download the 2070X configuration file, and if appropriate paired host data.
 * This application assumes MCU to be connected over the UART although minor
 * changes are required to support different interface, for example SPI.
 *
 * The sample app performs as a Bluetooth keyboard.  To modify the behavior
 * change HID descriptor in the wiced_bt_sdp_db.c file.
 *
 * The sample Windows ClientControl application is provided to show sample
 * MCU implementation on Windows platform.
 *
 * Features demonstrated
 *  - WICED BT HID Device (HIDD) APIs
 *  - Handling of the UART WICED protocol
 *  - SDP and HID descriptor configuration
 *  - Setting of the Local Bluetooth Device address from the host MCU
 *
 * On startup this demo:
 *  - Initializes the Bluetooth sub system
 *  - Receive NVRAM information from the host
 *
 */

#include <string.h>
#include <stdio.h>
#include "sparcommon.h"
#include "wiced.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_hidd.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_event.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_nvram.h"
#include "wiced_hal_wdog.h"
#include "wiced_hci.h"
#include "wiced_memory.h"
#include "wiced_gki.h"
#include "wiced_transport.h"
#include "wiced_platform.h"
#include "wiced_timer.h"
#include "hci_hid_device.h"
#include "hci_control_api.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define WICED_HID_EIR_BUF_MAX_SIZE      264
#define HCI_CONTROL_KEY_STORAGE_VS_ID   0x10
#define HCI_HID_SIZE_QUEUE_BUFFER       100     // Set to 0 not to queue data of host's request received before connection is established

#define KEY_INFO_POOL_BUFFER_SIZE   145 //Size of the buffer used for holding the peer device key info
#define KEY_INFO_POOL_BUFFER_COUNT  10  //Correspond's to the number of peer devices

#define BCM920719 20719

#ifdef HIDD_QOS
/* QoS settings */
const wiced_bt_hidd_qos_info_t hci_hid_qos =
{
    {
        0,          /* qos_flags */
        1,          /* service type */
        800,        /* token rate (bytes/second) */
        8,          /* token_bucket_size (bytes) */
        0,          /* peak_bandwidth (bytes/second) */
        0xffffffff, /* latency(microseconds) */
        0xffffffff  /* delay_variation(microseconds) */
    },  /* ctrl_ch */
    {
        0,          /* qos_flags */
        2,          /* service type */
        300,        /* token rate (bytes/second) */
        4,          /* token_bucket_size (bytes) */
        300,        /* peak_bandwidth (bytes/second) */
        11250,      /* latency(microseconds) */
        11250       /* delay_variation(microseconds) */
    },  /* int_ch */
    {
        0,          /* qos_flags */
        2,          /* service type */
        400,        /* token rate (bytes/second) */
        8,          /* token_bucket_size (bytes) */
        800,        /* peak_bandwidth (bytes/second) */
        11250,      /* latency(microseconds) */
        11250       /* delay_variation(microseconds) */
    }   /* hci */
};
#endif

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/
typedef struct
{
    void     *p_next;
    uint16_t nvram_id;
    uint8_t  chunk_len;
    uint8_t  data[1];
} hci_hid_nvram_chunk_t;


/******************************************************
 *               Function Declarations
 ******************************************************/
static void     hci_hid_control_transport_status( wiced_transport_type_t type );
static int      hci_hid_init( void *param );
static wiced_bt_dev_status_t hci_hid_management_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
static void     hci_hid_device_write_eir( void );
static uint32_t hci_hid_handle_command( uint8_t *p_data, uint32_t length );
static void 	hci_hid_handle_reset_cmd( void );
static void     hci_control_handle_trace_enable( uint8_t *p_data );
static void     hci_hid_handle_set_local_bda( uint8_t *p_bda);
static void     hci_hid_handle_accept_pairing_cmd( BOOLEAN enable );
static void     hci_hid_handle_host_connect( );
static void     hci_hid_handle_host_disconnect( void );
static void     hci_hid_handle_send_report( uint8_t channel, uint8_t type, uint8_t *p_data, uint16_t length );
static void     hci_hid_process_get_report( uint32_t data, wiced_bt_hidd_event_data_t *p_event_data );
static void     hci_hid_process_set_report( uint32_t data, wiced_bt_hidd_event_data_t *p_event_data );
static uint8_t  hci_hid_write_nvram( uint16_t nvram_id, uint8_t data_len, uint8_t *p_data, BOOLEAN from_host );
static uint8_t  hci_hid_read_nvram( uint16_t nvram_id, uint8_t data_len, uint8_t *p_data );
static void     hci_hid_delete_nvram( uint16_t nvram_id );
static void     hci_hid_send_pairing_complete( uint8_t result, uint8_t *p_bda);
static void     hci_hid_send_hid_opened( void );
static void     hci_hid_send_hid_closed( uint32_t reason );
static void     hci_hid_send_virtual_cable_unplugged( void );
static void     hci_hid_process_output_report( uint32_t rpt_type, wiced_bt_hidd_data_t *p_hidd_data );
static void     hci_hid_queue_data( uint8_t channel, uint8_t type, uint8_t *p_data, uint16_t length );
static int      hci_hid_send_queued_data( void *param );
static void     hci_hid_handle_pairing_host_info( uint8_t *p_data, uint16_t length );
static void     hci_hid_hci_trace_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data );
static void     hci_control_misc_handle_get_version( void );
static void     hci_control_send_device_started_evt( void );
static void     hci_control_send_command_status_evt( uint16_t code, uint8_t status );

extern void hidd_pm_proc_mode_change( UINT8 hci_status, UINT8 mode, UINT16 interval );
L2C_API extern UINT8 L2CA_SetDesireRole (UINT8 new_role);
void wiced_bt_hidd_init (void);

/******************************************************
 *               Variable Definitions
 ******************************************************/
hci_hid_nvram_chunk_t   *p_nvram_first = NULL;
wiced_bt_device_link_keys_t  paired_device;

uint8_t pincode[] = { 0x30, 0x30, 0x30, 0x30 };

#define HCI_HID_DEVICE_STATE_IDLE       0
#define HCI_HID_DEVICE_STATE_PAIRING    1
#define HCI_HID_DEVICE_STATE_PAIRED     2
#define HCI_HID_DEVICE_STATE_CONNECTING 3
#define HCI_HID_DEVICE_STATE_CONNECTED  4

uint8_t hci_hid_device_state = HCI_HID_DEVICE_STATE_IDLE;

wiced_bt_buffer_pool_t* p_key_info_pool;//Pool for storing the  key info

uint8_t         hci_hid_report_queue[HCI_HID_SIZE_QUEUE_BUFFER];
uint32_t        hci_hid_report_queue_size = 0;

const wiced_transport_cfg_t  transport_cfg =
{
    WICED_TRANSPORT_UART,
    { WICED_TRANSPORT_UART_HCI_MODE, HCI_UART_DEFAULT_BAUD },
    { 0, 0},
    hci_hid_control_transport_status,
    hci_hid_handle_command,
    NULL
};


/******************************************************
 *               Function Definitions
 ******************************************************/
/*
 *  Application Start, the entry point to the application.
 */

APPLICATION_START( )
{
    wiced_result_t result;

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

    WICED_BT_TRACE( "BT HIDD APP START\n" );

    /* Register the dynamic configurations */
    wiced_bt_stack_init( hci_hid_management_cback, &hci_hid_cfg_settings, hci_hid_cfg_buf_pools );
}

/*
 *  Pass protocol traces up through the UART
 */
void hci_hid_hci_trace_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data )
{
    //WICED_BT_TRACE( "HCI event type %d len %d\n", type, length );
    //WICED_BT_TRACE_ARRAY(  p_data, length, "event data" );

    //send the trace
    wiced_transport_send_hci_trace( NULL, type, length, p_data  );
}

/*
 * Bluetooth management event handler
 */
static wiced_bt_dev_status_t hci_hid_management_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t              result = WICED_BT_SUCCESS;
    wiced_bt_dev_status_t       status = WICED_BT_SUCCESS;
    wiced_bt_device_address_t   bda;
    uint8_t                     bytes_written, bytes_read;
    wiced_bt_dev_pairing_cplt_t *p_pairing_complete;

    WICED_BT_TRACE( "hci_hid_management_cback: event: %d\n", event);
    switch ( event )
    {
    case BTM_ENABLED_EVT:
        /* Bluetooth controller and host stack enabled */
        WICED_BT_TRACE( "BT Enable status: 0x%02x\n", p_event_data->enabled.status);

        if ( p_event_data->enabled.status == WICED_BT_SUCCESS )
        {
            wiced_bt_dev_read_local_addr( bda );
            WICED_BT_TRACE( "Local Bluetooth Address: [%02X:%02X:%02X:%02X:%02X:%02X]\n", bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);

            /* Initialize the HID device */
            hci_hid_init( NULL );

            //Creating a buffer pool for holding the peer devices's key info
            p_key_info_pool = wiced_bt_create_pool( KEY_INFO_POOL_BUFFER_SIZE, KEY_INFO_POOL_BUFFER_COUNT );
            WICED_BT_TRACE( "wiced_bt_create_pool %x\n", p_key_info_pool );

            /*
             * The application running on the 20719 will not start if the Wiced UART is already
             * opened.
             * If we reach this point, this means that the Wiced UART is not opened, so we cannot
             * send any event.
             * The p_status_handler callback of the wiced_transport_cfg_t will be called as soon
             * as the Host MCU will open the UART.
             * We will send the HCI_CONTROL_EVENT_DEVICE_STARTED message in this callback.
             */
        }
        break;
        
    case BTM_PIN_REQUEST_EVT:
        WICED_BT_TRACE("BTM_PIN_REQUEST_EVT\n");
        {
            WICED_BT_TRACE("remote address= %B\n", p_event_data->pin_request.bd_addr);
            wiced_bt_dev_pin_code_reply(*p_event_data->pin_request.bd_addr,result/*WICED_BT_SUCCESS*/,4, &pincode[0]);
        }
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:
        /* Use the default security for BR/EDR*/
        WICED_BT_TRACE("\n\n BTM_PAIRING_IO_CAPABILITIES_REQUEST_EVT bda %B\n\n",
                        p_event_data->pairing_io_capabilities_br_edr_request.bd_addr);
        p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
        p_event_data->pairing_io_capabilities_br_edr_request.auth_req = BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_NO;
        break;

    case BTM_USER_CONFIRMATION_REQUEST_EVT:
        WICED_BT_TRACE("BTM_USER_CONFIRMATION_REQUEST_EVT\n");
        /* User confirmation request for pairing (sample app always accepts) */
        wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr );
        break;

    case BTM_PAIRING_COMPLETE_EVT:
        WICED_BT_TRACE("BTM_PAIRING_COMPLETE_EVT\n");
        p_pairing_complete = &p_event_data->pairing_complete;

        WICED_BT_TRACE( "Pairing Complete: %d %B\n", p_pairing_complete->pairing_complete_info.br_edr.status,
                        p_pairing_complete->bd_addr);

        hci_hid_send_pairing_complete( p_pairing_complete->pairing_complete_info.br_edr.status, p_pairing_complete->bd_addr );
        break;

    case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
        WICED_BT_TRACE("BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT\n");
        /* Request to restore local identity keys from NVRAM (requested during Bluetooth start up) */
        /* (sample app does not store keys to NVRAM. New local identity keys will be generated).   */
        status = WICED_BT_NO_RESOURCES;
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
        WICED_BT_TRACE("BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT\n");
        bytes_written = hci_hid_write_nvram( HCI_CONTROL_KEY_STORAGE_VS_ID, sizeof(wiced_bt_device_link_keys_t),
                        ( uint8_t* ) &p_event_data->paired_device_link_keys_update, FALSE );
        WICED_BT_TRACE( "NVRAM write: %d\n", bytes_written);
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        WICED_BT_TRACE("BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT\n");
        bytes_read = hci_hid_read_nvram( HCI_CONTROL_KEY_STORAGE_VS_ID, sizeof(wiced_bt_device_link_keys_t), ( uint8_t* ) &paired_device );

        if ( ( result == WICED_SUCCESS ) && ( memcmp( p_event_data->paired_device_link_keys_request.bd_addr, paired_device.bd_addr, BD_ADDR_LEN ) == 0 ) )
        {
            result = WICED_BT_SUCCESS;
            memcpy( &p_event_data->paired_device_link_keys_request, &paired_device, sizeof(wiced_bt_device_link_keys_t) );
            WICED_BT_TRACE( "Key retrieval success\n" );
        }
        else
        {
            result = WICED_BT_ERROR;
            WICED_BT_TRACE( "Key retrieval failure\n" );
        }
        break;

    case BTM_ENCRYPTION_STATUS_EVT:
        WICED_BT_TRACE( "Encryption status:%d\n", p_event_data->encryption_status.result );
        break;

    case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
        WICED_BT_TRACE( "BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT\n" );
        /* Request to store newly generated local identity keys to NVRAM */
        /* (sample app does not store keys to NVRAM) */
        break;

    case BTM_POWER_MANAGEMENT_STATUS_EVT:
        WICED_BT_TRACE( "BTM_POWER_MANAGEMENT_STATUS_EVT\n" );
        hidd_pm_proc_mode_change( p_event_data->power_mgmt_notification.hci_status, p_event_data->power_mgmt_notification.status, p_event_data->power_mgmt_notification.value );
        break;

    default:
        WICED_BT_TRACE( "BT management callback: unhandled event (%d)\n", event);
        break;
    }

    return ( status );
}

/*
 * This function handles events received from the HID stack
 */
static void hci_hid_cback( wiced_bt_hidd_cback_event_t event, uint32_t data, wiced_bt_hidd_event_data_t *p_event_data )
{
    uint8_t res_code;

    WICED_BT_TRACE( "hci_hid_cback: event: %d\n", event);
    switch ( event )
    {
    case WICED_BT_HIDD_EVT_OPEN:                                    /**< Connected to host with Interrupt and Control  Data = 1 if Virtual Cable */
        WICED_BT_TRACE( "HID connection opened to %B\n", p_event_data->host_bdaddr);
        hci_hid_device_state = HCI_HID_DEVICE_STATE_CONNECTED;

        /* Disable discoverability */
        wiced_bt_dev_set_discoverability( BTM_NON_DISCOVERABLE, BTM_DEFAULT_DISC_WINDOW, BTM_DEFAULT_DISC_INTERVAL );

        /* Disable connectability  */
        wiced_bt_dev_set_connectability( WICED_FALSE, BTM_DEFAULT_CONN_WINDOW, BTM_DEFAULT_CONN_INTERVAL );

//        BTM_EnableAsymmetricSniff(p_event_data->host_bdaddr, 10);

        wiced_app_event_serialize( hci_hid_send_queued_data, NULL );

        hci_hid_send_hid_opened( );
        break;

    case WICED_BT_HIDD_EVT_CLOSE:                                   /**< Connection with host is closed.            Data=Reason Code. */
        WICED_BT_TRACE( "WICED_BT_HIDD_EVT_CLOSE\n", event);
        hci_hid_send_hid_closed( data );
        if ( ( hci_hid_device_state == HCI_HID_DEVICE_STATE_CONNECTING ) || ( hci_hid_device_state == HCI_HID_DEVICE_STATE_CONNECTED ) )
            hci_hid_device_state = HCI_HID_DEVICE_STATE_PAIRED;
        WICED_BT_TRACE( "HID connection closed (reason code = 0x%x)\n", data);
        break;

    case WICED_BT_HIDD_EVT_RETRYING:                                /**< Lost connection is being re-connected.     Data=Retrial number */
        WICED_BT_TRACE( "WICED_BT_HIDD_EVT_RETRYING\n", event);
        break;

    case WICED_BT_HIDD_EVT_MODE_CHG:                                /**< Device changed power mode.                 Data=new power mode */
        WICED_BT_TRACE( "HID power mode change (new mode = 0x%x)\n", data );
        break;

    case WICED_BT_HIDD_EVT_PM_FAILED:                               /**< Device power mode change failed */
        WICED_BT_TRACE( "HID power mode change failed\n" );
        break;

    case WICED_BT_HIDD_EVT_CONTROL:                                 /**< Host sent HID_CONTROL                      Data=Control Operation */
        WICED_BT_TRACE( "WICED_BT_HIDD_EVT_CONTROL\n" );
        if ( data == HID_PAR_CONTROL_VIRTUAL_CABLE_UNPLUG )
        {
            hci_hid_delete_nvram( HCI_CONTROL_KEY_STORAGE_VS_ID );
            hci_hid_send_virtual_cable_unplugged( );
        }
        break;

    case WICED_BT_HIDD_EVT_GET_REPORT:                              /**< Host sent GET_REPORT                       Data=Length,  pdata=get_report details */
        WICED_BT_TRACE( "HID GET_REPORT received (len=%d)\n", (int)data );
        hci_hid_process_get_report( data, p_event_data );
        break;

    case WICED_BT_HIDD_EVT_SET_REPORT:                              /**< Host sent SET_REPORT                       Data=Length pdata=details.*/
        WICED_BT_TRACE( "HID SET_REPORT received (len=%d)\n", (int)data );
        hci_hid_process_set_report( data, p_event_data );
        break;

    case WICED_BT_HIDD_EVT_GET_PROTO:                               /**< Host sent GET_PROTOCOL                     Data=NA*/
        WICED_BT_TRACE( "HID GET_PROTOCOL received (len=%d)\n", (int)data );
        wiced_bt_hidd_hand_shake (HID_PAR_HANDSHAKE_RSP_ERR_UNSUPPORTED_REQ);
        break;

    case WICED_BT_HIDD_EVT_SET_PROTO:                               /**< Host sent SET_PROTOCOL         Data=1 for Report, 0 for Boot*/
        WICED_BT_TRACE( "HID SET_PROTOCOL received (protocol = %s)\n", ( data ? "REPORT_MODE" : "BOOT_MODE" ) );
        wiced_bt_hidd_hand_shake (HID_PAR_HANDSHAKE_RSP_ERR_UNSUPPORTED_REQ);
        break;


    case WICED_BT_HIDD_EVT_GET_IDLE:                                /**< Host sent GET_IDLE                         Data=NA */
        WICED_BT_TRACE( "HID GET_IDLE received\n" );
        wiced_bt_hidd_hand_shake( HID_PAR_HANDSHAKE_RSP_SUCCESS );
        break;

    case WICED_BT_HIDD_EVT_SET_IDLE:                                /**< Host sent SET_IDLE                         Data=Idle Rate */
        WICED_BT_TRACE( "HID SET_IDLE received (idle rate:%d)\n", (int)data );
        wiced_bt_hidd_hand_shake( HID_PAR_HANDSHAKE_RSP_SUCCESS );
        break;

    case WICED_BT_HIDD_EVT_DATA:
        WICED_BT_TRACE( "HID DATA received (len:%d)\n", p_event_data->data.len );
        hci_hid_process_output_report( data, &p_event_data->data );
        break;

    case WICED_BT_HIDD_EVT_DATC:
        WICED_BT_TRACE( "HID DATC received\n" );
        break;

    case WICED_BT_HIDD_EVT_L2CAP_CONGEST:                           /**< L2CAP channel congested */
        WICED_BT_TRACE( "HID Congestion\n" );
        break;

    default:
        WICED_BT_TRACE( "unhandled HID event 0x%x\n", event );
        break;
    }
}

/*
 * Initialize the HID Device
 */
int hci_hid_init( void * param )
{
    /* Initialize SDP server database for rfcble_app */
    wiced_bt_sdp_db_init( ( uint8_t * ) wiced_bt_sdp_db, wiced_bt_sdp_db_size );

    hci_hid_device_write_eir( );

    wiced_bt_hidd_init( );

    WICED_BT_TRACE( "HIDD initialized\n" );

    return 0;
}

/*
 *  Prepare extended inquiry response data.  Current version HID service.
 */
void hci_hid_device_write_eir( void )
{
    uint8_t *pBuf = NULL;
    uint8_t *p = NULL;
    uint8_t length = strlen( ( char * ) hci_hid_cfg_settings.device_name );

    /* Allocate buffer of sufficient size to hold the EIR data */
    pBuf = ( uint8_t* ) wiced_bt_get_buffer( WICED_HID_EIR_BUF_MAX_SIZE );
    if ( NULL == pBuf )
    {
        WICED_BT_TRACE( "ERROR: Out of memory for EIR ...\n");
        return;
    }

    p = pBuf;

    /* Update the length of the name (Account for the type field(1 byte) as well) */
    *p++ = ( 1 + length );
    *p++ = 0x09;            // EIR type full name

    /* Copy the device name */memcpy( p, hci_hid_cfg_settings.device_name, length );
    p += length;

    *p++ = ( 1 * 2 ) + 1;     // length of services + 1
    *p++ = 0x02;            // EIR type full list of 16 bit service UUIDs
    *p++ = UUID_SERVCLASS_HUMAN_INTERFACE & 0xff;
    *p++ = ( UUID_SERVCLASS_HUMAN_INTERFACE >> 8 ) & 0xff;
    *p++ = 0;

    // print EIR data
    wiced_bt_trace_array( "EIR :", ( uint8_t* ) ( pBuf + 1 ), MIN( p - ( uint8_t* ) pBuf, 100 ) );
    wiced_bt_dev_write_eir( pBuf, ( uint16_t ) ( p - pBuf ) );

    /* Allocated buffer not anymore needed. Free it */
    wiced_bt_free_buffer( pBuf );

    return;
}

/*
 * process get report from the host
 */
void hci_hid_process_get_report( uint32_t data, wiced_bt_hidd_event_data_t *p_event_data )
{
    uint8_t report_type = p_event_data->get_rep.rep_type;
    uint8_t report_id   = p_event_data->get_rep.rep_id;

    WICED_BT_TRACE( "%s: Get Report Type:%d ReportId:0x%x\n", __FUNCTION__, report_type, report_id );

    if ( report_type == HID_PAR_REP_TYPE_INPUT )
    {
        switch ( report_id )
        {
        case HCI_HID_REPORT_ID_KEYBOARD:
            wiced_bt_hidd_send_data( WICED_TRUE, HID_PAR_REP_TYPE_INPUT, hci_hid_last_keyboard_report, HCI_HID_KEYBOARD_REPORT_SIZE );
            break;

        case HCI_HID_REPORT_ID_MOUSE:
            wiced_bt_hidd_send_data( WICED_TRUE, HID_PAR_REP_TYPE_INPUT, hci_hid_last_mouse_report, HCI_HID_MOUSE_REPORT_SIZE );
            break;

        case HCI_HID_REPORT_ID_CONSUMER:
            wiced_bt_hidd_send_data( WICED_TRUE, HID_PAR_REP_TYPE_INPUT, hci_hid_last_consumer_report, HCI_HID_CONSUMER_REPORT_SIZE );
            break;

        default:
            wiced_bt_hidd_hand_shake( HID_PAR_HANDSHAKE_RSP_ERR_INVALID_REP_ID );
            break;
        }
    }
    else if (report_type == HID_PAR_REP_TYPE_OUTPUT)
    {
        switch ( report_id )
        {
        case HCI_HID_REPORT_ID_KEYBOARD:
            wiced_bt_hidd_send_data( WICED_TRUE, HID_PAR_REP_TYPE_OUTPUT, hci_hid_last_keyboard_out_report, HCI_HID_KEYBOARD_OUT_REPORT_SIZE );
            break;

        default:
            wiced_bt_hidd_hand_shake( HID_PAR_HANDSHAKE_RSP_ERR_INVALID_REP_ID );
            break;
        }
    }
    else
    {
        // no (report_id == HID_PAR_REP_TYPE_FEATURE)
        wiced_bt_hidd_hand_shake( HID_PAR_HANDSHAKE_RSP_ERR_INVALID_PARAM );
    }
}

/*
 * process set report from the host
 */
void hci_hid_process_set_report( uint32_t data, wiced_bt_hidd_event_data_t *p_event_data )
{
    uint8_t res_code    = HID_PAR_HANDSHAKE_RSP_SUCCESS;
    uint8_t report_type = data;
    uint8_t report_id   = p_event_data->data.p_data[0];
    uint8_t report_len  = p_event_data->data.len;

    WICED_BT_TRACE( "%s: Set Report Type:%d ReportId:0x%x\n", __FUNCTION__, report_type, report_id );

    if ( report_type == HID_PAR_REP_TYPE_INPUT )
    {
        switch ( report_id )
        {
        case HCI_HID_REPORT_ID_KEYBOARD:
            if ( report_len != HCI_HID_KEYBOARD_REPORT_SIZE )
            {
                res_code = HID_PAR_HANDSHAKE_RSP_ERR_INVALID_PARAM;
            }
            break;

        case HCI_HID_REPORT_ID_MOUSE:
            if ( report_len != HCI_HID_MOUSE_REPORT_SIZE )
            {
                res_code = HID_PAR_HANDSHAKE_RSP_ERR_INVALID_PARAM;
            }
            break;

        case HCI_HID_REPORT_ID_CONSUMER:
            if ( p_event_data->data.len != HCI_HID_CONSUMER_REPORT_SIZE )
            {
                res_code = HID_PAR_HANDSHAKE_RSP_ERR_INVALID_PARAM;
            }
            break;

        default:
            res_code = HID_PAR_HANDSHAKE_RSP_ERR_INVALID_REP_ID;
            break;
        }
    }
    else if ( report_type == HID_PAR_REP_TYPE_OUTPUT )
    {
        if ( report_id == HCI_HID_REPORT_ID_KEYBOARD )
        {
            if ( p_event_data->data.len != HCI_HID_KEYBOARD_OUT_REPORT_SIZE )
            {
                res_code = HID_PAR_HANDSHAKE_RSP_ERR_INVALID_PARAM;
            }
        }
        else
        {
            res_code = HID_PAR_HANDSHAKE_RSP_ERR_INVALID_REP_ID;
        }
    }
    else
    {
        res_code = HID_PAR_HANDSHAKE_RSP_ERR_INVALID_PARAM;
    }
    wiced_bt_hidd_hand_shake( res_code );
}

/*
 * Handle command received over the UART.  First buffer of the command is the opcode
 * of the operation.  Rest are parameters specific for particular command.
 *
 */
uint32_t hci_hid_handle_command( uint8_t *p_data, uint32_t length )
{
    uint16_t handle;
    uint8_t  hs_cmd;
    uint8_t  bytes_written;
    uint16_t opcode;
    uint16_t payload_len;
    uint8_t* p_rx_buf = p_data;

    if ( !p_rx_buf )
    {
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }
    //Expected minimum 4 byte as the wiced header
    if( length < 4 )
    {
        WICED_BT_TRACE("invalid params\n");
        wiced_transport_free_buffer( p_rx_buf );
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }


    opcode = p_data[0] + ( p_data[1] << 8 );     // Get opcode
    payload_len = p_data[2] + ( p_data[3] << 8 );     // Get len
    p_data += 4;
    length -= 4;

    WICED_BT_TRACE( "hci_hid_handle_command 0x%04x\n", opcode );

    switch ( opcode )
    {
    case HCI_CONTROL_COMMAND_RESET:
        WICED_BT_TRACE( "HCI_CONTROL_COMMAND_RESET\n", bytes_written );
        hci_hid_handle_reset_cmd( );
        break;

    case HCI_CONTROL_COMMAND_TRACE_ENABLE:
        WICED_BT_TRACE( "HCI_CONTROL_COMMAND_TRACE_ENABLE\n", bytes_written );
        hci_control_handle_trace_enable( p_data );
        break;

    case HCI_CONTROL_COMMAND_SET_LOCAL_BDA:
        WICED_BT_TRACE( "HCI_CONTROL_COMMAND_SET_LOCAL_BDA\n", bytes_written );
        hci_hid_handle_set_local_bda( p_data );
        break;

    case HCI_CONTROL_COMMAND_PUSH_NVRAM_DATA:
        bytes_written = hci_hid_write_nvram( p_data[0] | ( p_data[1] << 8 ), length - 2, &p_data[2], TRUE );
        WICED_BT_TRACE( "NVRAM write: %d\n", bytes_written );
        break;

    case HCI_CONTROL_HIDD_COMMAND_ACCEPT_PAIRING:
        WICED_BT_TRACE( "HCI_CONTROL_HIDD_COMMAND_ACCEPT_PAIRING\n" );
        hci_hid_handle_accept_pairing_cmd( *p_data );
        break;

    case HCI_CONTROL_HID_COMMAND_PUSH_PAIRING_HOST_INFO:
        WICED_BT_TRACE( "HCI_CONTROL_HID_COMMAND_PUSH_PAIRING_HOST_INFO\n" );
        hci_hid_handle_pairing_host_info( p_data, length );
        break;

    case HCI_CONTROL_HIDD_COMMAND_CONNECT:
        WICED_BT_TRACE( "HCI_CONTROL_HIDD_COMMAND_CONNECT\n" );
        hci_hid_handle_host_connect( );
        break;

    case HCI_CONTROL_HIDD_COMMAND_DISCONNECT:
        WICED_BT_TRACE( "HCI_CONTROL_HIDD_COMMAND_DISCONNECT\n" );
        hci_hid_handle_host_disconnect( );
        break;

    case HCI_CONTROL_HIDD_COMMAND_SEND_REPORT:
        WICED_BT_TRACE( "HCI_CONTROL_HIDD_COMMAND_SEND_REPORT\n" );
        hci_hid_handle_send_report( p_data[0], p_data[1], &p_data[2], ( uint16_t ) ( length - 2 ) );
        break;

    case HCI_CONTROL_MISC_COMMAND_GET_VERSION:
        WICED_BT_TRACE( "HCI_CONTROL_MISC_COMMAND_GET_VERSION\n" );
        hci_control_misc_handle_get_version();
        break;

    default:
        WICED_BT_TRACE( "command %d not handled!\n" );
        break;
    }

    //Freeing the buffer in which data is received
    wiced_transport_free_buffer( p_rx_buf );
    return ( 0 );
}

/*
 * handle reset command from UART
 */
void hci_hid_handle_reset_cmd( void )
{
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
        /* Register callback for receiving hci traces */
        wiced_bt_dev_register_hci_trace( hci_hid_hci_trace_cback );
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
 * Handle host command to set device pairable.  This is typically a HID device button push.
 */
void hci_hid_handle_accept_pairing_cmd( BOOLEAN enable )
{
    wiced_bt_hidd_reg_info_t hidd_reg_info = { { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff }, /* Host bd address (FF:FF:FF:FF:FF:FF = any host) */

    // Removing this for now. Broadcom's BTW on Win7 does not negotiate QoS parameters correctly.
    // Which causes connection failure. We may want to restore when BTW is not
    // supported anymore.  Most of the HID devices do not use it regardless of the spec recommendation.
    //( wiced_bt_hidd_qos_info_t * ) &hci_hid_qos, /* QoS Information */
    NULL, /* QoS Information */

    hci_hid_cback /* Event callback  */
    };

    WICED_BT_TRACE( "accept_pairing_cmd %d\n", enable );

    if ( !enable )
    {
        /* Disable discoverability */
        wiced_bt_dev_set_discoverability( BTM_NON_DISCOVERABLE, BTM_DEFAULT_DISC_WINDOW, BTM_DEFAULT_DISC_INTERVAL );

        /* Disable connectability  */
        wiced_bt_dev_set_connectability( WICED_FALSE, BTM_DEFAULT_CONN_WINDOW, BTM_DEFAULT_CONN_INTERVAL );

        /* do not allow peer to pair */
        wiced_bt_set_pairable_mode(WICED_FALSE, 0);

        return;
    }

    /* disconnect any connections if active */
    wiced_bt_hidd_disconnect( );

    /* Host instructed us to start pairing.  Delete saved information from the NVRAM */
    /* If pairing fails, host better restor the NVRAM */
    hci_hid_delete_nvram( HCI_CONTROL_KEY_STORAGE_VS_ID );

    /*
     * We will deregister HIDD without waiting the result of the HIDD Disconnection above.
     * Call wiced_bt_hidd_init to reset HIDD state.
     */
    wiced_bt_hidd_init();

    /* deregister with HIDD and send new registration accepting connections from all devices */
    wiced_bt_hidd_deregister( );

    hci_hid_device_state = HCI_HID_DEVICE_STATE_PAIRING;

    wiced_bt_hidd_register( &hidd_reg_info );

    /* Enable connectability (use default connectability window/interval) */
    wiced_bt_dev_set_connectability( WICED_TRUE, BTM_DEFAULT_DISC_WINDOW, BTM_DEFAULT_DISC_INTERVAL );

    /* Enable discoverability (use default discoverability window/interval) */
    wiced_bt_dev_set_discoverability( BTM_GENERAL_DISCOVERABLE, BTM_DEFAULT_DISC_WINDOW, BTM_DEFAULT_DISC_INTERVAL );

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);
}

/*
 * Handle command to establish connection to a known host
 */
void hci_hid_handle_host_connect( void )
{
    WICED_BT_TRACE( "connect state:%d\n", hci_hid_device_state );

    switch ( hci_hid_device_state )
    {
    case HCI_HID_DEVICE_STATE_IDLE:
    case HCI_HID_DEVICE_STATE_PAIRING:
    case HCI_HID_DEVICE_STATE_CONNECTING:
    case HCI_HID_DEVICE_STATE_CONNECTED:
        break;

    case HCI_HID_DEVICE_STATE_PAIRED:
        hci_hid_device_state = HCI_HID_DEVICE_STATE_CONNECTING;
        wiced_bt_hidd_connect( );
        break;
    }
}

/*
 * Handle command to release HID connection
 */
void hci_hid_handle_host_disconnect( void )
{
    WICED_BT_TRACE( "HIDD Disconnect state:%d\n", hci_hid_device_state );

    switch ( hci_hid_device_state )
    {
    case HCI_HID_DEVICE_STATE_IDLE:
    case HCI_HID_DEVICE_STATE_PAIRING:
    case HCI_HID_DEVICE_STATE_CONNECTING:
    case HCI_HID_DEVICE_STATE_PAIRED:
        WICED_BT_TRACE( "Err: HIDD Disconnect wrong state:%d\n", hci_hid_device_state );
        break;

    case HCI_HID_DEVICE_STATE_CONNECTED:
        wiced_bt_hidd_disconnect( );
        break;
    }
}

/*
 * Handle host command to send HID report.
 */
void hci_hid_handle_send_report( uint8_t channel, uint8_t type, uint8_t *p_data, uint16_t length )
{
    WICED_BT_TRACE( "send report %d bytes state:%d\n", length, hci_hid_device_state );

    switch ( hci_hid_device_state )
    {
    case HCI_HID_DEVICE_STATE_IDLE:
    case HCI_HID_DEVICE_STATE_PAIRING:
        WICED_BT_TRACE ( "Err: No known HID host\n" );
        break;

    case HCI_HID_DEVICE_STATE_CONNECTING:
        WICED_BT_TRACE ( "HID Connecting...\n" );
        hci_hid_queue_data( channel, type, p_data, length );
        break;

    case HCI_HID_DEVICE_STATE_PAIRED:
        WICED_BT_TRACE ( "Initiate HID Connection\n" );
        hci_hid_device_state = HCI_HID_DEVICE_STATE_CONNECTING;
        wiced_bt_hidd_connect( );
        hci_hid_queue_data( channel, type, p_data, length );
        break;

    case HCI_HID_DEVICE_STATE_CONNECTED:
        wiced_bt_hidd_send_data( channel == HCI_CONTROL_HID_REPORT_CHANNEL_CONTROL, type, p_data, length );

        switch ( *p_data )
        {
        case HCI_HID_REPORT_ID_KEYBOARD:
            memcpy( hci_hid_last_keyboard_report, p_data, HCI_HID_KEYBOARD_REPORT_SIZE < length ? HCI_HID_KEYBOARD_REPORT_SIZE : length );
            break;

        case HCI_HID_REPORT_ID_MOUSE:
            memcpy( hci_hid_last_mouse_report, p_data, HCI_HID_MOUSE_REPORT_SIZE < length ? HCI_HID_MOUSE_REPORT_SIZE : length );
            break;

        case HCI_HID_REPORT_ID_CONSUMER:
            memcpy( hci_hid_last_consumer_report, p_data, HCI_HID_CONSUMER_REPORT_SIZE < length ? HCI_HID_CONSUMER_REPORT_SIZE : length );
            break;
        }
    }
}

/*
 * Queue HID report if request from the host received before connection is established.
 */
void hci_hid_queue_data( uint8_t channel, uint8_t type, uint8_t *p_data, uint16_t length )
{
#if HCI_HID_SIZE_QUEUE_BUFFER
    int i;

    // report too long to save
    if ( 4 + length >= HCI_HID_SIZE_QUEUE_BUFFER )
        return;

    // if queue is full, it is probably a good idea to save more recent reports
    while ( hci_hid_report_queue_size + 4 + length >= HCI_HID_SIZE_QUEUE_BUFFER )
    {
        // drop first report
        uint16_t report_len = hci_hid_report_queue[2] + ( hci_hid_report_queue[3] << 8 );
        for ( i = 0; i < hci_hid_report_queue_size - ( report_len + 4 ); i++ )
        {
            hci_hid_report_queue[i] = hci_hid_report_queue[report_len + 4 + i];
        }
        hci_hid_report_queue_size -= ( report_len + 4 );
    }
    hci_hid_report_queue[hci_hid_report_queue_size++] = channel;
    hci_hid_report_queue[hci_hid_report_queue_size++] = type;
    hci_hid_report_queue[hci_hid_report_queue_size++] = length & 0xff;
    hci_hid_report_queue[hci_hid_report_queue_size++] = (length >> 8 ) & 0xff;
    memcpy( &hci_hid_report_queue[hci_hid_report_queue_size], p_data, length );
    hci_hid_report_queue_size += length;

    WICED_BT_TRACE( "report queued total len:%d\n", hci_hid_report_queue_size );
#endif
}

/*
 * Send queued reports when connection is established
 */
int hci_hid_send_queued_data( void *param )
{
    int i = 0;
    while ( hci_hid_report_queue_size != 0 )
    {
        uint16_t report_len = hci_hid_report_queue[i + 2] + ( hci_hid_report_queue[i + 3] >> 8 );
        wiced_bt_hidd_send_data( hci_hid_report_queue[i] == HCI_CONTROL_HID_REPORT_CHANNEL_CONTROL, hci_hid_report_queue[i + 1],
                        &hci_hid_report_queue[i + 4], report_len );

        hci_hid_report_queue_size -= ( report_len + 4 );
        i                         += ( report_len + 4 );
    }
    return 0;
}

/*
 * handle command to send local Bluetooth device address
 */
void hci_hid_handle_set_local_bda( uint8_t *bda )
{
    BD_ADDR bd_addr;
    STREAM_TO_BDADDR(bd_addr,bda);
    wiced_bt_set_local_bdaddr( bd_addr, BLE_ADDR_PUBLIC );
    WICED_BT_TRACE( "Local Bluetooth Address: [%02X:%02X:%02X:%02X:%02X:%02X]\n", bda[5], bda[4], bda[3], bda[2], bda[1], bda[0] );

    L2CA_SetDesireRole(HCI_ROLE_SLAVE);

    WICED_BT_TRACE( "Set Name %s\n", hci_hid_cfg_settings.device_name);
}

/*
 * Send notification to the host that pairing has been completed
 */
void hci_hid_send_pairing_complete( uint8_t result, uint8_t *p_bda )
{
    uint8_t tx_buf[12];
    uint8_t *p = tx_buf;
    int i;

    *p++ = result;
    for ( i = 0; i < 6; i++ )
        *p++ = p_bda[5 - i];

    *p++ = BT_DEVICE_TYPE_BREDR;

    wiced_transport_send_data( HCI_CONTROL_EVENT_PAIRING_COMPLETE, tx_buf, ( int ) ( p - tx_buf ) );
}

/*
 * Send HID OPENED event to the host
 */
void hci_hid_send_hid_opened( void )
{
    wiced_transport_send_data( HCI_CONTROL_HID_EVENT_OPENED, NULL, 0 );
}

/*
 * Send HID CLOSED event to the host
 */
void hci_hid_send_hid_closed( uint32_t reason )
{
    uint8_t u8 = ( uint8_t ) reason;
    wiced_transport_send_data( HCI_CONTROL_HID_EVENT_CLOSED, &u8, 1 );
}

/*
 * Send Virtual Cable Unplugged event to the host
 */
void hci_hid_send_virtual_cable_unplugged( void )
{
    wiced_transport_send_data( HCI_CONTROL_HID_EVENT_VIRTUAL_CABLE_UNPLUGGED, NULL, 0 );
}

/*
 * Send Data event to the host
 */
void hci_hid_process_output_report( uint32_t rpt_type, wiced_bt_hidd_data_t *p_hidd_data )
{
    uint8_t tx_buf[202];

    if ( ( p_hidd_data->p_data[0] == HCI_HID_REPORT_ID_KEYBOARD ) && ( p_hidd_data->len == HCI_HID_KEYBOARD_OUT_REPORT_SIZE ) )
    {
        WICED_BT_TRACE( "%s type:%d len:%d\n", __FUNCTION__, rpt_type, p_hidd_data->len );

        tx_buf[0] = ( uint8_t ) rpt_type;
        memcpy( &tx_buf[1], p_hidd_data->p_data, p_hidd_data->len < 200 ? p_hidd_data->len : 200 );
        memcpy( hci_hid_last_keyboard_out_report, p_hidd_data->p_data, p_hidd_data->len < HCI_HID_KEYBOARD_OUT_REPORT_SIZE ? p_hidd_data->len : HCI_HID_KEYBOARD_OUT_REPORT_SIZE );
        wiced_transport_send_data( HCI_CONTROL_HID_EVENT_DATA, tx_buf, ( int ) ( p_hidd_data->len < 200 ? 1 + p_hidd_data->len : 200 ) );
    }
    else
    {
        WICED_BT_TRACE( "%s ignored type:%d len:%d\n", __FUNCTION__, rpt_type, p_hidd_data->len );
    }
}

/*
 * Host sends information about previously paired host
 */
void hci_hid_handle_pairing_host_info( uint8_t *p_data, uint16_t length )
{
    wiced_bt_device_link_keys_t *p_host_info = ( wiced_bt_device_link_keys_t * ) p_data;
    uint8_t bytes_written;
    wiced_bt_hidd_reg_info_t hidd_reg_info = { { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff }, /* Host bd address (FF:FF:FF:FF:FF:FF = any host) */
#ifdef HIDD_QOS
    ( wiced_bt_hidd_qos_info_t * ) &hci_hid_qos, /* QoS Information */
#else
        NULL,
#endif
    hci_hid_cback /* Event callback  */
    };

    if ( length < sizeof(wiced_bt_device_link_keys_t) )
    {
        WICED_BT_TRACE( " pairing_host_info bad length:%d\n", length );
        return;
    }
    bytes_written = hci_hid_write_nvram( HCI_CONTROL_KEY_STORAGE_VS_ID, sizeof(wiced_bt_device_link_keys_t), p_data, TRUE );

    wiced_bt_hidd_deregister( );

    hci_hid_device_state = HCI_HID_DEVICE_STATE_PAIRED;

    memcpy( hidd_reg_info.host_addr, p_host_info->bd_addr, BD_ADDR_LEN );

    wiced_bt_hidd_register( &hidd_reg_info );
}

/*
 * Write NVRAM function is called to store information in the RAM.  This can be called when
 * stack requires persistent storage, for example to save link keys.  In this case
 * data is also formatted and send to the host for real NVRAM storage.  The same function is
 * called when host pushes NVRAM chunks during the startup.  Parameter from_host in this
 * case is set to FALSE indicating that data does not need to be forwarded.
 */
uint8_t hci_hid_write_nvram( uint16_t nvram_id, uint8_t data_len, uint8_t *p_data, BOOLEAN from_host )
{
    uint8_t tx_buf[257];
    uint8_t *p = tx_buf;
    hci_hid_nvram_chunk_t *p1, *p2;

    /* first check if this ID is being reused and release the memory chunk */
    hci_hid_delete_nvram( nvram_id );

    if ( ( p1 = ( hci_hid_nvram_chunk_t * ) wiced_bt_get_buffer_from_pool( p_key_info_pool ) ) == NULL )
    {
        WICED_BT_TRACE( "Failed to alloc:%d\n", data_len );
        return ( 0 );
    }
    if ( wiced_bt_get_buffer_size( p1 ) < ( sizeof( hci_hid_nvram_chunk_t ) + data_len - 1 ) )
    {
        WICED_BT_TRACE( "Insufficient buffer size, Buff Size %d, Len %d  \n",
                        wiced_bt_get_buffer_size( p1 ), ( sizeof( hci_hid_nvram_chunk_t ) + data_len - 1 ) );
        wiced_bt_free_buffer( p1 );
        return ( 0 );
    }
    p1->p_next = p_nvram_first;
    p1->nvram_id = nvram_id;
    p1->chunk_len = data_len;
    memcpy( p1->data, p_data, data_len );

    p_nvram_first = p1;

    /* If NVRAM chunk arrived from host, no need to send it back, otherwise send over transport */
    if ( !from_host )
    {
        *p++ = nvram_id & 0xff;
        *p++ = ( nvram_id >> 8 ) & 0xff;
        memcpy( p, p_data, data_len );

        wiced_transport_send_data( HCI_CONTROL_EVENT_NVRAM_DATA, tx_buf, ( int ) ( data_len + 2 ) );
    }
    return ( data_len );
}

/*
 * Read NVRAM actually finds the memory chunk in the RAM
 */
uint8_t hci_hid_read_nvram( uint16_t nvram_id, uint8_t data_len, uint8_t *p_data )
{
    hci_hid_nvram_chunk_t *p1;
    uint8_t data_read = 0;

    /* Go through the linked list of chunks */
    for ( p1 = p_nvram_first; p1 != NULL; p1 = ( hci_hid_nvram_chunk_t * ) p1->p_next )
    {
        if ( p1->nvram_id == nvram_id )
        {
            data_read = ( data_len < p1->chunk_len ) ? data_len : p1->chunk_len;
            memcpy( p_data, p1->data, data_read );
        }
    }
    return ( data_read );
}

/*
 * Delete NVRAM function is called when host deletes NVRAM chunk from the persistent storage.
 */
void hci_hid_delete_nvram( uint16_t nvram_id )
{
    hci_hid_nvram_chunk_t *p1, *p2;

    if ( p_nvram_first == NULL )
        return;

    /* Special case when need to remove the first chunk */
    if ( ( p_nvram_first != NULL ) && ( p_nvram_first->nvram_id == nvram_id ) )
    {
        p1 = p_nvram_first;

        WICED_BT_TRACE( "Removing device from bonded list\n" );
        wiced_bt_dev_delete_bonded_device(p1->data);

        p_nvram_first = ( hci_hid_nvram_chunk_t * ) p_nvram_first->p_next;
        wiced_bt_free_buffer( p1 );
        return;
    }

    /* Go through the linked list of chunks */
    for ( p1 = p_nvram_first; p1 != NULL; p1 = ( hci_hid_nvram_chunk_t * ) p1->p_next )
    {
        p2 = ( hci_hid_nvram_chunk_t * ) p1->p_next;
        if ( ( p2 != NULL ) && ( p2->nvram_id == nvram_id ) )
        {
            p1->p_next = p2->p_next;
            WICED_BT_TRACE( "Removing device from bonded list(2)\n" );
            wiced_bt_dev_delete_bonded_device(p2->data);
            wiced_bt_free_buffer( p2 );
            return;
        }
    }
}

/* Handle get version command */
void hci_control_misc_handle_get_version( void )
{
    uint8_t   tx_buf[15];
    uint8_t   cmd = 0;

    uint32_t  chip = BCM920719;

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
    tx_buf[cmd++] = HCI_CONTROL_GROUP_HIDD;

    wiced_transport_send_data( HCI_CONTROL_MISC_EVENT_VERSION, tx_buf, cmd );
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
 *  Send Device Started event through UART
 */
void hci_control_send_device_started_evt( void )
{
    wiced_transport_send_data( HCI_CONTROL_EVENT_DEVICE_STARTED, NULL, 0 );

    WICED_BT_TRACE( "maxLinks:%d maxChannels:%d maxpsm:%d rfcom max links%d, rfcom max ports:%d\n",
            hci_hid_cfg_settings.l2cap_application.max_links,
            hci_hid_cfg_settings.l2cap_application.max_channels,
            hci_hid_cfg_settings.l2cap_application.max_psm,
            hci_hid_cfg_settings.rfcomm_cfg.max_links,
            hci_hid_cfg_settings.rfcomm_cfg.max_ports );
}

/*
* transfer command status event to UART
*/
static void hci_control_send_command_status_evt( uint16_t code, uint8_t status )
{
    wiced_transport_send_data( code, &status, 1 );
}


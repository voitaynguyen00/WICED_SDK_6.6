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
 * BLE HID Device Sample Application for 2070X devices.
 *
 * Refer to Bluetooth SIG HID over GATT Profile 1.0
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
 * change HID descriptor in the hci_ble_hid_dev_db.c file.
 *
 * The sample Windows ClientControl application is provided to show sample
 * MCU implementation on Windows platform.
 *
 * Features demonstrated
 *  - WICED BLE HID Device
 *  - Handling of the UART WICED protocol
 *  - GATT and HID descriptor configuration
 *  - Setting of the Local Bluetooth Device address from the host MCU
 *
 * On startup this demo:
 *  - Initializes the Bluetooth sub system
 *  - Receive NVRAM information from the host
 *
 */
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced_hal_gpio.h"
#include "wiced_bt_uuid.h"
#include "wiced_hal_nvram.h"
#if ( defined(CYW20706A2) || defined(CYW20719B1) || defined(CYW20719B0) || defined(CYW20721B1) || defined(CYW20735B0) || defined(CYW43012C0) )
#include "wiced_bt_app_common.h"
#include "wiced_bt_app_hal_common.h"
#endif
#include "wiced_platform.h"
#include "wiced_bt_trace.h"
#include "hci_ble_hid_dev_db.h"
#include "wiced_bt_cfg.h"
#include "hci_control_api.h"
#include "wiced_bt_gatt.h"
#include "wiced_transport.h"
#include "wiced_bt_stack.h"
#include "wiced_hal_wdog.h"
#include "string.h"
#include "hci_ble_hid_dev.h"
#ifdef AUDIO_SUPPORT
#include "hci_ble_hid_dev_audio.h"
#endif

#define HID_OVER_UART

extern const wiced_bt_cfg_settings_t hci_ble_hid_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t hci_ble_hid_cfg_buf_pools[];

/*******************************************************************
 * Constant Definitions
 ******************************************************************/
#define HCI_BLE_HID_DEV_KEY_STORAGE_VS_ID   0x10
#define HCI_BLE_HID_DEV_GATTS_MAX_CONN      1

#define HCI_BLE_HID_DEV_INPUT_REPORT_SIZE   8
#define HCI_BLE_HID_DEV_OUTPUT_REPORT_SIZE  1
#define HCI_BLE_HID_DEV_INPUT_REPORT2_SIZE  4

#define BCM920719                           20719

#define READ_LITTLE_ENDIAN_TO_UINT16(into, m,dl)      \
        (into) = ((m)[0] | ((m)[1]<<8));\
        (m) +=2; (dl)-=2;

#ifndef MIN
#define MIN(a,b)                            (a<b?a:b)
#endif

/******************************************************
 *                    Structures
 ******************************************************/
// host information as saved in the NVRAM
typedef struct
{
    BD_ADDR  remote_addr;
    uint8_t  key_data[sizeof(wiced_bt_device_sec_keys_t)];        /**< [in/out] Key data */
    uint8_t  remote_addr_type;
    uint16_t input_report_client_configuration;
    uint16_t input_report2_client_configuration;
    uint16_t input_audio_ctrl_client_configuration;
    uint16_t input_audio_data_client_configuration;
} hci_ble_hid_dev_hostinfo_t;

typedef struct
{
    hci_ble_hid_dev_hostinfo_t  host_info;  // paired host information if paired

    wiced_bool_t                paired;                 // if true, paired/virtually cabled with a host
    wiced_bool_t                encrypted;              // if true, link is encrypted
    wiced_bool_t                host_started_pairing;   // if TRUE host started and not paired yet
    uint16_t                    conn_id;                // connection_id or 0 if not connected
    wiced_bt_ble_advert_mode_t  adv_mode;               // Current Advertisement mode
#ifdef AUDIO_SUPPORT
    uint16_t                    mtu;
#endif

    uint8_t input_report[HCI_BLE_HID_DEV_INPUT_REPORT_SIZE];
    uint8_t input_report2[HCI_BLE_HID_DEV_INPUT_REPORT2_SIZE];

} hci_ble_hid_dev_state_t;

hci_ble_hid_dev_state_t hci_ble_hid_dev_state;

void hci_ble_hid_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len );
void hci_hid_control_transport_status( wiced_transport_type_t type );

/******************************************************
 *               Variable Definitions
 ******************************************************/
//wiced_bt_device_link_keys_t   paired_device;
#ifdef HID_OVER_UART
static uint32_t                hci_ble_hid_dev_handle_command( uint8_t *p_data, uint32_t length );
#endif
const wiced_transport_cfg_t  transport_cfg =
{
    WICED_TRANSPORT_UART,
    {{ WICED_TRANSPORT_UART_HCI_MODE, HCI_UART_DEFAULT_BAUD }},
    { 0, 0},
#ifdef HID_OVER_UART
    hci_hid_control_transport_status,
    hci_ble_hid_dev_handle_command,
#else
    NULL,
    NULL,
#endif
    NULL
};


/*******************************************************************
 * Function Prototypes
 ******************************************************************/
static void                   hci_ble_hid_dev_app_init               ( void );
static wiced_bt_dev_status_t  hci_ble_hid_dev_management_callback    ( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
static void                   hci_ble_hid_dev_set_advertisement_data ( void );
static void                   hci_ble_hid_dev_interrupt_handler      ( void* user_data, uint8_t pin );
static void                   hci_ble_hid_dev_reset_device           ( void );

/* GATT Registration Callbacks */
static wiced_bt_gatt_status_t hci_ble_hid_dev_write_handler          ( wiced_bt_gatt_write_t *p_write_req, uint16_t conn_id );
static wiced_bt_gatt_status_t hci_ble_hid_dev_read_handler           ( wiced_bt_gatt_read_t *p_read_req, uint16_t conn_id );
static wiced_bt_gatt_status_t hci_ble_hid_dev_connect_callback       ( wiced_bt_gatt_connection_status_t *p_conn_sts  );
static wiced_bt_gatt_status_t hci_ble_hid_dev_server_callback        ( uint16_t conn_id, wiced_bt_gatt_request_type_t type, wiced_bt_gatt_request_data_t *p_data );
static wiced_bt_gatt_status_t hci_ble_hid_dev_event_handler          ( wiced_bt_gatt_evt_t  event, wiced_bt_gatt_event_data_t *p_event_data );

static void                   hci_ble_hid_dev_connection_up( wiced_bt_gatt_connection_status_t *p_status );
static void                   hci_ble_hid_dev_connection_down( wiced_bt_gatt_connection_status_t *p_status );

#ifdef HID_OVER_UART
static void                   hci_ble_hid_dev_handle_reset_cmd( void );
static void                   hci_ble_hid_dev_handle_set_local_bda( uint8_t *p_bda);
static void                   hci_ble_hid_dev_handle_accept_pairing_cmd( BOOLEAN enable );
static void                   hci_ble_hid_dev_handle_host_connect(void);
static void                   hci_ble_hid_dev_handle_host_disconnect( void );
static uint8_t                hci_ble_hid_dev_write_nvram( uint8_t data_len, uint8_t *p_data, BOOLEAN from_host );
static void                   hci_ble_hid_dev_delete_nvram( void );
static void                   hci_ble_hid_dev_send_pairing_complete( uint8_t result, uint8_t *p_bda);
static void                   hci_ble_hid_dev_send_device_started_evt( void );
static void                   hci_ble_hid_dev_send_advertisement_state_event( uint8_t state );
static void                   hci_ble_hid_dev_send_data( uint8_t rpt_type, uint8_t report_id, uint8_t *p_data, uint16_t len );
static void                   hci_ble_hid_dev_send_alert_value( uint8_t value );
static void                   hci_ble_hid_dev_handle_pairing_host_info( uint8_t *p_data, uint16_t length );
static void                   hci_ble_hid_hci_trace_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data );
static void                   hci_control_misc_handle_get_version( void );
#endif

/*******************************************************************
 * Function Definitions
 ******************************************************************/

/*
 * Entry point to the application. Set device configuration and start BT
 * stack initialization.  The actual application initialization will happen
 * when stack reports that BT device is ready
 */
APPLICATION_START()
{
    wiced_transport_init( &transport_cfg );

#ifdef WICED_BT_TRACE_ENABLE
     // Set the debug uart to enable the debug traces
    // Configure the Debug Traces to PUART by default (only way to see the debug traces during boot)
    // The MCU may send a command to change the Debug trace route later

    // WICED_ROUTE_DEBUG_TO_PUART  to send debug string over the PUART
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );

    // Set the debug uart as WICED_ROUTE_DEBUG_NONE to get rid of prints
    //wiced_set_debug_uart( WICED_ROUTE_DEBUG_NONE );

    WICED_BT_TRACE("APP START\n");
#endif

    memset( &hci_ble_hid_dev_state, 0, sizeof( hci_ble_hid_dev_state ) );

    /* Initialize Stack and Register Management Callback */
    wiced_bt_stack_init( &hci_ble_hid_dev_management_callback, &hci_ble_hid_cfg_settings , hci_ble_hid_cfg_buf_pools);

    /* Register callback for receiving hci traces */
    wiced_bt_dev_register_hci_trace( hci_ble_hid_hci_trace_cback );
}

/*
 *  Pass protocol traces up through the UART
 */
void hci_ble_hid_hci_trace_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data )
{
    //send the trace
    wiced_transport_send_hci_trace( NULL, type, length, p_data  );
}


/*
 * This function is executed in the BTM_ENABLED_EVT management callback.
 */
void hci_ble_hid_dev_app_init(void)
{

    /* Configure buttons available on the platform */
    wiced_platform_register_button_callback( WICED_PLATFORM_BUTTON_1, hci_ble_hid_dev_interrupt_handler, NULL, WICED_PLATFORM_BUTTON_BOTH_EDGE);

    /* Set Advertisement Data */
    hci_ble_hid_dev_set_advertisement_data();

    /* Register with stack to receive GATT callback */
    wiced_bt_gatt_register( hci_ble_hid_dev_event_handler );

    /* GATT DB Initialization */
    wiced_bt_gatt_db_init( gatt_database, gatt_database_len );

    // if controlled over UART wait for a command
#ifndef HID_OVER_UART
    // If not paired to a host start advertisements so that a host can connect
    if ( !hci_ble_hid_dev_state.paired )
    {
        wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
    }
#endif

#ifdef AUDIO_SUPPORT
    hci_ble_hid_dev_audio_init();
#endif
}

/* This is the interrupt handler */
static void hci_ble_hid_dev_interrupt_handler( void* user_data, uint8_t pin )
{
#ifdef AUDIO_SUPPORT
    hci_ble_hid_dev_audio_button_handler(wiced_hal_gpio_get_pin_input_status(WICED_GPIO_PIN_BUTTON_1));
#endif
}

/* Set Advertisement Data */
static void hci_ble_hid_dev_set_advertisement_data( void )
{
    /* 16-bit Service UUIDs */
    uint16_t service_uuid_16_data[] =
    {
        UUID_SERVICE_HID,                // 16-bit Service UUID for 'Human Interface Device'
        UUID_SERVICE_IMMEDIATE_ALERT,    // 16-bit Service UUID for 'Immediate Alert'
    };

    wiced_bt_ble_advert_elem_t adv_elem[4];
    uint8_t num_elem = 0;
    uint8_t flag = BTM_BLE_LIMITED_DISCOVERABLE_FLAG|BTM_BLE_BREDR_NOT_SUPPORTED;
    uint16_t appearence = APPEARANCE_GENERIC_HID_DEVICE;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = sizeof(uint8_t);
    adv_elem[num_elem].p_data       = &flag;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_16SRV_PARTIAL;
    adv_elem[num_elem].len          = sizeof(service_uuid_16_data);
    adv_elem[num_elem].p_data       = ( uint8_t* )&service_uuid_16_data;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_APPEARANCE;
    adv_elem[num_elem].len          = sizeof(uint16_t);
    adv_elem[num_elem].p_data       = ( uint8_t* )&appearence;
    num_elem++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = strlen( (char *)hci_ble_hid_cfg_settings.device_name );
    adv_elem[num_elem].p_data       = ( uint8_t* )hci_ble_hid_cfg_settings.device_name;
    num_elem++;

    wiced_bt_ble_set_raw_advertisement_data(num_elem, adv_elem);
}

/* TODO: This function should be called when the device needs to be reset */
static void hci_ble_hid_dev_reset_device( void )
{
    wiced_result_t res = WICED_SUCCESS;

    /* TODO: Clear any additional persistent values used by the application from NVRAM */

    // Reset the device
    wiced_hal_wdog_reset_system( );
}

/* Bluetooth Management Event Handler */
static wiced_bt_dev_status_t hci_ble_hid_dev_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_bt_dev_status_t             status = WICED_BT_SUCCESS;
    wiced_bt_dev_encryption_status_t *p_status;
    wiced_bt_dev_ble_pairing_info_t  *p_info;
    wiced_bt_device_address_t         bda = { 0 };
    uint8_t                           bytes_written, bytes_read;
    wiced_bt_ble_advert_mode_t       *p_adv_mode = NULL;
    wiced_bt_ble_advert_mode_t        previous_adv_mode;
    wiced_bt_device_address_t         null_bdaddr = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    wiced_bt_ble_connection_param_update_t *p_ble_conn_param_update;

    WICED_BT_TRACE( "hci_ble_hid_dev_management_callback event:%d\n", event );

    switch (event)
    {
    case BTM_ENABLED_EVT:
        /* Bluetooth Controller and Host Stack Enabled */
        WICED_BT_TRACE("Bluetooth Enabled (%s)\n", ((WICED_BT_SUCCESS == p_event_data->enabled.status) ? "success" : "failure"));

        if (WICED_BT_SUCCESS == p_event_data->enabled.status)
        {
            wiced_bt_dev_read_local_addr(bda);
            WICED_BT_TRACE("Local Bluetooth Address: [%B]\n", bda);

            /* Bluetooth enabled. Perform application-specific initialization */
            hci_ble_hid_dev_app_init();

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

    case BTM_DISABLED_EVT:
        /* Bluetooth Controller and Host Stack Disabled */
        WICED_BT_TRACE("Bluetooth Disabled\n");
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
        p_event_data->pairing_io_capabilities_ble_request.local_io_cap  = BTM_IO_CAPABILITIES_NONE;
        p_event_data->pairing_io_capabilities_ble_request.oob_data   = BTM_OOB_NONE;
        p_event_data->pairing_io_capabilities_ble_request.auth_req   = BTM_LE_AUTH_REQ_BOND | BTM_LE_AUTH_REQ_MITM;
        p_event_data->pairing_io_capabilities_ble_request.max_key_size  = 0x10;
        p_event_data->pairing_io_capabilities_ble_request.init_keys     = BTM_LE_KEY_PENC | BTM_LE_KEY_PID;
        p_event_data->pairing_io_capabilities_ble_request.resp_keys     = BTM_LE_KEY_PENC | BTM_LE_KEY_PID;
        break;

    case BTM_PAIRING_COMPLETE_EVT:
        p_info = &p_event_data->pairing_complete.pairing_complete_info.ble;

        if ( p_info->reason == WICED_SUCCESS )
        {
            wiced_bool_t res;

            WICED_BT_TRACE( "Pairing Complete: %d level:%d %B\n", p_info->reason, p_info->sec_level, p_event_data->pairing_complete.bd_addr);

            hci_ble_hid_dev_state.paired               = WICED_TRUE;
            hci_ble_hid_dev_state.host_started_pairing = WICED_FALSE;

            /* Add this device to the white-list (not sure if it's really needed) */
            res = wiced_bt_ble_update_advertising_white_list( WICED_TRUE, p_event_data->pairing_complete.bd_addr );
            if (res == WICED_FALSE)
            {
                WICED_BT_TRACE("Err: wiced_bt_ble_update_advertising_white_list failed\n");
            }
            hci_ble_hid_dev_send_pairing_complete( p_info->reason, p_event_data->pairing_complete.bd_addr );
        }
        break;

    case BTM_ENCRYPTION_STATUS_EVT:
        /* Encryption Status Change */
        WICED_BT_TRACE("Encryption Status event: bd ( %B ) res %d\n", p_event_data->encryption_status.bd_addr, p_event_data->encryption_status.result);
        if ( p_event_data->encryption_status.result == WICED_SUCCESS )
        {
            hci_ble_hid_dev_state.encrypted = TRUE;
#ifdef AUDIO_SUPPORT
            /* FW Bug workaround: The packet size should be changed upon reception of the
             * MTU Response. Do it here for the moment.
             */
            status = BTM_SetDataChannelPDULength(hci_ble_hid_dev_state.host_info.remote_addr, 180);
            if (status != 0)
                WICED_BT_TRACE("Err: BTM_SetDataChannelPDULength failed %d\n", status);
#endif
        }
        break;

    case BTM_SECURITY_REQUEST_EVT:
        wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
        WICED_BT_TRACE("UpdateLinkKey for BdAddr:%B AddrType:%d",
                p_event_data->paired_device_link_keys_update.bd_addr,
                p_event_data->paired_device_link_keys_update.key_data.ble_addr_type);
        /* Workaround if the BT Stack sends a Null BdAddr (in case of Pairing Failure) */
        if (memcmp(p_event_data->paired_device_link_keys_update.bd_addr, null_bdaddr, BD_ADDR_LEN) != 0)
        {
            /* Save the peer device info (BdAddr, BdAddr-Type and Link key */
            hci_ble_hid_dev_state.host_info.remote_addr_type = 
                    p_event_data->paired_device_link_keys_update.key_data.ble_addr_type;
            memcpy( hci_ble_hid_dev_state.host_info.remote_addr, p_event_data->paired_device_link_keys_update.bd_addr, BD_ADDR_LEN );
            memcpy( &hci_ble_hid_dev_state.host_info.key_data, &p_event_data->paired_device_link_keys_update.key_data, sizeof(wiced_bt_device_sec_keys_t) );

            /* Ask the MCU to save these info in its NVRAM */
            bytes_written = hci_ble_hid_dev_write_nvram( sizeof(hci_ble_hid_dev_state.host_info),
                    ( uint8_t* ) &hci_ble_hid_dev_state.host_info, FALSE );
            if (bytes_written != sizeof(hci_ble_hid_dev_state.host_info))
            {
                status = WICED_BT_ERROR;
            }
        }	
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        WICED_BT_TRACE("BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT addr:%B\n",
                p_event_data->paired_device_link_keys_request.bd_addr);
        if ( memcmp( p_event_data->paired_device_link_keys_request.bd_addr,
                hci_ble_hid_dev_state.host_info.remote_addr, BD_ADDR_LEN ) == 0 )
        {
            status = WICED_BT_SUCCESS;
            memcpy( &p_event_data->paired_device_link_keys_request.key_data,
                    &hci_ble_hid_dev_state.host_info.key_data, sizeof(wiced_bt_device_sec_keys_t) );
            WICED_BT_TRACE("Key retrieval success\n");
        }
        else
        {
            status = WICED_BT_ERROR;
            WICED_BT_TRACE("Key retrieval failure\n");
        }
        break;

    case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
        /* Request to store newly generated local identity keys to NVRAM */
        /* (sample app does not store keys to NVRAM) */
        break;


    case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
        /* Request to restore local identity keys from NVRAM (requested during Bluetooth start up) */
        /* (sample app does not store keys to NVRAM. New local identity keys will be generated).   */
        status = WICED_BT_NO_RESOURCES;
        break;

    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
        /* Advertisement State Changed */
        previous_adv_mode = hci_ble_hid_dev_state.adv_mode;

        /* Save the New Advertisement mode */
        p_adv_mode = &p_event_data->ble_advert_state_changed;
        hci_ble_hid_dev_state.adv_mode = *p_adv_mode;

        hci_ble_hid_dev_send_advertisement_state_event(*p_adv_mode);

        WICED_BT_TRACE("Advertisement State Change: %d -> %d\n", previous_adv_mode, *p_adv_mode);

        /* To (re)connect we try first High Duty Directed Advertisement. If we cannot connect, the
         * BT Stack switch automatically to Low Duty Directed Advertisement.
         * As Low Duty Directed Advertisement is not really useful, we enter directly in
         * High Duty Undirected Advertisement (see HOGP Spec V1.0 chapter 5.1.3).
         * This modification is needed by some peer devices which do not accept Directed
         * Advertisement when their Random Address changes.
         */
        if ((previous_adv_mode == BTM_BLE_ADVERT_DIRECTED_HIGH) &&
            (*p_adv_mode == BTM_BLE_ADVERT_DIRECTED_LOW) &&
            (hci_ble_hid_dev_state.conn_id == 0))
        {
            WICED_BT_TRACE( "Send High Duty Undirected Adv\n" );
            wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );
        }
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
        WICED_BT_TRACE("Unhandled Bluetooth Management Event:%d\n", event);
        break;
    }

    return status;
}

/* Get a value */
wiced_bt_gatt_status_t hci_ble_hid_dev_get_value( wiced_bt_gatt_read_t *p_read_req )
{
    int i;
    wiced_bt_gatt_status_t res = WICED_BT_GATT_INVALID_HANDLE;
    uint8_t *attr_val_ptr = NULL;
    uint32_t remaining_bytes = 0;

    // some characteristics are processed outside of main table
    switch ( p_read_req->handle )
    {
    case HDLC_HUMAN_INTERFACE_DEVICE_REPORT_INPUT_REPORT_VALUE:
        if (*p_read_req->p_val_len >= HCI_BLE_HID_DEV_INPUT_REPORT_SIZE )
        {
            *p_read_req->p_val_len = HCI_BLE_HID_DEV_INPUT_REPORT_SIZE;
            memcpy( p_read_req->p_val, hci_ble_hid_dev_state.input_report, HCI_BLE_HID_DEV_INPUT_REPORT_SIZE );
            res = WICED_BT_GATT_SUCCESS;
        }
        else
        {
            res = WICED_BT_GATT_INVALID_ATTR_LEN;
        }
        break;

    case HDLC_HUMAN_INTERFACE_DEVICE_REPORT_INPUT_REPORT2_VALUE:
        if ( *p_read_req->p_val_len >= HCI_BLE_HID_DEV_INPUT_REPORT2_SIZE )
        {
            *p_read_req->p_val_len = HCI_BLE_HID_DEV_INPUT_REPORT2_SIZE;
            memcpy( p_read_req->p_val, hci_ble_hid_dev_state.input_report2, HCI_BLE_HID_DEV_INPUT_REPORT2_SIZE );
            res = WICED_BT_GATT_SUCCESS;
        }
        else
        {
            res = WICED_BT_GATT_INVALID_ATTR_LEN;
        }
        break;

    case HDLD_HUMAN_INTERFACE_DEVICE_REPORT_INPUT_REPORT_CLIENT_CONFIGURATION:
        if ( *p_read_req->p_val_len >= 2 )
        {
            *p_read_req->p_val_len = 2;
            p_read_req->p_val[0] = hci_ble_hid_dev_state.host_info.input_report_client_configuration & 0xff;
            p_read_req->p_val[1] = (hci_ble_hid_dev_state.host_info.input_report_client_configuration >> 8) & 0xff;
            res = WICED_BT_GATT_SUCCESS;
        }
        else
        {
            res = WICED_BT_GATT_INVALID_ATTR_LEN;
        }
        break;

    case HDLD_HUMAN_INTERFACE_DEVICE_REPORT_INPUT_REPORT2_CLIENT_CONFIGURATION:
        if ( *p_read_req->p_val_len >= 2 )
        {
            *p_read_req->p_val_len = 2;
            p_read_req->p_val[0] = hci_ble_hid_dev_state.host_info.input_report2_client_configuration & 0xff;
            p_read_req->p_val[1] = (hci_ble_hid_dev_state.host_info.input_report2_client_configuration >> 8) & 0xff;
            res = WICED_BT_GATT_SUCCESS;
        }
        else
        {
            res = WICED_BT_GATT_INVALID_ATTR_LEN;
        }
        break;

#ifdef AUDIO_SUPPORT
    case HDLD_HUMAN_INTERFACE_DEVICE_REPORT_AUDIO_CTRL_INPUT_CLIENT_CONFIGURATION:
        if ( *p_read_req->p_val_len >= 2 )
        {
            *p_read_req->p_val_len = 2;
            p_read_req->p_val[0] = hci_ble_hid_dev_state.host_info.input_audio_ctrl_client_configuration & 0xff;
            p_read_req->p_val[1] = (hci_ble_hid_dev_state.host_info.input_audio_ctrl_client_configuration >> 8) & 0xff;
            res = WICED_BT_GATT_SUCCESS;
        }
        else
        {
            res = WICED_BT_GATT_INVALID_ATTR_LEN;
        }
        break;

    case HDLD_HUMAN_INTERFACE_DEVICE_REPORT_AUDIO_DATA_INPUT_CLIENT_CONFIGURATION:
        if ( *p_read_req->p_val_len >= 2 )
        {
            *p_read_req->p_val_len = 2;
            p_read_req->p_val[0] = hci_ble_hid_dev_state.host_info.input_audio_data_client_configuration & 0xff;
            p_read_req->p_val[1] = (hci_ble_hid_dev_state.host_info.input_audio_data_client_configuration >> 8) & 0xff;
            res = WICED_BT_GATT_SUCCESS;
        }
        else
        {
            res = WICED_BT_GATT_INVALID_ATTR_LEN;
        }
        break;
#endif

    default:
        // Check for a matching handle entry
        for (i = 0; i < hci_ble_hid_dev_gatt_db_ext_attr_tbl_size; i++)
        {
            if (hci_ble_hid_dev_gatt_db_ext_attr_tbl[i].handle == p_read_req->handle)
            {
                // Detected a matching handle in the external lookup table

                //check if this is read req. is for a long attribute value, if so take care of the offset as well
                if(p_read_req->is_long)
                {
                    attr_val_ptr = hci_ble_hid_dev_gatt_db_ext_attr_tbl[i].p_data + p_read_req->offset;
                    remaining_bytes = hci_ble_hid_dev_gatt_db_ext_attr_tbl[i].max_len -  p_read_req->offset;
                }
                else
                {
                    attr_val_ptr = hci_ble_hid_dev_gatt_db_ext_attr_tbl[i].p_data;
                    remaining_bytes = hci_ble_hid_dev_gatt_db_ext_attr_tbl[i].max_len;
                }
                //if the attribute value length is less than the size of MTU update *p_read_req->p_val_len accordingly
                if(remaining_bytes<*p_read_req->p_val_len)
                {
                    *p_read_req->p_val_len = remaining_bytes;
                }
                // copy over the value to the supplied buffer(entirely if it fits, data worth of MTU size)
                // if we have only sent partial value of an attribute we expect the peer to issue a read blob request to get the
                // rest of the attribute value.
                memcpy( p_read_req->p_val, attr_val_ptr, *p_read_req->p_val_len );
                res = WICED_BT_GATT_SUCCESS;
                WICED_BT_TRACE("Sending %d bytes from the offset %d \n", *p_read_req->p_val_len, p_read_req->offset);
                break;
            }
        }
        break;
    }
    return res;
}

/* Set a value */
wiced_bt_gatt_status_t hci_ble_hid_dev_set_value( uint16_t attr_handle, uint8_t *p_val, uint16_t len )
{
    wiced_bool_t store_nvram   = WICED_FALSE;
    wiced_bt_gatt_status_t res = WICED_BT_GATT_INVALID_HANDLE;

    switch ( attr_handle )
    {
    case HDLD_HUMAN_INTERFACE_DEVICE_REPORT_INPUT_REPORT_CLIENT_CONFIGURATION:
        if ( len == 2 )
        {
            hci_ble_hid_dev_state.host_info.input_report_client_configuration = p_val[0] + ( p_val[1] << 8 );
            WICED_BT_TRACE( "input report ccc:%d\n", hci_ble_hid_dev_state.host_info.input_report_client_configuration );
            store_nvram = WICED_TRUE;
            res = WICED_BT_GATT_SUCCESS;
        }
        else
        {
            res = WICED_BT_GATT_INVALID_ATTR_LEN;
        }
        break;

    case HDLD_HUMAN_INTERFACE_DEVICE_REPORT_INPUT_REPORT2_CLIENT_CONFIGURATION:
        if ( len == 2 )
        {
            hci_ble_hid_dev_state.host_info.input_report2_client_configuration = p_val[0] + ( p_val[1] << 8 );
            WICED_BT_TRACE( "input report2 ccc:%d\n", hci_ble_hid_dev_state.host_info.input_report2_client_configuration );
            store_nvram = WICED_TRUE;
            res = WICED_BT_GATT_SUCCESS;
        }
        else
        {
            res = WICED_BT_GATT_INVALID_ATTR_LEN;
        }
        break;

    case HDLC_HUMAN_INTERFACE_DEVICE_REPORT_OUTPUT_REPORT_VALUE:
        if ( len == HCI_BLE_HID_DEV_OUTPUT_REPORT_SIZE )
        {
		    /* Save the received Report Data */
            hci_ble_hid_dev_human_interface_device_report_output_report[0] = *p_val;
#ifdef HID_OVER_UART
            hci_ble_hid_dev_send_data( HID_REPORT_TYPE_OUTPUT, HID_OUTPUT_REPORT_ID, p_val, len );
#endif
            res = WICED_BT_GATT_SUCCESS;
        }
        break;

    case HDLC_HUMAN_INTERFACE_DEVICE_HID_CONTROL_POINT_VALUE:
        if ( len == 1 )
        {
            // ToDo process suspend/resume
            res = WICED_BT_GATT_SUCCESS;
        }
        else
        {
            res = WICED_BT_GATT_INVALID_ATTR_LEN;
        }
        break;

    case HDLC_HUMAN_INTERFACE_DEVICE_PROTOCOL_MODE_VALUE:
        if ( len == 1 )
        {
            WICED_BT_TRACE("HID Host SetProcol:%d\n", *p_val);
            res = WICED_BT_GATT_SUCCESS;
            hci_ble_hid_dev_human_interface_device_protocol_mode[0] = *p_val;
        }
        else
        {
            res = WICED_BT_GATT_INVALID_ATTR_LEN;
        }
        break;

#ifdef AUDIO_SUPPORT
    case HDLD_HUMAN_INTERFACE_DEVICE_REPORT_AUDIO_CTRL_INPUT_CLIENT_CONFIGURATION:
        if ( len == 2 )
        {
            hci_ble_hid_dev_state.host_info.input_audio_ctrl_client_configuration = p_val[0] + ( p_val[1] << 8 );
            WICED_BT_TRACE( "input report audio ctrl ccc:%d\n", hci_ble_hid_dev_state.host_info.input_audio_ctrl_client_configuration );
            store_nvram = WICED_TRUE;
            res = WICED_BT_GATT_SUCCESS;
        }
        else
        {
            res = WICED_BT_GATT_INVALID_ATTR_LEN;
        }
        break;

    case HDLD_HUMAN_INTERFACE_DEVICE_REPORT_AUDIO_DATA_INPUT_CLIENT_CONFIGURATION:
        if ( len == 2 )
        {
            hci_ble_hid_dev_state.host_info.input_audio_data_client_configuration = p_val[0] + ( p_val[1] << 8 );
            WICED_BT_TRACE( "input report audio data ccc:%d\n", hci_ble_hid_dev_state.host_info.input_audio_data_client_configuration );
            store_nvram = WICED_TRUE;
            res = WICED_BT_GATT_SUCCESS;
        }
        else
        {
            res = WICED_BT_GATT_INVALID_ATTR_LEN;
        }
        break;

    case HDLC_HUMAN_INTERFACE_DEVICE_REPORT_AUDIO_CTRL_FEATURE_VALUE:
        hci_ble_hid_dev_audio_write_ctrl_feature_handler(p_val, len);
        res = WICED_BT_GATT_SUCCESS;
        break;
#endif

    case HDLC_IMMEDIATE_ALERT_ALERT_LEVEL_VALUE:
        if ( len == 1 )
        {
#ifdef HID_OVER_UART
            hci_ble_hid_dev_send_alert_value( *p_val );
#endif
            res = WICED_BT_GATT_SUCCESS;
        }
        else
        {
            res = WICED_BT_GATT_INVALID_ATTR_LEN;
        }
        break;
    }
    if ( store_nvram )
    {
        hci_ble_hid_dev_write_nvram( sizeof( hci_ble_hid_dev_hostinfo_t ), (uint8_t *)&hci_ble_hid_dev_state.host_info, FALSE );
    }
    return res;
}

/* Handles Write Requests received from Client device */
static wiced_bt_gatt_status_t hci_ble_hid_dev_write_handler( wiced_bt_gatt_write_t *p_write_req, uint16_t conn_id )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INVALID_HANDLE;

    /* Attempt to perform the Write Request */
    status = hci_ble_hid_dev_set_value(p_write_req->handle, p_write_req->p_val, p_write_req->val_len);

    return status;
}

/* Handles Read Requests received from Client device */
static wiced_bt_gatt_status_t hci_ble_hid_dev_read_handler( wiced_bt_gatt_read_t *p_read_req, uint16_t conn_id )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INVALID_HANDLE;

    /* Attempt to perform the Read Request */
    status = hci_ble_hid_dev_get_value(p_read_req);

    return status;
}

/* GATT Connection Status Callback */
static wiced_bt_gatt_status_t hci_ble_hid_dev_connect_callback( wiced_bt_gatt_connection_status_t *p_conn_sts )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

    if ( p_conn_sts->connected )
    {
        hci_ble_hid_dev_connection_up( p_conn_sts );
    }
    else
    {
#ifdef AUDIO_SUPPORT
        hci_ble_hid_dev_audio_disconnect();
#endif
        hci_ble_hid_dev_connection_down( p_conn_sts );
    }

    return status;
}

/* GATT Server Event Callback */
static wiced_bt_gatt_status_t hci_ble_hid_dev_server_callback( uint16_t conn_id, wiced_bt_gatt_request_type_t type, wiced_bt_gatt_request_data_t *p_data )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;

    switch ( type )
    {
    case GATTS_REQ_TYPE_READ:
        status = hci_ble_hid_dev_read_handler( &p_data->read_req, conn_id );
        break;
    case GATTS_REQ_TYPE_WRITE:
        status = hci_ble_hid_dev_write_handler( &p_data->write_req, conn_id );
        break;
#ifdef AUDIO_SUPPORT
    case GATTS_REQ_TYPE_MTU:
        WICED_BT_TRACE("Client set MTU to %d bytes\n", p_data->mtu);
        hci_ble_hid_dev_state.mtu = p_data->mtu;
        status = WICED_BT_GATT_SUCCESS;
       break;
#endif
    }

    return status;
}

/* GATT Event Handler */
static wiced_bt_gatt_status_t hci_ble_hid_dev_event_handler( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;
    wiced_bt_gatt_attribute_request_t *p_attr_req = NULL;

    switch ( event )
    {
    case GATT_CONNECTION_STATUS_EVT:
        status = hci_ble_hid_dev_connect_callback( &(p_event_data->connection_status));
        break;
    case GATT_ATTRIBUTE_REQUEST_EVT:
        status = hci_ble_hid_dev_server_callback( p_event_data->attribute_request.conn_id, p_event_data->attribute_request.request_type, &p_event_data->attribute_request.data);
        break;
    default:
        status = WICED_BT_GATT_SUCCESS;
        break;
    }

    return status;
}

/*
 *  transfer connection event to uart
 */
void hci_control_le_send_connect_event( uint8_t addr_type, BD_ADDR addr, uint16_t con_handle, uint8_t role )
{
    int i;
    uint8_t   tx_buf [30];
    uint8_t   *p = tx_buf;

    *p++ = addr_type;
    for ( i = 0; i < 6; i++ )
        *p++ = addr[5 - i];
    *p++ = con_handle & 0xff;
    *p++ = ( con_handle >> 8 ) & 0xff;
    *p++ = role;

    wiced_transport_send_data ( HCI_CONTROL_LE_EVENT_CONNECTED, tx_buf, ( int )( p - tx_buf ) );
}

/* This function is invoked when connection is established */
void hci_ble_hid_dev_connection_up( wiced_bt_gatt_connection_status_t *p_status  )
{
    wiced_result_t  result;

    WICED_BT_TRACE( "[%s] %B id:%d\n", __FUNCTION__, p_status->bd_addr, p_status->conn_id );

    /* Update the connection handler. */
    hci_ble_hid_dev_state.conn_id = p_status->conn_id;
    hci_ble_hid_dev_state.encrypted = WICED_FALSE;
#ifdef AUDIO_SUPPORT
    hci_ble_hid_dev_state.mtu = GATT_DEF_BLE_MTU_SIZE;
#endif
    /* Stop advertising */
    wiced_bt_start_advertisements( BTM_BLE_ADVERT_OFF, 0, NULL );

    /* If we support MTU size greater than the default value, start MTU exchange.
     * Ideally this should be initiated by the client and not server, but since some
     * clients(like Android) are not initiating MTU exchange, we are compensating that. */
    if(hci_ble_hid_cfg_settings.gatt_cfg.max_attr_len > GATT_DEF_BLE_MTU_SIZE)
    {
//        wiced_bt_gatt_configure_mtu(p_status->conn_id, hci_ble_hid_cfg_settings.gatt_cfg.max_attr_len);
    }

    /* A BLE Device is not supposed to initiate Pairing */
    /* The BLE Client will do it if it needs to access attributes requiring authentication */

    // propagate link up event to to host (application)
    hci_control_le_send_connect_event( p_status->addr_type, p_status->bd_addr, p_status->conn_id, p_status->link_role );
}

/*
 *  transfer disconnection event to UART
 */
void hci_control_le_send_disconnect_evt( uint8_t reason, uint16_t con_handle )
{
    uint8_t   tx_buf [3];
    uint8_t   *p = tx_buf;

    *p++ = con_handle & 0xff;
    *p++ = ( con_handle >> 8 ) & 0xff;
    *p++ = reason;

    wiced_transport_send_data ( HCI_CONTROL_LE_EVENT_DISCONNECTED, tx_buf, ( int )( p - tx_buf ) );
}

/* This function is invoked when connection is dropped */
void hci_ble_hid_dev_connection_down( wiced_bt_gatt_connection_status_t *p_status  )
{
    wiced_result_t              result;
    hci_ble_hid_dev_hostinfo_t  paired_host;
    uint16_t                    bytes_read;

    WICED_BT_TRACE( "[%s] conn_id:%d reason:%d host_pairing:%d\n", __FUNCTION__,
            p_status->conn_id, p_status->reason, hci_ble_hid_dev_state.host_started_pairing );

    /* Update the connection handler. */
    hci_ble_hid_dev_state.conn_id = 0;

    /* if host requested pairing mode, do it again */
    if ( hci_ble_hid_dev_state.host_started_pairing )
    {
        // start advertisements so that a host can connect
        wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );
    }
    // propagate event to host (uppoer layer application)
    hci_control_le_send_disconnect_evt( p_status->reason, p_status->conn_id );
}

#ifdef HID_OVER_UART

/*
 * Handle command received over the UART.  First buffer of the command is the opcode
 * of the operation.  Rest are parameters specific for particular command.
 *
 */
uint32_t hci_ble_hid_dev_handle_command( uint8_t *p_data, uint32_t length )
{
    uint16_t opcode;
    uint16_t payload_len;
    uint8_t status = HCI_CONTROL_STATUS_SUCCESS;
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

    READ_LITTLE_ENDIAN_TO_UINT16(opcode, p_data, length);     // Get opcode
    READ_LITTLE_ENDIAN_TO_UINT16(payload_len, p_data, length); // Get len

    WICED_BT_TRACE("Opcode:%04X\n", opcode);

    hci_ble_hid_handle_command( opcode, p_data, payload_len );

    //Freeing the buffer in which data is received
    wiced_transport_free_buffer( p_rx_buf );
    return ( 0 );
}

/*
 * handle reset command from UART
 */
void hci_ble_hid_dev_handle_reset_cmd( void )
{
    // trip watch dog now.
    wiced_hal_wdog_reset_system( );
}

/*
 * Handle host command to set device pairable.  This is typically a HID device button push.
 */
void hci_ble_hid_dev_handle_accept_pairing_cmd( BOOLEAN enable )
{
    WICED_BT_TRACE( "accept_pairing_cmd %d\n", enable );

    wiced_bt_set_pairable_mode(enable, 0);

    if ( !enable )
    {
        wiced_bt_start_advertisements( BTM_BLE_ADVERT_OFF, 0, NULL );
        return;
    }

    /* Host instructed us to start pairing.  Delete saved information from the NVRAM */
    /* If pairing fails, host better restor the NVRAM */
    hci_ble_hid_dev_delete_nvram( );

    /* disconnect any connections if active */
    if ( hci_ble_hid_dev_state.conn_id != 0 )
    {
        wiced_bt_gatt_disconnect( hci_ble_hid_dev_state.conn_id );
    }

    hci_ble_hid_dev_state.paired                = WICED_FALSE;
    hci_ble_hid_dev_state.host_started_pairing  = WICED_TRUE;

    // start advertisements so that a host can connect
    wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );
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
        wiced_bt_dev_register_hci_trace( hci_ble_hid_hci_trace_cback );
    }
    else
    {
        wiced_bt_dev_register_hci_trace( NULL);
    }
    wiced_set_debug_uart( route_debug );
}

/*
 * Handle command to establish connection to a known host
 */
void hci_ble_hid_dev_handle_host_connect( void )
{
    WICED_BT_TRACE( "HOGPD connect paired:%d\n", hci_ble_hid_dev_state.paired );

    if ( !hci_ble_hid_dev_state.paired || ( hci_ble_hid_dev_state.conn_id != 0 ) )
    {
        return;
    }

    WICED_BT_TRACE( "Send High Duty Directed Adv to :%B\n",
            hci_ble_hid_dev_state.host_info.remote_addr );
    wiced_bt_start_advertisements(BTM_BLE_ADVERT_DIRECTED_HIGH,
            hci_ble_hid_dev_state.host_info.remote_addr_type,
            hci_ble_hid_dev_state.host_info.remote_addr);
}

/*
 * Handle command to disconnect the HID Connection
 */
void hci_ble_hid_dev_handle_host_disconnect( void )
{
    WICED_BT_TRACE( "HOGPD disconnect\n" );

    if ( hci_ble_hid_dev_state.conn_id == 0 )
    {
        WICED_BT_TRACE( "HIDD not connected\n" );
        return;
    }
    wiced_bt_gatt_disconnect( hci_ble_hid_dev_state.conn_id );
}

/*
 * Handle host command to send HID report.
 */
void hci_ble_hid_dev_handle_send_report( uint8_t type, uint8_t *p_data, uint16_t length )
{
    uint8_t  report_id = *p_data++;
    uint16_t client_configuration_descriptor = 0;
    uint16_t attr_handle = HDLC_HUMAN_INTERFACE_DEVICE_REPORT_INPUT_REPORT_VALUE;
    wiced_bt_gatt_status_t gatt_status;

    WICED_BT_TRACE( "send report type:%d id:0x%X length:%d conn_id:%d\n", type, report_id,
            length, hci_ble_hid_dev_state.conn_id );

    if ( --length > 0 )
    {
        if ( type == HID_REPORT_TYPE_INPUT )
        {
            // need to save data in case host decides to read it
            if ( ( report_id == HID_INPUT_REPORT_ID ) && ( length == sizeof( hci_ble_hid_dev_state.input_report ) ) )
            {
                attr_handle = HDLC_HUMAN_INTERFACE_DEVICE_REPORT_INPUT_REPORT_VALUE;
                client_configuration_descriptor = hci_ble_hid_dev_state.host_info.input_report_client_configuration;
                memcpy ( hci_ble_hid_dev_state.input_report, p_data, length );
            }
            else if ( ( report_id == HID_INPUT_REPORT2_ID ) && ( length == sizeof( hci_ble_hid_dev_state.input_report2 ) ) )
            {
                attr_handle = HDLC_HUMAN_INTERFACE_DEVICE_REPORT_INPUT_REPORT2_VALUE;
                client_configuration_descriptor = hci_ble_hid_dev_state.host_info.input_report2_client_configuration;
                memcpy ( hci_ble_hid_dev_state.input_report2, p_data, length );
            }
#ifdef AUDIO_SUPPORT
            else if ( report_id == HID_AUDIO_CTRL_INPUT_REPORT_ID )
            {
                attr_handle = HDLC_HUMAN_INTERFACE_DEVICE_REPORT_AUDIO_CTRL_INPUT_VALUE;
                client_configuration_descriptor = hci_ble_hid_dev_state.host_info.input_audio_ctrl_client_configuration;
            }
            else if ( report_id == HID_AUDIO_DATA_INPUT_REPORT_ID )
            {
                attr_handle = HDLC_HUMAN_INTERFACE_DEVICE_REPORT_AUDIO_DATA_INPUT_VALUE;
                if (hci_ble_hid_dev_state.host_info.input_audio_data_client_configuration & GATT_CLIENT_CONFIG_NOTIFICATION)
                {
                    /* If the MTU has not been set, split the Notification in 3 packets (20 bytes each) */
                    if (hci_ble_hid_dev_state.mtu < length)
                    {
                        wiced_bt_gatt_send_notification( hci_ble_hid_dev_state.conn_id, attr_handle, 20, p_data );
                        wiced_bt_gatt_send_notification( hci_ble_hid_dev_state.conn_id, attr_handle, 20, p_data + 20);
                        wiced_bt_gatt_send_notification( hci_ble_hid_dev_state.conn_id, attr_handle, 20, p_data + 40);
                    }
                    else
                    {
                        gatt_status = wiced_bt_gatt_send_notification( hci_ble_hid_dev_state.conn_id, attr_handle, length, p_data );
                        if (gatt_status != WICED_BT_GATT_SUCCESS)
                            WICED_BT_TRACE("Err: wiced_bt_gatt_send_notification (audio) failed %d\n", gatt_status);
                    }
                }
            }
#endif
        }
    }
    if ( hci_ble_hid_dev_state.paired && ( hci_ble_hid_dev_state.conn_id == 0 ) )
    {
        // Start the connection.  Please note that this version of the app does not queue
        // reports.  This is ok for mice and is not ok for keyboards/remote controls.  Please
        // see hci_hid_device sample if you need help saving reports.
        wiced_bt_start_advertisements( BTM_BLE_ADVERT_DIRECTED_HIGH, hci_ble_hid_dev_state.host_info.remote_addr_type, hci_ble_hid_dev_state.host_info.remote_addr );
    }
    else if ( client_configuration_descriptor & GATT_CLIENT_CONFIG_NOTIFICATION )
    {
        WICED_BT_TRACE("Sending notification... : attr_handle %d, length %d, \n", attr_handle, length);
        gatt_status = wiced_bt_gatt_send_notification( hci_ble_hid_dev_state.conn_id, attr_handle, length, p_data );
        if (gatt_status != WICED_BT_GATT_SUCCESS)
            WICED_BT_TRACE("Err: wiced_bt_gatt_send_notification failed %d\n", gatt_status);
    }
}

/*
 * handle command to send local Bluetooth device address
 */
void hci_ble_hid_dev_handle_set_local_bda( uint8_t *bda )
{
    int result;

    BD_ADDR bd_addr;
    STREAM_TO_BDADDR (bd_addr, bda);
    wiced_bt_set_local_bdaddr( bd_addr, BLE_ADDR_PUBLIC );
    WICED_BT_TRACE( "Local Bluetooth Address: [%B]\n", bd_addr);
}

/*
 * Send notification to the host that pairing has been completed
 */
void hci_ble_hid_dev_send_pairing_complete( uint8_t result, uint8_t *p_bda )
{
    uint8_t tx_buf[12];
    uint8_t *p = tx_buf;
    int i;

    *p++ = result;

    for ( i = 0; i < 6; i++ )
        *p++ = p_bda[5 - i];

    *p++ = BT_DEVICE_TYPE_BLE;

    wiced_transport_send_data( HCI_CONTROL_EVENT_PAIRING_COMPLETE, tx_buf, ( int ) ( p - tx_buf ) );
}

/*
 *  Send Device Started event through UART
 */
void hci_ble_hid_dev_send_device_started_evt( void )
{
    wiced_transport_send_data( HCI_CONTROL_EVENT_DEVICE_STARTED, NULL, 0 );
}

/*
 *  transfer advertise  event to uart
 */
void hci_ble_hid_dev_send_advertisement_state_event( uint8_t state )
{
    wiced_transport_send_data ( HCI_CONTROL_LE_EVENT_ADVERTISEMENT_STATE, &state, 1 );
}

/*
 * Send Data event to the host
 */
void hci_ble_hid_dev_send_data( uint8_t report_type, uint8_t report_id, uint8_t *p_data, uint16_t len )
{
    uint8_t tx_buf[202];

    tx_buf[0] = report_type;
    tx_buf[1] = report_id;
    memcpy( &tx_buf[2], p_data, len < 200 ? len : 200 );

    wiced_transport_send_data( HCI_CONTROL_HID_EVENT_DATA, tx_buf, ( int ) ( len < 200 ? 2 + len : 200 ) );
}

/*
 * Send Data event to the host
 */
void hci_ble_hid_dev_send_alert_value( uint8_t value )
{
    wiced_transport_send_data( HCI_CONTROL_ALERT_EVENT_NOTIFICATION, &value, 1 );
}


/*
 * Host sends information about previously paired host
 */
void hci_ble_hid_dev_handle_pairing_host_info( uint8_t *p_data, uint16_t length )
{
    wiced_result_t result;
    wiced_bt_device_link_keys_t pairing_info;

    WICED_BT_TRACE( "hci_ble_hid_dev_handle_pairing_host_info\n");

    if ( length != sizeof( hci_ble_hid_dev_state.host_info) )
    {
        WICED_BT_TRACE( " pairing_host_info bad length:%d\n", length);
        return;
    }

    memcpy( &hci_ble_hid_dev_state.host_info, p_data, sizeof( hci_ble_hid_dev_state.host_info) );

    hci_ble_hid_dev_write_nvram( sizeof( hci_ble_hid_dev_hostinfo_t ), (uint8_t *)&hci_ble_hid_dev_state.host_info, TRUE );

    hci_ble_hid_dev_state.paired = WICED_TRUE;

    memcpy( &pairing_info.bd_addr, &hci_ble_hid_dev_state.host_info.remote_addr, sizeof(wiced_bt_device_address_t) );
    memcpy( &pairing_info.key_data, &hci_ble_hid_dev_state.host_info.key_data, sizeof(wiced_bt_device_sec_keys_t) );

    /* Add this peer device in the security database */
    result = wiced_bt_dev_add_device_to_address_resolution_db( &pairing_info );
    if (result != WICED_BT_SUCCESS)
    {
        WICED_BT_TRACE( "Err: wiced_bt_dev_add_device_to_address_resolution_db failed: %d\n", result);
    }
}

/*
 * Write NVRAM function is called to store information in the RAM.  This can be called when
 * stack requires persistent storage, for example to save link keys.  In this case
 * data is also formatted and send to the host for real NVRAM storage.  The same function is
 * called when host pushes NVRAM chunks during the startup.  Parameter from_host in this
 * case is set to FALSE indicating that data does not need to be forwarded.
 */
uint8_t hci_ble_hid_dev_write_nvram( uint8_t data_len, uint8_t *p_data, BOOLEAN from_host )
{
    uint8_t tx_buf[257];
    uint8_t *p = tx_buf;

    WICED_BT_TRACE( "NVRAM write: %d store:%d\n", data_len, !from_host );

    /* If NVRAM chunk arrived from host, no need to send it back, otherwise send over transport */
    if ( !from_host )
    {
        *p++ = HCI_BLE_HID_DEV_KEY_STORAGE_VS_ID & 0xff;
        *p++ = ( HCI_BLE_HID_DEV_KEY_STORAGE_VS_ID >> 8 ) & 0xff;
        memcpy( p, p_data, data_len );

        wiced_transport_send_data( HCI_CONTROL_EVENT_NVRAM_DATA, tx_buf, ( int ) ( data_len + 2 ) );
    }
    return ( data_len );
}

/*
 * Delete NVRAM function is called when host deletes NVRAM chunk from the persistent storage.
 */
void hci_ble_hid_dev_delete_nvram( )
{
    wiced_result_t result;

    /* Delete the device from the bonded devices list (in the BT stack) */
    result = wiced_bt_dev_delete_bonded_device( hci_ble_hid_dev_state.host_info.remote_addr );
    if (result != WICED_BT_SUCCESS)
    {
        WICED_BT_TRACE( "Err: hci_ble_hid_dev_delete_nvram unbond failed:%d\n", result );
    }
    memset( &hci_ble_hid_dev_state.host_info, 0, sizeof( hci_ble_hid_dev_state.host_info ) );
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
    tx_buf[cmd++] = HCI_CONTROL_GROUP_HIDD;

    wiced_transport_send_data( HCI_CONTROL_MISC_EVENT_VERSION, tx_buf, cmd );
}

/*
 * hci_control_transport_status
 * This callback function is called when the MCU opens the Wiced UART
 */
void hci_hid_control_transport_status( wiced_transport_type_t type )
{
    WICED_BT_TRACE( "hci_control_transport_status %x \n", type );

    // Tell Host that App is started
    hci_ble_hid_dev_send_device_started_evt();
}

void hci_ble_hid_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len )
{
    uint8_t bytes_written;
    switch( cmd_opcode )
    {
    case HCI_CONTROL_COMMAND_RESET:
        hci_ble_hid_dev_handle_reset_cmd( );
        break;

    case HCI_CONTROL_COMMAND_TRACE_ENABLE:
        hci_control_handle_trace_enable( p_data );
        break;

    case HCI_CONTROL_COMMAND_SET_LOCAL_BDA:
        hci_ble_hid_dev_handle_set_local_bda( p_data );
        break;

    case HCI_CONTROL_HIDD_COMMAND_ACCEPT_PAIRING:
        hci_ble_hid_dev_handle_accept_pairing_cmd( *p_data );
        break;

    case HCI_CONTROL_HID_COMMAND_PUSH_PAIRING_HOST_INFO:
    case HCI_CONTROL_COMMAND_PUSH_NVRAM_DATA:
        hci_ble_hid_dev_handle_pairing_host_info( p_data, data_len );
        break;

    case HCI_CONTROL_HIDD_COMMAND_CONNECT:
        hci_ble_hid_dev_handle_host_connect( );
        break;

    case HCI_CONTROL_HIDD_COMMAND_DISCONNECT:
        hci_ble_hid_dev_handle_host_disconnect( );
        break;

    case HCI_CONTROL_HIDD_COMMAND_VIRTUAL_UNPLUG:
        hci_ble_hid_dev_delete_nvram();
        /* disconnect any connections if active */
        if ( hci_ble_hid_dev_state.conn_id != 0 )
        {
            wiced_bt_gatt_disconnect( hci_ble_hid_dev_state.conn_id );
        }
        hci_ble_hid_dev_state.paired                = WICED_FALSE;
        break;

    case HCI_CONTROL_HIDD_COMMAND_SEND_REPORT:
        WICED_BT_TRACE( "HCI_CONTROL_HID_COMMAND_SEND_REPORT: ");
        wiced_trace_array(p_data, data_len);
        // for BLE there is no channel, skip p_data[0]
        hci_ble_hid_dev_handle_send_report( p_data[1], &p_data[2], ( uint16_t ) ( data_len - 2 ) );
        break;

    case HCI_CONTROL_MISC_COMMAND_GET_VERSION:
        hci_control_misc_handle_get_version();
        break;

    default:
        WICED_BT_TRACE( "??? Unknown command opcode:0x%04X\n", cmd_opcode);
        break;
    }
}

#endif

#ifdef AUDIO_SUPPORT
/*
 * hci_ble_hid_update_ble_conn_params
 * This function is used to change the BLE Connection parameters
 */
void hci_ble_hid_update_ble_conn_params(uint16_t min_int, uint16_t max_int, uint16_t latency, uint16_t timeout)
{
    WICED_BT_TRACE( "Changing BLE Conn Param to min:%d max:%d latency:%d lsto:%d\n", min_int, max_int, latency, timeout);
    wiced_bt_l2cap_update_ble_conn_params(hci_ble_hid_dev_state.host_info.remote_addr, min_int, max_int, latency, timeout);
}
#endif


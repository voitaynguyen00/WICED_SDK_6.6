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
 * Entry point to Bluetooth (BT) classic keyboard application.
 *
 */

#include "sparcommon.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_stack.h"
#include "wiced_hal_nvram.h"
#include "wiced_hal_wdog.h"
#include "wiced_platform.h"
#include "wiced_memory.h"
#include "wiced_hidd_lib.h"
#include "keyboard.h"
#include "bthidlink.h"
#include "bthostlist.h"

extern const uint8_t wiced_bt_sdp_db[];
extern const uint16_t wiced_bt_sdp_db_size;
extern wiced_bt_hidd_pm_pwr_state_t bthid_powerStateList[];
extern const uint8_t bthid_powerStateList_num;
extern wiced_bt_hidd_pm_pwr_state_t bthid_SSRPowerStatesList[];
extern const uint8_t bthid_SSRPowerStatesList_num;
extern const wiced_bt_cfg_settings_t wiced_bt_hid_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_bt_hid_cfg_buf_pools[];

extern wiced_bt_device_link_keys_t  bthidlink_link_keys;

#ifdef TESTING_USING_HCI
#include "hci_control_api.h"
#include "wiced_transport.h"
#if ( defined(CYW20706A2) || defined(CYW20719B1) || defined(CYW20719B0) || defined(CYW20721B1) || defined(CYW20735B0) || defined(CYW43012C0) )
#include "wiced_bt_app_common.h"
#endif
#define READ_LITTLE_ENDIAN_TO_UINT16(into, m,dl)      \
        (into) = ((m)[0] | ((m)[1]<<8));\
        (m) +=2; (dl)-=2;

void hci_hid_hci_trace_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data );
uint32_t hci_hid_dev_handle_command( uint8_t *p_data, uint32_t length );
void hci_control_misc_handle_get_version( void );


/******************************************************************************
 *                          HCI control support functions
 ******************************************************************************/
const wiced_transport_cfg_t  transport_cfg =
{
    WICED_TRANSPORT_UART,
    { WICED_TRANSPORT_UART_HCI_MODE, 115200 },
    { 0, 0},
    NULL,
    hci_hid_dev_handle_command,
    NULL
};

extern KbAppConfig kbAppConfig;
extern tKbAppState *kbAppState;
#endif


/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

wiced_result_t hid_app_init(void)
{
    /*SDP database init */
    wiced_bt_sdp_db_init(( uint8_t* )wiced_bt_sdp_db, wiced_bt_sdp_db_size);

    /* general hid app init */
    wiced_hidd_app_init(BT_DEVICE_TYPE_BREDR);

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);

    /* BT HID power management configuration  */
    wiced_bt_hidd_configure_power_management_params(bthid_powerStateList, bthid_powerStateList_num, 
                                                bthid_SSRPowerStatesList, bthid_SSRPowerStatesList_num);

    /* start keyboard  */
    kbapp_create();

    return WICED_BT_SUCCESS;
}

/*
 * kb bt link management callbacks 
 * */
wiced_result_t kb_management_cback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result = WICED_BT_SUCCESS;    

    switch( event )
    {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:
#ifdef TESTING_USING_HCI
            /* Register callback for receiving hci traces */
            wiced_bt_dev_register_hci_trace( hci_hid_hci_trace_cback );
#endif        
            /* Initialize the HID device */
            hid_app_init();
            break;

        case BTM_POWER_MANAGEMENT_STATUS_EVT:
            WICED_BT_TRACE("BTM_POWER_MANAGEMENT_STATUS_EVT\n");
            break;

        case BTM_PIN_REQUEST_EVT:
            WICED_BT_TRACE("BTM_PIN_REQUEST_EVT\n");
            bthidlink_pinCodeRequest((wiced_bt_dev_name_and_class_t *)p_event_data);
            break;
            
        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            WICED_BT_TRACE("BTM_USER_CONFIRMATION_REQUEST_EVT\n");
            wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr);
            break;
            
        case BTM_PASSKEY_REQUEST_EVT:
            WICED_BT_TRACE("BTM_PASSKEY_REQUEST_EVT\n");
#ifdef USE_KEYBOARD_IO_CAPABILITIES
            bthidlink_passKeyRequest(p_event_data);
#else
            wiced_bt_dev_pass_key_req_reply(WICED_BT_SUCCESS,p_event_data->user_passkey_request.bd_addr, 0);
#endif
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:
            WICED_BT_TRACE("BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT\n");
#ifdef USE_KEYBOARD_IO_CAPABILITIES            
            p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap = BTM_IO_CAPABILITIES_KEYBOARD_ONLY;
#else
            p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
#endif
            p_event_data->pairing_io_capabilities_br_edr_request.oob_data = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_br_edr_request.auth_req = BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_NO;
            p_event_data->pairing_io_capabilities_br_edr_request.is_orig = FALSE;
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_RESPONSE_EVT:
            WICED_BT_TRACE("BTM_PAIRING_IO_CAPABILITIES_BR_EDR_RESPONSE_EVT peer_bd_addr: %B, peer_io_cap: %d, peer_oob_data: %d, peer_auth_req: %d\n", 
                        p_event_data->pairing_io_capabilities_br_edr_response.bd_addr,
                        p_event_data->pairing_io_capabilities_br_edr_response.io_cap,
                        p_event_data->pairing_io_capabilities_br_edr_response.oob_data,
                        p_event_data->pairing_io_capabilities_br_edr_response.auth_req);
            break;
            
        case BTM_PAIRING_COMPLETE_EVT:
            WICED_BT_TRACE("BTM_PAIRING_COMPLETE_EVT\n");
            if(p_event_data->pairing_complete.transport == BT_TRANSPORT_BR_EDR)
            {
                //bonding successful
                if (!p_event_data->pairing_complete.pairing_complete_info.br_edr.status)
                {
                    WICED_BT_TRACE( "BONDED successful\n");
                }
            }
            break;
            
        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            WICED_BT_TRACE("BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT");
            memcpy(&bthidlink_link_keys, &p_event_data->paired_device_link_keys_update, sizeof(wiced_bt_device_link_keys_t));
            WICED_BT_TRACE(" %B\n", bthidlink_link_keys.bd_addr );
            wiced_trace_array((uint8_t*)&bthidlink_link_keys.key_data, BTM_SECURITY_KEY_DATA_LEN);
            
            //add to pairing host list
            wiced_bt_hidd_host_info_add_host_at_top(p_event_data->paired_device_link_keys_update.bd_addr, &bthidlink_link_keys, 0);
            break;
                
        case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        {
            wiced_bt_device_link_keys_t *pLinkKeys = (wiced_bt_device_link_keys_t *)wiced_bt_hidd_host_info_get_linkkey_by_index(0);
            WICED_BT_TRACE("BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT\n");
            
            if (pLinkKeys && !memcmp(pLinkKeys->bd_addr, p_event_data->paired_device_link_keys_request.bd_addr, BD_ADDR_LEN))
            {
                memcpy(&p_event_data->paired_device_link_keys_request, pLinkKeys, sizeof(wiced_bt_device_link_keys_t));
            }
            else
            {
                result = WICED_BT_ERROR;
            }
        }
        break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            /* Request to store newly generated local identity keys to NVRAM */
            /* (sample app does not store keys to NVRAM) */
            break;

        case  BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:

            /* Request to restore local identity keys from NVRAM (requested during Bluetooth start up) */
            /* (sample app does not store keys to NVRAM. New local identity keys will be generated).   */
            result = WICED_BT_NO_RESOURCES;
            break;

        case BTM_SECURITY_FAILED_EVT:
            WICED_BT_TRACE("BTM_SECURITY_FAILED_EVT. hci_status:%d\n", p_event_data->security_failed.hci_status);
            bt_hidd_link.security_failed = p_event_data->security_failed.hci_status;
            memset(&bthidlink_link_keys, 0, sizeof(wiced_bt_device_link_keys_t));
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            WICED_BT_TRACE("BTM_ENCRYPTION_STATUS_EVT, result=%d\n", p_event_data->encryption_status.result);
            if (p_event_data->encryption_status.result == WICED_SUCCESS)
            {
                bt_hidd_link.encrypt_status.encrypted = WICED_TRUE;
            }
            else
            {
                bt_hidd_link.encrypt_status.encrypted = WICED_FALSE;
                
            }
            memcpy(bt_hidd_link.encrypt_status.bdAddr, p_event_data->encryption_status.bd_addr, BD_ADDR_LEN);
            break;

        default:
            WICED_BT_TRACE("UNPROCESSED KB_MANAGEMENT_CBACK: %d!!!\n", event );
            break;
    }

    return result;
}

#ifdef TESTING_USING_HCI
/*
 *  Pass protocol traces up through the UART
 */
void hci_hid_hci_trace_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data )
{
    //send the trace
    wiced_transport_send_hci_trace( NULL, type, length, p_data  );
}

/*
 * handle reset command from UART
 */
void hci_hid_dev_handle_reset_cmd( void )
{
    // trip watch dog now.
    wiced_hal_wdog_reset_system( );
}


/*
 * Handle host command to send HID report.
 */
void hci_hid_dev_handle_send_report( uint8_t channel, uint8_t type, uint8_t *p_data, uint16_t length )
{
    uint8_t  report_id = *p_data;
    uint16_t attr_handle;
    wiced_bool_t OK_to_send = WICED_FALSE;

    //send report only when link is connected
    if(wiced_bt_hidd_link_is_connected())
    {
        if(wiced_bt_hid_cfg_settings.security_requirement_mask)
        {
            if (bt_hidd_link.encrypt_status.encrypted)
            {
                OK_to_send = WICED_TRUE;
            }
        }
        else
        {
            OK_to_send = WICED_TRUE;
        }
    }
    else //enter discoverable/reconnecting
    {
        wiced_bt_hidd_link_connect();
    }

    if ( OK_to_send && (wiced_bt_buffer_poolutilization (HCI_ACL_POOL_ID) < 80) && (length > 0 ))
    {
        if (channel == HCI_CONTROL_HID_REPORT_CHANNEL_CONTROL)
        {
            WICED_BT_TRACE( "Do NOT support initiating control channel report.\n");
        }
        else if ( type == HID_PAR_REP_TYPE_INPUT )
        {
            WICED_BT_TRACE( "send input report (reportID=%d) %d bytes\n", report_id, length);

            if (report_id == kbAppConfig.stdRptID)
            {
                if (length == kbAppState->stdRptSize)
                {
                    // Send the std report
                    wiced_bt_hidd_send_data(FALSE, type, p_data, length);
                }
                else
                {
                    WICED_BT_TRACE("report size incorrect. expected length: %d\n", kbAppState->stdRptSize);
                }
            }
            else if (report_id == kbAppConfig.bitReportID)
            {
                if (length == kbAppState->bitReportSize)
                {
                    // Send the bit rpt.
                    wiced_bt_hidd_send_data(FALSE, type, p_data, length);
                }
                else
                {
                    WICED_BT_TRACE("report size incorrect. expected length: %d\n", kbAppState->bitReportSize);
                }
            }
            else if (report_id == kbAppConfig.sleepReportID)
            {
                if (length == sizeof(KeyboardSleepReport))
                {
                    // Send the sleep rpt.
                    wiced_bt_hidd_send_data(FALSE, type, p_data, length);
                }
                else
                {
                    WICED_BT_TRACE("report size incorrect. expected length: %d\n", sizeof(KeyboardSleepReport));
                }
            }
            else if (report_id == kbAppConfig.pinReportID)
            {
                if (length == sizeof(KeyboardPinEntryReport))
                {
                    // Send the pin rpt.
                    wiced_bt_hidd_send_data(FALSE, type, p_data, length);
                }
                else
                {
                    WICED_BT_TRACE("report size incorrect. expected length: %d\n", sizeof(KeyboardPinEntryReport));
                }
            }
            else if (report_id == kbAppConfig.funcLockReportID)
            {
                if (length == sizeof(KeyboardFuncLockReport))
                {
                    // Send the function lock rpt.
                    wiced_bt_hidd_send_data(FALSE, type, p_data, length);
                }
                else
                {
                    WICED_BT_TRACE("report size incorrect. expected length: %d\n", sizeof(KeyboardFuncLockReport));
                }
            }
            else if (report_id == kbAppConfig.scrollReportID)
            {
                if (length == kbAppConfig.scrollReportLen)
                {
                    // Send the scroll rpt.
                    wiced_bt_hidd_send_data(FALSE, type, p_data, length);
                }
                else
                {
                    WICED_BT_TRACE("report size incorrect. expected length: %d\n", kbAppConfig.scrollReportLen);
                }
            }
            else
            {
                WICED_BT_TRACE("invalid report ID\n");
            }
        }
        else if ( type == HID_PAR_REP_TYPE_FEATURE )
        {
            WICED_BT_TRACE( "send feature report %d bytes\n", length);
            // Send the feature rpt.
            wiced_bt_hidd_send_data(FALSE, type, p_data, length);
        }
        else
        {
            WICED_BT_TRACE("invalid report type\n");
        }
    }
}

void hci_hid_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len )
{
    uint8_t bytes_written;

    switch( cmd_opcode )
    {
    case HCI_CONTROL_COMMAND_RESET:
        hci_hid_dev_handle_reset_cmd( );
        break;
        
    case HCI_CONTROL_HID_COMMAND_ACCEPT_PAIRING:
        if (*p_data)
        {
            kbapp_connectButtonPressed();
        }
        else
        {
            if (wiced_bt_hidd_link_is_discoverable())
                wiced_bt_hidd_link_enter_disconnected();
            else
                wiced_bt_hidd_link_disable_page_and_inquiry_scans();
        }
        break;
        
    case HCI_CONTROL_HID_COMMAND_CONNECT:
        wiced_bt_hidd_link_connect();
        break;

    case HCI_CONTROL_HIDD_COMMAND_DISCONNECT:
        wiced_bt_hidd_link_disconnect();
        break;

    case HCI_CONTROL_HIDD_COMMAND_VIRTUAL_UNPLUG:
        wiced_bt_hidd_link_virtual_cable_unplug();
        break;

    case HCI_CONTROL_HID_COMMAND_SEND_REPORT:
        WICED_BT_TRACE( "HCI_CONTROL_HID_COMMAND_SEND_REPORT: ");
        wiced_trace_array(p_data, data_len);
        hci_hid_dev_handle_send_report( p_data[0], p_data[1], &p_data[2], ( uint16_t )( data_len - 2 ) );
        break;

    case HCI_CONTROL_MISC_COMMAND_GET_VERSION:
        hci_control_misc_handle_get_version();
        break;

    default:
        WICED_BT_TRACE( "??? Don't care command code : %d\n", cmd_opcode);
        break;
    }
}

uint32_t hci_hid_dev_handle_command( uint8_t *p_data, uint32_t length )
{
    uint16_t opcode;
    uint16_t payload_len;
    uint8_t status = HCI_CONTROL_STATUS_SUCCESS;
    uint8_t* p_rx_buf = p_data;

    WICED_BT_TRACE("hci_hid_dev_handle_command\n");

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

    hci_hid_handle_command( opcode, p_data, payload_len );

    //Freeing the buffer in which data is received
    wiced_transport_free_buffer( p_rx_buf );
    return ( 0 );
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

#endif

/*
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that BT device is ready.
 */
void application_start( void )
{
#ifdef WICED_BT_TRACE_ENABLE
#ifdef TESTING_USING_HCI
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#else
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_PUART);
#endif
#endif

    //restore content from AON memory when wake up from shutdown sleep (SDS)
    kbapp_aon_restore();

    wiced_bt_stack_init (kb_management_cback, &wiced_bt_hid_cfg_settings, wiced_bt_hid_cfg_buf_pools);
       
#ifdef TESTING_USING_HCI
    wiced_transport_init( &transport_cfg );
#endif
}


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
 * WICED Bluetooth OTA Upgrade
 *
 * This file provides function required to support Over the Air WICED Upgrade.
 * Both secure and none secure services are supported.  In the non-secure
 * case the software is only protected by the checksum.  In the secure case
 * the image is protected with a digital signature which is verified using
 * the private key passed by the application.
 *
 * To download host sends command to download with length of the patch to be
 * transmitted.Commands and portions of data is sent over SPP.
 * In case of an error Error Response indicates failure to the host.
 * Host sends fixed chunks of data.  After all the bytes has been downloaded
 * and acknowledged host sends verify command that includes CRC32 of the
 * whole patch.  During the download device saves data directly to the
 * serial flash.  At the verification stage device reads data back from the
 * NVRAM and calculates checksum of the data stored there.  Result of the
 * verification is indicated in the SPP Response.
 *
 */
 #if (OTA_FW_UPGRADE_TYPE == SPP_OTA_FW_UPGRADE)
#include "spp_ota_fw_upgrade.h"
#include "bt_types.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_ota_firmware_upgrade.h"
#include "wiced_bt_trace.h"
#include "wiced_gki.h"
#include "wiced_timer.h"
#include "sha2.h"
#include "p_256_multprecision.h"
#include "p_256_ecc_pp.h"
#include "wiced_bt_spp.h"
#include "ota_fw_upgrade.h"

/******************************************************
 *                      Constants
 ******************************************************/

#define OTA_FW_UPGRADE_CONTROL_COMMAND              01
#define OTA_FW_UPGRADE_DATA                         02
#define OTA_FW_UPGRADE_EVENT                        03

wiced_bool_t ota_fw_upgrade_initialized = WICED_FALSE;

/******************************************************
 *               Variables Definitions
 ******************************************************/
extern ota_fw_upgrade_state_t                       ota_fw_upgrade_common_state;
extern Point                                        *p_ecdsa_public_key;
extern wiced_ota_firmware_upgrade_status_callback_t *ota_fw_upgrade_status_callback;
/*
 * Initializes global data
 */
void ota_fw_upgrade_init_data()
{
    ota_fw_upgrade_common_state.state           = OTA_STATE_IDLE;
}

/*
 * Return TRUE if status of the OTA Firmware Upgrade process is not IDLE or ABORTED
 */
wiced_bool_t wiced_ota_fw_upgrade_is_active(void)
{
    return (ota_fw_upgrade_initialized &&
           (ota_fw_upgrade_common_state.state != OTA_STATE_IDLE) &&
           (ota_fw_upgrade_common_state.state != OTA_STATE_ABORTED));
}

/*
 * Connection with peer device is established
 */
void wiced_ota_fw_upgrade_connection_status_event(wiced_bt_gatt_connection_status_t *p_status)
{
    // State is now idle
    ota_fw_upgrade_common_state.state = OTA_STATE_IDLE;
    if (p_status->connected)
    {
        memcpy(ota_fw_upgrade_common_state.bdaddr, p_status->bd_addr, BD_ADDR_LEN);
    }
}

wiced_result_t  wiced_spp_ota_firmware_upgrade_send_status( uint16_t handle, uint8_t status )
{
    wiced_result_t result = WICED_SUCCESS;
    uint8_t data[3];
    data[0] = OTA_FW_UPGRADE_EVENT << 4 | status;
    data[1] = 0;
    data[2] = 0; // Length 2 bytes; set to 0 as there is no payload attached to event at the moment

    if (!wiced_bt_spp_send_session_data( handle, data, 3 ) )
    {
        result = WICED_ERROR;
    }
    return result;
}

wiced_result_t wiced_spp_ota_firmware_upgrade_command_handler(uint16_t handle, uint8_t* p_data, uint32_t data_len )
{
    wiced_result_t result = WICED_ERROR;
    uint32_t       len = 0;
    uint8_t        command = *p_data & 0x0F;
    len = p_data[1] + (p_data[2] << 8);

    WICED_BT_TRACE("OTA Handler OTA_FW_UPGRADE_COMMAND packet %x data_len:%d  payload len: %d\n", *p_data, data_len, len );

    if( len == ( data_len - 3 ))
    {
       if ( ota_fw_upgrade_command_handler( handle, command, p_data+3, len ) )
       {
           result = WICED_SUCCESS;
       }
    }
    else
    {
        WICED_BT_TRACE("OTA Handler OTA_FW_UPGRADE_COMMAND payload length does not match");
    }
    return result;
}

wiced_result_t wiced_spp_ota_firmware_upgrade_data_handler(uint16_t handle, uint8_t* p_data, uint32_t data_len )
{
    wiced_result_t result = WICED_ERROR;
    uint32_t       len = 0;
    len = p_data[1] + (p_data[2] << 8);
    WICED_BT_TRACE("OTA Handler OTA_FW_UPGRADE_DATA packet data_len:%d  payload len: %d\n", data_len, len );
    if( len == ( data_len - 3 ))
    {
        if ( ota_fw_upgrade_image_data_handler(handle, p_data + 3, len ) )
        {
            result = WICED_SUCCESS;
            wiced_spp_ota_firmware_upgrade_send_status( handle, WICED_OTA_UPGRADE_STATUS_CONTINUE );
        }
    }
    else
    {
        WICED_BT_TRACE("OTA Handler OTA_FW_UPGRADE_DATA payload length does not match");
    }
    return result;
}

wiced_result_t wiced_spp_ota_firmware_upgrade_handler(uint16_t handle, uint8_t* p_data, uint32_t data_len )
{
    wiced_result_t result = WICED_ERROR;
    uint8_t type          = *p_data >> 4;

#if OTA_UPGRADE_DEBUG
    WICED_BT_TRACE("OTA Handler Type:%x\n", type );
#endif
    if ( type == OTA_FW_UPGRADE_DATA )
    {
        result = wiced_spp_ota_firmware_upgrade_data_handler(handle, p_data,data_len );
    }
    else if ( type == OTA_FW_UPGRADE_CONTROL_COMMAND )
    {
        result = wiced_spp_ota_firmware_upgrade_command_handler( handle, p_data, data_len );
    }
    return result;
}

wiced_result_t  wiced_ota_firmware_upgrade_send_status( uint16_t handle, uint8_t status )
{
    return wiced_spp_ota_firmware_upgrade_send_status(handle, status );
}

wiced_bool_t wiced_ota_indication_enabled( void )
{
    return 0;
}
#endif

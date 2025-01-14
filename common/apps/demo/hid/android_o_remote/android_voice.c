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
 * Android TV Voice Service protocol implementation.
 *
 */
//#include "bt_types.h"
#include "spar_utils.h"
#include "wiced_bt_gatt.h"
#include "android_voice.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_event.h"
#include "wiced_memory.h"
#include "ble_remote.h"
#include "wiced_hidd_micaudio.h"

/******************************************************
 *                      Constants
 ******************************************************/

/******************************************************
 *               Variables Definitions
 ******************************************************/ 
extern wiced_bool_t blehidlink_connection_param_updated;
extern tMicAudio micAudioDriver;
extern uint16_t characteristic_client_configuration[];
extern void     bleremoteapp_updateClientConfFlags(uint16_t enable, uint16_t feature_bit);

/* 
 * Set new value for client configuration descriptor
 */
void android_voice_set_rx_client_configuration(uint16_t client_config)
{
    uint8_t  notification = client_config & GATT_CLIENT_CONFIG_NOTIFICATION;
    
    WICED_BT_TRACE("android_voice_set_rx_client_configuration: %d\n", client_config);
    
    bleremoteapp_updateClientConfFlags(notification, KBAPP_CLIENT_CONFIG_NOTIF_ATV_VOICE_RX);
}

/*
 * Set new value for client configuration descriptor
 */
void android_voice_set_ctl_client_configuration(uint16_t client_config)
{
    uint8_t  notification = client_config & GATT_CLIENT_CONFIG_NOTIFICATION;
    
    WICED_BT_TRACE("android_voice_set_ctl_client_configuration: %d\n", client_config);
    
    bleremoteapp_updateClientConfFlags(notification, KBAPP_CLIENT_CONFIG_NOTIF_ATV_VOICE_CTL);
}

/*
 * Send Notification if allowed, or indication if allowed, or return error
 */
int android_voice_send_notification_serialized(void *data)
{
    uint16_t *buffer = (uint16_t *)data;
    uint16_t conn_id, attr_handle, val_len;

    conn_id     = *buffer++;
    attr_handle = *buffer++;
    val_len     = *buffer++;


    uint16_t cccd = (attr_handle == HANDLE_ATV_VOICE_RX_CHARACTERISTIC_VALUE) ?
            characteristic_client_configuration[KBAPP_CLIENT_CONFIG_NOTIF_ATV_VOICE_RX_BIT_OFFSET] : characteristic_client_configuration[KBAPP_CLIENT_CONFIG_NOTIF_ATV_VOICE_CTL_BIT_OFFSET];

    if (cccd & GATT_CLIENT_CONFIG_NOTIFICATION)
    {
        wiced_bt_gatt_send_notification(conn_id, attr_handle, val_len, (uint8_t *)buffer);
    }
    else if (cccd & GATT_CLIENT_CONFIG_INDICATION)
    {
        wiced_bt_gatt_send_indication(conn_id, attr_handle, val_len, (uint8_t *)buffer);
    }
    else
    {
        WICED_BT_TRACE("Notification failed\n");
    }
    wiced_bt_free_buffer(data);

    return 0;
}

wiced_bt_gatt_status_t android_voice_send_notification(uint16_t conn_id, uint16_t attr_handle, uint16_t val_len, uint8_t *p_val, wiced_bool_t serialized)
{
    uint16_t *buffer;
    uint16_t *p;

    if ((buffer = (uint16_t *)wiced_bt_get_buffer(6 + val_len)) != NULL)
    {
        p = buffer;
        *p++ = conn_id;
        *p++ = attr_handle;
        *p++ = val_len;
        memcpy(p, p_val, val_len);
        if (serialized)
        {
            wiced_app_event_serialize (android_voice_send_notification_serialized, (void *)buffer);
        }
        else
        {
            android_voice_send_notification_serialized(buffer);
        }
        return WICED_BT_GATT_SUCCESS;
    }
    
    return WICED_BT_GATT_ERROR;
}

/*
 * Process GATT Read request
 */
wiced_bt_gatt_status_t android_voice_read_handler(uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data)
{
    switch (p_read_data->handle)
    {
    case HANDLE_ATV_VOICE_RX_CLIENT_CONFIGURATION_DESCRIPTOR:   
    case HANDLE_ATV_VOICE_CTL_CLIENT_CONFIGURATION_DESCRIPTOR:
        if (p_read_data->offset >= 2)
            return WICED_BT_GATT_INVALID_OFFSET;

        if (*p_read_data->p_val_len < 2)
            return WICED_BT_GATT_INVALID_ATTR_LEN;

        if (p_read_data->offset == 1)
        {
            p_read_data->p_val[0]   = 0;
            *p_read_data->p_val_len = 1;
        }
        else
        {
            if (p_read_data->handle == HANDLE_ATV_VOICE_RX_CLIENT_CONFIGURATION_DESCRIPTOR)
            {
                p_read_data->p_val[0] = characteristic_client_configuration[KBAPP_CLIENT_CONFIG_NOTIF_ATV_VOICE_RX_BIT_OFFSET];
            }
            else 
            {
                p_read_data->p_val[0] = characteristic_client_configuration[KBAPP_CLIENT_CONFIG_NOTIF_ATV_VOICE_CTL_BIT_OFFSET];
            }

            p_read_data->p_val[1]   = 0;
            *p_read_data->p_val_len = 2;
        }
        return WICED_BT_GATT_SUCCESS;

        
    case HANDLE_ATV_VOICE_RX_CHARACTERISTIC_VALUE:
        break;
    case HANDLE_ATV_VOICE_CTL_CHARACTERISTIC_VALUE:
        break;

    default: 
        break;
    }
    return WICED_BT_GATT_INVALID_HANDLE;
}



/*
 * Process GATT Write request
 */
wiced_bt_gatt_status_t android_voice_write_handler(uint16_t conn_id, wiced_bt_gatt_write_t *p_write_data)
{
    switch (p_write_data->handle)
    {
    case HANDLE_ATV_VOICE_TX_CHARACTERISTIC_VALUE:
    {
        uint8_t *p = p_write_data->p_val;
        uint16_t len = p_write_data->val_len;
        uint8_t rsp_data[9]={ATV_VOICE_SERVICE_GET_CAPS_RESP, 0, 4, 0, 1, 0, 0x86, 0, 0x14}; //version 0.4; codec supported: ADPCM 8khz/16bit; bytes/frames: 0x0086; bytes/characteristic: 0x0014 . From the TV data, it looks like it is using big endian.
        uint8_t command, temp;
        uint16_t tv_codec;
        

        command = *p++;
        len--;
        WICED_BT_TRACE("ATVV_CHAR_TX cmd:%d\n", command);
        
        if (command == ATV_VOICE_SERVICE_GET_CAPS_REQUEST)
        {
            if (len < 4)
            {
                WICED_BT_TRACE("len too short: %d\n", len);
                return WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            wiced_trace_array(p, 4);
            WICED_BT_TRACE("TV version: %d.%d\n", *p++, *p++);
            temp = *p++;
            WICED_BT_TRACE("TV codec supported: 0x%x\n", (temp<<8)|(*p));

            android_voice_send_notification(conn_id, HANDLE_ATV_VOICE_CTL_CHARACTERISTIC_VALUE, 9, rsp_data, WICED_TRUE);
            return WICED_BT_GATT_SUCCESS;
        }
        else if (command == ATV_VOICE_SERVICE_MIC_OPEN)
        {
            if (len < 2)
            {
                WICED_BT_TRACE("len too short: %d\n", len);
                return WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            wiced_trace_array(p, 2);
            temp = *p++;
            tv_codec = (temp<<8)|(*p);
            WICED_BT_TRACE("TV codec useded: 0x%x\n", tv_codec);
            
            //TODO: if codec used by TV can not be supported. we need to send back CHAR_CTL: MIC_OPEN_ERROR
            if (tv_codec == 1)
            {
                micAudioDriver.codec_sampling_freq = WICED_HIDD_CODEC_SAMP_FREQ_8K;
            }
            else if (tv_codec == 2)
            {
                micAudioDriver.codec_sampling_freq = WICED_HIDD_CODEC_SAMP_FREQ_16K;
            }
            
            bleremoteapp_handleVoiceTxCommand(ATV_VOICE_SERVICE_MIC_OPEN);
            
            return WICED_BT_GATT_SUCCESS;
        }
        else if (command == ATV_VOICE_SERVICE_MIC_CLOSE)
        {
            bleremoteapp_handleVoiceTxCommand(ATV_VOICE_SERVICE_MIC_CLOSE);
            
            return WICED_BT_GATT_SUCCESS;
        }
        return WICED_BT_GATT_ILLEGAL_PARAMETER;
    }
    
    case HANDLE_ATV_VOICE_RX_CLIENT_CONFIGURATION_DESCRIPTOR:
        if (p_write_data->val_len != 2)
        {
            return WICED_BT_GATT_INVALID_ATTR_LEN;
        }
        android_voice_set_rx_client_configuration(p_write_data->p_val[0] + (p_write_data->p_val[1] << 8));
        break;

    case HANDLE_ATV_VOICE_CTL_CLIENT_CONFIGURATION_DESCRIPTOR:
        if (p_write_data->val_len != 2)
        {
            return WICED_BT_GATT_INVALID_ATTR_LEN;
        }
        android_voice_set_ctl_client_configuration(p_write_data->p_val[0] + (p_write_data->p_val[1] << 8));
        break;

    default:
        return WICED_BT_GATT_INVALID_HANDLE;
        break;
    }
    return WICED_BT_GATT_SUCCESS;
}






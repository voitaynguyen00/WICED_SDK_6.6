/*
* Copyright 2021, Cypress Semiconductor Corporation or a subsidiary of 
 * Cypress Semiconductor Corporation. All Rights Reserved.
 * 
 * This software, associated documentation and materials ("Software"),
 * is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
*/


/**
 * @file wiced_hci_bt_a2dp.c
 *
 * This file contains implementation of the audio sink interface.
 *
 */
#include <string.h>
#include <stdlib.h>
#include "wiced_hci.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_a2dp_sink.h"
#include "wwd_debug.h"
#include "wiced_hci_bt_internal_common.h"
#include "wiced_bt_audio.h"

/******************************************************
 *                    Constants
 ******************************************************/

#define AV_SOURCE_ROLE              0x00
#define AV_SINK_ROLE                0x01

#define A2DP_CONN_MAX_LIMIT     2

/******************************************************
 *                   Structures
 ******************************************************/
/* Type for A2DP sink control block */
typedef struct
{
    uint8_t                          codec_id;
    wiced_bt_a2dp_sink_control_cb_t  control_cb;     /* Application registered control callback function */
} wiced_bt_hci_a2dp_sink_cb_t;

typedef struct
{
    wiced_bt_device_address_t        peer_addr;
    uint16_t                         handle;
    wiced_bool_t                     in_use;
    uint8_t                          av_stream_label; /* This is updated on AV_START ind and later used in AV_START RSP while responding to 20706 */
} wiced_bt_hci_a2dp_remote_dev_t;

/*****************************************************************************
** Global data
*****************************************************************************/
/* A2DP sink control block */
wiced_bt_hci_a2dp_sink_cb_t         wiced_bt_hci_a2dp_sink_cb;
wiced_bt_hci_a2dp_remote_dev_t      wiced_bt_hci_a2dp_remote_dev[A2DP_CONN_MAX_LIMIT] =
{
    {
        .peer_addr       = {0,},
        .handle          = 0,
        .in_use          = WICED_FALSE,
        .av_stream_label = 0
    },
    {
        .peer_addr       = {0,},
        .handle          = 0,
        .in_use          = WICED_FALSE,
        .av_stream_label = 0
    }
};

wiced_bt_a2dp_sink_data_cb_t        wiced_bt_hci_a2dp_sink_data_cb; //Data Callback

/******************************************************
 *               Static Function Declarations
 ******************************************************/
static void wiced_hci_audio_cb(uint16_t event, uint8_t* payload, uint32_t len);
static int get_a2d_context_index_by_handle(uint16_t    handle);
static int set_a2d_context_index_peer(wiced_bt_device_address_t remote_addr);

/******************************************************
 *               Function Definitions
 ******************************************************/

/*******************************************************************************
**
** Function         wiced_bt_a2dp_sink_utils_bdcpy
**
** Description      Copy bd addr b to a.
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_a2dp_sink_utils_bdcpy(wiced_bt_device_address_t a,
    const wiced_bt_device_address_t b)
{
    int i;
    for(i = BD_ADDR_LEN; i!=0; i--)
    {
        *a++ = *b++;
    }
}

/*******************************************************************************
**
** Function         wiced_bt_a2dp_sink_utils_bdcmp
**
** Description          Compare bd addr b to a.
**
** Returns              Zero if b==a, nonzero otherwise (like memcmp).
**
*******************************************************************************/
int wiced_bt_a2dp_sink_utils_bdcmp(const wiced_bt_device_address_t a,
    const wiced_bt_device_address_t b)
{
    int i;
    for(i = BD_ADDR_LEN; i!=0; i--)
    {
        if( *a++ != *b++ )
        {
            return -1;
        }
    }
    return 0;
}

wiced_result_t wiced_bt_a2dp_sink_init(wiced_bt_a2dp_config_data_t *p_config_data,
    wiced_bt_a2dp_sink_control_cb_t control_cb )
{
    uint8_t*        data  = NULL;
    uint8_t*        p     = NULL;
    uint16_t      length  = 0;
    uint16_t      length_of_codec_info;

    if(p_config_data == NULL || control_cb == NULL)
    {
        WPRINT_APP_ERROR(("%s: Invalid input parameters", __FUNCTION__));
        return WICED_BADARG;
    }

    /* Copy the callback information */
    wiced_bt_hci_a2dp_sink_cb.control_cb = control_cb;

    wiced_hci_set_event_callback(AUDIO_SINK, wiced_hci_audio_cb);

    if(p_config_data->codec_capabilities.info->codec_id == WICED_BT_A2DP_CODEC_SBC)
        length_of_codec_info = sizeof(uint8_t)  + sizeof(wiced_bt_a2d_sbc_cie_t);
    else if(p_config_data->codec_capabilities.info->codec_id == WICED_BT_A2DP_CODEC_M12)
        length_of_codec_info = sizeof(uint8_t)  + sizeof(wiced_bt_a2d_m12_cie_t);
    else if(p_config_data->codec_capabilities.info->codec_id == WICED_BT_A2DP_CODEC_M24)
        length_of_codec_info = sizeof(uint8_t)  + sizeof(wiced_bt_a2d_m24_cie_t);
    else if(p_config_data->codec_capabilities.info->codec_id == WICED_BT_A2DP_CODEC_M24)
        length_of_codec_info = sizeof(uint8_t)  + sizeof(wiced_bt_a2d_vendor_cie_t);
    else
        return WICED_BADARG;

    length += ( sizeof(uint8_t) + sizeof(uint8_t) + sizeof(uint16_t) + length_of_codec_info);
    /* allocate memory for the data */
    data = (uint8_t*)calloc(length, sizeof(uint8_t));
    p = data;

    UINT8_TO_STREAM(p, AV_SINK_ROLE);
    UINT8_TO_STREAM(p, p_config_data->feature_mask & 0xff);
    UINT16_TO_STREAM(p, length_of_codec_info);

    if(p_config_data->codec_capabilities.info->codec_id == WICED_BT_A2DP_CODEC_SBC)
    {
        UINT8_TO_STREAM(p, p_config_data->codec_capabilities.info->codec_id);
        UINT8_TO_STREAM(p, p_config_data->codec_capabilities.info->cie.sbc.samp_freq);
        UINT8_TO_STREAM(p, p_config_data->codec_capabilities.info->cie.sbc.ch_mode);
        UINT8_TO_STREAM(p, p_config_data->codec_capabilities.info->cie.sbc.block_len);
        UINT8_TO_STREAM(p, p_config_data->codec_capabilities.info->cie.sbc.num_subbands);
        UINT8_TO_STREAM(p, p_config_data->codec_capabilities.info->cie.sbc.alloc_mthd);
        UINT8_TO_STREAM(p, p_config_data->codec_capabilities.info->cie.sbc.max_bitpool);
        UINT8_TO_STREAM(p, p_config_data->codec_capabilities.info->cie.sbc.min_bitpool);
    }
    //TODO: Handling of Non SBC codec details.

 /*Comenting below code since It is decided that 2070X Embedded application itself handles this functionality.
      So this function can made stub function later. */
//    wiced_hci_send(HCI_CONTROL_AUDIO_COMMAND_INIT, data, length);

    free(data);
    return WICED_SUCCESS;
}

void wiced_bt_a2dp_sink_register_data_cback( wiced_bt_a2dp_sink_data_cb_t p_cback )
{
    wiced_bt_hci_a2dp_sink_data_cb = p_cback;
}


wiced_result_t wiced_bt_a2dp_sink_deinit(void)
{
    wiced_hci_set_event_callback(AUDIO_SINK, NULL);
    memset(&wiced_bt_hci_a2dp_sink_cb, 0 ,sizeof(wiced_bt_hci_a2dp_sink_cb));
    memset(&wiced_bt_hci_a2dp_remote_dev, 0, sizeof(wiced_bt_hci_a2dp_remote_dev));
    memset(&wiced_bt_hci_a2dp_sink_data_cb, 0, sizeof(wiced_bt_hci_a2dp_sink_data_cb));

    return WICED_SUCCESS;
}

wiced_result_t wiced_bt_a2dp_sink_connect(wiced_bt_device_address_t bd_address)
{
    uint8_t*                data  = NULL;
    uint8_t*                p     = NULL;
    uint16_t                length  = 0;


    length += ( sizeof(wiced_bt_device_address_t) + sizeof(uint8_t));
    /* allocate memory for the data */
    data = (uint8_t*)calloc(length, sizeof(uint8_t));
    p = data;
    BDADDR_TO_STREAM(p, bd_address);
    UINT8_TO_STREAM(p, AUDIO_ROUTE_I2S);
    wiced_hci_send(HCI_CONTROL_AUDIO_SINK_COMMAND_CONNECT, data, length);
    free(data);
    return WICED_SUCCESS;
}

wiced_result_t wiced_bt_a2dp_sink_disconnect( uint16_t handle )
{
    uint8_t*        data  = NULL;
    uint8_t*        p     = NULL;
    uint16_t        length  = 0;
    int             context_index;

    length = sizeof(uint16_t);
    /* allocate memory for the data */
    data = (uint8_t*)calloc(length, sizeof(uint8_t));
    p = data;
    context_index = get_a2d_context_index_by_handle(handle);
    if(context_index == A2DP_CONN_MAX_LIMIT)
    {
        WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
        free(data);
        return WICED_BADARG;
    }
    UINT16_TO_STREAM(p, wiced_bt_hci_a2dp_remote_dev[context_index].handle);
    wiced_hci_send(HCI_CONTROL_AUDIO_SINK_COMMAND_DISCONNECT, data, length);
    free(data);
    return WICED_SUCCESS;
}

wiced_result_t wiced_bt_a2dp_sink_start( uint16_t handle )
{
    uint8_t*        data  = NULL;
    uint8_t*        p     = NULL;
    uint16_t        length  = 0;
    int             context_index;

    length = sizeof(uint16_t);
    /* allocate memory for the data */
    data = (uint8_t*)calloc(length, sizeof(uint8_t));
    p = data;
    context_index = get_a2d_context_index_by_handle(handle);
    if(context_index == A2DP_CONN_MAX_LIMIT)
    {
        WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
        free(data);
        return WICED_BADARG;
    }
    UINT16_TO_STREAM(p, wiced_bt_hci_a2dp_remote_dev[context_index].handle);
    wiced_hci_send(HCI_CONTROL_AUDIO_SINK_COMMAND_START, data, length);
    free(data);
    return WICED_SUCCESS;
}

wiced_result_t wiced_bt_a2dp_sink_send_start_response( uint16_t handle, uint8_t label, uint8_t status )
{
    uint8_t*        data  = NULL;
    uint8_t*        p     = NULL;
    uint16_t        length  = 0;

    length = sizeof(uint16_t) + sizeof(uint8_t) + sizeof(uint8_t);

    /* allocate memory for the data */
    data = (uint8_t*)calloc(length, sizeof(uint8_t));
    p = data;

    UINT16_TO_STREAM(p, handle);
    UINT8_TO_STREAM(p, label);
    UINT8_TO_STREAM(p, status);

    wiced_hci_send(HCI_CONTROL_AUDIO_SINK_COMMAND_START_RSP, data, length);
    free(data);
    return WICED_SUCCESS;
}

wiced_result_t wiced_bt_a2dp_sink_suspend(uint16_t handle)
{
    uint8_t*        data  = NULL;
    uint8_t*        p     = NULL;
    uint16_t        length  = 0;
    int             context_index;

    length = sizeof(uint16_t);
    /* allocate memory for the data */
    data = (uint8_t*)calloc(length, sizeof(uint8_t));
    p = data;
    context_index = get_a2d_context_index_by_handle(handle);
    if(context_index == A2DP_CONN_MAX_LIMIT)
    {
        WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
        free(data);
        return WICED_BADARG;
    }
    UINT16_TO_STREAM(p, wiced_bt_hci_a2dp_remote_dev[context_index].handle);
    wiced_hci_send(HCI_CONTROL_AUDIO_SINK_COMMAND_STOP, data, length);
    free(data);
    return WICED_SUCCESS;
}

wiced_bool_t wiced_bt_a2dp_sink_update_route_config( uint16_t handle, wiced_bt_a2dp_sink_route_config *route_config )
{
    uint8_t*        data  = NULL;
    uint8_t*        p     = NULL;
    uint16_t        length  = 0;
    int             context_index;

    length = sizeof(uint16_t) + sizeof(uint8_t) + sizeof(uint8_t);
    /* allocate memory for the data */
    data = (uint8_t*)calloc(length, sizeof(uint8_t));
    p = data;
    context_index = get_a2d_context_index_by_handle(handle);
    if(context_index == A2DP_CONN_MAX_LIMIT)
    {
        WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
        free(data);
        return WICED_BADARG;
    }
    UINT16_TO_STREAM(p, handle);
    UINT8_TO_STREAM(p, route_config->route);
    UINT8_TO_STREAM(p, route_config->is_master);
    wiced_hci_send(HCI_CONTROL_AUDIO_SINK_COMMAND_CHANGE_ROUTE, data, length);
    free(data);
    return WICED_SUCCESS;
}

static int get_a2d_context_index_by_handle(uint16_t    handle)
{
    int i=0;

    while(i < A2DP_CONN_MAX_LIMIT)
    {
        if( (wiced_bt_hci_a2dp_remote_dev[i].in_use == WICED_TRUE) &&
            (wiced_bt_hci_a2dp_remote_dev[i].handle == handle))
        {
            WICED_HCI_DEBUG_LOG(( "get_a2d_context_index_by_handle Already present i:%d\n",i ));
            return i;
        }
        i++;
    }
    WICED_HCI_DEBUG_LOG(("get_a2d_context_index_by_handle Invalid!!\n" ));
    return A2DP_CONN_MAX_LIMIT;
}

static int set_a2d_context_index_peer(wiced_bt_device_address_t remote_addr)
{
    int i=0;
    while(i < A2DP_CONN_MAX_LIMIT)
    {
        if(wiced_bt_hci_a2dp_remote_dev[i].in_use == WICED_TRUE)
        {
            if(!wiced_bt_a2dp_sink_utils_bdcmp(wiced_bt_hci_a2dp_remote_dev[i].peer_addr, remote_addr))
             {
                 WICED_HCI_DEBUG_LOG(( "set_context_index Already present i:%d\n",i ));
                 return i;
             }
             else
             {
                 i=i+1;
                 continue;
             }
        }
        else
        {
            wiced_bt_hci_a2dp_remote_dev[i].in_use = WICED_TRUE;
            WICED_HCI_DEBUG_LOG( ( "set_context_index New entry i:%d\n",i ));
            return i;
        }
    }
    WICED_HCI_DEBUG_LOG(("set_a2d_context_index_peer Invalid!!\n" ));
    return A2DP_CONN_MAX_LIMIT;
}

static void wiced_hci_audio_cb(uint16_t event, uint8_t* payload, uint32_t len)
{
    uint8_t*                        p = payload;
    int                             context_index;
    wiced_bt_a2dp_sink_event_data_t sink_evt_data;

    //WICED_HCI_DEBUG_LOG(("%s event:%x\n", __func__, event));

    switch(event)
    {
        case HCI_CONTROL_AUDIO_SINK_EVENT_COMMAND_STATUS:
        {
            WICED_HCI_DEBUG_LOG(("%s: command status:%d\n", __FUNCTION__, *p));
        }
        break;
        case HCI_CONTROL_AUDIO_SINK_EVENT_CONNECTED:
        {
            STREAM_TO_BDADDR(sink_evt_data.connect.bd_addr, p);
            STREAM_TO_UINT16(sink_evt_data.connect.handle, p);
            sink_evt_data.connect.result = WICED_SUCCESS;

            WICED_HCI_DEBUG_LOG(("%s HCI_CONTROL_AUDIO_EVENT_CONNECTED: bdaddr: "BDADDR_FORMAT " handle:0x%x\n",
                __FUNCTION__,
                EXPAND_BDADDR(sink_evt_data.connect.bd_addr),
                sink_evt_data.connect.handle));

            context_index = set_a2d_context_index_peer(sink_evt_data.connect.bd_addr);
            if(context_index == A2DP_CONN_MAX_LIMIT)
            {
                WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
                break;
            }
            wiced_bt_a2dp_sink_utils_bdcpy( wiced_bt_hci_a2dp_remote_dev[context_index].peer_addr,
                sink_evt_data.connect.bd_addr);
            wiced_bt_hci_a2dp_remote_dev[context_index].handle = sink_evt_data.connect.handle;

            (*wiced_bt_hci_a2dp_sink_cb.control_cb)( WICED_BT_A2DP_SINK_CONNECT_EVT, &sink_evt_data );
        }
        break;
        case HCI_CONTROL_AUDIO_SINK_EVENT_CONNECTION_FAILED:
        {
            WICED_HCI_DEBUG_LOG(("%s: Connection FAILED!!\n", __FUNCTION__));
            sink_evt_data.connect.result = WICED_BT_ERROR;
            (*wiced_bt_hci_a2dp_sink_cb.control_cb)( WICED_BT_A2DP_SINK_CONNECT_EVT, &sink_evt_data );
        }
        break;
        case HCI_CONTROL_AUDIO_SINK_EVENT_DISCONNECTED:
        {
            WICED_HCI_DEBUG_LOG(("%s HCI_CONTROL_AUDIO_EVENT_DISCONNECTED\n", __FUNCTION__));
            STREAM_TO_UINT16(sink_evt_data.disconnect.handle, p);
            STREAM_TO_UINT8(sink_evt_data.disconnect.result, p);

            context_index = get_a2d_context_index_by_handle(sink_evt_data.disconnect.handle);
            if(context_index == A2DP_CONN_MAX_LIMIT)
            {
                WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
                break;
            }
            wiced_bt_a2dp_sink_utils_bdcpy(sink_evt_data.disconnect.bd_addr, wiced_bt_hci_a2dp_remote_dev[context_index].peer_addr);

            wiced_bt_hci_a2dp_remote_dev[context_index].in_use = WICED_FALSE;
            memset(wiced_bt_hci_a2dp_remote_dev[context_index].peer_addr, 0, sizeof(wiced_bt_device_address_t));
            wiced_bt_hci_a2dp_remote_dev[context_index].handle = 0;

            (*wiced_bt_hci_a2dp_sink_cb.control_cb)( WICED_BT_A2DP_SINK_DISCONNECT_EVT, &sink_evt_data );
        }
        break;
        case HCI_CONTROL_AUDIO_SINK_EVENT_STARTED:
        {
            WICED_HCI_DEBUG_LOG(("%s HCI_CONTROL_AUDIO_EVENT_STARTED\n", __FUNCTION__));
            STREAM_TO_UINT16(sink_evt_data.start_cfm.handle, p);
            sink_evt_data.start_cfm.result = WICED_SUCCESS;

            context_index = get_a2d_context_index_by_handle(sink_evt_data.start_cfm.handle);
            if(context_index == A2DP_CONN_MAX_LIMIT)
            {
                WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
                break;
            }
            wiced_bt_a2dp_sink_utils_bdcpy(sink_evt_data.start_cfm.bd_addr, wiced_bt_hci_a2dp_remote_dev[context_index].peer_addr);

            (*wiced_bt_hci_a2dp_sink_cb.control_cb)(WICED_BT_A2DP_SINK_START_CFM_EVT, &sink_evt_data);
        }
        break;
        case HCI_CONTROL_AUDIO_SINK_EVENT_START_IND:
        {
            STREAM_TO_UINT16(sink_evt_data.start_ind.handle, p);
            STREAM_TO_UINT8(sink_evt_data.start_ind.label, p);
            sink_evt_data.start_ind.result = A2D_SUCCESS;
            /* TODO: enable the code in 20706 Headset App to get status. */
            WICED_HCI_DEBUG_LOG(("%s HCI_CONTROL_AUDIO_EVENT_START_IND handle:%x label:%x\n",
            __FUNCTION__, sink_evt_data.start_ind.handle, sink_evt_data.start_ind.label));

            context_index = get_a2d_context_index_by_handle(sink_evt_data.start_ind.handle);
            if(context_index == A2DP_CONN_MAX_LIMIT)
            {
                WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
                break;
            }
            wiced_bt_hci_a2dp_remote_dev[context_index].av_stream_label = sink_evt_data.start_ind.label;

            (*wiced_bt_hci_a2dp_sink_cb.control_cb)(WICED_BT_A2DP_SINK_START_IND_EVT, &sink_evt_data);
        }
        break;
        case HCI_CONTROL_AUDIO_SINK_EVENT_STOPPED:
        {
            WICED_HCI_DEBUG_LOG(("%s HCI_CONTROL_AUDIO_EVENT_STOPPED\n", __FUNCTION__));
            STREAM_TO_UINT16(sink_evt_data.suspend.handle, p);

            context_index = get_a2d_context_index_by_handle(sink_evt_data.suspend.handle);
            if(context_index == A2DP_CONN_MAX_LIMIT)
            {
                WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
                break;
            }
            wiced_bt_a2dp_sink_utils_bdcpy(sink_evt_data.suspend.bd_addr, wiced_bt_hci_a2dp_remote_dev[context_index].peer_addr);
            sink_evt_data.suspend.result = WICED_SUCCESS;

            (*wiced_bt_hci_a2dp_sink_cb.control_cb)(WICED_BT_A2DP_SINK_SUSPEND_EVT, &sink_evt_data);
        }
        break;
        case HCI_CONTROL_AUDIO_SINK_EVENT_CODEC_CONFIGURED:
        {
            WICED_HCI_DEBUG_LOG(("%s HCI_CONTROL_AUDIO_EVENT_CODEC_CONFIGURED\n", __FUNCTION__));
            // TODO: handle should be sent from the Embedded App.
            sink_evt_data.codec_config.handle = wiced_bt_hci_a2dp_remote_dev[0].handle;
            STREAM_TO_UINT8(sink_evt_data.codec_config.codec.codec_id, p);
            wiced_bt_hci_a2dp_sink_cb.codec_id = sink_evt_data.codec_config.codec.codec_id;
            STREAM_TO_UINT8(sink_evt_data.codec_config.codec.cie.sbc.samp_freq, p);
            STREAM_TO_UINT8(sink_evt_data.codec_config.codec.cie.sbc.ch_mode, p);
            STREAM_TO_UINT8(sink_evt_data.codec_config.codec.cie.sbc.block_len, p);
            STREAM_TO_UINT8(sink_evt_data.codec_config.codec.cie.sbc.num_subbands, p);
            STREAM_TO_UINT8(sink_evt_data.codec_config.codec.cie.sbc.alloc_mthd, p);
            STREAM_TO_UINT8(sink_evt_data.codec_config.codec.cie.sbc.max_bitpool, p);
            STREAM_TO_UINT8(sink_evt_data.codec_config.codec.cie.sbc.min_bitpool, p);
            (*wiced_bt_hci_a2dp_sink_cb.control_cb)(WICED_BT_A2DP_SINK_CODEC_CONFIG_EVT, &sink_evt_data);
        }
        break;
        case HCI_CONTROL_AUDIO_SINK_EVENT_AUDIO_DATA:
        {
            wiced_bt_a2dp_sink_audio_data_t audio_data;
            uint16_t media_info;
            uint8_t media_header_len;
            BT_HDR  *p_pkt;
            uint8_t* p_avdtp_header = p;
            BE_STREAM_TO_UINT16(media_info, p);
            BE_STREAM_TO_UINT16(audio_data.seq_num, p);
            BE_STREAM_TO_UINT32(audio_data.timestamp, p);
            // AVDTP media header length = 12 + (csrc count << 2 )
            media_header_len = 12 + ( ( (media_info & 0x0F00) >> 8 )  << 2 );
            p_pkt = (BT_HDR*)(p_avdtp_header + media_header_len -sizeof(BT_HDR));
            p_pkt->event = 0; // not relevant
            p_pkt->layer_specific = 0; // not relevant
            p_pkt->len = len - media_header_len;
            p_pkt->offset = 0; //offset from where the media payload starts.
            audio_data.p_pkt     = p_pkt;
            audio_data.m_pt      = (media_info & 0x00FF); //Marker and Payload Type
            wiced_bt_hci_a2dp_sink_data_cb(wiced_bt_hci_a2dp_sink_cb.codec_id,&audio_data);
        }
        break;
        default:
            WICED_HCI_DEBUG_LOG(("%s EVENT not handled\n", __FUNCTION__));
            break;
    }
}

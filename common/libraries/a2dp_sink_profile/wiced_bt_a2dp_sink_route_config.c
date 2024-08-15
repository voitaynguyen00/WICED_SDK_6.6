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
 * This file contains functions for audio sink lite host adaptation.
 */

#include <string.h>
#include "wiced_gki.h"
#include "wiced_bt_avdt.h"
#include "wiced_bt_a2dp_sink_int.h"
#include "wiced_audio_sink.h"

#if (defined(WICED_BT_A2DP_SINK_USE_LITEHOST) && WICED_BT_A2DP_SINK_USE_LITEHOST == TRUE)

#define WICED_BT_MAX_CONNECTION_SUPPORTED 2

typedef struct
{
    uint16_t handle;
    uint32_t audio_route;
    wiced_bt_a2dp_codec_info_t codec_config; /* Codec configuration information */
    uint16_t cp_type; /* Content protection type */
    wiced_bool_t is_started;
    wiced_bool_t in_use_context;
    wiced_bool_t is_master;
} a2dp_sink_route_config_t;

static a2dp_sink_route_config_t sink_cfg[WICED_BT_MAX_CONNECTION_SUPPORTED] =
{
    {
        .handle = 0,
        .audio_route  = 0,
        .codec_config = {0,},
        .cp_type      = 0,
        .is_started   = WICED_FALSE,
        .in_use_context = WICED_FALSE
    },
    {
        .handle = 0,
        .audio_route  = 0,
        .codec_config = {0,},
        .cp_type      = 0,
        .is_started   = WICED_FALSE,
        .in_use_context = WICED_FALSE
    }
};

static int get_context_index( uint16_t handle )
{
    int i=0;

    while( i < WICED_BT_MAX_CONNECTION_SUPPORTED )
    {
        if( ( sink_cfg[i].in_use_context == WICED_TRUE ) && ( sink_cfg[i].handle == handle ) )
        {
            return i;
        }
        i++;
    }
    WICED_BT_TRACE( "get_context_index Invalid!!\n" );
    return WICED_BT_MAX_CONNECTION_SUPPORTED;
}

static int set_context_index( uint16_t handle )
{
    int i=0;

    while( i < WICED_BT_MAX_CONNECTION_SUPPORTED )
    {
        if( sink_cfg[i].in_use_context == WICED_TRUE )
        {
            if( sink_cfg[i].handle == handle )
            {
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
            sink_cfg[i].in_use_context = WICED_TRUE;
            return i;
        }
    }
    WICED_BT_TRACE( "set_context_index Invalid!!\n" );
    return WICED_BT_MAX_CONNECTION_SUPPORTED;
}

void wiced_bt_a2dp_sink_set_route_codec_config( uint32_t audio_route, wiced_bt_a2dp_codec_info_t *codec_config, uint16_t handle, uint16_t cp_type, wiced_bool_t is_master )
{
    int context_index;

    context_index = set_context_index( handle );
    sink_cfg[context_index].handle = handle;
    memcpy( &sink_cfg[context_index].codec_config, codec_config, sizeof( wiced_bt_a2dp_codec_info_t ) );
    sink_cfg[context_index].audio_route = audio_route;
    sink_cfg[context_index].cp_type = cp_type;
    sink_cfg[context_index].is_master = is_master;
}

void wiced_bt_a2dp_sink_set_handle( uint16_t handle )
{
    int context_index;

    context_index = get_context_index( handle );
    sink_cfg[context_index].handle = handle;
}

wiced_result_t wiced_bt_a2dp_sink_streaming_configure_route( uint16_t handle )
{
    wiced_result_t  result;
    int             context_index;

    context_index = get_context_index(handle);

    WICED_BT_TRACE("%s\n", __FUNCTION__);

    result = wiced_audio_sink_configure( sink_cfg[context_index].handle, sink_cfg[context_index].is_master,
                                    sink_cfg[context_index].audio_route,
                                    sink_cfg[context_index].cp_type,
                                    &sink_cfg[context_index].codec_config );

    sink_cfg[context_index].is_started = (result == WICED_SUCCESS);
    WICED_BT_TRACE("%s result:%d\n", __FUNCTION__, result);
    return result;
}

wiced_result_t wiced_bt_a2dp_sink_streaming_stop( uint16_t handle )
{
    int             context_index;
    context_index = get_context_index( handle );
    wiced_result_t result = WICED_NOT_FOUND;

    if ( sink_cfg[context_index].is_started )
    {
        result = wiced_audio_sink_reset( handle );
        sink_cfg[context_index].is_started = !( result == WICED_SUCCESS );
    }
    return result;
}

void wiced_bt_a2dp_sink_stream_close( uint16_t handle )
{
    int             context_index;
    context_index = get_context_index(handle);

    if ( sink_cfg[context_index].in_use_context == WICED_TRUE )
    {
        sink_cfg[context_index].in_use_context = WICED_FALSE;
    }
}

wiced_bool_t wiced_bt_a2dp_sink_update_route_config (uint16_t handle, wiced_bt_a2dp_sink_route_config *route_params)
{
    wiced_bool_t    status = WICED_TRUE;
    int             context_index;
    context_index = get_context_index(handle);

    if (sink_cfg[context_index].in_use_context == WICED_TRUE)
    {
        switch(route_params->route)
        {
            case AUDIO_ROUTE_I2S:
                if (sink_cfg[context_index].codec_config.codec_id != WICED_BT_A2DP_CODEC_SBC)
                {
                    if (wiced_bt_a2dp_sink_cb.p_config_data->ext_codec.codec_id == WICED_AUDIO_CODEC_NONE)
                    {
                        WICED_BT_TRACE("\n\n [ERROR] %s: For codec %d audio data path should not be AUDIO_ROUTE_I2S \n\n", __FUNCTION__,sink_cfg[context_index].codec_config.codec_id);
                        status = WICED_FALSE;
                    }
                }
                break;
            case AUDIO_ROUTE_UART:
            case AUDIO_ROUTE_COMPRESSED_TRANSPORT:
                break;
            default:
                status = WICED_FALSE;
                break;
        }
    }
    else
    {
        status = WICED_FALSE;
    }

    if (status)
    {
        sink_cfg[context_index].audio_route = route_params->route;
        sink_cfg[context_index].is_master = route_params->is_master;
    }
    return status;
}

/*
 * wiced_bt_a2dp_sink_route_config_lrac_switch_get
 */
wiced_result_t wiced_bt_a2dp_sink_route_config_lrac_switch_get(void *p_opaque,
        uint16_t *p_sync_data_len)
{
    wiced_result_t status;
    uint16_t sync_data_len;

    if (p_opaque == NULL)
    {
        WICED_BT_TRACE("Err %s: p_opaque is NULL\n", __FUNCTION__);
        return WICED_BT_BADARG;
    }

    if (p_sync_data_len == NULL)
    {
        WICED_BT_TRACE("Err %s: p_sync_data_len is NULL\n", __FUNCTION__);
        return WICED_BT_BADARG;
    }

    if (*p_sync_data_len < sizeof(sink_cfg))
    {
        WICED_BT_TRACE("Err %s: buffer too small (%d/%d)\n", __FUNCTION__,
                *p_sync_data_len, sizeof(sink_cfg));
        return WICED_BT_BADARG;
    }

    /* Get the A2DP Sink Route Sync data */
    memcpy(p_opaque, &sink_cfg, sizeof(sink_cfg));

    *p_sync_data_len = sizeof(sink_cfg);

    return WICED_BT_SUCCESS;
}

/*
 * wiced_bt_a2dp_sink_route_config_lrac_switch_set
 */
wiced_result_t wiced_bt_a2dp_sink_route_config_lrac_switch_set(void *p_opaque,
        uint16_t sync_data_len)
{
    wiced_result_t status;

    if (p_opaque == NULL)
    {
        WICED_BT_TRACE("Err %s: p_opaque is NULL\n", __FUNCTION__);
        return WICED_BT_BADARG;
    }

    if (sync_data_len < sizeof(sink_cfg))
    {
        WICED_BT_TRACE("Err %s: bad buffer size (%d/%d)\n", __FUNCTION__,
                sync_data_len, sizeof(sink_cfg));
        return WICED_BT_BADARG;
    }
    /* Set the A2DP Sink's Sync data */
    memcpy(&sink_cfg, p_opaque, sizeof(sink_cfg));

    return WICED_BT_SUCCESS;
}

#endif

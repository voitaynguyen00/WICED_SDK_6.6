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
 * AV Sink Sample Application for 2070X devices
 *
 */
#include "wiced_bt_a2d.h"
#include "wiced_bt_a2d_sbc.h"
#include "wiced_bt_a2dp_sink.h"
#include "wiced_bt_trace.h"
#include "a2dp_sink.h"
#include "string.h"

/******************************************************************************
 *                         Type Definitions
 ******************************************************************************/

typedef struct
{
    wiced_bt_device_address_t peerBda;             /* Peer bd address */
    AV_STATE                  state;               /* AVDT State machine state */
    AV_STREAM_STATE           audio_stream_state;  /* Audio Streaming to host state */
    wiced_bt_a2dp_codec_info_t codec_config;       /* Codec configuration information */
} tAV_APP_CB;

/******************************************************************************
 *                         Variable Definitions
 ******************************************************************************/

/* A2DP module control block */
static tAV_APP_CB av_app_cb;
extern wiced_bt_a2dp_config_data_t bt_audio_config;
static uint8_t mute = 1;

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/
static const char *dump_control_event_name(wiced_bt_a2dp_sink_event_t event)
{
    switch((int)event)
    {
        CASE_RETURN_STR(WICED_BT_A2DP_SINK_CONNECT_EVT)
        CASE_RETURN_STR(WICED_BT_A2DP_SINK_DISCONNECT_EVT)
        CASE_RETURN_STR(WICED_BT_A2DP_SINK_START_IND_EVT)
        CASE_RETURN_STR(WICED_BT_A2DP_SINK_START_CFM_EVT)
        CASE_RETURN_STR(WICED_BT_A2DP_SINK_SUSPEND_EVT)
        CASE_RETURN_STR(WICED_BT_A2DP_SINK_CODEC_CONFIG_EVT)
    }

    return NULL;
}

static const char *dump_state_name(AV_STATE state)
{
    switch((int)state)
    {
        CASE_RETURN_STR(AV_STATE_IDLE)
        CASE_RETURN_STR(AV_STATE_CONFIGURED)
        CASE_RETURN_STR(AV_STATE_CONNECTED)
        CASE_RETURN_STR(AV_STATE_STARTED)
    }

    return NULL;
}

void a2dp_sink_mute_unmute_audio( void )
{
    /* If streaming is started */
    if ( av_app_cb.state == AV_STATE_STARTED )
    {
        if ( mute )
        {
            WICED_BT_TRACE( "Audio Mute \n" );
            wiced_bt_a2dp_sink_mute_audio( 1, 500 );
            mute = 0;
        }
        else
        {
            WICED_BT_TRACE( "Audio Unmute \n" );
            wiced_bt_a2dp_sink_mute_audio( 0, 500 );
            mute = 1;
        }
    }
}

/******************************************************************************
 *                 Data Callback
 ******************************************************************************/

/* ****************************************************************************
 * Function: a2dp_sink_data_cback
 *
 * Parameters:
 *          p_a2dp_data   - A2DP media data
 *          a2dp_data_len - A2DP data length
 *
 * Description:
 *          Data supplied by  the a2dp sink profile code.
 * ***************************************************************************/
void a2dp_sink_data_cback( uint8_t* p_a2dp_data, uint32_t a2dp_data_len )
{
    WICED_BT_TRACE( "A2DP data %x, %d\n",p_a2dp_data, a2dp_data_len );
}

/******************************************************************************
 *                 Control Profile Callback
 ******************************************************************************/

/* ****************************************************************************
 * Function: a2dp_sink_control_cback
 *
 * Parameters:
 *          event - control event called back
 *          p_data - event data
 *
 * Description:
 *          Control callback supplied by  the a2dp sink profile code.
 * ***************************************************************************/
static void a2dp_sink_control_cback( wiced_bt_a2dp_sink_event_t event,
                                     wiced_bt_a2dp_sink_event_data_t* p_data )
{
    wiced_bt_a2dp_sink_route_config route_config;
    WICED_BT_TRACE( "[%s] Event: (%d) %s state: (%d) %s\n\r", __FUNCTION__, event,
                    dump_control_event_name(event),
                    av_app_cb.state, dump_state_name(av_app_cb.state));

    switch(event)
    {
        case WICED_BT_A2DP_SINK_CODEC_CONFIG_EVT: /**< Codec config event, received when codec config for a streaming session is updated */

            /* Maintain State */
            av_app_cb.state = AV_STATE_CONFIGURED;

            /* Save the configuration to setup the decoder if necessary. */
            memcpy( &av_app_cb.codec_config, &p_data->codec_config.codec, sizeof( wiced_bt_a2dp_codec_info_t ) );

            // Please update route configure setting based on requirements
            {
                route_config.route = AUDIO_ROUTE_I2S;
                route_config.is_master = WICED_TRUE;
            }
            wiced_bt_a2dp_sink_update_route_config( p_data->codec_config.handle, &route_config);

            WICED_BT_TRACE(" a2dp sink codec configuration done\n");
            break;

        case WICED_BT_A2DP_SINK_CONNECT_EVT:      /**< Connected event, received on establishing connection to a peer device. Ready to stream. */

            if ( p_data->connect.result == WICED_SUCCESS)
            {
                uint16_t settings = HCI_ENABLE_MASTER_SLAVE_SWITCH;// HCI_DISABLE_ALL_LM_MODES;

                WICED_BT_TRACE( "[%s] connected to addr: <%B> Handle:%d\n\r", __FUNCTION__, p_data->connect.bd_addr, p_data->connect.handle );

                /* Save the address of the remote device on remote connection */
                memcpy(av_app_cb.peerBda, p_data->connect.bd_addr, sizeof(wiced_bt_device_address_t));

                /* Maintain State */
                av_app_cb.state = AV_STATE_CONNECTED;

                WICED_BT_TRACE(" a2dp sink connected \n");
            }
            else
            {
                WICED_BT_TRACE(" a2dp sink connection failed %d \n", p_data->connect.result );
            }
            break;

        case WICED_BT_A2DP_SINK_DISCONNECT_EVT:   /**< Disconnected event, received on disconnection from a peer device */

            /* Maintain State */
            av_app_cb.state = AV_STATE_IDLE;
            WICED_BT_TRACE(" a2dp sink disconnected \n");
            break;

        case WICED_BT_A2DP_SINK_START_IND_EVT:        /**< Start stream indication, Send response */
            WICED_BT_TRACE("  a2dp sink start indication from Peer @handle: %d, %x \n", p_data->start_ind.handle, p_data->start_ind.label );
            if ( !wiced_bt_a2dp_sink_send_start_response( p_data->start_ind.handle, p_data->start_ind.label, A2D_SUCCESS ) )
            {
                mute = 1; // reset the mute state
                /* Maintain State */
                av_app_cb.state = AV_STATE_STARTED;
                WICED_BT_TRACE(" a2dp sink streaming started \n");
            }
            break;

        case WICED_BT_A2DP_SINK_START_CFM_EVT:        /**< Start stream event, received when audio streaming is about to start */
            /* Maintain State */
            av_app_cb.state = AV_STATE_STARTED;
            WICED_BT_TRACE(" a2dp sink streaming started handle:%d\n", p_data->start_cfm.handle);
            break;

        case WICED_BT_A2DP_SINK_SUSPEND_EVT:      /**< Suspend stream event, received when audio streaming is suspended */
            /* Maintain State */
            av_app_cb.state = AV_STATE_CONNECTED;
            WICED_BT_TRACE(" a2dp sink streaming suspended \n");
            break;

        default:
            break;
    }
}

/******************************************************************************
 *                     Application Initialization
 ******************************************************************************/

wiced_result_t av_app_start (void)
{
    wiced_result_t result;

    WICED_BT_TRACE( "[%s] Application start\n\r", __FUNCTION__ );

    /* Register with the A2DP sink profile code */
    result = wiced_bt_a2dp_sink_init( &bt_audio_config,
                                      a2dp_sink_control_cback );
    return result;
}

/*
 * AV application initialization
 */
void av_app_init( void )
{
    av_app_cb.state = AV_STATE_IDLE;
    av_app_start( ); /* start the application */
    WICED_BT_TRACE ("[%s] exit\n", __FUNCTION__ );
}

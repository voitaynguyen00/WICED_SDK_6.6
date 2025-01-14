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
 * This is the implementation file for audio sink call-out functions.
 */

#include "wiced_bt_a2dp_sink_int.h"
#include "wiced_bt_audio.h"
#include "string.h"

/****************************************************************************\
* Audio definitions
\****************************************************************************/
/* Optional audio codecs are unsupported by default */
#ifdef A2DP_SINK_AAC_ENABLED
#define WICED_BT_A2DP_SINK_CO_M12_SUPPORT               TRUE
#else
#define WICED_BT_A2DP_SINK_CO_M12_SUPPORT               FALSE
#endif

#ifdef A2DP_SINK_AAC_ENABLED
#define WICED_BT_A2DP_SINK_CO_M24_SUPPORT               TRUE
#else
#define WICED_BT_A2DP_SINK_CO_M24_SUPPORT               FALSE
#endif

#ifndef WICED_BT_A2DP_SINK_CO_VENDOR_SPECIFIC_SUPPORT
#define WICED_BT_A2DP_SINK_CO_VENDOR_SPECIFIC_SUPPORT   FALSE
#endif

#if (WICED_BT_A2DP_SINK_CO_M12_SUPPORT == TRUE)
#include "wiced_bt_a2d_m12.h"
#endif

#if (WICED_BT_A2DP_SINK_CO_M24_SUPPORT == TRUE)
#include "wiced_bt_a2d_m24.h"
#endif

/*******************************************************************************
**
** Function         wiced_bt_a2dp_sink_cfg_setconfig_rsp
**
** Description      This function must be called in response to function
**                  wiced_bt_a2dp_sink_setcfg_req_handler().
**                  Parameter err_code is set to an AVDTP status value;
**                  AVDT_SUCCESS if the codec configuration is ok,
**                  otherwise error.
**
** Returns          void
**
*******************************************************************************/
static void wiced_bt_a2dp_sink_cfg_setconfig_rsp(uint8_t err_code, uint8_t category,
        wiced_bt_device_address_t bd_addr)
{
    wiced_bt_a2dp_sink_setconfig_rsp_t buf;
    memset(&buf, 0, sizeof(buf));

    WICED_BTA2DP_TRACE("%s \n", __FUNCTION__);

    buf.hdr.event = (err_code == AVDT_SUCCESS) ?
        WICED_BT_A2DP_SINK_STR_CONFIG_RSP_OK_EVT : WICED_BT_A2DP_SINK_STR_CONFIG_RSP_FAIL_EVT;
    buf.err_code = err_code;
    buf.category = category;
    memcpy(buf.peer_addr, bd_addr, sizeof(wiced_bt_device_address_t));

    wiced_bt_a2dp_sink_hdl_event((BT_HDR*)&buf);
}

/*******************************************************************************
**
** Function         wiced_bt_a2dp_sink_cfg_init
**
** Description      This function is used to initialize and build
**                  codec configuration information from codec capability structure.
**
** Returns          Stream codec and content protection capabilities info.
**
*******************************************************************************/
wiced_bool_t wiced_bt_a2dp_sink_cfg_init(wiced_bt_a2dp_codec_info_t *p_codec_info,
        uint8_t *p_built_codec_info,
        uint8_t *p_num_protect, uint8_t *p_protect_info)
{
    uint8_t a2d_status = A2D_FAIL;

    WICED_BTA2DP_TRACE("%s: codec_id:%d \n", __FUNCTION__, p_codec_info->codec_id);

    //SCMS-T is supported as of now
    if ( *p_num_protect )
    {
        *p_protect_info++ = 02; //LOSC - Length of Service Capability
        *p_protect_info++ = AVDT_CP_TYPE_SCMST & 0xFF; //SCMS-T
        *p_protect_info   = ( AVDT_CP_TYPE_SCMST >> 8 ) & 0xFF;
    }

    switch(p_codec_info->codec_id)
    {
    case WICED_BT_A2DP_CODEC_SBC: /* SBC */
        /* Setup for SBC codec */
        a2d_status = wiced_bt_a2d_bld_sbc_info( AVDT_MEDIA_AUDIO,
                                    (wiced_bt_a2d_sbc_cie_t *) &p_codec_info->cie.sbc,
                                    p_built_codec_info);

        WICED_BTA2DP_TRACE("%s: status: %d \n", __FUNCTION__, a2d_status);
        WICED_BTA2DP_TRACE("sbc [0x%02x;0x%02x;0x%02x;0x%02x;0x%02x;0x%02x] \n",
            p_built_codec_info[1], p_built_codec_info[2], p_built_codec_info[3],
            p_built_codec_info[4], p_built_codec_info[5], p_built_codec_info[6]);

        return (A2D_SUCCESS == a2d_status) ? WICED_TRUE : WICED_FALSE;

#if (WICED_BT_A2DP_SINK_CO_M12_SUPPORT == TRUE)
    case WICED_BT_A2DP_CODEC_M12: /* MP3 */
        /* Set up for MP3 codec */
        wiced_bt_a2d_bld_m12info(AVDT_MEDIA_AUDIO,
            (wiced_bt_a2d_m12_cie_t *) &p_codec_info->cie.m12,
            p_built_codec_info);
        WICED_BTA2DP_TRACE("m12 [0x%02x;0x%02x;0x%02x;0x%02x;0x%02x;0x%02x] \n", \
            p_built_codec_info[1], p_built_codec_info[2], p_built_codec_info[3],
            p_built_codec_info[4], p_built_codec_info[5], p_built_codec_info[6]);
        return WICED_TRUE;
#endif

#if (WICED_BT_A2DP_SINK_CO_M24_SUPPORT == TRUE)
    case WICED_BT_A2DP_CODEC_M24: /* AAC */
        /* Set up for AAC codec */
        wiced_bt_a2d_bld_m24info(AVDT_MEDIA_AUDIO,
            (wiced_bt_a2d_m24_cie_t *) &p_codec_info->cie.m24,
            p_built_codec_info);
        WICED_BTA2DP_TRACE("m24 [0x%02x;0x%02x;0x%02x;0x%02x;0x%02x;0x%02x] \n", \
            p_built_codec_info[1], p_built_codec_info[2], p_built_codec_info[3],
            p_built_codec_info[4], p_built_codec_info[5], p_built_codec_info[6]);
        return WICED_TRUE;
#endif

#if (WICED_BT_A2DP_SINK_CO_VENDOR_SPECIFIC_SUPPORT == TRUE)
    case WICED_BT_A2DP_CODEC_VENDOR_SPECIFIC: /* Vendor specific */
        /* Set up for vendor specific codec */
        /* The restriction of AVDT_CODEC_SIZE is because of AVDTP structure wiced_bt_avdt_cfg_t */
        memcpy(p_built_codec_info, p_codec_info->cie.vsp.cie, AVDT_CODEC_SIZE);
        WICED_BTA2DP_TRACE("m24 [0x%02x;0x%02x;0x%02x;0x%02x;0x%02x;0x%02x] \n", \
            p_built_codec_info[1], p_built_codec_info[2], p_built_codec_info[3],
            p_built_codec_info[4], p_built_codec_info[5], p_built_codec_info[6]);
        return WICED_TRUE;
#endif

    default:
        break;
    }

    return WICED_FALSE;
}

/*******************************************************************************
**
** Function         wiced_bt_a2dp_sink_cfg_setcfg_ind_handler
**
** Description      This handler function is executed when the peer a2dp src does a setcfg.
**                  It checks whether the configuration set by peer src is supported in
**                  codec capabilities or not and responds accordingly.
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_a2dp_sink_cfg_setcfg_ind_handler(
    wiced_bt_a2dp_codec_info_list_t *p_codec_capabilities,
     wiced_bt_a2dp_sink_str_msg_t *str_msg)
{
    wiced_bt_a2dp_codec_t codec_type = str_msg->cfg.codec_info[AVDT_CODEC_TYPE_INDEX];
    uint8_t *p_codec_info = str_msg->cfg.codec_info;
    uint8_t num_protect = str_msg->cfg.num_protect;
    uint8_t status   = A2D_FAIL;
    uint8_t category = AVDT_ASC_CODEC;
    uint16_t cp_type = 0;
    uint8_t codec_idx = 0;
    wiced_bt_a2dp_codec_info_t* p_codec_capability_info = wiced_bt_a2dp_sink_cb.p_config_data->codec_capabilities.info;
    wiced_bt_a2dp_codec_info_t codec_config;
    wiced_bt_a2dp_sink_codec_config_t api_params;

    WICED_BTA2DP_TRACE("%s \n", __FUNCTION__);
    codec_config.codec_id = codec_type;

    for ( codec_idx = 0; codec_idx < wiced_bt_a2dp_sink_cb.p_config_data->codec_capabilities.count; codec_idx++ )
    {
        if ( codec_type == p_codec_capability_info->codec_id )
        {
            /* Set decoder configuration */
            switch(codec_type)
            {
            case WICED_BT_A2DP_CODEC_SBC:
                if ((status = wiced_bt_a2dp_sbc_cfg_in_cap(p_codec_info,
                    (wiced_bt_a2d_sbc_cie_t *) &(p_codec_capability_info->cie.sbc))) == A2D_SUCCESS)
                {
                    //SCMS-T is supported as of now
                    if ( num_protect)
                    {
                        cp_type = str_msg->cfg.protect_info[1] | ( str_msg->cfg.protect_info[2] << 8 );
                        if ( ( str_msg->cfg.protect_info[0] != 2 ) || ( cp_type != AVDT_CP_TYPE_SCMST ) )
                        {
                            status = A2D_BAD_CP_TYPE;
                            category = AVDT_ASC_PROTECT;
                            break;
                        }
                    }
                    status =
                          wiced_bt_a2d_pars_sbc_info(&codec_config.cie.sbc,
                                          p_codec_info,
                                          WICED_FALSE);
                }
                break;

#if (WICED_BT_A2DP_SINK_CO_M12_SUPPORT == TRUE)
            case WICED_BT_A2DP_CODEC_M12:
                /* Verify MP3 configuration */
                if ((status = wiced_bt_a2dp_m12_cfg_in_cap(p_codec_info,
                    (wiced_bt_a2d_m12_cie_t *) &(p_codec_capability_info->cie.m12))) == A2D_SUCCESS)
                {
                    /* Do not allow content protection for now */
                    if (num_protect != 0)
                    {
                        status = A2D_BAD_CP_TYPE;
                        category = AVDT_ASC_PROTECT;
                    } else {
                    wiced_bt_a2d_pars_m12info(&codec_config.cie.m12,
                            p_codec_info, WICED_FALSE);
                    }
                }
                break;
#endif

#if (WICED_BT_A2DP_SINK_CO_M24_SUPPORT == TRUE)
            case WICED_BT_A2DP_CODEC_M24:
                /* Verify AAC configuration */
                if ((status = wiced_bt_a2dp_m24_cfg_in_cap(p_codec_info,
                    (wiced_bt_a2d_m24_cie_t *) &(p_codec_capability_info->cie.m24))) == A2D_SUCCESS)
                {
                    /* Do not allow content protection for now */
                    if (num_protect != 0)
                    {
                        status = A2D_BAD_CP_TYPE;
                        category = AVDT_ASC_PROTECT;
                    } else {
                    wiced_bt_a2d_pars_m24info(&codec_config.cie.m24,
                            p_codec_info, WICED_FALSE);
                    }
                }

                break;
#endif

                case WICED_BT_A2DP_CODEC_VENDOR_SPECIFIC:
#if (WICED_BT_A2DP_SINK_CO_VENDOR_SPECIFIC_SUPPORT == TRUE)
                    if (!memcmp(p_codec_info, p_codec_capability_info->cie.vsp.cie,
                        AVDT_CODEC_SIZE))
                    {
                        status = A2D_SUCCESS;
                        codec_config.cie.vsp.cie_length = AVDT_CODEC_SIZE;
                        codec_config.cie.vsp.cie = p_codec_info;
                    }
#endif
                    break;

                default:
                    break;
            }
            if ( status == A2D_SUCCESS )
                break;
       }
       p_codec_capability_info++;
    }
    WICED_BTA2DP_TRACE("%s status:%d \n", __FUNCTION__, status);
    wiced_bt_a2dp_sink_cfg_setconfig_rsp(status, category, str_msg->bd_addr);

    if (status == A2D_SUCCESS)
    {
#if (defined(WICED_BT_A2DP_SINK_USE_LITEHOST) && WICED_BT_A2DP_SINK_USE_LITEHOST == TRUE)
        wiced_bt_a2dp_sink_set_route_codec_config( AUDIO_ROUTE_I2S, &codec_config, str_msg->handle, cp_type, WICED_TRUE );
#endif
        memcpy(&api_params.codec,&codec_config,sizeof(wiced_bt_a2dp_codec_info_t));
        api_params.handle = str_msg->handle;

        (*wiced_bt_a2dp_sink_cb.control_cb)(WICED_BT_A2DP_SINK_CODEC_CONFIG_EVT,
            (wiced_bt_a2dp_sink_event_data_t*) &api_params);
    }
}



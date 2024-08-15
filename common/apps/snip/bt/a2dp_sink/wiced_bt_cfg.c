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
 * Runtime Bluetooth stack configuration parameters
 *
 */
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_avrc.h"
#include "wiced_bt_avrc_defs.h"
#include "wiced_bt_a2dp_sink.h"
#include "wiced_bt_a2d_sbc.h"
#include "a2dp_sink.h"
#include "wiced_bt_audio.h"

#if ( ( defined(CYW20719B1) || defined(CYW20721B1) ) && (WICED_A2DP_EXT_CODEC == WICED_TRUE) )
extern wiced_codec_interface_functions_t AAC_codec_function_table;
#endif

/*****************************************************************************
 * wiced_bt core stack configuration
 ****************************************************************************/
const wiced_bt_cfg_settings_t a2dp_sink_cfg_settings =
{
    .device_name                         = ( uint8_t* )"wiced a2dp sink",                                /**< Local device name ( NULL terminated ) */
    .device_class                        = {0x24, 0x04, 0x18},                                           /**< Local device class */
    .security_requirement_mask           = ( BTM_SEC_IN_AUTHENTICATE | BTM_SEC_OUT_AUTHENTICATE | BTM_SEC_ENCRYPT ), /**< Security requirements mask ( BTM_SEC_NONE, or combination of BTM_SEC_IN_AUTHENTICATE, BTM_SEC_OUT_AUTHENTICATE, BTM_SEC_ENCRYPT ( see #wiced_bt_sec_level_e ) ) */

    .max_simultaneous_links              = 3,                                                            /**< Maximum number simultaneous links to different devices */

    .br_edr_scan_cfg =                                                                                   /* BR/EDR scan config */
    {
        .inquiry_scan_type               = BTM_SCAN_TYPE_STANDARD,                                     /**< Inquiry scan type ( BTM_SCAN_TYPE_STANDARD or BTM_SCAN_TYPE_INTERLACED ) */
        .inquiry_scan_interval           = WICED_BT_CFG_DEFAULT_INQUIRY_SCAN_INTERVAL,                 /**< Inquiry scan interval  ( 0 to use default ) */
        .inquiry_scan_window             = WICED_BT_CFG_DEFAULT_INQUIRY_SCAN_WINDOW,                   /**< Inquiry scan window ( 0 to use default ) */

        .page_scan_type                  = BTM_SCAN_TYPE_STANDARD,                                     /**< Page scan type ( BTM_SCAN_TYPE_STANDARD or BTM_SCAN_TYPE_INTERLACED ) */
        .page_scan_interval              = WICED_BT_CFG_DEFAULT_PAGE_SCAN_INTERVAL,                    /**< Page scan interval  ( 0 to use default ) */
        .page_scan_window                = WICED_BT_CFG_DEFAULT_PAGE_SCAN_WINDOW                       /**< Page scan window ( 0 to use default ) */
    },

    .ble_scan_cfg =                                                                                    /* BLE scan settings  */
    {
        .scan_mode                       = BTM_BLE_SCAN_MODE_ACTIVE,                                   /**< BLE scan mode ( BTM_BLE_SCAN_MODE_PASSIVE, BTM_BLE_SCAN_MODE_ACTIVE, or BTM_BLE_SCAN_MODE_NONE ) */

        /* Advertisement scan configuration */
        .high_duty_scan_interval         = 96,                                                         /**< High duty scan interval */
        .high_duty_scan_window           = 48,                                                         /**< High duty scan window */
        .high_duty_scan_duration         = 30,                                                         /**< High duty scan duration in seconds ( 0 for infinite ) */

        .low_duty_scan_interval          = 2048,                                                       /**< Low duty scan interval  */
        .low_duty_scan_window            = 48,                                                         /**< Low duty scan window */
        .low_duty_scan_duration          = 30,                                                         /**< Low duty scan duration in seconds ( 0 for infinite ) */

        /* Connection scan configuration */
        .high_duty_conn_scan_interval    = 96,                                                         /**< High duty cycle connection scan interval */
        .high_duty_conn_scan_window      = 48,                                                         /**< High duty cycle connection scan window */
        .high_duty_conn_duration         = 30,                                                         /**< High duty cycle connection duration in seconds ( 0 for infinite ) */

        .low_duty_conn_scan_interval     = 2048,                                                       /**< Low duty cycle connection scan interval */
        .low_duty_conn_scan_window       = 48,                                                         /**< Low duty cycle connection scan window */
        .low_duty_conn_duration          = 30,                                                         /**< Low duty cycle connection duration in seconds ( 0 for infinite ) */

        /* Connection configuration */
        .conn_min_interval               = WICED_BT_CFG_DEFAULT_CONN_MIN_INTERVAL,                     /**< Minimum connection interval */
        .conn_max_interval               = WICED_BT_CFG_DEFAULT_CONN_MAX_INTERVAL,                     /**< Maximum connection interval */
        .conn_latency                    = WICED_BT_CFG_DEFAULT_CONN_LATENCY,                          /**< Connection latency */
        .conn_supervision_timeout        = WICED_BT_CFG_DEFAULT_CONN_SUPERVISION_TIMEOUT,              /**< Connection link supervision timeout */
    },

    .ble_advert_cfg =                                                                                  /* BLE advertisement settings */
    {
        .channel_map                     = BTM_BLE_ADVERT_CHNL_37 |                                    /**< Advertising channel map ( mask of BTM_BLE_ADVERT_CHNL_37, BTM_BLE_ADVERT_CHNL_38, BTM_BLE_ADVERT_CHNL_39 ) */
                                           BTM_BLE_ADVERT_CHNL_38 |
                                           BTM_BLE_ADVERT_CHNL_39,

        .high_duty_min_interval          = WICED_BT_CFG_DEFAULT_HIGH_DUTY_ADV_MIN_INTERVAL,            /**< High duty undirected connectable minimum advertising interval */
        .high_duty_max_interval          = WICED_BT_CFG_DEFAULT_HIGH_DUTY_ADV_MAX_INTERVAL,            /**< High duty undirected connectable maximum advertising interval */
        .high_duty_duration              = 30,                                                         /**< High duty undirected connectable advertising duration in seconds ( 0 for infinite ) */

        .low_duty_min_interval           = WICED_BT_CFG_DEFAULT_LOW_DUTY_ADV_MIN_INTERVAL,             /**< Low duty undirected connectable minimum advertising interval */
        .low_duty_max_interval           = WICED_BT_CFG_DEFAULT_LOW_DUTY_ADV_MAX_INTERVAL,             /**< Low duty undirected connectable maximum advertising interval */
        .low_duty_duration               = 60,                                                         /**< Low duty undirected connectable advertising duration in seconds ( 0 for infinite ) */

        .high_duty_directed_min_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MIN_INTERVAL,   /**< High duty directed connectable minimum advertising interval */
        .high_duty_directed_max_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MAX_INTERVAL,   /**< High duty directed connectable maximum advertising interval */

        .low_duty_directed_min_interval  = WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MIN_INTERVAL,    /**< Low duty directed connectable minimum advertising interval */
        .low_duty_directed_max_interval  = WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MAX_INTERVAL,    /**< Low duty directed connectable maximum advertising interval */
        .low_duty_directed_duration      = 30,                                                         /**< Low duty directed connectable advertising duration in seconds ( 0 for infinite ) */

        .high_duty_nonconn_min_interval  = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MIN_INTERVAL,    /**< High duty non-connectable minimum advertising interval */
        .high_duty_nonconn_max_interval  = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MAX_INTERVAL,    /**< High duty non-connectable maximum advertising interval */
        .high_duty_nonconn_duration      = 30,                                                         /**< High duty non-connectable advertising duration in seconds ( 0 for infinite ) */

        .low_duty_nonconn_min_interval   = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MIN_INTERVAL,     /**< Low duty non-connectable minimum advertising interval */
        .low_duty_nonconn_max_interval   = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MAX_INTERVAL,     /**< Low duty non-connectable maximum advertising interval */
        .low_duty_nonconn_duration       = 0                                                           /**< Low duty non-connectable advertising duration in seconds ( 0 for infinite ) */
    },

    .gatt_cfg =                                                                                        /* GATT configuration */
    {
        .appearance                     = APPEARANCE_GENERIC_TAG,                                      /**< GATT appearance ( see gatt_appearance_e ) */
        .client_max_links               = 3,                                                           /**< Client config: maximum number of servers that local client can connect to  */
        .server_max_links               = 3,                                                           /**< Server config: maximum number of remote clients connections allowed by the local */
        .max_attr_len                   = 360,                                                         /**< Maximum attribute length; gki_cfg must have a corresponding buffer pool that can hold this length */
#ifndef CYW20706A2
        .max_mtu_size                   = 365                                                          /**< Maximum MTU size for GATT connections, should be between 23 and (max_attr_len + 5) */
#endif
    },

    .rfcomm_cfg =                                                                                      /* RFCOMM configuration */
    {
        .max_links                      = 0,                                                           /**< Maximum number of simultaneous RFCOMM ports */
        .max_links                      = 0                                                            /**< Maximum number of simultaneous RFCOMM connections */
    },

    .l2cap_application =                                                                             /* Application managed l2cap protocol configuration */
    {
        .max_links                      = 0,                                                           /**< Maximum number of application-managed l2cap links (BR/EDR and LE) */

        /* BR EDR l2cap configuration */
        .max_psm                        = 0,                                                           /**< Maximum number of application-managed BR/EDR PSMs */
        .max_channels                   = 0,                                                           /**< Maximum number of application-managed BR/EDR channels  */

        /* LE L2cap connection-oriented channels configuration */
        .max_le_psm                     = 0,                                                           /**< Maximum number of application-managed LE PSMs */
        .max_le_channels                = 0,                                                           /**< Maximum number of application-managed LE channels */
#ifndef CYW20706A2
        .max_le_l2cap_fixed_channels    = 0,	                                                       /**< Maximum number of application managed fixed channels supported (in addition to mandatory channels 4, 5 and 6). > */
#endif
    },

    .avdt_cfg =                                                                                        /* Audio/Video Distribution configuration */
    {
        .max_links                      = 1,                                                           /**< Maximum simultaneous audio/video links */
#ifndef CYW20706A2
        .max_seps                       = 3,                                                           /**< Maximum number of stream end points */
#endif
    },

    .avrc_cfg =                                                                                       /* Audio/Video Remote Control configuration */
    {
        .roles                          = 1,                                                           /**< 1 if AVRC_CONN_ACCEPTOR is supported */
        .max_links                      = 1,                                                           /**< Maximum simultaneous remote control links */
    },
    .addr_resolution_db_size            = 5,                                                           /**< LE Address Resolution DB settings - effective only for pre 4.2 controller*/
#ifdef CYW20706A2
    .max_mtu_size                       = 365,                                                         /**< Maximum MTU size for GATT connections, should be between 23 and (max_attr_len + 5) */
    .max_pwr_db_val                     = 12                                                           /**< Max. power level of the device */
#else
    .max_number_of_buffer_pools         = 6,                                                           /**< Maximum number of buffer pools in p_btm_cfg_buf_pools and by wiced_create_pool */
    .rpa_refresh_timeout                = WICED_BT_CFG_DEFAULT_RANDOM_ADDRESS_NEVER_CHANGE,            /**< Interval of  random address refreshing - secs */
#endif

};


/*****************************************************************************
 * SDP database for the hci_control application
 ****************************************************************************/
// SDP Record handle for AVDT Sink
#define HANDLE_AVDT_SINK                        0x10001
// SDP Record handle for AVRC TARGET
#define HANDLE_AVRC_TARGET                      0x10002
// SDP Record handle for AVRC TARGET
#define HANDLE_AVRC_CONTROLLER                  0x10003
// SDP Record handle for SPP
#define HANDLE_SPP                              0x10004

#ifdef USE_AVRCP_CTRLR
const uint8_t a2dp_sink_sdp_db[A2DP_SINK_SDP_DB_SIZE] =
{
    SDP_ATTR_SEQUENCE_1( 198 ),                                   // length is the sum of all records
#else
const uint8_t a2dp_sink_sdp_db[A2DP_SINK_SDP_DB_SIZE] =
{
    SDP_ATTR_SEQUENCE_1( 79 ),
#endif

    // SDP Record for A2DP Sink
    SDP_ATTR_SEQUENCE_1(77),
        SDP_ATTR_RECORD_HANDLE(HANDLE_AVDT_SINK),
        SDP_ATTR_CLASS_ID(UUID_SERVCLASS_AUDIO_SINK),
        SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST), SDP_ATTR_SEQUENCE_1(16),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),
                SDP_ATTR_VALUE_UINT2(BT_PSM_AVDTP),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_AVDTP),
                SDP_ATTR_VALUE_UINT2(0x103),
        SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_SERVCLASS_ADV_AUDIO_DISTRIBUTION),
                SDP_ATTR_VALUE_UINT2(0x103),
        SDP_ATTR_SERVICE_NAME(16),
                'W', 'I', 'C', 'E', 'D', ' ', 'A', 'u', 'd', 'i', 'o', ' ', 'S', 'i', 'n', 'k',
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, 0x000B),


#ifdef USE_AVRCP_CTRLR
    // SDP Record for AVRC Target
    SDP_ATTR_SEQUENCE_1(56),
        SDP_ATTR_RECORD_HANDLE(HANDLE_AVRC_TARGET),
        SDP_ATTR_ID(ATTR_ID_SERVICE_CLASS_ID_LIST),
            SDP_ATTR_SEQUENCE_1(3),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REM_CTRL_TARGET),
        SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST), SDP_ATTR_SEQUENCE_1(16),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),
                SDP_ATTR_VALUE_UINT2(BT_PSM_AVCTP),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_AVCTP),
                SDP_ATTR_VALUE_UINT2(0x100),
        SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REMOTE_CONTROL),
                SDP_ATTR_VALUE_UINT2(AVRC_REV_1_0),
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, AVRC_SUPF_TG_CAT1),

    // SDP Record for AVRC Controller
    SDP_ATTR_SEQUENCE_1(59),
        SDP_ATTR_RECORD_HANDLE(HANDLE_AVRC_CONTROLLER),
        SDP_ATTR_ID(ATTR_ID_SERVICE_CLASS_ID_LIST),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REMOTE_CONTROL),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REM_CTRL_CONTROL),
        SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST), SDP_ATTR_SEQUENCE_1(16),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),
                SDP_ATTR_VALUE_UINT2(BT_PSM_AVCTP),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_AVCTP),
                SDP_ATTR_VALUE_UINT2(0x104),
        SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REMOTE_CONTROL),
                SDP_ATTR_VALUE_UINT2(AVRC_REV_1_5),
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, AVRC_SUPF_CT_CAT2),
#endif
};


/*****************************************************************************
 * wiced_bt  buffer pool configuration
 *
 * Configure buffer pools used by the stack  according to application's requirement
 *
 * Pools must be ordered in increasing buf_size.
 * If a pool runs out of buffers, the next  pool will be used
 *****************************************************************************/
const wiced_bt_cfg_buf_pool_t a2dp_sink_cfg_buf_pools[] =
{
/*  { buf_size, buf_count } */
    { 64,       12  },      /* Small Buffer Pool */
    { 272,      6   },      /* Medium Buffer Pool (used for HCI & RFCOMM control messages, min recommended size is 360) */
    { 1056,     6   },      /* Large Buffer Pool  (used for HCI ACL messages) */
    { 1056,     4   },      /* Extra Large Buffer Pool - Used for avdt media packets and miscellaneous (if not needed, set buf_count to 0) */
};

/*****************************************************************************
 *   codec and audio tuning configurations
 ****************************************************************************/
/*  Recommended max_bitpool for high quality audio */
#define BT_AUDIO_A2DP_SBC_MAX_BITPOOL   53

/* Array of decoder capabilities information. */
wiced_bt_a2dp_codec_info_t bt_audio_codec_capabilities[] =
{
    {
        .codec_id = WICED_BT_A2DP_CODEC_SBC,
        .cie =
        {
            .sbc =
            {
                (A2D_SBC_IE_SAMP_FREQ_44 | A2D_SBC_IE_SAMP_FREQ_48),    /* samp_freq */
                (A2D_SBC_IE_CH_MD_MONO   | A2D_SBC_IE_CH_MD_STEREO |
                 A2D_SBC_IE_CH_MD_JOINT  | A2D_SBC_IE_CH_MD_DUAL),      /* ch_mode */
                (A2D_SBC_IE_BLOCKS_16    | A2D_SBC_IE_BLOCKS_12 |
                 A2D_SBC_IE_BLOCKS_8     | A2D_SBC_IE_BLOCKS_4),        /* block_len */
                (A2D_SBC_IE_SUBBAND_4    | A2D_SBC_IE_SUBBAND_8),       /* num_subbands */
                (A2D_SBC_IE_ALLOC_MD_L   | A2D_SBC_IE_ALLOC_MD_S),      /* alloc_mthd */
                BT_AUDIO_A2DP_SBC_MAX_BITPOOL,                          /* max_bitpool for high quality audio */
                A2D_SBC_IE_MIN_BITPOOL                                  /* min_bitpool */
            }
        }
    },

#ifdef A2DP_SINK_AAC_ENABLED
    {
        .codec_id = WICED_BT_A2DP_CODEC_M24,
        .cie =
        {
            .m24 =
            {
                (A2D_M24_IE_OBJ_MSK),                                   /* obj_type */
                (A2D_M24_IE_SAMP_FREQ_44 | A2D_M24_IE_SAMP_FREQ_48),    /*samp_freq */
                (A2D_M24_IE_CHNL_MSK),                                  /* chnl */
                (A2D_M24_IE_VBR_MSK),                                   /* b7: VBR */
                (A2D_M24_IE_BITRATE_MSK)                                /* bitrate - b7-b0 of octect 3, all of octect4, 5*/
            }
        }
    }
#endif
};

/** A2DP sink configuration data */
wiced_bt_a2dp_config_data_t bt_audio_config =
{
#ifdef A2DP_SINK_ENABLE_CONTENT_PROTECTION
    .feature_mask = WICED_BT_A2DP_SINK_FEAT_PROTECT | WICED_BT_A2DP_SINK_FEAT_DELAY_RPT,    /* feature mask */
#else
    .feature_mask = WICED_BT_A2DP_SINK_FEAT_DELAY_RPT,                                      /* feature mask */
#endif
    .codec_capabilities =
    {
        .count = sizeof(bt_audio_codec_capabilities) / sizeof(bt_audio_codec_capabilities[0]),
        .info  = bt_audio_codec_capabilities,                                           /* codec configuration */
    },
    .p_param =
    {
        .buf_depth_ms                   = 300,                                          /* in msec */
        .start_buf_depth                = 50,                                           /* start playback percentage of the buffer depth */
        .target_buf_depth               = 50,                                           /* target level percentage of the buffer depth */
        .overrun_control                = WICED_BT_A2DP_SINK_OVERRUN_CONTROL_FLUSH_DATA,/* overrun flow control flag */
        .adj_ppm_max                    = +300,                                         /* Max PPM adjustment value */
        .adj_ppm_min                    = -300,                                         /* Min PPM adjustment value */
        .adj_ppb_per_msec               = 200,                                          /* PPM adjustment per milli second */
        .lvl_correction_threshold_high  = +2000,                                        /* Level correction threshold high value */
        .lvl_correction_threshold_low   = -2000,                                        /* Level correction threshold low value */
        .adj_proportional_gain          = 20,                                           /* Proportional component of total PPM adjustment */
        .adj_integral_gain              = 2,                                            /* Integral component of total PPM adjustment */
    },
    .ext_codec =
    {
#if ( ( defined(CYW20719B1) || defined(CYW20721B1) ) && (WICED_A2DP_EXT_CODEC == WICED_TRUE) )
        .codec_id        = WICED_AUDIO_CODEC_AAC_DEC,
        .codec_functions = &AAC_codec_function_table,
#else
        .codec_id        = WICED_AUDIO_CODEC_NONE,
        .codec_functions = 0,
#endif
    },
};

/**  Audio buffer configuration configuration */
const wiced_bt_audio_config_buffer_t a2dp_sink_audio_buf_config = {
    .role                       =   WICED_AUDIO_SINK_ROLE,
    .audio_tx_buffer_size       =   0,
#if ( ( defined(CYW20719B1) || defined(CYW20721B1) ) && (WICED_A2DP_EXT_CODEC == WICED_TRUE) )
    .audio_codec_buffer_size    =   0xD800
#else
    .audio_codec_buffer_size    =   0x5000
#endif
};



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

#include "wiced_app_cfg.h"
#include "wiced_bt_avrc_defs.h"
#include "wiced_bt_avrc.h"
#include "wiced_bt_avrc_tg.h"

#include "wiced_bt_dev.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_sdp.h"

/* If APP_AVRC_TRACK_INFO_SUPPORTED, APP_AVRC_PLAY_STATUS_SUPPORTED or APP_AVRC_SETTING_CHANGE_SUPPORTED are supported, set the AVRC profile */
/* version as 1.3, else set it to 1.0 */
#if (defined(APP_AVRC_TRACK_INFO_SUPPORTED) || defined(APP_AVRC_PLAY_STATUS_SUPPORTED) || defined(APP_AVRC_SETTING_CHANGE_SUPPORTED))
#define AVRC_PROFILE_VER  AVRC_REV_1_3
#else
#define AVRC_PROFILE_VER  AVRC_REV_1_0
#endif

/*
 * Definitions
 */
// SDP Record handle for AVDT Source
#define HANDLE_AVDT_SOURCE                      0x10001
// SDP Record handle for AVRC TARGET
#define HANDLE_AVRC_TARGET                      0x10002
// SDP Record handle for AVRC TARGET
#define HANDLE_AVRC_CONTROLLER                  0x10003
// SDP Record handle for PNP (Device Information)
#define HANDLE_PNP                              0x10006

#define WICED_DEVICE_NAME                       "Wiced BLE HID Host"

const uint8_t pincode[WICED_PIN_CODE_LEN] = { 0x30, 0x30, 0x30, 0x30 };

/*****************************************************************************
 * wiced_bt core stack configuration
 ****************************************************************************/
const wiced_bt_cfg_settings_t wiced_bt_cfg_settings =
{
        .device_name                         = (uint8_t *)WICED_DEVICE_NAME,                             // Local device name (NULL terminated)
        .device_class                        = { 0x00, 0x05, 0xc0 },                                     // Local device class
        .security_requirement_mask           = (  BTM_SEC_NONE   ),                                      // Security requirements mask (BTM_SEC_NONE, or combinination of BTM_SEC_IN_AUTHENTICATE, BTM_SEC_OUT_AUTHENTICATE, BTM_SEC_ENCRYPT (see #wiced_bt_sec_level_e))
        .max_simultaneous_links              = 3,                                                        // Maximum number simultaneous links to different devices

        .br_edr_scan_cfg =                                                                               // BR/EDR scan settings
        {
            .inquiry_scan_type               = BTM_SCAN_TYPE_STANDARD,                                   // Inquiry scan type (BTM_SCAN_TYPE_STANDARD or BTM_SCAN_TYPE_INTERLACED)
            .inquiry_scan_interval           = WICED_BT_CFG_DEFAULT_INQUIRY_SCAN_INTERVAL,               // Inquiry scan interval  (0 to use default)
            .inquiry_scan_window             = WICED_BT_CFG_DEFAULT_INQUIRY_SCAN_WINDOW,                 // Inquiry scan window (0 to use default)

            .page_scan_type                  = BTM_SCAN_TYPE_STANDARD,                                   // Page scan type (BTM_SCAN_TYPE_STANDARD or BTM_SCAN_TYPE_INTERLACED)
            .page_scan_interval              = WICED_BT_CFG_DEFAULT_PAGE_SCAN_INTERVAL,                  // Page scan interval  (0 to use default)
            .page_scan_window                = WICED_BT_CFG_DEFAULT_PAGE_SCAN_WINDOW                     // Page scan window (0 to use default)
        },

        .ble_scan_cfg =                                                                                  // BLE scan settings
        {
            .scan_mode                       = BTM_BLE_SCAN_MODE_PASSIVE,                                 // BLE scan mode (BTM_BLE_SCAN_MODE_PASSIVE, BTM_BLE_SCAN_MODE_ACTIVE, or BTM_BLE_SCAN_MODE_NONE)

            .high_duty_scan_interval         = WICED_BT_CFG_DEFAULT_HIGH_DUTY_SCAN_INTERVAL,             // High duty scan interval
            .high_duty_scan_window           = WICED_BT_CFG_DEFAULT_HIGH_DUTY_SCAN_WINDOW,               // High duty scan window
            .high_duty_scan_duration         = 5,                                                        // High duty scan duration in seconds (0 for infinite)

            .low_duty_scan_interval          = 800,                                                      // Low duty scan interval
            .low_duty_scan_window            = 400,                                                      // Low duty scan window
            .low_duty_scan_duration          = 5,                                                        // Low duty scan duration in seconds (0 for infinite)

            /* Connection scan intervals */
            .high_duty_conn_scan_interval    = WICED_BT_CFG_DEFAULT_HIGH_DUTY_CONN_SCAN_INTERVAL,        // High duty cycle connection scan interval
            .high_duty_conn_scan_window      = WICED_BT_CFG_DEFAULT_HIGH_DUTY_CONN_SCAN_WINDOW,          // High duty cycle connection scan window
            .high_duty_conn_duration         = 5,                                                        // High duty cycle connection duration in seconds (0 for infinite)

            .low_duty_conn_scan_interval     = WICED_BT_CFG_DEFAULT_LOW_DUTY_CONN_SCAN_INTERVAL,         // Low duty cycle connection scan interval
            .low_duty_conn_scan_window       = WICED_BT_CFG_DEFAULT_LOW_DUTY_CONN_SCAN_WINDOW,           // Low duty cycle connection scan window
            .low_duty_conn_duration          = 5,                                                        // Low duty cycle connection duration in seconds (0 for infinite)

            /* Connection configuration */
            .conn_min_interval               = WICED_BT_CFG_DEFAULT_CONN_MIN_INTERVAL,                    // Minimum connection interval, 24 * 1.25 = 30ms.
            .conn_max_interval               = WICED_BT_CFG_DEFAULT_CONN_MAX_INTERVAL,                    // Maximum connection interval, 40 * 1.25 = 50ms.
            .conn_latency                    = WICED_BT_CFG_DEFAULT_CONN_LATENCY,                         // Connection latency, ~1sec
            .conn_supervision_timeout        = WICED_BT_CFG_DEFAULT_CONN_SUPERVISION_TIMEOUT              // Connection link supervsion timeout
        },

        .ble_advert_cfg =                                                                                // BLE advertisement settings
        {
            .channel_map                     = BTM_BLE_ADVERT_CHNL_37 |                                  // Advertising channel map (mask of BTM_BLE_ADVERT_CHNL_37, BTM_BLE_ADVERT_CHNL_38, BTM_BLE_ADVERT_CHNL_39)
                                               BTM_BLE_ADVERT_CHNL_38 |
                                               BTM_BLE_ADVERT_CHNL_39,

            .high_duty_min_interval          = WICED_BT_CFG_DEFAULT_HIGH_DUTY_ADV_MIN_INTERVAL,          // High duty undirected connectable minimum advertising interval 48 *0.625 = 30ms
            .high_duty_max_interval          = WICED_BT_CFG_DEFAULT_HIGH_DUTY_ADV_MAX_INTERVAL,          // High duty undirected connectable maximum advertising interval
            .high_duty_duration              = 30,                                                       // High duty undirected connectable advertising duration in seconds (0 for infinite)

            .low_duty_min_interval           = 1024,                                                     // Low duty undirected connectable minimum advertising  interval. 1024 *0.625 = 640ms
            .low_duty_max_interval           = 1024,                                                     // Low duty undirected connectable maximum advertising interval
            .low_duty_duration               = 60,                                                       // Low duty undirected connectable advertising duration in seconds (0 for infinite)

            .high_duty_directed_min_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MIN_INTERVAL, // High duty directed connectable minimum advertising interval
            .high_duty_directed_max_interval = WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MAX_INTERVAL, // High duty directed connectable maximum advertising interval

            .low_duty_directed_min_interval  = WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MIN_INTERVAL,  // Low duty directed connectable minimum advertising interval
            .low_duty_directed_max_interval  = WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MAX_INTERVAL,  // Low duty directed connectable maximum advertising interval
            .low_duty_directed_duration      = 30,                                                       // Low duty directed connectable advertising duration in seconds (0 for infinite)

            .high_duty_nonconn_min_interval  = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MIN_INTERVAL,  // High duty non-connectable minimum advertising interval
            .high_duty_nonconn_max_interval  = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MAX_INTERVAL,  // High duty non-connectable maximum advertising interval
            .high_duty_nonconn_duration      = 30,                                                       // High duty non-connectable advertising duration in seconds (0 for infinite)

            .low_duty_nonconn_min_interval   = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MIN_INTERVAL,   // Low duty non-connectable minimum advertising interval
            .low_duty_nonconn_max_interval   = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MAX_INTERVAL,   // Low duty non-connectable maximum advertising interval
            .low_duty_nonconn_duration       = 0                                                         // Low duty non-connectable advertising duration in seconds (0 for infinite)
        },

        .gatt_cfg =                                                                                      // GATT configuration
        {
            .appearance                     = APPEARANCE_GENERIC_TAG,                                    // GATT appearance (see gatt_appearance_e)
            .client_max_links               = 3,                                                         // Client config: maximum number of servers that local client can connect to
            .server_max_links               = 0,                                                         // Server config: maximum number of remote clients connections allowed by the local
            .max_attr_len                   = 180,                                                       // Maximum attribute length; gki_cfg must have a corresponding buffer pool that can hold this length
            .max_mtu_size                   = 185                                                        // Maximum MTU size for GATT connections, should be between 23 and (max_attr_len + 5 )
        },

        .rfcomm_cfg =                                                                                    // RFCOMM configuration
        {
            .max_links                      = 0,                                                         // Maximum number of simultaneous connected remote devices*/
            .max_ports                      = 0                                                          // Maximum number of simultaneous RFCOMM ports
        },

        .l2cap_application =                                                                             // Application managed l2cap protocol configuration
        {
            .max_links                      = 0,                                                         // Maximum number of application-managed l2cap links (BR/EDR and LE)

            /* BR EDR l2cap configuration */
            .max_psm                        = 0,                                                         // Maximum number of application-managed BR/EDR PSMs
            .max_channels                   = 0,                                                         // Maximum number of application-managed BR/EDR channels

            /* LE L2cap connection-oriented channels configuration */
            .max_le_psm                     = 0,                                                         // Maximum number of application-managed LE PSMs
            .max_le_channels                = 0,                                                         // Maximum number of application-managed LE channels

            /* LE L2cap fixed channel configuration */
            .max_le_l2cap_fixed_channels    = 0                                                          // Maximum number of application managed fixed channels supported (in addition to mandatory channels 4, 5 and 6).
        },

        .avdt_cfg =                                                                                      // Audio/Video Distribution configuration
        {
            .max_links                      = 0,                                                         // Maximum simultaneous audio/video links
            .max_seps                       = 0                                                          // Maximum number of stream end points
        },

        . avrc_cfg =                                                                                     // Audio/Video Remote Control configuration
        {
            .roles                          = 0,                                                         // Mask of local roles supported (AVRC_CONN_INITIATOR|AVRC_CONN_ACCEPTOR)
            .max_links                      = 0                                                          // Maximum simultaneous remote control links
        },
        .addr_resolution_db_size            = 5,                                                         // LE Address Resolution DB settings - effective only for pre 4.2 controller
        .max_number_of_buffer_pools         = 6,                                                         // Maximum number of buffer pools in p_btm_cfg_buf_pools and by wiced_create_pool
        .rpa_refresh_timeout                = WICED_BT_CFG_DEFAULT_RANDOM_ADDRESS_NEVER_CHANGE,          // Interval of  random address refreshing - secs
};

/*
 * This is the SDP database for the whole hci_control application
 */
const uint8_t wiced_app_cfg_sdp_record[] =
{
    // length is the sum of all records
    SDP_ATTR_SEQUENCE_2(0
              + 69 + 2                                                      // PNP
              ),

    // SDP record Device ID (total = 69 + 2 = 71)
    SDP_ATTR_SEQUENCE_1(69),                                                // 2 bytes, length of the record
        SDP_ATTR_RECORD_HANDLE(HANDLE_PNP),                                 // 8 bytes
        SDP_ATTR_CLASS_ID(UUID_SERVCLASS_PNP_INFORMATION),                  // 8
        SDP_ATTR_PROTOCOL_DESC_LIST(1),                                     // 18
        SDP_ATTR_UINT2(ATTR_ID_SPECIFICATION_ID, 0x103),                    // 6
        SDP_ATTR_UINT2(ATTR_ID_VENDOR_ID, 0x0131),                          // 6
        SDP_ATTR_UINT2(ATTR_ID_PRODUCT_ID, 0x0201),                         // 6
        SDP_ATTR_UINT2(ATTR_ID_PRODUCT_VERSION, 0x0001),                    // 6
        SDP_ATTR_BOOLEAN(ATTR_ID_PRIMARY_RECORD, 0x01),                     // 5
        SDP_ATTR_UINT2(ATTR_ID_VENDOR_ID_SOURCE, DI_VENDOR_ID_SOURCE_BTSIG),// 6
};

/*****************************************************************************
 * wiced_bt  buffer pool configuration
 *
 * Configure buffer pools used by the stack  according to application's requirement
 *
 * Pools must be ordered in increasing buf_size.
 * If a pool runs out of buffers, the next  pool will be used
 *****************************************************************************/
const wiced_bt_cfg_buf_pool_t wiced_app_cfg_buf_pools[WICED_BT_CFG_NUM_BUF_POOLS] =
{
/*  { buf_size, buf_count } */
    { 64,       12   },      /* Small Buffer Pool */
    { 360,      10   },      /* Medium Buffer Pool (used for HCI & RFCOMM control messages, min recommended size is 360) */
    { 1024,     6    },      /* Large Buffer Pool  (used for HCI ACL messages) */
    { 2048,     4    },      /* Extra Large Buffer Pool - Used for avdt media packets and miscellaneous (if not needed, set buf_count to 0) */
};

/*
 * wiced_app_cfg_buf_pools_get_num
 */
const wiced_bt_cfg_settings_t *wiced_app_cfg_get_settings(void)
{
    return &wiced_bt_cfg_settings;
}

/*
 * wiced_app_cfg_buf_pools_get
 */
const wiced_bt_cfg_buf_pool_t *wiced_app_cfg_buf_pools_get(void)
{
    return wiced_app_cfg_buf_pools;
}

/*
 * wiced_app_cfg_sdp_record_get
 */
uint8_t *wiced_app_cfg_sdp_record_get(void)
{
    return (uint8_t *)wiced_app_cfg_sdp_record;
}

/*
 * wiced_app_cfg_sdp_record_get_size
 */
uint16_t wiced_app_cfg_sdp_record_get_size(void)
{
    return (uint16_t)sizeof(wiced_app_cfg_sdp_record);
}


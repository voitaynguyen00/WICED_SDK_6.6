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

#pragma once

#include "bt_types.h"
#include "wiced_bt_cfg.h"
#include "hci_control_api.h"
#include "wiced_bt_hfp_hf_int.h"
#include "wiced_bt_audio.h"

// SDP Record for Hands-Free Unit
#define HDLR_HANDS_FREE_UNIT                    0x10001
#define HDLR_HEADSET_UNIT                       0x10002
#define HANDS_FREE_SCN                          0x01
#define HEADSET_SCN                             0x02
#define HANDS_FREE_DEVICE_NAME                  "free-hands"
#define BT_AUDIO_HFP_VOLUME_MIN                 1
#define BT_AUDIO_HFP_VOLUME_MAX                 15
#define TRANS_UART_BUFFER_SIZE                  1024
#define BT_AUDIO_INVALID_SCO_INDEX              0xFFFF
#define HANDSFREE_NVRAM_ID                      0x46

#define WICED_HS_EIR_BUF_MAX_SIZE               264
#define KEY_INFO_POOL_BUFFER_SIZE               145 //Size of the buffer used for holding the peer device key info
#define KEY_INFO_POOL_BUFFER_COUNT              10  //Correspond's to the number of peer devices

extern const wiced_bt_cfg_settings_t handsfree_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t handsfree_cfg_buf_pools[];
extern const wiced_bt_audio_config_buffer_t handsfree_audio_buf_config;
extern uint32_t  hci_control_proc_rx_cmd( uint8_t *p_data, uint32_t length );

extern const uint8_t handsfree_sdp_db[];

#ifndef BTM_SCO_PKT_TYPES_MASK_HV1
#define BTM_INVALID_SCO_INDEX           0xFFFF
#define BTM_SCO_LINK_ALL_PKT_MASK       0x003F
#define BTM_SCO_LINK_ONLY_MASK          0x0007
#define BTM_SCO_PKT_TYPES_MASK_HV3      0x0004
#define BTM_SCO_PKT_TYPES_MASK_EV3      0x0008
#define BTM_SCO_PKT_TYPES_MASK_EV4      0x0010
#define BTM_SCO_PKT_TYPES_MASK_EV5      0x0020
#define BTM_SCO_PKT_TYPES_MASK_NO_2_EV3 0x0040
#define BTM_SCO_PKT_TYPES_MASK_NO_3_EV3 0x0080
#define BTM_SCO_PKT_TYPES_MASK_NO_2_EV5 0x0100
#define BTM_SCO_PKT_TYPES_MASK_NO_3_EV5 0x0200
#define BTM_ESCO_RETRANS_POWER          2       // For S4 and T2 Retransmission_Effort=2
#define BTM_ESCO_RETRANS_QUALITY        2
#endif

#define SCO_CONNECTION_WAIT_TIMEOUT     1000    // If AG won't trigger sco connection in 1000msec of time, we will initiate SCO connection.

#define HANDS_FREE_SCO_PKT_TYPES    ( BTM_SCO_PKT_TYPES_MASK_HV3 | \
                                 BTM_SCO_PKT_TYPES_MASK_EV3 | \
                                 BTM_SCO_PKT_TYPES_MASK_EV4 | \
                                 BTM_SCO_PKT_TYPES_MASK_EV5 | \
                                 BTM_SCO_PKT_TYPES_MASK_NO_3_EV3 | \
                                 BTM_SCO_PKT_TYPES_MASK_NO_3_EV5 )

#if (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE)
#define SUPPORTED_FEATURES_ATT           ( WICED_BT_HFP_HF_SDP_FEATURE_ECNR | \
                                           WICED_BT_HFP_HF_SDP_FEATURE_3WAY_CALLING | \
                                           WICED_BT_HFP_HF_SDP_FEATURE_CLIP | \
                                           WICED_BT_HFP_HF_SDP_FEATURE_VRECG | \
                                           WICED_BT_HFP_HF_SDP_FEATURE_REMOTE_VOL_CTRL | \
                                           WICED_BT_HFP_HF_SDP_FEATURE_WIDEBAND_SPEECH )

#define BT_AUDIO_HFP_SUPPORTED_FEATURES  ( WICED_BT_HFP_HF_FEATURE_3WAY_CALLING | \
                                           WICED_BT_HFP_HF_FEATURE_CLIP_CAPABILITY | \
                                           WICED_BT_HFP_HF_FEATURE_VOICE_RECOGNITION_ACTIVATION | \
                                           WICED_BT_HFP_HF_FEATURE_REMOTE_VOLUME_CONTROL| \
                                           WICED_BT_HFP_HF_FEATURE_ENHANCED_CALL_STATUS | \
                                           WICED_BT_HFP_HF_FEATURE_ENHANCED_CALL_CONTROL | \
                                           WICED_BT_HFP_HF_FEATURE_CODEC_NEGOTIATION | \
                                           WICED_BT_HFP_HF_FEATURE_HF_INDICATORS)
#else
#define SUPPORTED_FEATURES_ATT           ( WICED_BT_HFP_HF_SDP_FEATURE_ECNR | \
                                           WICED_BT_HFP_HF_SDP_FEATURE_3WAY_CALLING | \
                                           WICED_BT_HFP_HF_SDP_FEATURE_CLIP | \
                                           WICED_BT_HFP_HF_SDP_FEATURE_VRECG | \
                                           WICED_BT_HFP_HF_SDP_FEATURE_REMOTE_VOL_CTRL )

#define BT_AUDIO_HFP_SUPPORTED_FEATURES  ( WICED_BT_HFP_HF_FEATURE_3WAY_CALLING | \
                                           WICED_BT_HFP_HF_FEATURE_CLIP_CAPABILITY | \
                                           WICED_BT_HFP_HF_FEATURE_VOICE_RECOGNITION_ACTIVATION | \
                                           WICED_BT_HFP_HF_FEATURE_REMOTE_VOLUME_CONTROL| \
                                           WICED_BT_HFP_HF_FEATURE_ENHANCED_CALL_STATUS | \
                                           WICED_BT_HFP_HF_FEATURE_ENHANCED_CALL_CONTROL | \
                                           WICED_BT_HFP_HF_FEATURE_HF_INDICATORS)
#endif

/** HF device indicators. */
typedef enum
{
    WICED_BT_HFP_HF_SERVICE_IND     =   1,
    WICED_BT_HFP_HF_CALL_IND        =   2,
    WICED_BT_HFP_HF_CALL_SETUP_IND  =   3,
    WICED_BT_HFP_HF_CALL_HELD_IND   =   4,
    WICED_BT_HFP_HF_SIGNAL_IND      =   5,
    WICED_BT_HFP_HF_ROAM_IND        =   6,
    WICED_BT_HFP_HF_BATTERY_IND     =   7
}wiced_bt_hfp_hf_indicator_t;
typedef struct
{
    wiced_bt_device_address_t               peer_bd_addr;
    wiced_bt_hfp_hf_connection_state_t      connection_status;
    int                                     call_active;
    int                                     call_held;
    wiced_bt_hfp_hf_callsetup_state_t       call_setup;
    wiced_bt_hfp_hf_inband_ring_state_t     inband_ring_status;
    uint8_t                                 mic_volume;
    uint8_t                                 spkr_volume;
    uint16_t                                sco_index;
    uint16_t                                rfcomm_handle;
    wiced_bool_t                            init_sco_conn;
    wiced_bool_t                            is_sco_connected;
} bluetooth_hfp_context_t;

extern bluetooth_hfp_context_t handsfree_ctxt_data;

/* data associated with HF_OPEN_EVT */
typedef struct
{
    BD_ADDR             bd_addr;
    uint8_t             status;
} hci_control_hf_open_t;

/* data associated with AT command response event */
typedef struct
{
    uint16_t            num;
    char                str[WICED_BT_HFP_HF_MAX_AT_CMD_LEN];
} hci_control_hf_value_t;

/* data associated with HF_CONNECTED_EVT */
typedef struct
{
    uint32_t           peer_features;
    uint8_t            profile_selected;
} hci_control_hf_connect_t;

/* union of data associated with HS callback */
typedef union
{
    hci_control_hf_open_t    open;
    hci_control_hf_connect_t conn;
    hci_control_hf_value_t   val;
} hci_control_hf_event_t;

typedef struct
{
    uint8_t pairing_allowed;
    hci_control_hf_connect_t connect;
    wiced_timer_t hfp_timer;
} handsfrees_app_globals;

extern handsfrees_app_globals handsfree_app_states;

/* External Function Definitions */
extern char *utl_strcpy( char *p_dst, char *p_src );
extern int utl_strlen( char *p_str );
extern int util_itoa(int i, char *p_s);
extern void wiced_bt_trace_array( const char *string, const uint8_t* array, const uint16_t len );
extern uint16_t wiced_app_cfg_sdp_record_get_size(void);
extern void hci_control_send_device_started_evt( void );
extern void hci_control_send_pairing_completed_evt( uint8_t status , wiced_bt_device_address_t bdaddr );
extern int hci_control_find_nvram_id(uint8_t *p_data, int len);
extern int hci_control_alloc_nvram_id( );
extern int hci_control_write_nvram( int nvram_id, int data_len, void *p_data, wiced_bool_t from_host );
extern int hci_control_read_nvram( int nvram_id, void *p_data, int data_len );
extern void hci_control_delete_nvram( int nvram_id ,wiced_bool_t from_host);

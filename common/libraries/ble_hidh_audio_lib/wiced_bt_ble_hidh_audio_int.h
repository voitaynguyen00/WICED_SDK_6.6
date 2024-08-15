/*
 *  Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of Cypress Semiconductor 
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

#include "wiced_bt_ble_hidh.h"
#include "wiced_bt_ble_hidh_audio.h"
#include "wiced_bt_ble_hidh_audio_core.h"
#include "wiced_bt_trace.h"
#include "sbc_decoder.h"

/******************************************************
 *                     Constants
 ******************************************************/
/* Enable this compile option to enable BLE HID Debug traces */
//#define WICED_BT_BLE_HIDH_AUDIO_TRACE_ENABLED

/* BLE HID Audio Trace macro(s) */
#ifdef WICED_BT_BLE_HIDH_AUDIO_TRACE_ENABLED
/* WICED_BT_BLE_HIDH_AUDIO_TRACE can be enabled/disabled */
#define WICED_BT_BLE_HIDH_AUDIO_TRACE(format, ...)      WICED_BT_TRACE("%s " format,\
                                                               __FUNCTION__, ##__VA_ARGS__)
#else
#define WICED_BT_BLE_HIDH_AUDIO_TRACE(...)
#endif

/* WICED_BT_BLE_HIDH_AUDIO_TRACE_ERR is always enabled */
#define WICED_BT_BLE_HIDH_AUDIO_TRACE_ERR(format, ...)  WICED_BT_TRACE("Err: %s " format, \
                                                                __FUNCTION__, ##__VA_ARGS__)

/*
 * Report Id containing Audio Control
 */
/* ReportId  */
#define WICED_BT_BLE_HIDH_AUDIO_CTRL_REPORT_ID          0xF8

/* From Host to HID device */
#define WICED_BT_BLE_HIDH_AUDIO_CTRL_CMD_MIC_START      0x02
#define WICED_BT_BLE_HIDH_AUDIO_CTRL_CMD_MIC_STOP       0x03

/* From HID device to Host */
#define WICED_BT_BLE_HIDH_AUDIO_CTRL_EVT_MIC_START      0x0C
#define WICED_BT_BLE_HIDH_AUDIO_CTRL_EVT_MIC_STOP       0x0D

/*
 * Report Id containing Audio Data
 */
/* ReportId */
#define WICED_BT_BLE_HIDH_AUDIO_DATA_REPORT_ID          0xF7

/* mSBC Format */
#define WICED_BT_BLE_HIDH_AUDIO_MSBC_SYNCWORD           0xAD
#define WICED_BT_BLE_HIDH_AUDIO_MSBC_SIZE               57
#define WICED_BT_BLE_HIDH_AUDIO_MSBC_PCM_SAMPLES        120

#define WICED_BT_BLE_HIDH_AUDIO_PADDING_SIZE            1

/*
 * WICED_BT_BLE_HIDH_AUDIO_FORMAT_MSBC_1 Format
 */
/* H2 Header */
#define WICED_BT_BLE_HIDH_AUDIO_MSCB_1_H2_HDR_SIZE      2
#define WICED_BT_BLE_HIDH_AUDIO_MSCB_1_H2_SYNC_LSB      0x01
#define WICED_BT_BLE_HIDH_AUDIO_MSCB_1_H2_SYNC_MSB_MSK  0x0F
#define WICED_BT_BLE_HIDH_AUDIO_MSCB_1_H2_SYNC_MSB      0x08
#define WICED_BT_BLE_HIDH_AUDIO_MSCB_1_H2_SEQ_MSK       0xF0
#define WICED_BT_BLE_HIDH_AUDIO_MSCB_1_H2_SEQ_0         0x00
#define WICED_BT_BLE_HIDH_AUDIO_MSCB_1_H2_SEQ_1         0x30
#define WICED_BT_BLE_HIDH_AUDIO_MSCB_1_H2_SEQ_2         0xC0
#define WICED_BT_BLE_HIDH_AUDIO_MSCB_1_H2_SEQ_3         0xF0

/*
 * WICED_BT_BLE_HIDH_AUDIO_FORMAT_MSBC_3 Format
 */
#define WICED_BT_BLE_HIDH_AUDIO_MSBC_3_HDR_SIZE         1
#define WICED_BT_BLE_HIDH_AUDIO_MSBC_3_PACKET_SIZE      (1 + (3 * WICED_BT_BLE_HIDH_AUDIO_MSBC_SIZE))

/*
 * mSBC Decoder
 */
/* The mSBC Decoder requires a 60 bytes input buffer (H2 + mSBC + padding) */
#define WICED_BT_BLE_HIDH_AUDIO_MSBC_DECODER_SIZE       (WICED_BT_BLE_HIDH_AUDIO_MSCB_1_H2_HDR_SIZE + \
                                                         WICED_BT_BLE_HIDH_AUDIO_MSBC_SIZE + \
                                                         WICED_BT_BLE_HIDH_AUDIO_PADDING_SIZE)
/* mSBC Working and Scratch buffer */
#define WICED_BT_BLE_HIDH_AUDIO_MSBC_STATIC_MEM_SIZE    1920
#define WICED_BT_BLE_HIDH_AUDIO_MSBC_SCRATCH_MEM_SIZE   2048



#define WICED_BT_BLE_HIDH_AUDIO_INVALID_HANDLE              0xFFFF


typedef enum
{
    WICED_BT_BLE_HIDH_AUDIO_FORMAT_AUTO = 0,    /* Auto audio Format detection */
    WICED_BT_BLE_HIDH_AUDIO_FORMAT_MSBC_1,      /* Legacy mSBC format (H2, 1 mSBC frame, padding) */
    WICED_BT_BLE_HIDH_AUDIO_FORMAT_MSBC_3,      /* new mSBC format (SeqN, 3 mSBC frames) */
    WICED_BT_BLE_HIDH_AUDIO_FORMAT_CELT_1,      /* CELT format (SeqN, 1 CELT frame) */

    /* Add other Audio format here ... */

    WICED_BT_BLE_HIDH_AUDIO_FORMAT_UNKNOW,
} wiced_bt_ble_hidh_audio_sink_format_t;



/* BLE HIDH Audio receive State */
typedef enum
{
    WICED_BT_BLE_HIDH_AUDIO_SINK_STATE_IDLE = 0,
    WICED_BT_BLE_HIDH_AUDIO_SINK_STATE_W4_H2_MSB,
    WICED_BT_BLE_HIDH_AUDIO_SINK_STATE_W4_MSBC_SYNCWORD,
    WICED_BT_BLE_HIDH_AUDIO_SINK_STATE_W4_MSBC_DATA
} wiced_bt_ble_hidh_audio_sink_state_t;

/******************************************************
 *                     Structures
 ******************************************************/
/* SBC Decoder Control Block  (Parameters and working Memories) */
typedef struct
{
    /* Internal memory used by SBC Decoder */
    int32_t static_mem[WICED_BT_BLE_HIDH_AUDIO_MSBC_STATIC_MEM_SIZE/sizeof(int32_t)];
    /* Internal scratch memory used by SBC Decoder */
    int32_t scratch_mem[WICED_BT_BLE_HIDH_AUDIO_MSBC_SCRATCH_MEM_SIZE/sizeof(int32_t)];
    /* Decompressed PCM samples (120 samples = 240 bytes) */
    int16_t pcm_out[WICED_BT_BLE_HIDH_AUDIO_MSBC_PCM_SAMPLES];
    /* Received Data (H2 Header, mSBC Frame and Padding */
    uint8_t rx_data[WICED_BT_BLE_HIDH_AUDIO_MSBC_DECODER_SIZE];
    uint16_t rx_data_len;
    /* SBC Decoder Parameters */
    SBC_DEC_PARAMS decoder_param;
    wiced_bool_t initialized;
} wiced_bt_ble_hidh_audio_sbc_dec_t;

/* BLE HIDH Audio Sink Control Block */
typedef struct
{
    uint16_t handle;
    wiced_bt_ble_hidh_audio_sink_state_t state;
    wiced_bt_ble_hidh_audio_sbc_dec_t sbc_dec;
    uint8_t seq_expected;
    uint8_t seq_received;
    wiced_bool_t playing;
    wiced_bt_ble_hidh_audio_sink_format_t packet_format;
} wiced_bt_ble_hidh_audio_sink_cb_t;

/* BLE HIDH Audio Control Block */
typedef struct
{
    wiced_bt_ble_hidh_audio_cback_t *p_callback;
    wiced_bt_ble_hidh_audio_sink_cb_t sink;
    uint8_t set_report_pending_nb;
} wiced_bt_ble_hidh_audio_cb_t;

/******************************************************
 *                 Global Variables
 ******************************************************/
extern wiced_bt_ble_hidh_audio_cb_t wiced_bt_ble_hidh_audio_cb;


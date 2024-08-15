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

#include "wiced_bt_ble_hidh_audio_core.h"
#include "wiced_bt_a2d.h"
#include "wiced_bt_a2d_sbc.h"

/******************************************************
 *                 Local functions
 ******************************************************/
static wiced_bool_t wiced_bt_ble_hidh_audio_report_filter(wiced_bt_ble_hidh_report_t *p_report);
static wiced_bt_ble_hidh_audio_sink_format_t wiced_bt_ble_hidh_audio_format_detect(uint8_t *p_data,
        uint8_t length);
static void wiced_bt_ble_hidh_audio_mscb_1_data_handler(uint8_t *p_data, uint16_t length);
static void wiced_bt_ble_hidh_audio_mscb_3_data_handler(uint8_t *p_data, uint16_t length);
static void wiced_bt_ble_hidh_audio_mscb_decode(uint8_t *p_data, uint16_t length);
static wiced_bt_ble_hidh_status_t wiced_bt_ble_hidh_audio_mic_start_req_handle(
        wiced_bt_ble_hidh_report_t *p_report);
static wiced_bt_ble_hidh_status_t wiced_bt_ble_hidh_audio_mic_stop_req_handle(
        wiced_bt_ble_hidh_report_t *p_report);

/******************************************************
 *                 Global Variables
 ******************************************************/
/* BLE HIDH Audio control Block */
wiced_bt_ble_hidh_audio_cb_t wiced_bt_ble_hidh_audio_cb;

/*
 * wiced_bt_ble_hidh_audio_cback
 */
wiced_bool_t wiced_bt_ble_hidh_audio_cback(wiced_bt_ble_hidh_event_t event,
        wiced_bt_ble_hidh_event_data_t *p_event_data)
{
    wiced_bool_t event_filtered = WICED_FALSE;
    wiced_bt_ble_hidh_audio_sink_cb_t *p_sink_cb = &wiced_bt_ble_hidh_audio_cb.sink;
    wiced_bt_ble_hidh_audio_event_data_t audio_evt;

    switch(event)
    {
    case WICED_BT_BLE_HIDH_OPEN_EVT:
    case WICED_BT_BLE_HIDH_DESCRIPTOR_EVT:
    case WICED_BT_BLE_HIDH_GATT_CACHE_EVT:
    case WICED_BT_BLE_HIDH_GET_REPORT_EVT:
    case WICED_BT_BLE_HIDH_VIRTUAL_UNPLUG_EVT:
    case WICED_BT_BLE_HIDH_SET_PROTOCOL_EVT:
    default:
        /* Event not needed by BLE HID Host Audio */
        break;

    case WICED_BT_BLE_HIDH_REPORT_EVT:
        event_filtered = wiced_bt_ble_hidh_audio_report_filter(&p_event_data->report);
        break;

    case WICED_BT_BLE_HIDH_SET_REPORT_EVT:
        if (wiced_bt_ble_hidh_audio_cb.set_report_pending_nb != 0)
        {
            wiced_bt_ble_hidh_audio_cb.set_report_pending_nb--;
            event_filtered = WICED_TRUE;
        }
        break;

    case WICED_BT_BLE_HIDH_CLOSE_EVT:
        /* If the Audio is playing and this device disconnect */
        if ((p_sink_cb->playing) &&
            (p_sink_cb->handle == p_event_data->disconnected.handle))
        {
            p_sink_cb->playing = WICED_FALSE;
            if (wiced_bt_ble_hidh_audio_cb.p_callback != NULL)
            {
                audio_evt.stop.handle = p_sink_cb->handle;
                wiced_bt_ble_hidh_audio_cb.p_callback(WICED_BT_BLE_HIDH_AUDIO_STOP_EVT, &audio_evt);
            }
            p_sink_cb->handle = WICED_BT_BLE_HIDH_AUDIO_INVALID_HANDLE;
        }
        break;
    }

    return event_filtered;
}

/*
 * wiced_bt_ble_hidh_audio_sbc_dec_init
 */
wiced_bt_ble_hidh_status_t wiced_bt_ble_hidh_audio_sbc_dec_init(void)
{
#ifndef WICEDX
    wiced_bt_ble_hidh_audio_sbc_dec_t *p_sbc_dec;
    SBC_DEC_PARAMS *p_sbc_dec_param;
    void *p_buff;
    int16_t sbc_status;

    WICED_BT_BLE_HIDH_AUDIO_TRACE("sizeof(wiced_bt_ble_hidh_audio_sbc_dec_t):%d\n",
            sizeof(wiced_bt_ble_hidh_audio_sbc_dec_t));
    p_sbc_dec = &wiced_bt_ble_hidh_audio_cb.sink.sbc_dec;
    memset(p_sbc_dec, 0, sizeof(*p_sbc_dec));

    p_sbc_dec_param = &p_sbc_dec->decoder_param;

    p_sbc_dec_param->s32StaticMem = p_sbc_dec->static_mem;
    p_sbc_dec_param->s32ScratchMem = p_sbc_dec->scratch_mem;

    p_sbc_dec_param->sbc_mode = SBC_MODE_WB; //Wide Band Mode (mSBC)
    p_sbc_dec_param->sbc_option = 0; //SBC_PLC_DISABLED;

    /* Initialize SBC Decoder (will be done every time it is started) */
    sbc_status = SBC_Decoder_decode_Init(p_sbc_dec_param);
    if (sbc_status != SBC_SUCCESS)
        WICED_BT_BLE_HIDH_AUDIO_TRACE_ERR("SBC_Decoder_decode_Init failed\n");
#endif
    return WICED_BT_BLE_HIDH_STATUS_SUCCESS;
}

/*
 * wiced_bt_ble_hidh_audio_report_filter
 */
static wiced_bool_t wiced_bt_ble_hidh_audio_report_filter(wiced_bt_ble_hidh_report_t *p_report)
{
    wiced_bool_t event_filtered = WICED_FALSE;
    uint8_t *p = p_report->p_data;
    uint8_t ctrl;
    uint8_t tx_report_data[5];
    wiced_bt_ble_hidh_status_t status;
    wiced_bt_ble_hidh_audio_sink_cb_t *p_sink_cb = &wiced_bt_ble_hidh_audio_cb.sink;
    wiced_bt_ble_hidh_audio_event_data_t audio_evt;

    if (p == NULL)
        return event_filtered;

    switch(p_report->report_id)
    {
    case WICED_BT_BLE_HIDH_AUDIO_DATA_REPORT_ID:
        /* If this is Audio Data for the currently playing device */
        if ((p_sink_cb->handle == p_report->handle) &&
            (p_sink_cb->playing))
        {
            /* If the Audio packet format is not yet known, try to detect it */
            if (p_sink_cb->packet_format == WICED_BT_BLE_HIDH_AUDIO_FORMAT_AUTO)
            {
                p_sink_cb->packet_format = wiced_bt_ble_hidh_audio_format_detect(p_report->p_data,
                        p_report->length);
            }
            switch ((int)p_sink_cb->packet_format)
            {
            case WICED_BT_BLE_HIDH_AUDIO_FORMAT_MSBC_1:
                wiced_bt_ble_hidh_audio_mscb_1_data_handler(p_report->p_data, p_report->length);
                break;

            case WICED_BT_BLE_HIDH_AUDIO_FORMAT_MSBC_3:
                wiced_bt_ble_hidh_audio_mscb_3_data_handler(p_report->p_data, p_report->length);
                break;

            case WICED_BT_BLE_HIDH_AUDIO_FORMAT_CELT_1:
                WICED_BT_BLE_HIDH_AUDIO_TRACE_ERR("CELT Decoder not yet implemented\n");
                /* Send Request to Stop Audio */
                wiced_bt_ble_hidh_audio_mic_stop_req_handle(p_report);
                break;

            case WICED_BT_BLE_HIDH_AUDIO_FORMAT_UNKNOW:
                WICED_BT_BLE_HIDH_AUDIO_TRACE_ERR("Unable to detect Audio Format from data:%02X %02X %02X...\n",
                        p_report->p_data[0], p_report->p_data[1], p_report->p_data[2]);
                /* Send Request to Stop Audio */
                wiced_bt_ble_hidh_audio_mic_stop_req_handle(p_report);
                break;
            }
        }
        event_filtered = WICED_TRUE;
        break;

    case WICED_BT_BLE_HIDH_AUDIO_CTRL_REPORT_ID:
        /* Extract the Command/Event */
        STREAM_TO_UINT8(ctrl, p);
        switch(ctrl)
        {
        /* Peer device Request to Start its Microphone (Audio button pressed) */
        case WICED_BT_BLE_HIDH_AUDIO_CTRL_EVT_MIC_START:
            if (p_sink_cb->playing)
            {
                WICED_BT_BLE_HIDH_AUDIO_TRACE_ERR("Sink already active\n");
                break;
            }
            WICED_BT_BLE_HIDH_AUDIO_TRACE("Audio Start Event received\n");
            wiced_bt_ble_hidh_audio_mic_start_req_handle(p_report);
            event_filtered = WICED_TRUE;
            break;

        /* Peer device Request to Stop its Microphone (Audio button released) */
        case WICED_BT_BLE_HIDH_AUDIO_CTRL_EVT_MIC_STOP:
            if (p_sink_cb->playing == WICED_FALSE)
            {
                WICED_BT_BLE_HIDH_AUDIO_TRACE_ERR("Sink not playing\n");
                break;
            }
            WICED_BT_BLE_HIDH_AUDIO_TRACE("Audio Stop Event received\n");
            wiced_bt_ble_hidh_audio_mic_stop_req_handle(p_report);
            event_filtered = WICED_TRUE;
            break;

        default:
            WICED_BT_BLE_HIDH_AUDIO_TRACE_ERR("Unknown ctrl:%d\n", ctrl);
            break;
        }
        break;

    default:
        /* Unhandled ReportId */
        break;
    }

    return event_filtered;
}

/*
 * wiced_bt_ble_hidh_audio_mscb_1_data_handler
 * This function will handle reception of audio data in mSBC_1 format
 * Some RC don't support MTU, so we need to parse the received packet to detect the beginning
 * of a valid Frame. The format is:
 *
 *  +--------------------------------------------------------+
 *  | Byte 0 | Byte 1 | Byte 2 |       mSBC Data   | Byte 59 |
 *  +--------------------------------------------------------+
 *  | H2 LSB | H2 MSB |  0xAD  |  56 bytes         | 00 (pad)|
 *  +--------------------------------------------------------+
 * H2 MSB = contains the Sequence number (see HFP Specification)
 */
static void wiced_bt_ble_hidh_audio_mscb_1_data_handler(uint8_t *p_data, uint16_t length)
{
    wiced_bt_ble_hidh_audio_sink_cb_t *p_sink_cb = &wiced_bt_ble_hidh_audio_cb.sink;
    wiced_bt_ble_hidh_audio_sbc_dec_t *p_sbc_dec;
    uint8_t byte;
    int cpy_len;

    p_sbc_dec = &p_sink_cb->sbc_dec;

    while (length > 0)
    {
        switch(p_sink_cb->state)
        {
        case WICED_BT_BLE_HIDH_AUDIO_SINK_STATE_IDLE:
            STREAM_TO_UINT8(byte, p_data);
            length--;
            /* This is the first byte of the H2 Synchronization Header */
            if (byte == WICED_BT_BLE_HIDH_AUDIO_MSCB_1_H2_SYNC_LSB)
            {
                p_sbc_dec->rx_data_len = 0;
                p_sbc_dec->rx_data[p_sbc_dec->rx_data_len++] = byte;
                p_sink_cb->state = WICED_BT_BLE_HIDH_AUDIO_SINK_STATE_W4_H2_MSB;
            }
            else
                WICED_BT_BLE_HIDH_AUDIO_TRACE("ignore byte:0x%02X\n", byte);
            break;

        case WICED_BT_BLE_HIDH_AUDIO_SINK_STATE_W4_H2_MSB:
            STREAM_TO_UINT8(byte, p_data);
            length--;
            /* The H2 Synchronization part (4 bits) is correct */
            if ((byte & WICED_BT_BLE_HIDH_AUDIO_MSCB_1_H2_SYNC_MSB_MSK) ==
                    WICED_BT_BLE_HIDH_AUDIO_MSCB_1_H2_SYNC_MSB)
            {
                p_sbc_dec->rx_data[p_sbc_dec->rx_data_len++] = byte;
                p_sink_cb->state = WICED_BT_BLE_HIDH_AUDIO_SINK_STATE_W4_MSBC_SYNCWORD;

                /* Check if the Sequence Number coding is correct */
                switch (byte & WICED_BT_BLE_HIDH_AUDIO_MSCB_1_H2_SEQ_MSK)
                {
                case WICED_BT_BLE_HIDH_AUDIO_MSCB_1_H2_SEQ_0:
                    p_sink_cb->seq_received = 0;
                    break;
                case WICED_BT_BLE_HIDH_AUDIO_MSCB_1_H2_SEQ_1:
                    p_sink_cb->seq_received = 1;
                    break;
                case WICED_BT_BLE_HIDH_AUDIO_MSCB_1_H2_SEQ_2:
                    p_sink_cb->seq_received = 2;
                    break;
                case WICED_BT_BLE_HIDH_AUDIO_MSCB_1_H2_SEQ_3:
                    p_sink_cb->seq_received = 3;
                    break;
                default:
                    WICED_BT_BLE_HIDH_AUDIO_TRACE_ERR("bad seq:0x%02X\n",
                            byte & WICED_BT_BLE_HIDH_AUDIO_MSCB_1_H2_SEQ_MSK);
                    p_sink_cb->state = WICED_BT_BLE_HIDH_AUDIO_SINK_STATE_IDLE;
                    p_sbc_dec->rx_data_len = 0;
                    break;
                }
            }
            else
            {
                WICED_BT_BLE_HIDH_AUDIO_TRACE_ERR("bad h2 MSB:0x%02X\n",
                        byte & WICED_BT_BLE_HIDH_AUDIO_MSCB_1_H2_SYNC_MSB_MSK);
                p_sink_cb->state = WICED_BT_BLE_HIDH_AUDIO_SINK_STATE_IDLE;
                p_sbc_dec->rx_data_len = 0;
            }
            break;

        case WICED_BT_BLE_HIDH_AUDIO_SINK_STATE_W4_MSBC_SYNCWORD:
            STREAM_TO_UINT8(byte, p_data);
            length--;
            /* Found an mSBC Synchronization byte. This seems to be a good frame... */
            if (byte == WICED_BT_BLE_HIDH_AUDIO_MSBC_SYNCWORD)
            {
                p_data--;   /* Re-introduce the mSBC Sync word */
                length++;
                cpy_len = MIN(length, WICED_BT_BLE_HIDH_AUDIO_MSBC_SIZE +
                                      WICED_BT_BLE_HIDH_AUDIO_PADDING_SIZE);
                memcpy(&p_sbc_dec->rx_data[p_sbc_dec->rx_data_len], p_data, cpy_len);
                p_sbc_dec->rx_data_len += cpy_len;
                length -= cpy_len;
                p_data += cpy_len;

                /* If a Full packet has been received, decode it */
                if (p_sbc_dec->rx_data_len == WICED_BT_BLE_HIDH_AUDIO_MSBC_DECODER_SIZE)
                {
                    wiced_bt_ble_hidh_audio_mscb_decode(p_sbc_dec->rx_data, p_sbc_dec->rx_data_len);
                    p_sbc_dec->rx_data_len = 0;
                    p_sink_cb->state = WICED_BT_BLE_HIDH_AUDIO_SINK_STATE_IDLE;
                }
                else
                {
                    /* Partial mSBC Frame received. Wait for the remaining parts */
                    p_sink_cb->state = WICED_BT_BLE_HIDH_AUDIO_SINK_STATE_W4_MSBC_DATA;
                }
            }
            else
            {
                WICED_BT_BLE_HIDH_AUDIO_TRACE_ERR("bad mSBC Hdr:0x%02X\n", byte);
                p_sink_cb->state = WICED_BT_BLE_HIDH_AUDIO_SINK_STATE_IDLE;
                p_sbc_dec->rx_data_len = 0;
            }
            break;

        case WICED_BT_BLE_HIDH_AUDIO_SINK_STATE_W4_MSBC_DATA:
            cpy_len = MIN(length, sizeof(p_sbc_dec->rx_data) - p_sbc_dec->rx_data_len);
            memcpy(&p_sbc_dec->rx_data[p_sbc_dec->rx_data_len], p_data, cpy_len);
            p_sbc_dec->rx_data_len += cpy_len;
            length -= cpy_len;
            p_data += cpy_len;

            /* If a Full packet has been received, decode it */
            if (p_sbc_dec->rx_data_len == WICED_BT_BLE_HIDH_AUDIO_MSBC_DECODER_SIZE)
            {
                wiced_bt_ble_hidh_audio_mscb_decode(p_sbc_dec->rx_data, p_sbc_dec->rx_data_len);
                p_sbc_dec->rx_data_len = 0;
                p_sink_cb->state = WICED_BT_BLE_HIDH_AUDIO_SINK_STATE_IDLE;
            }
            else
            {
                /* Partial mSBC Frame received. Wait for the remaining parts */
            }
            break;

        default:
            WICED_BT_BLE_HIDH_AUDIO_TRACE_ERR("bad state:%d\n", p_sink_cb->state);
            break;
        }
    }
}

/*
 * wiced_bt_ble_hidh_audio_mscb_3_data_handler
 * This function will handle reception of audio data in mSBC_3 format
 * Such RC requires the MTU to be big enough to sent 3 mSBC frames in one packet.
 * The format is:
 *
 *  +-----------------------------------------------------------------------+
 *  | Byte 0 | Byte 1 |  mSBC Frame 1   |  mSBC Frame 2   |  mSBC Frame 3   |
 *  +-----------------------------------------------------------------------+
 *  |  SeqN  |  0xAD  | 0xAD | 56 bytes | 0xAD | 56 bytes | 0xAD | 56 bytes |
 *  +-----------------------------------------------------------------------+
 */
static void wiced_bt_ble_hidh_audio_mscb_3_data_handler(uint8_t *p_data, uint16_t length)
{
    wiced_bt_ble_hidh_audio_sink_cb_t *p_sink_cb = &wiced_bt_ble_hidh_audio_cb.sink;
    wiced_bt_ble_hidh_audio_sbc_dec_t *p_sbc_dec;
    uint8_t *p;
    uint8_t byte;
    int nb_msbc_frames;
    uint8_t *p_msbc_decoder;
    uint16_t h2_hdr;

    if (length < WICED_BT_BLE_HIDH_AUDIO_MSBC_3_PACKET_SIZE)
    {
        WICED_BT_BLE_HIDH_AUDIO_TRACE_ERR("mSBC_3 frame too small %d\n", length);
        return;
    }

    p = p_data;
    p_sbc_dec = &p_sink_cb->sbc_dec;

    /* Read the first byte of the payload (Sequence Number). Ignore it */
    STREAM_TO_UINT8(byte, p);
    if (byte); // makes compiler happy

    /* This packet will contain 3 mSCB frames */
    nb_msbc_frames = 3;
    do
    {
        /* The mSBC Decoder requires an H2 Header. Build, a fake, one */
        p_msbc_decoder = &p_sbc_dec->rx_data[0];
        h2_hdr = WICED_BT_BLE_HIDH_AUDIO_MSCB_1_H2_SYNC_LSB |
                 (WICED_BT_BLE_HIDH_AUDIO_MSCB_1_H2_SYNC_MSB << 8);

        if (p_sink_cb->seq_received == 1)
        {
            h2_hdr |= (WICED_BT_BLE_HIDH_AUDIO_MSCB_1_H2_SEQ_1 << 8);
        }
        else if (p_sink_cb->seq_received == 2)
        {
            h2_hdr |= (WICED_BT_BLE_HIDH_AUDIO_MSCB_1_H2_SEQ_2 << 8);
        }
        else if (p_sink_cb->seq_received == 3)
        {
            h2_hdr |= (WICED_BT_BLE_HIDH_AUDIO_MSCB_1_H2_SEQ_3 << 8);
        }
        else
        {
            p_sink_cb->seq_received = 0;
            h2_hdr |= (WICED_BT_BLE_HIDH_AUDIO_MSCB_1_H2_SEQ_0 << 8);
        }
        UINT16_TO_STREAM(p_msbc_decoder, h2_hdr);

        memcpy(p_msbc_decoder, p, WICED_BT_BLE_HIDH_AUDIO_MSBC_SIZE);
        p += WICED_BT_BLE_HIDH_AUDIO_MSBC_SIZE;
        p_msbc_decoder += WICED_BT_BLE_HIDH_AUDIO_MSBC_SIZE;

        /* Write the Padding byte */
        UINT8_TO_STREAM(p_msbc_decoder, 0x00);

        /* We rebuilt an mSBC Frame. Call the mSBC decoder */
        wiced_bt_ble_hidh_audio_mscb_decode(p_sbc_dec->rx_data,
                WICED_BT_BLE_HIDH_AUDIO_MSBC_DECODER_SIZE);

        /* Update Sequence number for next frame */
        p_sink_cb->seq_received++;
        if (p_sink_cb->seq_received > 3)
            p_sink_cb->seq_received = 0;

        nb_msbc_frames--;
    } while (nb_msbc_frames > 0);
}

/*
 * wiced_bt_ble_hidh_audio_mscb_decode
 */
static void wiced_bt_ble_hidh_audio_mscb_decode(uint8_t *p_data, uint16_t length)
{
#ifndef WICEDX
    wiced_bt_ble_hidh_audio_event_data_t audio_evt;
    wiced_bt_ble_hidh_audio_sink_cb_t *p_sink_cb = &wiced_bt_ble_hidh_audio_cb.sink;
    int16_t sbc_status;

    if (p_sink_cb->sbc_dec.initialized == WICED_FALSE)
    {
        /* Initialize the mSBC Decoder */
        sbc_status = SBC_Decoder_decode_Init(&p_sink_cb->sbc_dec.decoder_param);
        if (sbc_status != SBC_SUCCESS)
        {
            WICED_BT_BLE_HIDH_AUDIO_TRACE_ERR("SBC_Decoder_decode_Init failed\n");
            return;
        }
        p_sink_cb->sbc_dec.initialized = WICED_TRUE;
    }


    /* Check the Sequence number */
    if (p_sink_cb->seq_expected != p_sink_cb->seq_received)
    {
        WICED_BT_BLE_HIDH_AUDIO_TRACE_ERR("Seq Error received:%d expected:%d\n",
                p_sink_cb->seq_received, p_sink_cb->seq_expected);
    }

    /* Update next Expected Sequence Number */
    p_sink_cb->seq_expected = p_sink_cb->seq_received + 1;
    if (p_sink_cb->seq_expected > 3)
        p_sink_cb->seq_expected = 0;

    /* Decompress the mSBC Frame */
    p_sink_cb->sbc_dec.decoder_param.nb_frame = 1;
    p_sink_cb->sbc_dec.decoder_param.frame_status[0] = 0;
    p_sink_cb->sbc_dec.decoder_param.frame_len = 57;
    sbc_status = SBC_Decoder_decoder(&p_sink_cb->sbc_dec.decoder_param, p_data, length,
            p_sink_cb->sbc_dec.pcm_out);
    if (sbc_status != WICED_BT_BLE_HIDH_AUDIO_MSBC_PCM_SAMPLES)
    {
        WICED_BT_BLE_HIDH_AUDIO_TRACE_ERR("SBC_Decoder_decoder returns:%d\n", sbc_status);
        WICED_BT_BLE_HIDH_AUDIO_TRACE_ERR("SBC frame:%02X %02X %02X %02X %02X %02X %02X %02X...\n",
                p_data[0], p_data[1], p_data[2], p_data[3],
                p_data[4], p_data[5], p_data[6], p_data[7]);
        return;
    }

    /* Send the Decompressed mSBC data (PCM) to Application */
    if (wiced_bt_ble_hidh_audio_cb.p_callback != NULL)
    {
        audio_evt.rx_data.p_data = (uint8_t *)p_sink_cb->sbc_dec.pcm_out;
        audio_evt.rx_data.handle = p_sink_cb->handle;
        audio_evt.rx_data.length = WICED_BT_BLE_HIDH_AUDIO_MSBC_PCM_SAMPLES * sizeof(uint16_t);
        wiced_bt_ble_hidh_audio_cb.p_callback(WICED_BT_BLE_HIDH_AUDIO_RX_DATA_EVT, &audio_evt);
    }
#endif
}

/*
 * wiced_bt_ble_hidh_audio_mic_start_req_handle
 */
static wiced_bt_ble_hidh_status_t wiced_bt_ble_hidh_audio_mic_start_req_handle(
        wiced_bt_ble_hidh_report_t *p_report)
{
    wiced_bt_ble_hidh_audio_sink_cb_t *p_sink_cb = &wiced_bt_ble_hidh_audio_cb.sink;
    wiced_bt_ble_hidh_audio_event_data_t audio_evt;
    uint8_t *p;
    uint8_t tx_report_data[5];
    wiced_bt_ble_hidh_status_t status;
    int16_t sbc_status;

    WICED_BT_BLE_HIDH_AUDIO_TRACE("Send Audio Start Cmd\n");

    /* For the moment, we don't know the Audio packet format */
    p_sink_cb->packet_format = WICED_BT_BLE_HIDH_AUDIO_FORMAT_AUTO;

    p_sink_cb->state = WICED_BT_BLE_HIDH_AUDIO_SINK_STATE_IDLE;
    p_sink_cb->sbc_dec.initialized = WICED_FALSE;
    p_sink_cb->seq_received = 0;
    p_sink_cb->seq_expected = 0;

    /* Build the Audio Start Command */
    p = &tx_report_data[0];
    UINT8_TO_STREAM(p, WICED_BT_BLE_HIDH_AUDIO_CTRL_CMD_MIC_START); /* Audio Start Cmd */
    UINT8_TO_STREAM(p, 0); /* channel */
    UINT8_TO_STREAM(p, 1); /* reserved: audio mode */
    UINT8_TO_STREAM(p, 0); /* reserved */
    UINT8_TO_STREAM(p, 0); /* data count */

    /* Send the Audio Start Command Report */
    status = wiced_bt_ble_hidh_set_report(p_report->handle,
            WICED_BT_BLE_HIDH_REPORT_TYPE_FEATURE,
            WICED_BT_BLE_HIDH_AUDIO_CTRL_REPORT_ID,
            tx_report_data, p - tx_report_data);
    if (status == WICED_BT_BLE_HIDH_STATUS_SUCCESS)
    {
        /* Report Sent. Remember that we sent one (to filter out the SetReportEvent */
        wiced_bt_ble_hidh_audio_cb.set_report_pending_nb++;

        p_sink_cb->playing = WICED_TRUE;
        p_sink_cb->handle = p_report->handle;

        if (wiced_bt_ble_hidh_audio_cb.p_callback != NULL)
        {
            audio_evt.start.channel_nb = 1; /* Mono */
            audio_evt.start.format = WICED_BT_BLE_HIDH_AUDIO_FORMAT_PCM_16;
            audio_evt.start.frequency = 16000;
            audio_evt.start.handle = p_sink_cb->handle;
            wiced_bt_ble_hidh_audio_cb.p_callback(WICED_BT_BLE_HIDH_AUDIO_START_EVT, &audio_evt);
        }
    }
    else
    {
        WICED_BT_BLE_HIDH_AUDIO_TRACE_ERR("wiced_bt_ble_hidh_set_report failed %d\n", status);
    }
    return status;
}

/*
 * wiced_bt_ble_hidh_audio_mic_stop_req_handle
 */
static wiced_bt_ble_hidh_status_t wiced_bt_ble_hidh_audio_mic_stop_req_handle(
        wiced_bt_ble_hidh_report_t *p_report)
{
    wiced_bt_ble_hidh_audio_sink_cb_t *p_sink_cb = &wiced_bt_ble_hidh_audio_cb.sink;
    wiced_bt_ble_hidh_audio_event_data_t audio_evt;
    uint8_t *p;
    uint8_t tx_report_data[10];
    wiced_bt_ble_hidh_status_t status;
    uint16_t sbc_enc_init_rv;

    WICED_BT_BLE_HIDH_AUDIO_TRACE("Send Audio Stop Cmd\n");

    /* Build Stop Req message */
    p = &tx_report_data[0];
    UINT8_TO_STREAM(p, WICED_BT_BLE_HIDH_AUDIO_CTRL_CMD_MIC_STOP); /* Audio Stop Cmd */
    UINT8_TO_STREAM(p, 0); /* channel */
    UINT8_TO_STREAM(p, 1); /* reserved: audio mode */
    UINT16_TO_STREAM(p, 0); /* reserved */
    UINT16_TO_STREAM(p, 0); /* data count */

    /* Send the Audio Stop Command Report */
    status = wiced_bt_ble_hidh_set_report(p_report->handle,
            WICED_BT_BLE_HIDH_REPORT_TYPE_FEATURE,
            WICED_BT_BLE_HIDH_AUDIO_CTRL_REPORT_ID,
            tx_report_data, p - tx_report_data);
    if (status != WICED_BT_BLE_HIDH_STATUS_SUCCESS)
    {
        WICED_BT_BLE_HIDH_AUDIO_TRACE_ERR("wiced_bt_ble_hidh_set_report failed %d\n", status);
    }
    else
    {
        /* Report Sent. Remember that we sent one (to filter out the SetReportEvent */
        wiced_bt_ble_hidh_audio_cb.set_report_pending_nb++;
    }
    p_sink_cb->playing = WICED_FALSE;
    if (wiced_bt_ble_hidh_audio_cb.p_callback != NULL)
    {
        audio_evt.stop.handle = p_sink_cb->handle;
        wiced_bt_ble_hidh_audio_cb.p_callback(WICED_BT_BLE_HIDH_AUDIO_STOP_EVT, &audio_evt);
    }
    p_sink_cb->handle = WICED_BT_BLE_HIDH_AUDIO_INVALID_HANDLE;

    return WICED_BT_BLE_HIDH_STATUS_SUCCESS;
}

/*
 * wiced_bt_ble_hidh_audio_format_detect
 */
static wiced_bt_ble_hidh_audio_sink_format_t wiced_bt_ble_hidh_audio_format_detect(uint8_t *p_data,
        uint8_t length)
{
    wiced_bt_ble_hidh_audio_sink_format_t audio_format;
    uint8_t *p;
    uint8_t byte;
    uint32_t celt_length;

    WICED_BT_BLE_HIDH_AUDIO_TRACE("Length:%d data:%02X %02X %02X... \n",
            length, p_data[0], p_data[1], p_data[2]);

    p = p_data;

    /* Read the first byte of the payload */
    STREAM_TO_UINT8(byte, p);

    /* If the First byte is the LSB is an H2 Synchronization word */
    if (byte == WICED_BT_BLE_HIDH_AUDIO_MSCB_1_H2_SYNC_LSB)
    {
        WICED_BT_BLE_HIDH_AUDIO_TRACE("H2 LSB detected\n");
        /* Read the second byte of the payload */
        STREAM_TO_UINT8(byte, p);

        /* Check if the second H2 Synchronization part (4 bits) is correct */
        if ((byte & WICED_BT_BLE_HIDH_AUDIO_MSCB_1_H2_SYNC_MSB_MSK) ==
                WICED_BT_BLE_HIDH_AUDIO_MSCB_1_H2_SYNC_MSB)
        {
            /* H2 Synchronization word (12 bits) found. */
            WICED_BT_BLE_HIDH_AUDIO_TRACE("H2 MSB detected\n");

            /* Check if the Sequence Number coding is correct */
            switch (byte & WICED_BT_BLE_HIDH_AUDIO_MSCB_1_H2_SEQ_MSK)
            {
            case WICED_BT_BLE_HIDH_AUDIO_MSCB_1_H2_SEQ_0:
            case WICED_BT_BLE_HIDH_AUDIO_MSCB_1_H2_SEQ_1:
            case WICED_BT_BLE_HIDH_AUDIO_MSCB_1_H2_SEQ_2:
            case WICED_BT_BLE_HIDH_AUDIO_MSCB_1_H2_SEQ_3:
                WICED_BT_BLE_HIDH_AUDIO_TRACE("H2 SeqN detected\n");
                /* Read the third byte of the payload */
                STREAM_TO_UINT8(byte, p);
                if (byte == WICED_BT_BLE_HIDH_AUDIO_MSBC_SYNCWORD)
                {
                    WICED_BT_BLE_HIDH_AUDIO_TRACE("mSBC(1) Syncword detected\n");
                    WICED_BT_BLE_HIDH_AUDIO_TRACE("Format detected is: mSBC_1\n");
                    return WICED_BT_BLE_HIDH_AUDIO_FORMAT_MSBC_1;
                }
                break;

            default:
                WICED_BT_BLE_HIDH_AUDIO_TRACE("mSBC_1 format not detected\n");
                break;
            }
        }
    }

    /* We have not been able to detect an mSBC_1 format. Check if it's an mSBC_2 format */
    p = p_data;

    /* Read the first byte of the payload */
    STREAM_TO_UINT8(byte, p);

    /* If the First byte is 0, this is 'perhaps' the first Sequence Number */
    if (byte == 0x00)
    {
        /* Read the second byte of the payload. It should be an mSBC Syncword */
        STREAM_TO_UINT8(byte, p);
        if (byte == WICED_BT_BLE_HIDH_AUDIO_MSBC_SYNCWORD)
        {
            WICED_BT_BLE_HIDH_AUDIO_TRACE("mSBC(2) Syncword detected\n");
            /* The RC sending mSBC_3 format support only MTU >= 172 bytes */
            /* So, the length of this packet must be bigger */
            if (length >= WICED_BT_BLE_HIDH_AUDIO_MSBC_3_PACKET_SIZE)
            {
                WICED_BT_BLE_HIDH_AUDIO_TRACE("Packet size >= %d\n",
                        WICED_BT_BLE_HIDH_AUDIO_MSBC_3_PACKET_SIZE);
                WICED_BT_BLE_HIDH_AUDIO_TRACE("Format detected is: mSBC_3\n");
                return WICED_BT_BLE_HIDH_AUDIO_FORMAT_MSBC_3;
            }
        }
    }

    /* Not able to detect neither mSBC_1 nor mSBC_3 format. */

    /* Try to detect CELT Frame */
    p = p_data;
    /* Read the first byte of the payload */
    STREAM_TO_UINT8(byte, p);

    /* If the First byte is 0, this is 'perhaps' the first Sequence Number */
    if (byte == 0x00)
    {
        /* Read the first 4 bytes of the payload (supposed to contain the length of the
         * CELT frame).
         * Note that this length is coded in Big Endian mode
         * */
        BE_STREAM_TO_UINT32(celt_length, p);

        /* The CELT frame is supposed to contain:
         * seqN(1 byte) + lenght (4 bytes) + range (4 bytes) + length (x bytes)
         */
        if (length == (sizeof(uint8_t) + sizeof(uint32_t) + sizeof(uint32_t) + celt_length))
        {
            WICED_BT_BLE_HIDH_AUDIO_TRACE("CELT Frame:%d size match packet length\n", celt_length);
            WICED_BT_BLE_HIDH_AUDIO_TRACE("Format detected is: CELT_1\n");
            return WICED_BT_BLE_HIDH_AUDIO_FORMAT_CELT_1;
        }
    }
    return WICED_BT_BLE_HIDH_AUDIO_FORMAT_UNKNOW;
}


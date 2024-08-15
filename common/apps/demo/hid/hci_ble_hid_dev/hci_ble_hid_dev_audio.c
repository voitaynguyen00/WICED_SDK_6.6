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
 *
 *  This file contains some test code to simulate Audio over BLE HID (used by Cypress Remote Control).
 *  The 20706A2 Eval Board does not have an Audio chip. So, we will send a Tone (encoded in mSBC
 *  format).
 *  Furthermore, the SW timer API of the Wiced SDK does not support accurate, periodic, timer.
 *  This implementation uses the HW Timer2 of the 20706A2.
 *  The SW6/BTN1 button is used to simulate the 'Audio Button' of the RC).
 */

#include "hci_ble_hid_dev_audio.h"
#include "hci_ble_hid_dev.h"
#include "wiced_bt_trace.h"
#include "wiced_timer.h"
#include "hci_ble_hid_dev_db.h"
#include "sbc_encoder.h"
#include "wiced_hal_gpio.h"
#include "bt_types.h"

/*******************************************************************
 * Constant Definitions
 ******************************************************************/
/* From Host to HID device */
#define HCI_BLE_HID_DEV_AUDIO_CTRL_CMD_MIC_START        0x02
#define HCI_BLE_HID_DEV_AUDIO_CTRL_CMD_MIC_STOP         0x03

/* From HID device to Host */
#define HCI_BLE_HID_DEV_AUDIO_CTRL_EVT_MIC_START        0x0C
#define HCI_BLE_HID_DEV_AUDIO_CTRL_EVT_MIC_STOP         0x0D

#define HCI_BLE_HID_DEV_AUDIO_NB_SAMPLES                120

#define HCI_BLE_HID_DEV_AUDIO_REPORT_ID_SIZE            1

#define HCI_BLE_HID_DEV_AUDIO_SEQ_SIZE                  1

#define HCI_BLE_HID_DEV_AUDIO_H2_SIZE                   2
#define HCI_BLE_HID_DEV_AUDIO_H2_SYNCWORD               0x0801
#define HCI_BLE_HID_DEV_AUDIO_H2_SN0                    0x0000
#define HCI_BLE_HID_DEV_AUDIO_H2_SN1                    0x3000
#define HCI_BLE_HID_DEV_AUDIO_H2_SN2                    0xC000
#define HCI_BLE_HID_DEV_AUDIO_H2_SN3                    0xF000

#define HCI_BLE_HID_DEV_AUDIO_MSBC_SIZE                 57
#define HCI_BLE_HID_DEV_AUDIO_PADDING_SIZE              1

#if 0
#define HCI_BLE_HID_DEV_AUDIO_TX_BUFFER_SIZE            (HCI_BLE_HID_DEV_AUDIO_REPORT_ID_SIZE  + \
                                                         HCI_BLE_HID_DEV_AUDIO_H2_SIZE +         \
                                                         HCI_BLE_HID_DEV_AUDIO_MSBC_SIZE +       \
                                                         HCI_BLE_HID_DEV_AUDIO_PADDING_SIZE)
#else

#define HCI_BLE_HID_DEV_AUDIO_NB_MSBC_PER_PACKET        3
#define HCI_BLE_HID_DEV_AUDIO_TX_BUFFER_SIZE            (HCI_BLE_HID_DEV_AUDIO_REPORT_ID_SIZE  + \
                                                         HCI_BLE_HID_DEV_AUDIO_SEQ_SIZE +        \
                                                         (HCI_BLE_HID_DEV_AUDIO_MSBC_SIZE *      \
                                                          HCI_BLE_HID_DEV_AUDIO_NB_MSBC_PER_PACKET))
#endif

/******************************************************
 *                    Structures
 ******************************************************/
typedef struct
{
    uint8_t angle;
    SBC_ENC_PARAMS sbc_encoder_param;
    uint8_t seq_number;
    wiced_bool_t audio_active;
    wiced_timer_t timer;
} hci_ble_hid_dev_audio_cb_t;

/******************************************************
 *               Variable Definitions
 ******************************************************/
hci_ble_hid_dev_audio_cb_t hci_ble_hid_dev_audio_cb;

/* PCM Samples of a tone (to simulate audio) */
int16_t hci_ble_hid_dev_audio_tone[] =
{
        0x0000, /* 0 */
        0x30FB, /* 12539 */
        0x5A82, /* 23170 */
        0x7641, /* 30273 */
        0x7FFF, /* 32767 */
        0x7641, /* 30273 */
        0x5A82, /* 23170 */
        0x30FB, /* 12539 */
        0x0000, /* 0 */
        0xCF05, /* -12539 */
        0xA57E, /* -23170 */
        0x89BF, /* -30273 */
        0x8001, /* -32767 */
        0x89BF, /* -30273 */
        0xA57E, /* -23170 */
        0xCF05, /* -12539 */
};

/******************************************************
 *               Local Functions
 ******************************************************/
static void hci_ble_hid_dev_audio_timer_callback(uint32_t param);
static uint8_t *hci_ble_hid_dev_audio_write_h2_hdr(uint8_t *p_buffer);

/*
 * hci_ble_hid_dev_audio_init
 */
void hci_ble_hid_dev_audio_init(void)
{
    wiced_result_t result;
    uint32_t old_addr;
    uint8_t *p;

    memset(&hci_ble_hid_dev_audio_cb, 0, sizeof(hci_ble_hid_dev_audio_cb));

    hci_ble_hid_dev_audio_cb.sbc_encoder_param.numOfSubBands = SBC_SUB_BANDS_8;
    hci_ble_hid_dev_audio_cb.sbc_encoder_param.numOfBlocks = 15;
    hci_ble_hid_dev_audio_cb.sbc_encoder_param.allocationMethod  = SBC_LOUDNESS;
    hci_ble_hid_dev_audio_cb.sbc_encoder_param.channelMode = SBC_MONO;
    hci_ble_hid_dev_audio_cb.sbc_encoder_param.samplingFreq = SBC_sf16000;
    hci_ble_hid_dev_audio_cb.sbc_encoder_param.bitPool = 26;
    hci_ble_hid_dev_audio_cb.sbc_encoder_param.sbc_mode = SBC_MODE_WB;

    /* Initialize SBC Decoder (will be done every time it is started) */
    SBC_Encoder_Init(&hci_ble_hid_dev_audio_cb.sbc_encoder_param);

    /* Initialize the Timer */
    wiced_init_timer(&hci_ble_hid_dev_audio_cb.timer, hci_ble_hid_dev_audio_timer_callback, 0,
            WICED_MILLI_SECONDS_PERIODIC_TIMER);
}

/*
 * hci_ble_hid_dev_audio_button_handler
 * This Function must be called when the Button controlling audio (SW6) is pressed or released
 */
void hci_ble_hid_dev_audio_button_handler(wiced_bool_t is_pressed)
{
    uint8_t tx_report_data[6];
    uint8_t *p = tx_report_data;

    WICED_BT_TRACE("hci_ble_hid_dev_audio_button_handler is_pressed:%d\n", is_pressed);

    /* Build the Audio Start/Stop Request Report */
    UINT8_TO_STREAM(p, HID_AUDIO_CTRL_INPUT_REPORT_ID);
    if (is_pressed)
    {
        /* Reduce the connection interval to send audio (18 frames means 18 * 1.25 = 22.5ms) */
        hci_ble_hid_update_ble_conn_params(18, 18, 0, 200);

        UINT8_TO_STREAM(p, HCI_BLE_HID_DEV_AUDIO_CTRL_EVT_MIC_START); /* Audio Start Cmd */
    }
    else
    {
        UINT8_TO_STREAM(p, HCI_BLE_HID_DEV_AUDIO_CTRL_EVT_MIC_STOP); /* Audio Start Cmd */
        /* Stop the Audio timer (in case there is a transmission problem with either Stop Req or Cmd */
       wiced_stop_timer(&hci_ble_hid_dev_audio_cb.timer);
    }

    UINT8_TO_STREAM(p, 0); /* channel */
    UINT8_TO_STREAM(p, 1); /* reserved: audio mode */
    UINT16_TO_STREAM(p, 0); /* data count (0 bytes following) */

    /* Send the Report to the HID Host */
    hci_ble_hid_dev_handle_send_report(HID_REPORT_TYPE_INPUT, tx_report_data, sizeof(tx_report_data));
}

/*
 * hci_ble_hid_dev_audio_write_ctrl_feature_handler
 * This Function must be called when the HID Host writes the Audio Control Feature Report Id
 */
void hci_ble_hid_dev_audio_write_ctrl_feature_handler(uint8_t *p_data, uint16_t length)
{
    wiced_result_t result;
    uint8_t byte;

    /* Extract the Command */
    STREAM_TO_UINT8(byte, p_data);

    /* HID Host sent command to Start Audio */
    if (byte == HCI_BLE_HID_DEV_AUDIO_CTRL_CMD_MIC_START)
    {
        hci_ble_hid_dev_audio_cb.audio_active = WICED_TRUE;
        hci_ble_hid_dev_audio_cb.seq_number = 0;

        /* Initialize SBC Decoder (done every time it is started) */
        SBC_Encoder_Init(&hci_ble_hid_dev_audio_cb.sbc_encoder_param);

        /*
         * We are supposed to use a timer of 22.5ms (3 * 7.5ms). But the period of a wiced timer
         * can only be an entier number of ms.
         * So, we use a 15 ms timer (2 * 7.5ms) and we will ignore 1 timer interrupt over 3.
         * Each mSBC frame contains 7.5 ms of audio
         */
        wiced_start_timer(&hci_ble_hid_dev_audio_cb.timer, 15);
    }
    /* HID Host sent command to Stop Audio */
    else  if (byte == HCI_BLE_HID_DEV_AUDIO_CTRL_CMD_MIC_STOP)
    {
        hci_ble_hid_dev_audio_cb.audio_active = WICED_FALSE;

        /* Stop the timer */
        wiced_stop_timer(&hci_ble_hid_dev_audio_cb.timer);

        /* Increase the connection interval when audio stopped (reduce power consumption) */
        hci_ble_hid_update_ble_conn_params(40, 40, 0, 200);
    }
}

/*
 * hci_ble_hid_dev_audio_disconnect
 * This function is called when the link is disconnected.
 * Stop audio timer in case the disconnection happens while Audio in playing
 */
void hci_ble_hid_dev_audio_disconnect(void)
{
    hci_ble_hid_dev_audio_cb.audio_active = WICED_FALSE;

    /* Stop the timer */
    wiced_stop_timer(&hci_ble_hid_dev_audio_cb.timer);
}

/*
 * hci_ble_hid_dev_audio_timer_callback
 */
static void hci_ble_hid_dev_audio_timer_callback(uint32_t param)
{
    int16_t pcm_buffer[HCI_BLE_HID_DEV_AUDIO_NB_SAMPLES];
    int angle;
    int nb_mscb_frames;
    uint16_t sbc_encoder_rv;
    uint8_t *p;
    uint8_t tx_buffer[HCI_BLE_HID_DEV_AUDIO_TX_BUFFER_SIZE];

    p = tx_buffer;

    /* Write the ReportId. It will be removed before transmission OTA */
    UINT8_TO_STREAM(p, HID_AUDIO_DATA_INPUT_REPORT_ID);

    /* Write the Sequence Number and increment it */
    UINT8_TO_STREAM(p, hci_ble_hid_dev_audio_cb.seq_number++);

    if ((hci_ble_hid_dev_audio_cb.seq_number % 3) == 0)
        return;

    for (nb_mscb_frames = 0 ; nb_mscb_frames < HCI_BLE_HID_DEV_AUDIO_NB_MSBC_PER_PACKET ; nb_mscb_frames++)
    {
        /* Create a PCM Buffer containing the tone */
        for (angle = 0 ; angle < HCI_BLE_HID_DEV_AUDIO_NB_SAMPLES ; angle++)
        {
            pcm_buffer[angle] = hci_ble_hid_dev_audio_tone[hci_ble_hid_dev_audio_cb.angle];
            hci_ble_hid_dev_audio_cb.angle++;
            if (hci_ble_hid_dev_audio_cb.angle >= (sizeof(hci_ble_hid_dev_audio_tone) /
                                                   sizeof(hci_ble_hid_dev_audio_tone[0])))
                hci_ble_hid_dev_audio_cb.angle = 0;
        }

        /* Write the Encoded mSBC data */
        sbc_encoder_rv = SBC_Encoder_encode(&hci_ble_hid_dev_audio_cb.sbc_encoder_param,
                pcm_buffer, p, HCI_BLE_HID_DEV_AUDIO_NB_SAMPLES);
        if (sbc_encoder_rv == HCI_BLE_HID_DEV_AUDIO_MSBC_SIZE); // makes compiler happy
        p += HCI_BLE_HID_DEV_AUDIO_MSBC_SIZE;
    }

    /* Send the Report to the HID Host */
    hci_ble_hid_dev_handle_send_report(HID_REPORT_TYPE_INPUT, tx_buffer, sizeof(tx_buffer));
}

/*
 * hci_ble_hid_dev_audio_write_h2_hdr
 */
static uint8_t *hci_ble_hid_dev_audio_write_h2_hdr(uint8_t *p_buffer)
{
    uint16_t h2_hdr;

    /* Build H2 Header from Sequence number */
    if (hci_ble_hid_dev_audio_cb.seq_number == 0)
        h2_hdr = HCI_BLE_HID_DEV_AUDIO_H2_SYNCWORD | HCI_BLE_HID_DEV_AUDIO_H2_SN0;
    else if (hci_ble_hid_dev_audio_cb.seq_number == 1)
        h2_hdr = HCI_BLE_HID_DEV_AUDIO_H2_SYNCWORD | HCI_BLE_HID_DEV_AUDIO_H2_SN1;
    else if (hci_ble_hid_dev_audio_cb.seq_number == 2)
        h2_hdr = HCI_BLE_HID_DEV_AUDIO_H2_SYNCWORD | HCI_BLE_HID_DEV_AUDIO_H2_SN2;
    else if (hci_ble_hid_dev_audio_cb.seq_number == 3)
        h2_hdr = HCI_BLE_HID_DEV_AUDIO_H2_SYNCWORD | HCI_BLE_HID_DEV_AUDIO_H2_SN3;

    /* Write it */
    UINT16_TO_STREAM(p_buffer, h2_hdr);

    /* Update Sequence number */
    hci_ble_hid_dev_audio_cb.seq_number++;
    hci_ble_hid_dev_audio_cb.seq_number &= 0x03;

    return p_buffer;
}


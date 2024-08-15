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
 * This file contains functions for processing AT commands and results.
 */

#include <string.h>
#include <stdlib.h>
#include "wiced_bt_trace.h"
#include "wiced_bt_hfp_hf.h"
#include "wiced_bt_hfp_hf_int.h"

extern char *utl_strcpy( char *p_dst, char *p_src );

/******************************************************
 * Indicator Enumerations
 ******************************************************/
#define WICED_BT_HFP_HF_CALL_IND_NAME "call"
#define WICED_BT_HFP_HF_CALL_SETUP_IND_NAME "call_setup"
#define WICED_BT_HFP_HF_CALLSETUP_IND_NAME "callsetup"
#define WICED_BT_HFP_HF_SERVICE_IND_NAME "service"
#define WICED_BT_HFP_HF_BATTERY_IND_NAME "battchg"
#define WICED_BT_HFP_HF_CALL_HELD_IND_NAME "callheld"
#define WICED_BT_HFP_HF_SIGNAL_IND_NAME "signal"
#define WICED_BT_HFP_HF_ROAM_IND_NAME "roam"

#define WICED_BT_HFP_HF_MAX_RSSI_LEVEL 5
#define WICED_BT_HFP_HF_MAX_BATTERY_CHARGE_LEVEL 5

/* State to track AT command transaction during service level connection setup */
/* TODO: Remove the dependency on the MACROS */
enum
{
    WICED_BT_HFP_HF_AT_SLC_STATE_INIT,
    WICED_BT_HFP_HF_AT_SLC_STATE_BRSF,
#if (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE)
    WICED_BT_HFP_HF_AT_SLC_STATE_BAC,
#endif
    WICED_BT_HFP_HF_AT_SLC_STATE_CIND_TEST,
    WICED_BT_HFP_HF_AT_SLC_STATE_CIND_READ,
    WICED_BT_HFP_HF_AT_SLC_STATE_CMER,
    WICED_BT_HFP_HF_AT_SLC_STATE_CHLD,
#if (WICED_BT_HFP_HF_VERSION >= WICED_BT_HFP_HF_VERSION_1_7 \
  && WICED_BT_HFP_HF_IND_SUPPORTED == TRUE)
    WICED_BT_HFP_HF_AT_SLC_STATE_BIND_SET,
    WICED_BT_HFP_HF_AT_SLC_STATE_BIND_TEST,
    WICED_BT_HFP_HF_AT_SLC_STATE_BIND_READ,
#endif
    WICED_BT_HFP_HF_AT_SLC_STATE_VGM,
    WICED_BT_HFP_HF_AT_SLC_STATE_VGS,
    WICED_BT_HFP_HF_AT_SLC_STATE_CLIP,
    WICED_BT_HFP_HF_AT_SLC_STATE_CCWA,
    WICED_BT_HFP_HF_AT_SLC_STATE_COPS,
    WICED_BT_HFP_HF_AT_SLC_STATE_CMEE,
    WICED_BT_HFP_HF_AT_SLC_STATE_DONE
};

/* AT result code table element */
typedef struct
{
    const char *p_cmd; /* AT cmd string */
} wiced_bt_hfp_hf_at_cmd_t;

/* AT command strings */
const wiced_bt_hfp_hf_at_cmd_t wiced_bt_hfp_hf_cmd_str[] =
{
    {"+VGS" },
    {"+VGM" },
    {"A"    },
    {"+BINP"},
    {"+BVRA"},
    {"+BLDN"},
    {"+CHLD"},
    {"+CHUP"},
    {"+CIND"},
    {"+CNUM"},
    {"D"    },
    {"+NREC"},
    {"+VTS" },
    {"+BTRH"},
    {"+COPS"},
    {"+CMEE"},
    {"+CLCC"},
    {"+BIA"},
    {"+BIEV"},
    {"+BCC"},
    {"+BCS"},
    {"+BAC"},
    {"+BIND"},
    {"+CKPD"},
    {"+BRSF"},
    {"+CLIP"},
    {"+CMER"},
    {"+CCWA"}
};

/* Handsfree Profile result codes from AG */
enum
{
     WICED_BT_HFP_HF_RES_OK,
     WICED_BT_HFP_HF_RES_ERROR,
     WICED_BT_HFP_HF_RES_CMEE,
     WICED_BT_HFP_HF_RES_RING,
     WICED_BT_HFP_HF_RES_VGS,
     WICED_BT_HFP_HF_RES_VGM,
     WICED_BT_HFP_HF_RES_CCWA,
     WICED_BT_HFP_HF_RES_CHLD,
     WICED_BT_HFP_HF_RES_CIND,
     WICED_BT_HFP_HF_RES_CLIP,
     WICED_BT_HFP_HF_RES_CIEV,
     WICED_BT_HFP_HF_RES_BINP,
     WICED_BT_HFP_HF_RES_BVRA,
     WICED_BT_HFP_HF_RES_BSIR,
     WICED_BT_HFP_HF_RES_CNUM,
     WICED_BT_HFP_HF_RES_BTRH,
     WICED_BT_HFP_HF_RES_COPS,
     WICED_BT_HFP_HF_RES_CLCC,
     WICED_BT_HFP_HF_RES_BIND,
     WICED_BT_HFP_HF_RES_BCS,
     WICED_BT_HFP_HF_RES_BUSY,
     WICED_BT_HFP_HF_RES_VGS_EQ,
     WICED_BT_HFP_HF_RES_VGM_EQ,
     WICED_BT_HFP_HF_RES_BVRA_EQ,
     WICED_BT_HFP_HF_RES_BRSF,
     WICED_BT_HFP_HF_RES_MAX
};

/* Handsfree Profile result codes from AG */
wiced_bt_hfp_hf_at_res_t wiced_bt_hfp_hf_res[] =
{
    {"OK"         , WICED_BT_HFP_HF_AT_FMT_NONE, WICED_BT_HFP_HF_OK_EVT},
    {"ERROR"      , WICED_BT_HFP_HF_AT_FMT_NONE, WICED_BT_HFP_HF_ERROR_EVT},
    {"+CME ERROR:", WICED_BT_HFP_HF_AT_FMT_INT , WICED_BT_HFP_HF_CME_ERROR_EVT},
    {"RING"       , WICED_BT_HFP_HF_AT_FMT_NONE, WICED_BT_HFP_HF_RING_EVT},
    {"+VGS:"      , WICED_BT_HFP_HF_AT_FMT_INT , WICED_BT_HFP_HF_VOLUME_CHANGE_EVT},
    {"+VGM:"      , WICED_BT_HFP_HF_AT_FMT_INT , WICED_BT_HFP_HF_VOLUME_CHANGE_EVT},
    {"+CCWA:"     , WICED_BT_HFP_HF_AT_FMT_STR , WICED_BT_HFP_HF_NO_EVT},
    {"+CHLD:"     , WICED_BT_HFP_HF_AT_FMT_STR , WICED_BT_HFP_HF_NO_EVT},
    {"+CIND:"     , WICED_BT_HFP_HF_AT_FMT_STR , WICED_BT_HFP_HF_NO_EVT},
    {"+CLIP:"     , WICED_BT_HFP_HF_AT_FMT_STR , WICED_BT_HFP_HF_CLIP_IND_EVT},
    {"+CIEV:"     , WICED_BT_HFP_HF_AT_FMT_STR , WICED_BT_HFP_HF_NO_EVT},
    {"+BINP:"     , WICED_BT_HFP_HF_AT_FMT_STR , WICED_BT_HFP_HF_BINP_EVT},
    {"+BVRA:"     , WICED_BT_HFP_HF_AT_FMT_INT , WICED_BT_HFP_HF_VOICE_RECOGNITION_EVT},
    {"+BSIR:"     , WICED_BT_HFP_HF_AT_FMT_INT , WICED_BT_HFP_HF_NO_EVT},
    {"+CNUM:"     , WICED_BT_HFP_HF_AT_FMT_STR , WICED_BT_HFP_HF_CNUM_EVT},
    {"+BTRH:"     , WICED_BT_HFP_HF_AT_FMT_INT , WICED_BT_HFP_HF_NO_EVT},
    {"+COPS:"     , WICED_BT_HFP_HF_AT_FMT_STR , WICED_BT_HFP_HF_NO_EVT},
    {"+CLCC:"     , WICED_BT_HFP_HF_AT_FMT_STR , WICED_BT_HFP_HFP_ACTIVE_CALL_EVT},
    {"+BIND:"     , WICED_BT_HFP_HF_AT_FMT_STR , WICED_BT_HFP_HF_BIND_EVT},
    {"+BCS:"      , WICED_BT_HFP_HF_AT_FMT_INT , WICED_BT_HFP_HFP_CODEC_SET_EVT},
    {"BUSY"       , WICED_BT_HFP_HF_AT_FMT_NONE, WICED_BT_HFP_HF_NO_EVT},
    {"+VGS="      , WICED_BT_HFP_HF_AT_FMT_INT , WICED_BT_HFP_HF_VOLUME_CHANGE_EVT},
    {"+VGM="      , WICED_BT_HFP_HF_AT_FMT_INT , WICED_BT_HFP_HF_VOLUME_CHANGE_EVT},
    {"+BVRA="     , WICED_BT_HFP_HF_AT_FMT_INT , WICED_BT_HFP_HF_NO_EVT},
    {"+BRSF:"     , WICED_BT_HFP_HF_AT_FMT_INT , WICED_BT_HFP_HF_AG_FEATURE_SUPPORT_EVT},
    {""           , WICED_BT_HFP_HF_AT_FMT_STR , WICED_BT_HFP_HF_NO_EVT}
};

#if (WICED_BT_HFP_HF_VERSION >= WICED_BT_HFP_HF_VERSION_1_7 \
  && WICED_BT_HFP_HF_IND_SUPPORTED == TRUE)
/*******************************************************************************
** Function         wiced_bt_hfp_hf_get_free_indicator_index
** Description      This function returns the index of the
**                  first empty HF indicator structure
** Returns          Index of the empty HF indicator structure
*******************************************************************************/
static uint8_t wiced_bt_hfp_hf_get_free_indicator_index(wiced_bt_hfp_hf_scb_t *p_scb)
{
    uint8_t i=0;
    for (i=0; i<WICED_BT_HFP_HF_MAX_NUM_PEER_IND; i++)
    {
        if (p_scb->peer_ind[i].ind_id == 0)
        {
            return i;
        }
    }
    return WICED_BT_HFP_HF_MAX_NUM_PEER_IND;
}

/*******************************************************************************
** Function         wiced_bt_hfp_hf_get_indicator
** Description      This function returns the index of the HF indicator
**                  structure using the indicator id
** Returns          Index of the HF indicator structure
*******************************************************************************/
static uint8_t wiced_bt_hfp_hf_get_indicator_index(wiced_bt_hfp_hf_scb_t *p_scb,
    uint32_t ind_id)
{
    uint8_t i=0;
    for (i=0; i<WICED_BT_HFP_HF_MAX_NUM_PEER_IND; i++)
    {
        if (p_scb->peer_ind[i].ind_id == ind_id)
        {
            return i;
        }
    }
    return WICED_BT_HFP_HF_MAX_NUM_PEER_IND;
}
#endif

/*******************************************************************************
** Function         wiced_bt_hfp_hf_at_send_cmd
** Description      Send AT command to peer.
*******************************************************************************/
void wiced_bt_hfp_hf_at_send_cmd(wiced_bt_hfp_hf_scb_t *p_scb, uint8_t cmd,
    uint8_t arg_type, uint8_t arg_format, const char *p_arg, int16_t int_arg)
{
    char    buf[WICED_BT_HFP_HF_AT_MAX_LEN + 16];
    char    *p = buf;

    memset (buf, 0, (WICED_BT_HFP_HF_AT_MAX_LEN+16));

    *p++ = 'A';
    *p++ = 'T';

    /* copy result code string */
    strncpy(p, wiced_bt_hfp_hf_cmd_str[cmd].p_cmd, WICED_BT_HFP_HF_AT_MAX_LEN);
    p += strlen(wiced_bt_hfp_hf_cmd_str[cmd].p_cmd);

    if(arg_type == WICED_BT_HFP_HF_AT_SET)
    {
        *p++ = '=';

    }
    else if(arg_type == WICED_BT_HFP_HF_AT_READ)
    {
        *p++ = '?';

    }
    else if(arg_type == WICED_BT_HFP_HF_AT_TEST)
    {
        *p++ = '=';
        *p++ = '?';

    }

    /* copy argument if any */
    if (arg_format == WICED_BT_HFP_HF_AT_FMT_INT)
    {
        p += wiced_bt_hfp_hf_utils_itoa((uint16_t) int_arg, p);
    }
    else if (arg_format == WICED_BT_HFP_HF_AT_FMT_STR)
    {
        utl_strcpy(p, (char *)p_arg);
        p += strlen(p_arg);
    }

    /* finish with \r*/
    *p++ = '\r';

    wiced_bt_hfp_hf_at_cmd_queue_enqueue(p_scb, cmd, buf, (uint16_t)(p-buf));
}

/*******************************************************************************
** Function         wiced_bt_hsp_at_slc
** Description      Send initial sequence of AT commands for HSP device
*******************************************************************************/
void wiced_bt_hsp_at_slc( wiced_bt_hfp_hf_scb_t *p_scb )
{
    // if initiator send AT+CKPD=200
    if(p_scb->is_server == WICED_FALSE)
    {
        wiced_bt_hfp_hf_at_send_cmd(p_scb, WICED_BT_HFP_HF_CMD_CKPD, WICED_BT_HFP_HF_AT_SET,
                WICED_BT_HFP_HF_AT_FMT_INT, NULL, 200);
    }

    wiced_bt_hfp_hf_at_send_cmd(p_scb, WICED_BT_HFP_HF_CMD_VGS, WICED_BT_HFP_HF_AT_SET,
            WICED_BT_HFP_HF_AT_FMT_INT, NULL, p_scb->speaker_volume);

    wiced_bt_hfp_hf_at_send_cmd(p_scb, WICED_BT_HFP_HF_CMD_VGM, WICED_BT_HFP_HF_AT_SET,
            WICED_BT_HFP_HF_AT_FMT_INT, NULL, p_scb->mic_volume);
}

/*******************************************************************************
** Function         wiced_bt_hfp_hf_at_slc
** Description      Send initial sequence of AT commands
*******************************************************************************/
void wiced_bt_hfp_hf_at_slc(wiced_bt_hfp_hf_scb_t *p_scb)
{
    /* TODO: add timer also check features */

    switch(p_scb->slc_at_init_state)
    {
        case WICED_BT_HFP_HF_AT_SLC_STATE_INIT:
            wiced_bt_hfp_hf_at_send_cmd(p_scb, WICED_BT_HFP_HF_CMD_BRSF, WICED_BT_HFP_HF_AT_SET,
                WICED_BT_HFP_HF_AT_FMT_INT, NULL, (int16_t) (p_scb->feature_mask));
            break;
        case WICED_BT_HFP_HF_AT_SLC_STATE_BRSF:
#if (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE)
            if ((p_scb->feature_mask & WICED_BT_HFP_HF_FEATURE_CODEC_NEGOTIATION)&&
                (p_scb->peer_feature_mask & WICED_BT_HFP_AG_FEATURE_CODEC_NEGOTIATION))
            {
                /* Supporting CVSD and mSBC */
                wiced_bt_hfp_hf_at_send_cmd(p_scb, WICED_BT_HFP_HF_CMD_BAC, WICED_BT_HFP_HF_AT_SET,
                    WICED_BT_HFP_HF_AT_FMT_STR, "1,2", 0);
                break;
            }
            else
            {
                p_scb->slc_at_init_state++;
            }
        case WICED_BT_HFP_HF_AT_SLC_STATE_BAC:
            wiced_bt_hfp_hf_at_send_cmd(p_scb, WICED_BT_HFP_HF_CMD_CIND, WICED_BT_HFP_HF_AT_TEST,
                WICED_BT_HFP_HF_AT_FMT_NONE, NULL, 0);
#else
            wiced_bt_hfp_hf_at_send_cmd(p_scb, WICED_BT_HFP_HF_CMD_CIND, WICED_BT_HFP_HF_AT_TEST,
                WICED_BT_HFP_HF_AT_FMT_NONE, NULL, 0);
#endif
            break;
        case WICED_BT_HFP_HF_AT_SLC_STATE_CIND_TEST:
            wiced_bt_hfp_hf_at_send_cmd(p_scb, WICED_BT_HFP_HF_CMD_CIND, WICED_BT_HFP_HF_AT_READ,
                WICED_BT_HFP_HF_AT_FMT_NONE, NULL, 0);
            break;
        case WICED_BT_HFP_HF_AT_SLC_STATE_CIND_READ:
            wiced_bt_hfp_hf_at_send_cmd(p_scb, WICED_BT_HFP_HF_CMD_CMER, WICED_BT_HFP_HF_AT_SET,
                WICED_BT_HFP_HF_AT_FMT_STR, "3,0,0,1", 1);
            break;
        case WICED_BT_HFP_HF_AT_SLC_STATE_CMER:
            if((p_scb->feature_mask & WICED_BT_HFP_HF_FEATURE_3WAY_CALLING)&&
                (p_scb->peer_feature_mask & WICED_BT_HFP_AG_FEATURE_3WAY_CALLING))
            {
                /* Sent CHLD if local device and peer device support 3 way calling */
                wiced_bt_hfp_hf_at_send_cmd(p_scb, WICED_BT_HFP_HF_CMD_CHLD, WICED_BT_HFP_HF_AT_TEST,
                    WICED_BT_HFP_HF_AT_FMT_NONE, NULL, 0);
              break;
            }
            else
            {
                p_scb->slc_at_init_state++;
            }
        case WICED_BT_HFP_HF_AT_SLC_STATE_CHLD:
#if (WICED_BT_HFP_HF_VERSION >= WICED_BT_HFP_HF_VERSION_1_7 \
  && WICED_BT_HFP_HF_IND_SUPPORTED == TRUE)
            if((p_scb->feature_mask & WICED_BT_HFP_HF_FEATURE_HF_INDICATORS) &&
               (p_scb->peer_feature_mask & WICED_BT_HFP_AG_FEATURE_HF_INDICATORS))
            {
                wiced_bt_hfp_hf_at_send_cmd(p_scb, WICED_BT_HFP_HF_CMD_BIND, WICED_BT_HFP_HF_AT_SET,
                    WICED_BT_HFP_HF_AT_FMT_STR, WICED_BT_HFP_HF_IND_INFO, 0);
                break;
            }
            else
            {
                p_scb->slc_at_init_state++;
            }
        case WICED_BT_HFP_HF_AT_SLC_STATE_BIND_SET:
            if((p_scb->feature_mask & WICED_BT_HFP_HF_FEATURE_HF_INDICATORS) &&
               (p_scb->peer_feature_mask & WICED_BT_HFP_AG_FEATURE_HF_INDICATORS))
            {
                wiced_bt_hfp_hf_at_send_cmd(p_scb, WICED_BT_HFP_HF_CMD_BIND, WICED_BT_HFP_HF_AT_TEST,
                    WICED_BT_HFP_HF_AT_FMT_NONE, NULL, 0);
                break;
            }
            else
            {
                p_scb->slc_at_init_state++;
            }
        case WICED_BT_HFP_HF_AT_SLC_STATE_BIND_TEST:
            if((p_scb->feature_mask & WICED_BT_HFP_HF_FEATURE_HF_INDICATORS) &&
               (p_scb->peer_feature_mask & WICED_BT_HFP_AG_FEATURE_HF_INDICATORS))
            {
                wiced_bt_hfp_hf_at_send_cmd(p_scb, WICED_BT_HFP_HF_CMD_BIND, WICED_BT_HFP_HF_AT_READ,
                    WICED_BT_HFP_HF_AT_FMT_NONE, NULL, 0);
                break;
            }
            else
            {
                p_scb->slc_at_init_state++;
            }
        case WICED_BT_HFP_HF_AT_SLC_STATE_BIND_READ:
#endif
            if(p_scb->feature_mask & WICED_BT_HFP_HF_FEATURE_REMOTE_VOLUME_CONTROL)
            {
                wiced_bt_hfp_hf_at_send_cmd(p_scb, WICED_BT_HFP_HF_CMD_VGM, WICED_BT_HFP_HF_AT_SET,
                    WICED_BT_HFP_HF_AT_FMT_INT, NULL, p_scb->mic_volume);
                break;
            }
            else
            {
                p_scb->slc_at_init_state++;
            }
        case WICED_BT_HFP_HF_AT_SLC_STATE_VGM:
            if(p_scb->feature_mask & WICED_BT_HFP_HF_FEATURE_REMOTE_VOLUME_CONTROL)
            {
                wiced_bt_hfp_hf_at_send_cmd(p_scb, WICED_BT_HFP_HF_CMD_VGS, WICED_BT_HFP_HF_AT_SET,
                    WICED_BT_HFP_HF_AT_FMT_INT, NULL, p_scb->speaker_volume);
                break;
            }
            else
            {
                p_scb->slc_at_init_state++;
            }
        case WICED_BT_HFP_HF_AT_SLC_STATE_VGS:
            if(p_scb->feature_mask & WICED_BT_HFP_HF_FEATURE_CLIP_CAPABILITY)
            {
                /* enable Caller ID if application supports it */
                wiced_bt_hfp_hf_at_send_cmd(p_scb, WICED_BT_HFP_HF_CMD_CLIP, WICED_BT_HFP_HF_AT_SET,
                    WICED_BT_HFP_HF_AT_FMT_INT, NULL, 1);
                break;
            }
            else
            {
                p_scb->slc_at_init_state++;
            }
        case WICED_BT_HFP_HF_AT_SLC_STATE_CLIP:
            if(p_scb->feature_mask & WICED_BT_HFP_HF_FEATURE_3WAY_CALLING)
            {
                /* enable Call Wait if application supports it */
                wiced_bt_hfp_hf_at_send_cmd(p_scb, WICED_BT_HFP_HF_CMD_CCWA, WICED_BT_HFP_HF_AT_SET,
                    WICED_BT_HFP_HF_AT_FMT_INT, NULL, 1);
            }
            else
            {
                p_scb->slc_at_init_state++;
            }
            /* TODO: Let's assume that peer HF version is > 1.5 */
            /* TODO: As a server we are not doing a SDP for now */
#if 0
            /* Initialization is complete for HFP 1.0 */
            WICED_BT_TRACE("%s: Peer Profile Version: %x", __FUNCTION__,
                p_scb->peer_version);
            if (p_scb->peer_version < WICED_BT_HFP_HF_VERSION_1_5)
            {
                p_scb->slc_at_init_state = WICED_BT_HFP_HF_AT_SLC_STATE_DONE-1;
                break;
            }
            else
#endif
            {
            if(p_scb->feature_mask & WICED_BT_HFP_HF_FEATURE_3WAY_CALLING)
                break;
            }
        case WICED_BT_HFP_HF_AT_SLC_STATE_CCWA:
            wiced_bt_hfp_hf_at_send_cmd(p_scb, WICED_BT_HFP_HF_CMD_COPS, WICED_BT_HFP_HF_AT_SET,
                WICED_BT_HFP_HF_AT_FMT_STR, "3,0", 0);
            break;

        case WICED_BT_HFP_HF_AT_SLC_STATE_COPS:
            wiced_bt_hfp_hf_at_send_cmd(p_scb, WICED_BT_HFP_HF_CMD_CMEE, WICED_BT_HFP_HF_AT_SET,
                WICED_BT_HFP_HF_AT_FMT_INT, NULL, 1);
            break;
    }
    p_scb->slc_at_init_state++;
    if(WICED_BT_HFP_HF_AT_SLC_STATE_DONE == p_scb->slc_at_init_state)
    {
         wiced_bt_hfp_hf_slc_open(p_scb, NULL);
    }
}

/*******************************************************************************
** Function       wiced_bt_hfp_hf_indicator_event
** Description    Parse and update indicator event from AG
** Returns        None
*******************************************************************************/
static void wiced_bt_hfp_hf_indicator_event(wiced_bt_hfp_hf_scb_t *p_scb, char * ind)
{
    uint8_t id, val;
    wiced_bool_t call_state_notify = FALSE, device_state_change = FALSE;
    wiced_bt_hfp_hf_event_t event;
    wiced_bt_hfp_hf_event_data_t app_data;

    memset(&app_data, 0, sizeof(wiced_bt_hfp_hf_event_data_t));
    app_data.handle = p_scb->rfcomm_handle;

    /* Skip any spaces in the front */
    while ( *ind == ' ' ) ind++;

    id = *ind - '0';
    ind++;
    while ( (*ind == ' ') || (*ind == ',')) ind++;
    val = *ind - '0';

    if (id == p_scb->call_ind_id)
    {
        call_state_notify = TRUE;
        if (val == 1)
        {
            /* Handle the case where call was just answered */
            p_scb->call_setup_ind_val = 0;
        }
        else if (val == 0)
        {
            /* Per the errata 2043, call=0 implies there is no held/active call
            ** Handle the case where phone doesnt send callheld=0
            ** https://www.bluetooth.org/errata/errata_view.cfm?errata_id=2043
            **/
           p_scb->callheld_ind_val = 0;
        }

        if (val <= TRUE)
        {
            p_scb->call_ind_val = val;
        }
    }
    else if (id == p_scb->call_setup_ind_id)
    {
        call_state_notify = TRUE;

        if (val <= WICED_BT_HFP_HF_CALLSETUP_STATE_WAITING)
        {
             p_scb->call_setup_ind_val = val;
        }
    }
    else if (id == p_scb->callheld_ind_id)
    {
        if (val == 0)
        {   // No calls held
            if (p_scb->callheld_ind_val == 1)
            {   // A call is held.
                /* Set call to active. */
                p_scb->call_ind_val = 1;
            }
            else
            {   // No existent calls are held.
                p_scb->call_ind_val = 0;
            }

            /* Set call held to 0. */
            p_scb->callheld_ind_val = 0;
        }
        else if (val == 1)
        {   // Call is placed on hold or active/held calls swapped
            /* AG has both an active and a held call */
            p_scb->call_ind_val     = 1;
            p_scb->callheld_ind_val = 1;
        }
        else if (val == 2)
        {   // Call on hold, no active call
            p_scb->call_ind_val     = 0;
            p_scb->callheld_ind_val = 1;
        }
        else
        {   // Not defined yet.
            return;
        }

        call_state_notify = TRUE;
    }
    else if (id == p_scb->service_state_ind_id)
    {
        if (val <= WICED_BT_HFP_HF_SERVICE_STATE_AVAILABLE)
        {
            device_state_change = TRUE;
            app_data.service_state = val;
            event = WICED_BT_HFP_HF_SERVICE_STATE_EVT;
        }
    }
    else if (id == p_scb->service_type_ind_id)
    {
        if (val <= WICED_BT_HFP_HF_SERVICE_TYPE_ROAMING)
        {
            device_state_change = TRUE;
            app_data.service_type = val;
            event = WICED_BT_HFP_HF_SERVICE_TYPE_EVT;
        }
    }
    else if (id == p_scb->battery_level_ind_id)
    {
        if (val <= WICED_BT_HFP_HF_MAX_BATTERY_CHARGE_LEVEL)
        {
            device_state_change = TRUE;
            app_data.battery_level = val;
            event = WICED_BT_HFP_HF_BATTERY_STATUS_IND_EVT;
        }
    }
    else if (id == p_scb->rssi_ind_id)
    {
        if (val <= WICED_BT_HFP_HF_MAX_RSSI_LEVEL)
        {
            device_state_change = TRUE;
            app_data.rssi = val;
            event = WICED_BT_HFP_HF_RSSI_IND_EVT;
        }
    }

    if(call_state_notify)
    {
        app_data.call_data.active_call_present = p_scb->call_ind_val;
        app_data.call_data.held_call_present = p_scb->callheld_ind_val;
        app_data.call_data.setup_state = p_scb->call_setup_ind_val;
        wiced_bt_hfp_hf_cb.p_event_cback(WICED_BT_HFP_HF_CALL_SETUP_EVT, &app_data);
    }

    if(device_state_change)
    {
        wiced_bt_hfp_hf_cb.p_event_cback(event, &app_data);
    }
}

static wiced_bool_t wiced_bt_hfp_check_ind_status( char * ind, uint8_t pos, uint8_t cmp, uint8_t * p_val )
{
    wiced_bool_t result = FALSE;
    int i = 0;
    for(i=0; i< strlen(ind) ; i++)
    {
        if(!pos)
        {
            if ((ind[i] - '0') <= cmp)
            {
                *p_val = ind[i] - '0';
                result = TRUE;
            }
            break;
        }
        else if(ind[i] == ',')
            pos--;
    }
    return result;
}
/*******************************************************************************
** Function         wiced_bt_hfp_hf_set_indicator_status
** Description      sets the current indicator
** Returns          void
*******************************************************************************/
static void wiced_bt_hfp_hf_set_indicator_status(wiced_bt_hfp_hf_scb_t *p_scb, char * ind)
{
    uint8_t i=0, pos=0;
    wiced_bt_hfp_hf_event_data_t app_data;

    p_scb->call_ind_val = 0;
    p_scb->call_setup_ind_val = 0;
    p_scb->callheld_ind_val = 0;

    memset(&app_data, 0, sizeof(wiced_bt_hfp_hf_event_data_t));
    app_data.handle = p_scb->rfcomm_handle;

    /* Skip any spaces in the front */
    while ( *ind == ' ' ) ind++;

    /* Get "call" indicator*/
    pos = p_scb->call_ind_id -1;
    if ( wiced_bt_hfp_check_ind_status( ind, pos, TRUE, (uint8_t*)&app_data.call_data.active_call_present ) )
    {
        p_scb->call_ind_val = app_data.call_data.active_call_present;
    }

    /* Get "callsetup" indicator*/
    pos = p_scb->call_setup_ind_id -1;
    if ( wiced_bt_hfp_check_ind_status( ind, pos, WICED_BT_HFP_HF_CALLSETUP_STATE_WAITING, (uint8_t *)&app_data.call_data.setup_state ) )
    {
        p_scb->call_setup_ind_val = app_data.call_data.setup_state;
    }

    /* Get "callheld" indicator*/
    pos = p_scb->callheld_ind_id -1;
    if ( wiced_bt_hfp_check_ind_status( ind, pos, TRUE, (uint8_t*)&app_data.call_data.held_call_present ) )
    {
        p_scb->callheld_ind_val = app_data.call_data.held_call_present;
    }

    /* Aggregate and pass on call information to the application */
    wiced_bt_hfp_hf_cb.p_event_cback(WICED_BT_HFP_HF_CALL_SETUP_EVT, &app_data);

    /* Get "service" indicator*/
    pos = p_scb->service_state_ind_id -1;
    if ( wiced_bt_hfp_check_ind_status( ind, pos, WICED_BT_HFP_HF_SERVICE_STATE_AVAILABLE, (uint8_t *)&app_data.service_state ) )
    {
        p_scb->service_state_ind_id = app_data.service_state;
        wiced_bt_hfp_hf_cb.p_event_cback(WICED_BT_HFP_HF_SERVICE_STATE_EVT, &app_data);
    }

    /* Get "roam" indicator*/
    pos = p_scb->service_type_ind_id -1;
    if ( wiced_bt_hfp_check_ind_status( ind, pos, WICED_BT_HFP_HF_SERVICE_TYPE_ROAMING, (uint8_t *)&app_data.service_type ) )
    {
        wiced_bt_hfp_hf_cb.p_event_cback(WICED_BT_HFP_HF_SERVICE_TYPE_EVT, &app_data);
    }

    /* Get "signal" indicator*/
    pos = p_scb->rssi_ind_id -1;
    if ( wiced_bt_hfp_check_ind_status( ind, pos, WICED_BT_HFP_HF_MAX_RSSI_LEVEL, &app_data.rssi ) )
    {
        wiced_bt_hfp_hf_cb.p_event_cback(WICED_BT_HFP_HF_RSSI_IND_EVT, &app_data);
    }

    /* Get "battchg" indicator*/
    pos = p_scb->battery_level_ind_id -1;
    if ( wiced_bt_hfp_check_ind_status( ind, pos, WICED_BT_HFP_HF_MAX_BATTERY_CHARGE_LEVEL, &app_data.battery_level ) )
    {
        wiced_bt_hfp_hf_cb.p_event_cback(WICED_BT_HFP_HF_BATTERY_STATUS_IND_EVT, &app_data);
    }
}

/*******************************************************************************
** Function         wiced_bt_hfp_hf_find_indicator_id
** Description      Parses the indicator string and finds the position of a field
** Returns          Index in the string
*******************************************************************************/
static uint8_t wiced_bt_hfp_hf_find_indicator_id(char *ind, char *field)
{
    uint16_t string_len = (uint16_t) strlen(ind);
    uint8_t i, id = 0;
    wiced_bool_t skip = FALSE;

    for(i=0; i< string_len ; i++)
    {
        if(ind[i] == '"')
        {
            /* robustness tester sends empty string, skip parsing it */
            if (ind[i+1] == '"')
                continue;
            if(!skip)
            {
                id++;
                if(!memcmp(&ind[i+1], field,strlen(field)) && (ind[i+1+strlen(field)] == '"'))
                {
                    WICED_BTHFP_TRACE("%s id:%s index:%d", __FUNCTION__, field, id);
                    return id;
                }
                else
                {
                    /* skip the next " */
                    skip = TRUE;
                }
            }
            else
            {
                skip = FALSE;
            }
        }
    }
    return 0;
}

/*******************************************************************************
** Function         wiced_bt_hfp_hf_decode_indicator
** Description      Process the indicator string and set the indicator ids
** Returns          void
*******************************************************************************/
static void wiced_bt_hfp_hf_decode_indicator(wiced_bt_hfp_hf_scb_t *p_scb, char * ind)
{
    p_scb->call_ind_id = wiced_bt_hfp_hf_find_indicator_id(ind,
        WICED_BT_HFP_HF_CALL_IND_NAME);
    p_scb->call_setup_ind_id = wiced_bt_hfp_hf_find_indicator_id(ind,
        WICED_BT_HFP_HF_CALLSETUP_IND_NAME);
    if(!p_scb->call_setup_ind_id)
    {
        p_scb->call_setup_ind_id = wiced_bt_hfp_hf_find_indicator_id(ind,
            WICED_BT_HFP_HF_CALL_SETUP_IND_NAME);
    }
    p_scb->service_state_ind_id = wiced_bt_hfp_hf_find_indicator_id(ind,
        WICED_BT_HFP_HF_SERVICE_IND_NAME);
    p_scb->battery_level_ind_id = wiced_bt_hfp_hf_find_indicator_id(ind,
        WICED_BT_HFP_HF_BATTERY_IND_NAME);
    p_scb->callheld_ind_id = wiced_bt_hfp_hf_find_indicator_id(ind,
        WICED_BT_HFP_HF_CALL_HELD_IND_NAME);
    p_scb->rssi_ind_id = wiced_bt_hfp_hf_find_indicator_id(ind,
        WICED_BT_HFP_HF_SIGNAL_IND_NAME);
    p_scb->service_type_ind_id = wiced_bt_hfp_hf_find_indicator_id(ind,
        WICED_BT_HFP_HF_ROAM_IND_NAME);
}


/*******************************************************************************
** Function         wiced_bt_hfp_hf_at__cback
** Description      AT command processing callback for Handsfree Profile.
*******************************************************************************/
void wiced_bt_hfp_hf_at_cback(void *user_data, uint16_t res, char *p_arg)
{
    wiced_bt_hfp_hf_event_data_t app_data;
    wiced_bt_hfp_hf_event_t      event = 0;
    uint8_t                      result = 0;
    wiced_bt_hfp_hf_scb_t       *p_scb = (wiced_bt_hfp_hf_scb_t *) user_data;
    uint16_t uuid;

#if (WICED_BT_HFP_HF_VERSION >= WICED_BT_HFP_HF_VERSION_1_7 \
  && WICED_BT_HFP_HF_IND_SUPPORTED == TRUE)
    uint8_t      i, index;
    wiced_bool_t find_ind = FALSE;
#endif

    if( p_scb->is_server)
    {
        uuid = wiced_bt_hfp_hf_get_uuid_by_handle(p_scb->rfcomm_handle);
    }
    else
    {
        uuid = ( wiced_bt_hfp_hf_cb.ag_profile_uuid == UUID_SERVCLASS_AG_HANDSFREE ) ? UUID_SERVCLASS_HF_HANDSFREE : UUID_SERVCLASS_HEADSET;
    }

    WICED_BTHFP_TRACE("[%s]\n", __FUNCTION__);

    if (res < WICED_BT_HFP_HF_RES_MAX)
    {
        event = wiced_bt_hfp_hf_res[res].app_evt;
    }

    memset(&app_data,0,sizeof(wiced_bt_hfp_hf_event_data_t));
    app_data.handle = p_scb->rfcomm_handle;

    switch (res)
    {
        case WICED_BT_HFP_HF_RES_OK:
        case WICED_BT_HFP_HF_RES_ERROR:
            /* If application needs to know we have to provide the AT command which failed */
            result = wiced_bt_hfp_hf_at_cmd_queue_handle_res(p_scb);
            /* Do not send result if the command was not initiated from the app. */
            if ((result == 0xFF) || !p_scb->slc_open)
                event = 0;
            break;
        case WICED_BT_HFP_HF_RES_CMEE:
            result = wiced_bt_hfp_hf_at_cmd_queue_handle_res(p_scb);
            /* Do not send result if the command was not initiated from the app. */
            if ((result == 0xFF) || !p_scb->slc_open)
            {
                event = 0;
            }
            else
            {
                result = (uint8_t) wiced_bt_hfp_hf_utils_str2int(p_arg);
                app_data.error_code = (uint8_t)result;
            }
            break;
        case WICED_BT_HFP_HF_RES_VGM:
            result = (uint8_t) wiced_bt_hfp_hf_utils_str2int(p_arg);
            p_scb->mic_volume = (uint8_t)result;
            app_data.volume.level = p_scb->mic_volume;
            wiced_bt_hfp_hf_cb.config_data.mic_volume = p_scb->mic_volume;
            app_data.volume.type = WICED_BT_HFP_HF_MIC;
            break;
        case WICED_BT_HFP_HF_RES_VGS:
            result = (uint8_t) wiced_bt_hfp_hf_utils_str2int(p_arg);
            p_scb->speaker_volume = (uint8_t)result;
            app_data.volume.level = p_scb->speaker_volume;
            wiced_bt_hfp_hf_cb.config_data.speaker_volume = p_scb->speaker_volume;
            app_data.volume.type = WICED_BT_HFP_HF_SPEAKER;
            break;
        case WICED_BT_HFP_HF_RES_VGM_EQ:
            result = (uint8_t)wiced_bt_hfp_hf_utils_str2int(p_arg);
            p_scb->mic_volume = (uint8_t)result;
            app_data.volume.level = p_scb->mic_volume;
            wiced_bt_hfp_hf_cb.config_data.mic_volume = p_scb->mic_volume;
            app_data.volume.type = WICED_BT_HFP_HF_MIC;
            break;
        case WICED_BT_HFP_HF_RES_VGS_EQ:
            result = (uint8_t)wiced_bt_hfp_hf_utils_str2int(p_arg);
            p_scb->speaker_volume = (uint8_t)result;
            app_data.volume.level = p_scb->speaker_volume;
            wiced_bt_hfp_hf_cb.config_data.speaker_volume = p_scb->speaker_volume;
            app_data.volume.type = WICED_BT_HFP_HF_SPEAKER;
            break;
        case WICED_BT_HFP_HF_RES_CLIP:
        case WICED_BT_HFP_HF_RES_BINP:
            {
                wiced_bt_hfp_hf_caller_num_t token;
                int i,j=0;
                // p_arg have caller number and type separated by ','

                //Copy number
                for (i=0;i<strlen(p_arg) && p_arg[i]!=',';i++)
                    token[i]=p_arg[i];
                token[i++] = '\0';

                strncpy(app_data.clip.caller_num, token, strlen(token)+1);
                //Copy type
                for (;i<strlen(p_arg);i++,j++)
                    token[j]=p_arg[i];
                token[j] = '\0';
                app_data.clip.type = (uint8_t) wiced_bt_hfp_hf_utils_str2int(token);
            }

            break;
        case WICED_BT_HFP_HF_RES_CLCC:
            {
               wiced_bt_hfp_hf_caller_num_t token;
               int i = 0,j=0;

               // Skip other symbols
               for(; p_arg[i]<'0' || p_arg[i]>'9'; i++);
               app_data.active_call.idx = p_arg[i++] - '0'; i++;
               app_data.active_call.dir = p_arg[i++] - '0'; i++;
               app_data.active_call.status = p_arg[i++] - '0'; i++;
               app_data.active_call.mode = p_arg[i++] - '0'; i++;
               app_data.active_call.is_conference = p_arg[i++] - '0'; i++;

               if ( i < strlen(p_arg) )
               {
                   // Skip other symbols
                   for(; p_arg[i]<'0' || p_arg[i]>'9'; i++);

                   //Copy number
                   for (j=0; i<strlen(p_arg) && ( p_arg[i]>='0' && p_arg[i]<='9') ; i++)
                   {
                       token[j++] = p_arg[i];
                   }
                   token[j++] = '\0';
                   strncpy(app_data.active_call.num, token, j);

                   // Skip other symbols
                   for(; p_arg[i]<'0' || p_arg[i]>'9';i++);

                   //Copy type
                   for (j=0; i<strlen(p_arg) && ( p_arg[i]>='0' && p_arg[i]<='9' ); i++)
                   {
                       token[j++] = p_arg[i];
                   }
                   token[j++] = '\0';
                   app_data.active_call.type = (uint8_t)wiced_bt_hfp_hf_utils_str2int(token);
               }
            }
            break;
        case WICED_BT_HFP_HF_RES_CNUM:
            strncpy(app_data.cnum_data, p_arg, strlen(p_arg));
            break;

        case WICED_BT_HFP_HF_RES_CCWA:
        case WICED_BT_HFP_HF_RES_CHLD:
        case WICED_BT_HFP_HF_RES_COPS:
            break;
        case WICED_BT_HFP_HF_RES_CIND:
            if (p_scb->ind_string_received)
            {
                 wiced_bt_hfp_hf_set_indicator_status(p_scb, p_arg);
            }
            else
            {
                 p_scb->ind_string_received = TRUE;
                 wiced_bt_hfp_hf_decode_indicator(p_scb, p_arg);
            }
            break;
        case WICED_BT_HFP_HF_RES_CIEV:
            wiced_bt_hfp_hf_indicator_event(p_scb, p_arg);
            break;

        case WICED_BT_HFP_HF_RES_BVRA_EQ:
        case WICED_BT_HFP_HF_RES_BSIR:
        case WICED_BT_HFP_HF_RES_BTRH:
            result = (uint8_t)wiced_bt_hfp_hf_utils_str2int(p_arg);
            break;
#if (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE )
        case WICED_BT_HFP_HF_RES_BCS:
            result = (uint8_t)wiced_bt_hfp_hf_utils_str2int(p_arg);
            app_data.selected_codec = result;
            // Send AT+BCS=<Codec ID> to AG
            wiced_bt_hfp_hf_at_send_cmd( p_scb, WICED_BT_HFP_HF_CMD_BCS,
                                WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_INT, NULL, result );
            break;
#endif

        case WICED_BT_HFP_HF_RES_BVRA:
            /* If both devices support Voice Recognition */
            if ((p_scb->feature_mask & WICED_BT_HFP_HF_FEATURE_VOICE_RECOGNITION_ACTIVATION) &&
                (p_scb->peer_feature_mask & WICED_BT_HFP_AG_FEATURE_VOICE_RECOGNITION_ACTIVATION))
            {
                /* Extract Voice Recognition state and send it to application */
                result = (uint8_t)wiced_bt_hfp_hf_utils_str2int(p_arg);
                app_data.voice_recognition = (uint8_t)result;
            }
            else
            {
                WICED_BTHFP_ERROR("Err: Unexpected BVRA cmd\n");
                event = 0;
            }
            break;

        case WICED_BT_HFP_HF_RES_BRSF:
            p_scb->peer_feature_mask = wiced_bt_hfp_hf_utils_str2int(p_arg);
            WICED_BTHFP_TRACE("%s: peer_feature_mask: %d", __FUNCTION__,
                p_scb->peer_feature_mask);
            app_data.ag_feature_flags = p_scb->peer_feature_mask;
            WICED_BTHFP_TRACE("\t        Peer Feature                                Supported \n");
            WICED_BTHFP_TRACE("\t WICED_BT_HFP_AG_FEATURE_3WAY_CALLING                   %d \n", (app_data.ag_feature_flags & WICED_BT_HFP_AG_FEATURE_3WAY_CALLING) ? 1 : 0 );
            WICED_BTHFP_TRACE("\t WICED_BT_HFP_AG_FEATURE_ECNR                           %d \n", (app_data.ag_feature_flags & WICED_BT_HFP_AG_FEATURE_ECNR) ? 1 : 0 );
            WICED_BTHFP_TRACE("\t WICED_BT_HFP_AG_FEATURE_VOICE_RECOGNITION_ACTIVATION   %d \n", (app_data.ag_feature_flags & WICED_BT_HFP_AG_FEATURE_VOICE_RECOGNITION_ACTIVATION) ? 1 : 0 );
            WICED_BTHFP_TRACE("\t WICED_BT_HFP_AG_FEATURE_INBAND_RING_TONE_CAPABILITY    %d \n", (app_data.ag_feature_flags & WICED_BT_HFP_AG_FEATURE_INBAND_RING_TONE_CAPABILITY) ? 1 : 0 );
            WICED_BTHFP_TRACE("\t WICED_BT_HFP_AG_FEATURE_ATTACH_NUMBER_TO_VOICE_TAG     %d \n", (app_data.ag_feature_flags & WICED_BT_HFP_AG_FEATURE_ATTACH_NUMBER_TO_VOICE_TAG) ? 1 : 0 );
            WICED_BTHFP_TRACE("\t WICED_BT_HFP_AG_FEATURE_ABILITY_TO_REJECT_CALL         %d \n", (app_data.ag_feature_flags & WICED_BT_HFP_AG_FEATURE_ABILITY_TO_REJECT_CALL) ? 1 : 0 );
            WICED_BTHFP_TRACE("\t WICED_BT_HFP_AG_FEATURE_ENHANCED_CALL_STATUS           %d \n", (app_data.ag_feature_flags & WICED_BT_HFP_AG_FEATURE_ENHANCED_CALL_STATUS) ? 1 : 0 );
            WICED_BTHFP_TRACE("\t WICED_BT_HFP_AG_FEATURE_ENHANCED_CALL_CONTROL          %d \n", (app_data.ag_feature_flags & WICED_BT_HFP_AG_FEATURE_ENHANCED_CALL_CONTROL) ? 1 : 0 );
            WICED_BTHFP_TRACE("\t WICED_BT_HFP_AG_FEATURE_EXTENDED_ERROR_RESULT_CODES    %d \n", (app_data.ag_feature_flags & WICED_BT_HFP_AG_FEATURE_EXTENDED_ERROR_RESULT_CODES) ? 1 : 0 );
            WICED_BTHFP_TRACE("\t WICED_BT_HFP_AG_FEATURE_CODEC_NEGOTIATION              %d \n", (app_data.ag_feature_flags & WICED_BT_HFP_AG_FEATURE_CODEC_NEGOTIATION) ? 1 : 0 );
            WICED_BTHFP_TRACE("\t WICED_BT_HFP_AG_FEATURE_HF_INDICATORS                  %d \n", (app_data.ag_feature_flags & WICED_BT_HFP_AG_FEATURE_HF_INDICATORS) ? 1 : 0 );
            WICED_BTHFP_TRACE("\t WICED_BT_HFP_AG_FEATURE_ESCO_S4_T2_SETTINGS_SUPPORT    %d \n", (app_data.ag_feature_flags & WICED_BT_HFP_AG_FEATURE_ESCO_S4_T2_SETTINGS_SUPPORT) ? 1 : 0 );
            WICED_BTHFP_TRACE("\t WICED_BT_HFP_AG_FEATURE_ENHANCED_VOICE_RECOGNITION     %d \n", (app_data.ag_feature_flags & WICED_BT_HFP_AG_FEATURE_ENHANCED_VOICE_RECOGNITION) ? 1 : 0 );
            break;
#if (WICED_BT_HFP_HF_VERSION >= WICED_BT_HFP_HF_VERSION_1_7 \
  && WICED_BT_HFP_HF_IND_SUPPORTED == TRUE)
        case WICED_BT_HFP_HF_RES_BIND:


            if (p_scb->slc_at_init_state == WICED_BT_HFP_HF_AT_SLC_STATE_BIND_TEST)
            {   /* Only in the BIND test case, the incoming cmd is in
                   the format of +BIND: (ind1,ind2....) */
                for (i = 0; i < WICED_BT_HFP_HF_AT_MAX_LEN; i++)
                {
                    /* Skip comma, white space and parathesis */
                    if (p_arg[i] == ',' || p_arg[i] == ' ' ||
                        p_arg[i] == '(' || p_arg[i] == ')')
                        continue;

                    if (p_arg[i] == '\0')
                        break;

                    index = wiced_bt_hfp_hf_get_free_indicator_index(p_scb);
                    if (index != WICED_BT_HFP_HF_MAX_NUM_PEER_IND)
                    {
                        p_scb->peer_ind[index].ind_id = (p_arg[i] - '0');
                    }
                }
            }
            else
            {
                /* The incoming cmd is in the format of +BIND: ind, state */
                for (i = 0; i < WICED_BT_HFP_HF_AT_MAX_LEN; i++)
                {
                    /* Skip comma, white space and parathesis */
                    if (p_arg[i] == ',' || p_arg[i] == ' ' )
                        continue;

                    if (p_arg[i] == '\0')
                        break;

                    if (find_ind == FALSE)
                    {
                        /* Find the index with ind_id */
                        index = wiced_bt_hfp_hf_get_indicator_index(p_scb, p_arg[i] - '0');
                        find_ind = TRUE;
                    }
                    else
                    {
                        /* Found index, change the is_enable associated with the ind_id */
                        if (index != WICED_BT_HFP_HF_MAX_NUM_PEER_IND)
                        {
                            p_scb->peer_ind[index].is_enable = (p_arg[i] - '0');
                            app_data.bind_data.ind_id = index;
                            app_data.bind_data.ind_value = p_scb->peer_ind[index].is_enable;
                        }
                    }
                }
            }
            break;
#endif
        default:
            break;
    }

    /* If service level connection is not fully established send the next AT command */
    if((p_scb->slc_at_init_state != WICED_BT_HFP_HF_AT_SLC_STATE_DONE)
        && ((res == WICED_BT_HFP_HF_RES_OK) || (res == WICED_BT_HFP_HF_RES_ERROR))
        && (uuid != UUID_SERVCLASS_HEADSET))
    {
        wiced_bt_hfp_hf_at_slc(p_scb);
    }

    /* Call callback */
    if (event != 0)
    {
        (*wiced_bt_hfp_hf_cb.p_event_cback)(event, &app_data);
    }
}

/*******************************************************************************
** Function         bta_hs_at_err_cback
** Description      AT command parser error callback.
*******************************************************************************/
void wiced_bt_hfp_hf_at_err_cback(void *user_data, wiced_bool_t unknown, char *p_arg)
{
    //TODO
    //wiced_bt_hfp_hf_scb_t *p_scb = (wiced_bt_hfp_hf_scb_t *) user_data;
    //WICED_BT_TRACE("%s: scb:%p", __FUNCTION__, p_scb);
}



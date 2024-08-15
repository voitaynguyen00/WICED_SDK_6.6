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
 * WICED HID Host interface
 *
 */

#pragma once

#include <stdint.h>
#include "wiced_bt_hidh.h"
#include "wiced_bt_sdp.h"


/*
 * Definitions
 */
#ifndef WICED_BT_HIDH_NUM_DEV_MAX
#define WICED_BT_HIDH_NUM_DEV_MAX   3       /* Maximum number of HID Devices */
#endif


typedef enum
{
    WICED_BT_HIDH_STATE_FREE = 0,           /* This device is Free */
    WICED_BT_HIDH_STATE_REGISTERED,         /* This device is Registered */
    WICED_BT_HIDH_STATE_IN_CONNECTING,      /* This device is Connecting (incoming connection) */
    WICED_BT_HIDH_STATE_OUT_CONNECTING,     /* This device is Connecting (outgoing connection) */
    WICED_BT_HIDH_STATE_CONNECTED,          /* This device is Connected */
    WICED_BT_HIDH_STATE_DISCONNECTING,      /* This device is Disconnecting (locally initiated) */
} wiced_bt_hidh_dev_state_t;

typedef enum
{
    WICED_BT_HIDH_REQUEST_IDLE = 0,           /* No pending Request */
    WICED_BT_HIDH_REQUEST_SET_PROTOCOL,       /* SetProtocol Request Pending */
    WICED_BT_HIDH_REQUEST_SET_REPORT,         /* SetReport Request Pending */
    WICED_BT_HIDH_REQUEST_GET_REPORT,         /* GetProtocol Request Pending */
} wiced_bt_hidh_request_t;

/*
 * Types
 */
typedef struct
{
    uint16_t          ctrl_cid;
    uint16_t          ctrl_mtu;
    uint16_t          intr_cid;
    uint16_t          intr_mtu;
} wiced_bt_hidh_con_t;

typedef struct
{
    wiced_bool_t                added;      /* Has this device been added (allowed to reconnect) */
    wiced_bt_hidh_dev_state_t   state;      /* Item/device state */
    uint16_t                    handle;     /* HIDH Connection Handle. */
    wiced_bt_device_address_t   bdaddr;     /* BdAddr of the peer HID Device. */
    wiced_bt_hidh_con_t         connection; /* HID (L2CAP) Connection information */
    wiced_bt_hidh_request_t     request;    /* Pending Request */
    uint8_t                     report_id;  /* Pending ReportId (if request is Set/GetReport) */
    wiced_bool_t                boot_supported; /* TRUE if peer device supports BOOT Protocol */
} wiced_bt_hidh_dev_t;

typedef struct
{
    wiced_bt_hidh_cback_t       *p_cback;   /* Application callback */
    wiced_bt_sdp_discovery_db_t *p_sdp_db;  /* SDP Database */
    wiced_bt_hidh_dev_t         *p_dev_sdp; /* Pointer to the devices which initiated SDP */
    wiced_bt_hidh_dev_t         devices[WICED_BT_HIDH_NUM_DEV_MAX];

} wiced_bt_hidh_cb_t;

/*
 * Globals
 */
extern wiced_bt_hidh_cb_t wiced_bt_hidh_cb;

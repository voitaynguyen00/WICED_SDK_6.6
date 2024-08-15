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

#include "wiced.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_ble_hidh.h"

/******************************************************
 *                     Constants
 ******************************************************/
/* Enable this compile option to enable BLE HID Debug traces */
//#define WICED_BT_BLE_HIDH_TRACE_ENABLED

/* BLE HID Trace macro(s) */
#ifdef WICED_BT_BLE_HIDH_TRACE_ENABLED
/* WICED_BT_BLE_HIDH_TRACE can be enabled/disabled */
#define WICED_BT_BLE_HIDH_TRACE(format, ...)        WICED_BT_TRACE("%s " format, __FUNCTION__, ##__VA_ARGS__)
#else
#define WICED_BT_BLE_HIDH_TRACE(...)
#endif

/* WICED_BT_BLE_HIDH_TRACE_ERR is always enabled */
#define WICED_BT_BLE_HIDH_TRACE_ERR(format, ...)    WICED_BT_TRACE("Err: %s " format, __FUNCTION__, ##__VA_ARGS__)

typedef enum
{
    WICED_BT_BLE_HIDH_GATTC_STATE_EMPTY = 0,
    WICED_BT_BLE_HIDH_GATTC_STATE_SEARCHING_SRV,
    WICED_BT_BLE_HIDH_GATTC_STATE_SEARCHING_CHAR,
    WICED_BT_BLE_HIDH_GATTC_STATE_SEARCHING_CHAR_DESC,
    WICED_BT_BLE_HIDH_GATTC_STATE_SEARCHING_REPORT_REF,
    WICED_BT_BLE_HIDH_GATTC_STATE_CONFIGURE_NOTIFICATION,
    WICED_BT_BLE_HIDH_GATTC_STATE_DONE
} wiced_bt_ble_hidh_gattc_state_t;

/******************************************************
 *                     Structures
 ******************************************************/
/* Structure containing BLE HID GATT Client information */
typedef struct
{
    wiced_bt_ble_hidh_gattc_state_t db_state;           /* GATT State machine */
    uint16_t start_handle;                              /* First HID GATT Handle */
    uint16_t end_handle;                                /* Last HID GATT Handle */

    uint16_t search_char_desc_index;                    /* Current Searched Descriptor Index */

    uint8_t characteristics_nb;                         /* Number of Characteristics in table */
    wiced_bt_ble_hidh_gatt_char_t characteristics[WICED_BT_BLE_HIDH_DEV_CHAR_MAX];

    uint8_t report_descs_nb;                            /* Number of Report Descriptors in table */
    wiced_bt_ble_hidh_gatt_report_t report_descs[WICED_BT_BLE_HIDH_REPORT_DESC_MAX];

    uint8_t report_val_hdl_nb;                          /* Number of Report Report Val Handle in table */
    uint16_t report_val_hdl[WICED_BT_BLE_HIDH_REPORT_DESC_MAX];

    uint8_t client_char_configs_nb;                     /* Number of Report Client Char Config in table */
    uint16_t client_char_configs[WICED_BT_BLE_HIDH_REPORT_DESC_MAX];
} wiced_bt_ble_hidh_gattc_dev_t;

/* Device state (bitfield) */
#define WICED_BT_BLE_HIDH_DEV_STATE_ALLOCATED        0x01
#define WICED_BT_BLE_HIDH_DEV_STATE_CONNECTED        0x02
#define WICED_BT_BLE_HIDH_DEV_STATE_ADDED            0x04
#define WICED_BT_BLE_HIDH_DEV_STATE_ENCRYPTED        0x08
#define WICED_BT_BLE_HIDH_DEV_STATE_OP_READ_DESC     0x10
#define WICED_BT_BLE_HIDH_DEV_STATE_OP_SET_REPORT    0x20
#define WICED_BT_BLE_HIDH_DEV_STATE_OP_GET_REPORT    0x40
#define WICED_BT_BLE_HIDH_DEV_STATE_OP_SET_PROTOCOL  0x80
#define WICED_BT_BLE_HIDH_DEV_STATE_OP_PENDING_MSK   (WICED_BT_BLE_HIDH_DEV_STATE_OP_READ_DESC |  \
                                                      WICED_BT_BLE_HIDH_DEV_STATE_OP_SET_REPORT | \
                                                      WICED_BT_BLE_HIDH_DEV_STATE_OP_GET_REPORT | \
                                                      WICED_BT_BLE_HIDH_DEV_STATE_OP_SET_PROTOCOL)
typedef uint8_t wiced_bt_ble_hidh_dev_state_t;

/* WakeUp pattern */
typedef struct
{
    /* ReportId 0 is invalid. Use this field to know if this element is in use */
    uint8_t report_id;
    uint16_t length;
    uint8_t pattern[WICED_BT_BLE_HIDH_WAKEUP_PATTERN_LEN_MAX];
} wiced_bt_ble_hidh_wakeup_pattern_t;

/* Structure containing the in information of a peer HID BLE Device */
typedef struct
{
    wiced_bt_device_address_t bdaddr;
    wiced_bt_ble_address_type_t addr_type;
    uint16_t conn_id;
    wiced_bt_ble_hidh_dev_state_t dev_state_msk;
    wiced_bt_ble_hidh_gattc_dev_t database;
    wiced_bt_ble_hidh_wakeup_pattern_t wakeup_patterns[WICED_BT_BLE_HIDH_WAKEUP_PATTERN_NB_MAX];
} wiced_bt_ble_hidh_dev_t;

/* Structure containing the in information of a peer HID BLE Device */
typedef struct
{
    uint8_t enable;
    uint32_t gpio_num;
    uint8_t polarity;
} wiced_bt_ble_hidh_wakeup_control_t;

typedef struct
{
    wiced_bt_ble_hidh_dev_t devices[WICED_BT_BLE_HIDH_DEV_MAX];
    wiced_bt_ble_hidh_cback_t *p_callback;
    wiced_bt_ble_hidh_wakeup_control_t wakeup_control;
    wiced_bt_ble_hidh_filter_cback_t *p_filter_callbacks[WICED_BT_BLE_HIDH_EVENT_FILTER_NB_MAX];
} wiced_bt_ble_hidh_cb_t;


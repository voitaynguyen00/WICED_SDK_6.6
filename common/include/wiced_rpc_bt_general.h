/***************************************************************************//**
* \file <wiced_rpc_bt_general.h>
*
* \brief
* 	WICED RPC Bluetooth general group definitions
*//*****************************************************************************
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
*******************************************************************************/


#pragma once

#include "hci_control_api.h"
#include <stdint.h>


/* WICED RPC Bluetooth general commands */
enum wiced_rpc_command_bt_general {
    WICED_RPC_COMMAND_BT_GENERAL_RESET             = 0x00ffu & HCI_CONTROL_COMMAND_RESET,             /* Restart controller */
    WICED_RPC_COMMAND_BT_GENERAL_TRACE_ENABLE      = 0x00ffu & HCI_CONTROL_COMMAND_TRACE_ENABLE,      /* Enable or disable WICED traces */
    WICED_RPC_COMMAND_BT_GENERAL_SET_LOCAL_BDA     = 0x00ffu & HCI_CONTROL_COMMAND_SET_LOCAL_BDA,     /* Set local device addrsss */
    WICED_RPC_COMMAND_BT_GENERAL_PUSH_NVRAM_DATA   = 0x00ffu & HCI_CONTROL_COMMAND_PUSH_NVRAM_DATA,   /* Download previously saved NVRAM chunk */
    WICED_RPC_COMMAND_BT_GENERAL_DELETE_NVRAM_DATA = 0x00ffu & HCI_CONTROL_COMMAND_DELETE_NVRAM_DATA, /* Delete NVRAM chunk currently stored in RAM */
    WICED_RPC_COMMAND_BT_GENERAL_INQUIRY           = 0x00ffu & HCI_CONTROL_COMMAND_INQUIRY,           /* Start/stop inquiry */
    WICED_RPC_COMMAND_BT_GENERAL_SET_VISIBILITY    = 0x00ffu & HCI_CONTROL_COMMAND_SET_VISIBILITY,    /* Set BR/EDR connectability and discoverability of the device */
    WICED_RPC_COMMAND_BT_GENERAL_SET_PAIRING_MODE  = 0x00ffu & HCI_CONTROL_COMMAND_SET_PAIRING_MODE,  /* Set Pairing Mode for the device 0 = Not pairable 1 = Pairable */
    WICED_RPC_COMMAND_BT_GENERAL_UNBOND            = 0x00ffu & HCI_CONTROL_COMMAND_UNBOND,            /* Delete bond with specified BDADDR */
    WICED_RPC_COMMAND_BT_GENERAL_USER_CONFIRMATION = 0x00ffu & HCI_CONTROL_COMMAND_USER_CONFIRMATION, /* User Confirmation during pairing, TRUE/FALSE passed as parameter */
    WICED_RPC_COMMAND_BT_GENERAL_ENABLE_COEX       = 0x00ffu & HCI_CONTROL_COMMAND_ENABLE_COEX,       /* Enable coex functionality */
    WICED_RPC_COMMAND_BT_GENERAL_DISABLE_COEX      = 0x00ffu & HCI_CONTROL_COMMAND_DISABLE_COEX,      /* Disable coex functionality */
    WICED_RPC_COMMAND_BT_GENERAL_SET_BATTERY_LEVEL = 0x00ffu & HCI_CONTROL_COMMAND_SET_BATTERY_LEVEL, /* Sets battery level in the GATT database */
    WICED_RPC_COMMAND_BT_GENERAL_READ_LOCAL_BDA    = 0x00ffu & HCI_CONTROL_COMMAND_READ_LOCAL_BDA,    /* Get local device addrsss */
    WICED_RPC_COMMAND_BT_GENERAL_BOND              = 0x00ffu & HCI_CONTROL_COMMAND_BOND,              /* Initiate Bonding with a peer device */
    WICED_RPC_COMMAND_BT_GENERAL_READ_BUFF_STATS   = 0x00ffu & HCI_CONTROL_COMMAND_READ_BUFF_STATS,   /* Read Buffer statistics */
    WICED_RPC_COMMAND_BT_GENERAL_SET_LOCAL_NAME    = 0x00ffu & HCI_CONTROL_COMMAND_SET_LOCAL_NAME,    /* Set the local name */
};
typedef uint8_t wiced_rpc_command_bt_general_t;

/* WICED RPC Bluetooth general events */
enum wiced_rpc_event_bt_general {
    WICED_RPC_EVENT_BT_GENERAL_COMMAND_STATUS                    = 0x00ffu & HCI_CONTROL_EVENT_COMMAND_STATUS,                    /* Command status event for the requested operation */
    WICED_RPC_EVENT_BT_GENERAL_WICED_TRACE                       = 0x00ffu & HCI_CONTROL_EVENT_WICED_TRACE,                       /* WICED trace packet */
    WICED_RPC_EVENT_BT_GENERAL_HCI_TRACE                         = 0x00ffu & HCI_CONTROL_EVENT_HCI_TRACE,                         /* Bluetooth protocol trace */
    WICED_RPC_EVENT_BT_GENERAL_NVRAM_DATA                        = 0x00ffu & HCI_CONTROL_EVENT_NVRAM_DATA,                        /* Request to MCU to save NVRAM chunk */
    WICED_RPC_EVENT_BT_GENERAL_DEVICE_STARTED                    = 0x00ffu & HCI_CONTROL_EVENT_DEVICE_STARTED,                    /* Device completed power up initialization */
    WICED_RPC_EVENT_BT_GENERAL_INQUIRY_RESULT                    = 0x00ffu & HCI_CONTROL_EVENT_INQUIRY_RESULT,                    /* Inquiry result */
    WICED_RPC_EVENT_BT_GENERAL_INQUIRY_COMPLETE                  = 0x00ffu & HCI_CONTROL_EVENT_INQUIRY_COMPLETE,                  /* Inquiry completed event */
    WICED_RPC_EVENT_BT_GENERAL_PAIRING_COMPLETE                  = 0x00ffu & HCI_CONTROL_EVENT_PAIRING_COMPLETE,                  /* Pairing Completed */
    WICED_RPC_EVENT_BT_GENERAL_ENCRYPTION_CHANGED                = 0x00ffu & HCI_CONTROL_EVENT_ENCRYPTION_CHANGED,                /* Encryption changed event */
    WICED_RPC_EVENT_BT_GENERAL_CONNECTED_DEVICE_NAME             = 0x00ffu & HCI_CONTROL_EVENT_CONNECTED_DEVICE_NAME,             /* Device name event */
    WICED_RPC_EVENT_BT_GENERAL_USER_CONFIRMATION                 = 0x00ffu & HCI_CONTROL_EVENT_USER_CONFIRMATION,                 /* User Confirmation during pairing */
    WICED_RPC_EVENT_BT_GENERAL_DEVICE_ERROR                      = 0x00ffu & HCI_CONTROL_EVENT_DEVICE_ERROR,                      /* Device Error event */
    WICED_RPC_EVENT_BT_GENERAL_READ_LOCAL_BDA                    = 0x00ffu & HCI_CONTROL_EVENT_READ_LOCAL_BDA,                    /* Local BDA Read event */
    WICED_RPC_EVENT_BT_GENERAL_MAX_NUM_OF_PAIRED_DEVICES_REACHED = 0x00ffu & HCI_CONTROL_EVENT_MAX_NUM_OF_PAIRED_DEVICES_REACHED, /* Key Buffer Pool Full */
    WICED_RPC_EVENT_BT_GENERAL_READ_BUFFER_STATS                 = 0x00ffu & HCI_CONTROL_EVENT_READ_BUFFER_STATS,                 /* Read Buffer statistics event */
    WICED_RPC_EVENT_BT_GENERAL_UPDATE_LINK_KEY                   = 0x00ffu & HCI_CONTROL_EVENT_UPDATE_LINK_KEY,                   /* Update link key info in the DCT */
    WICED_RPC_EVENT_BT_GENERAL_REQUEST_ID_KEYS                   = 0x00ffu & HCI_CONTROL_EVENT_REQUEST_ID_KEYS,                   /* Request the ID keys */
    WICED_RPC_EVENT_BT_GENERAL_READ_RSSI                         = 0x00ffu & HCI_CONTROL_EVENT_READ_RSSI,                         /* CYW20706 returns the RSSI of the desired link */
    WICED_RPC_EVENT_BT_GENERAL_DEVICE_INIT                       = 0x00ffu & HCI_CONTROL_EVENT_DEVICE_INIT,                       /* Sends a Device Started event at the end of application initialization. */
    WICED_RPC_EVENT_BT_GENERAL_SECURITY_REQ                      = 0x00ffu & HCI_CONTROL_EVENT_SECURITY_REQ,                      /* The app will respond to this event with a "wiced_bt_ble_security_grant". */
    WICED_RPC_EVENT_BT_GENERAL_SECURITY_FAILED                   = 0x00ffu & HCI_CONTROL_EVENT_SECURITY_FAILED,                   /* Security procedure/authentication failed. */
    WICED_RPC_EVENT_BT_GENERAL_IO_CAPABILITIES_BR_EDR_REQUEST    = 0x00ffu & HCI_CONTROL_EVENT_IO_CAPABILITIES_BR_EDR_REQUEST,    /* IO capablities request */
    WICED_RPC_EVENT_BT_GENERAL_KEYPRESS_NOTIFICATION             = 0x00ffu & HCI_CONTROL_EVENT_KEYPRESS_NOTIFICATION,             /* KeyPress notification */
    WICED_RPC_EVENT_BT_GENERAL_CONNECTION_STATUS                 = 0x00ffu & HCI_CONTROL_EVENT_CONNECTION_STATUS,                 /* Connection Status */
};

/* Define trace route types in WICED_RPC_COMMAND_BT_GENERAL_TRACE_ENABLE */
enum wiced_rpc_trace_route {
    WICED_RPC_TRACE_ROUTE_NONE               = 0, /* Traces are not generated. */
    WICED_RPC_TRACE_ROUTE_TO_TRACE_EVENT     = 1, /* Traces are forwarded to the WICED_RPC_EVENT_BT_GENERAL_WICED_TRACE event. */
    WICED_RPC_TRACE_ROUTE_TO_HCI_UART        = 2, /* Traces are forwarded to the HCI UART. */
    WICED_RPC_TRACE_ROUTE_TO_DEBUG_UART      = 3, /* Traces are forwarded to the debug UART. */
    WICED_RPC_TRACE_ROUTE_TO_PERIPHERAL_UART = 4, /* Traces are forwarded to the peripheral UART. */
};
typedef uint8_t wiced_rpc_trace_route_t;

/* Define WICED_RPC_COMMAND_BT_GENERAL_TRACE_ENABLE parameters */
typedef struct wiced_rpc_command_general_trace_enable {
    uint8_t is_enable;
    wiced_rpc_trace_route_t route;
} wiced_rpc_command_general_trace_enable_t;

/* Define status code returned in WICED_RPC_EVENT_BT_GENERAL_COMMAND_STATUS */
enum wiced_rpc_bt_general_command_status {
    WICED_RPC_BT_GENERAL_COMMAND_STATUS_SUCCESS               = HCI_CONTROL_STATUS_SUCCESS,
    WICED_RPC_BT_GENERAL_COMMAND_STATUS_IN_PROGRESS           = HCI_CONTROL_STATUS_IN_PROGRESS,
    WICED_RPC_BT_GENERAL_COMMAND_STATUS_ALREADY_CONNECTED     = HCI_CONTROL_STATUS_ALREADY_CONNECTED,
    WICED_RPC_BT_GENERAL_COMMAND_STATUS_NOT_CONNECTED         = HCI_CONTROL_STATUS_NOT_CONNECTED,
    WICED_RPC_BT_GENERAL_COMMAND_STATUS_BAD_HANDLE            = HCI_CONTROL_STATUS_BAD_HANDLE,
    WICED_RPC_BT_GENERAL_COMMAND_STATUS_WRONG_STATE           = HCI_CONTROL_STATUS_WRONG_STATE,
    WICED_RPC_BT_GENERAL_COMMAND_STATUS_INVALID_ARGS          = HCI_CONTROL_STATUS_INVALID_ARGS,
    WICED_RPC_BT_GENERAL_COMMAND_STATUS_FAILED                = HCI_CONTROL_STATUS_FAILED,
    WICED_RPC_BT_GENERAL_COMMAND_STATUS_UNKNOWN_GROUP         = HCI_CONTROL_STATUS_UNKNOWN_GROUP,
    WICED_RPC_BT_GENERAL_COMMAND_STATUS_UNKNOWN_COMMAND       = HCI_CONTROL_STATUS_UNKNOWN_COMMAND,
    WICED_RPC_BT_GENERAL_COMMAND_STATUS_CLIENT_NOT_REGISTERED = HCI_CONTROL_STATUS_CLIENT_NOT_REGISTERED,
    WICED_RPC_BT_GENERAL_COMMAND_STATUS_OUT_OF_MEMORY         = HCI_CONTROL_STATUS_OUT_OF_MEMORY,
    WICED_RPC_BT_GENERAL_COMMAND_STATUS_DISALLOWED            = HCI_CONTROL_STATUS_DISALLOWED,
};
typedef uint8_t wiced_rpc_bt_general_command_status_t;

/* Define WICED_RPC_EVENT_BT_GENERAL_HCI_TRACE parameters */
typedef struct wiced_rpc_event_bt_general_hci_trace {
    uint8_t type;
    uint8_t raw_hci_bytes[0];
} wiced_rpc_event_bt_general_hci_trace_t;

#endif /* WICED_RPC_BT_GENERAL_H_ */

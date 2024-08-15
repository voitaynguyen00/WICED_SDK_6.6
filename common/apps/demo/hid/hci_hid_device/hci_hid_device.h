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
 * HID Device sample application
 *
 */
#include "wiced_bt_dev.h"
#include "wiced_bt_cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
 * Constants
 ****************************************************************************/
/* define sizes of the input reports */
#define HCI_HID_KEYBOARD_REPORT_SIZE        9
#define HCI_HID_MOUSE_REPORT_SIZE           5
#define HCI_HID_CONSUMER_REPORT_SIZE        5
#define HCI_HID_KEYBOARD_OUT_REPORT_SIZE    2

#define HCI_HID_REPORT_ID_KEYBOARD          1
#define HCI_HID_REPORT_ID_MOUSE             2
#define HCI_HID_REPORT_ID_CONSUMER          3


/*****************************************************************************
 * Globals
 *****************************************************************************/
/* Stack and buffer pool configuration tables */
extern int hci_hid_get_num_buf_pools();
extern const wiced_bt_cfg_settings_t hci_hid_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t hci_hid_cfg_buf_pools[];

extern const uint8_t   wiced_bt_sdp_db[];
extern const uint16_t  wiced_bt_sdp_db_size;

extern uint8_t hci_hid_last_keyboard_report[HCI_HID_KEYBOARD_REPORT_SIZE];
extern uint8_t hci_hid_last_mouse_report[HCI_HID_MOUSE_REPORT_SIZE];
extern uint8_t hci_hid_last_consumer_report[HCI_HID_CONSUMER_REPORT_SIZE];
extern uint8_t hci_hid_last_keyboard_out_report[HCI_HID_KEYBOARD_OUT_REPORT_SIZE];

/*****************************************************************************
 * Function Prototypes
 *****************************************************************************/

#ifdef __cplusplus
}
#endif

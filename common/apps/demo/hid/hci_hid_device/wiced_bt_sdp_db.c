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
 * Pre-built SDP server database
 *
 * (Available soon: tool for generating SDP database)
 *
 */
#include "hci_hid_device.h"
#include "wiced_bt_sdp.h"
#include "bt_types.h"
#include "wiced_bt_hid_defs.h"

#if 1 // HID descriptor length < 255
#define HID_DESCRIPTOR   \
    0x05, 0x01,             /* Usage page Desktop 01 */         \
    0x09, 0x06,             /* Usage Keyboard 06     */         \
    0xa1, 0x01,             /* Collection application */        \
        0x05, 0x07,         /* Usage page Keyboard */           \
        0x85, HCI_HID_REPORT_ID_KEYBOARD,     /* Report ID 1 */ \
        0x19, 0xe0,        /* Usage minimum e0 (leftControl) */ \
        0x29, 0xe7,        /* Usage maximum e7 (right gui) */   \
        0x15, 0x00,        /* Logical minimum 0 */              \
        0x25, 0x01,        /* Logical Maximum 1 */              \
        0x75, 0x01,        /* Report size 1 */                  \
        0x95, 0x08,        /* Report count 8 */                 \
        0x81, 0x02,        /* Input Variable Abs */             \
        0x95, 0x01,        /* Report count 1 */                 \
        0x75, 0x08,        /* Report size 8 */                  \
        0x81, 0x01,        /* Iput constant variable   */       \
        0x95, 0x05,        /* report count 5 */                 \
        0x75, 0x01,        /* Report size 1 */                  \
        0x05, 0x08,        /* LED page */                       \
        0x19, 0x01,        /* Usage minimum 1 Num lock */       \
        0x29, 0x05,        /* Usage maximum 5 Kana  */          \
        0x91, 0x02,        /* Output Data, Variable, Absolute */\
        0x95, 0x01,        /* Report Count 1 */                 \
        0x75, 0x03,        /* Report Size 3 */                  \
        0x91, 0x01,        /* Output constant, Absolute  */     \
        0x95, 0x06,        /* Report Count 6 */                 \
        0x75, 0x08,        /* Report size 8 */                  \
        0x15, 0x00,        /* Logical minimum 0 */              \
        0x26, 0xa4, 0x00,  /* Logical Maximum 00a4 */           \
        0x05, 0x07,        /* Usage page Keyboard */            \
        0x19, 0x00,        /* Usage minimum 0 */                \
        0x29, 0xa4,        /* Usage maximum a4 */               \
        0x81, 0x00,        /* Input data array absolute */      \
    0xc0,                                                       \
    0x05, 0x01,             /* Usage page Desktop 01 */         \
    0x09, 0x02,             /* Usage 2 Mouse */                 \
    0xa1, 0x01,             /* Collection application */        \
        0x09, 0x01,         /* Usage 1 pointer */               \
        0xa1, 0x00,         /* Collection physical */           \
            0x85, HCI_HID_REPORT_ID_MOUSE,     /* report id 2 */\
            0x05, 0x09,     /* Usage page button */             \
            0x19, 0x01,     /* Usage minimum 1 */               \
            0x29, 0x03,     /* Usage maximum 3 */               \
            0x15, 0x00,     /* Logical minimum 0 */             \
            0x25, 0x01,     /* Logical maximum 1 */             \
            0x95, 0x03,     /* Report Count 3 */                \
            0x75, 0x01,     /* Report size 1 */                 \
            0x81, 0x02,     /* Input Variable Abs */            \
            0x95, 0x01,     /* Report Count 1 */                \
            0x75, 0x05,     /* Report size 5 */                 \
            0x81, 0x03,     /* Input const var Abs */           \
                                                                \
            0x05, 0x01,     /* Usage page Desktop 01 */         \
            0x09, 0x30,     /* Usage X */                       \
            0x09, 0x31,     /* Usage Y */                       \
            0x09, 0x38,     /* Usage Wheel */                   \
            0x15, 0x81,     /* Logical minimum -127 */          \
            0x25, 0x7f,     /* Logical Maximum 127 */           \
            0x75, 0x08,     /* Report size 8 */                 \
            0x95, 0x03,     /* Report Count 3 */                \
            0x81, 0x06,     /* Input Variable relative */       \
        0xc0,                                                   \
    0xc0,                                                       \
                                                                \
    0x05, 0x0c,            /* Usage page (consumer page) */     \
    0x09, 0x01,            /* Usage Consumer control  */        \
    0xa1, 0x01,            /* Collection application */         \
        0x85, HCI_HID_REPORT_ID_CONSUMER,    /* Report ID 3 */  \
        0x75, 0x10,        /* report size 16 */                 \
        0x95, 0x02,        /* report count 2 */                 \
        0x15, 0x01,        /* Logical minimum 1 */              \
        0x26, 0x8c, 0x02,  /* Logical Maximum 028c */           \
        0x19, 0x01,        /* Usage minimum 1 */                \
        0x2a, 0x8c, 0x02,  /* Usage maximum 028c */             \
        0x81, 0x60,        /* Input Data array absolute No preferred, Null state */  \
    0xc0
#endif

static const uint8_t hid_discriptor[] = {HID_DESCRIPTOR};
#define HID_DESCRIPTOR_LEN  (sizeof(hid_discriptor))

#define HID_LANGUAGE_LIST       0x35, 0x06, 0x09, 0x04, 0x09, 0x09, 0x01, 0x00
static const uint8_t hid_language_list[] = {HID_LANGUAGE_LIST};
#define HID_LANGUAGE_LIST_LEN   (sizeof(hid_language_list))

/*
 * Need to save last report sent out, so that we can answer GET_REPORT queries
 */
uint8_t hci_hid_last_keyboard_report[HCI_HID_KEYBOARD_REPORT_SIZE] = { 0x01 };
uint8_t hci_hid_last_mouse_report[HCI_HID_MOUSE_REPORT_SIZE] = { 0x02 };
uint8_t hci_hid_last_consumer_report[HCI_HID_CONSUMER_REPORT_SIZE] = { 0x03 };
uint8_t hci_hid_last_keyboard_out_report[HCI_HID_KEYBOARD_OUT_REPORT_SIZE] = { 0x01 };

const uint8_t wiced_bt_sdp_db[] =
{
#if 1 // HID descriptor len < 255
    SDP_ATTR_SEQUENCE_2(300 + HID_DESCRIPTOR_LEN + HID_LANGUAGE_LIST_LEN),  // length is the sum of all records

    // first SDP record: HID (total = 142 + HID_DESCRIPTOR_LEN + HID_LANGUAGE_LIST_LEN + 3 = 148 + HID_DESCRIPTOR_LEN + HID_LANGUAGE_LIST_LEN)
    SDP_ATTR_SEQUENCE_2(154 + HID_DESCRIPTOR_LEN + HID_LANGUAGE_LIST_LEN), // 3 bytes, length of the record
#else
    SDP_ATTR_SEQUENCE_2(303 + HID_DESCRIPTOR_LEN + HID_LANGUAGE_LIST_LEN),  // length is the sum of all records

    // first SDP record: HID (total = 142 + HID_DESCRIPTOR_LEN + HID_LANGUAGE_LIST_LEN + 3 = 148 + HID_DESCRIPTOR_LEN + HID_LANGUAGE_LIST_LEN)
    SDP_ATTR_SEQUENCE_2(157 + HID_DESCRIPTOR_LEN + HID_LANGUAGE_LIST_LEN),  // length is the sum of all records

#endif
    // 16
    SDP_ATTR_RECORD_HANDLE(0x10001),                                    // 8 byte
    SDP_ATTR_CLASS_ID(UUID_SERVCLASS_HUMAN_INTERFACE),                  // 8

    // 18
    SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST), SDP_ATTR_SEQUENCE_1(13),   // 3 + 2
    SDP_ATTR_SEQUENCE_1(6),                                             // 2
    SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),                               // 3
    SDP_ATTR_VALUE_UINT2(0x11),                                         // 3
    SDP_ATTR_SEQUENCE_1(3),                                             // 2
    SDP_ATTR_UUID16(UUID_PROTOCOL_HIDP),                                // 3

    // 8
    SDP_ATTR_BROWSE_LIST,                                               // 8

    // 14
    SDP_ATTR_ID(ATTR_ID_LANGUAGE_BASE_ATTR_ID_LIST), SDP_ATTR_SEQUENCE_1(9),      // 3 + 2
    SDP_ATTR_VALUE_UINT2(LANG_ID_CODE_ENGLISH),                         // 3
    SDP_ATTR_VALUE_UINT2(LANG_ID_CHAR_ENCODE_UTF8),                     // 3
    SDP_ATTR_VALUE_UINT2(LANGUAGE_BASE_ID),                             // 3

    // 13
    SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),  // 3 + 2
    SDP_ATTR_SEQUENCE_1(6),                                             // 2
    SDP_ATTR_UUID16(UUID_SERVCLASS_HUMAN_INTERFACE),                    // 3
    SDP_ATTR_VALUE_UINT2(0x101),                                        // 3 Version

    // 20
    SDP_ATTR_ID(ATTR_ID_ADDITION_PROTO_DESC_LISTS), SDP_ATTR_SEQUENCE_1(15),// 3 + 2
    SDP_ATTR_SEQUENCE_1(13),                                            // 2
    SDP_ATTR_SEQUENCE_1(6),                                             // 2
    SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),                               // 3
    SDP_ATTR_VALUE_UINT2(0x13),                                         // 3 (PSM=HID Interrupt)
    SDP_ATTR_SEQUENCE_1(3),                                             // 2
    SDP_ATTR_UUID16(UUID_PROTOCOL_HIDP),                                // 3

    // 6
    SDP_ATTR_ID(ATTR_ID_HID_DEVICE_RELNUM),                             // 3 x
    SDP_ATTR_VALUE_UINT2(0x100),                                        // 3 Version

    // 6
    SDP_ATTR_ID(ATTR_ID_HID_PARSER_VERSION),                            // 3 x
    SDP_ATTR_VALUE_UINT2(0x111),                                        // 3 Version

    // 5
    SDP_ATTR_ID(ATTR_ID_HID_DEVICE_SUBCLASS),                           // 3 x
    SDP_ATTR_VALUE_UINT1(0xC0),                                         // 2  (This needs to match low byte of the Class of device, see wiced_bt_cfg.c)

    // 5
    SDP_ATTR_ID(ATTR_ID_HID_COUNTRY_CODE),                              // 3 x
    SDP_ATTR_VALUE_UINT1(0x00),                                         // 2  (0 for not localized)

    // 5
    SDP_ATTR_ID(ATTR_ID_HID_VIRTUAL_CABLE),                             // 3 x
    SDP_ATTR_VALUE_BOOLEAN(1),                                          // 2

    // 5
    SDP_ATTR_ID(ATTR_ID_HID_RECONNECT_INITIATE),                        // 3 x
    SDP_ATTR_VALUE_BOOLEAN(1),                                          // 2

#if 1 // HID descriptor len < 255
    // HID_DESCRIPTOR_LEN + 6 + 5
    SDP_ATTR_ID(ATTR_ID_HID_DESCRIPTOR_LIST), SDP_ATTR_SEQUENCE_1(HID_DESCRIPTOR_LEN + 6),  // 3 + 2
        SDP_ATTR_SEQUENCE_1(HID_DESCRIPTOR_LEN + 4),                        // 2
            SDP_ATTR_VALUE_UINT1(HID_SDP_DESCRIPTOR_REPORT),                // 2
            SDP_ATTR_VALUE_TEXT_1(HID_DESCRIPTOR_LEN),                      // 2
                HID_DESCRIPTOR,                                             // HID_DESCRIPTOR_LEN
#else
    // HID_DESCRIPTOR_LEN + 6 + 8
    SDP_ATTR_ID(ATTR_ID_HID_DESCRIPTOR_LIST), SDP_ATTR_SEQUENCE_2(HID_DESCRIPTOR_LEN + 8),  // 3 + 3
        SDP_ATTR_SEQUENCE_2(HID_DESCRIPTOR_LEN + 5),                        // 3
            SDP_ATTR_VALUE_UINT1(HID_SDP_DESCRIPTOR_REPORT),                // 2
            SDP_ATTR_VALUE_TEXT_2(HID_DESCRIPTOR_LEN),                      // 3
                HID_DESCRIPTOR,                                             // HID_DESCRIPTOR_LEN
#endif

    // HID_LANGUAGE_LIST_LEN +  5
    SDP_ATTR_ID(ATTR_ID_HID_LANGUAGE_ID_BASE), SDP_ATTR_SEQUENCE_1(HID_LANGUAGE_LIST_LEN),  // 3 + 2
    HID_LANGUAGE_LIST,                                                  // HID_LANGUAGE_LIST_LEN

    // 6
    SDP_ATTR_ID(ATTR_ID_HID_PROFILE_VERSION),
    SDP_ATTR_VALUE_UINT2(0x100),                                        // 3 Version

    // 6
    SDP_ATTR_ID(ATTR_ID_HID_LINK_SUPERVISION_TO),
    SDP_ATTR_VALUE_UINT2(3200),                                        // 3 Link supervision TO 2 seconds

    // 5
    SDP_ATTR_ID(ATTR_ID_HID_BOOT_DEVICE),                               // 3 x
    SDP_ATTR_VALUE_BOOLEAN(0),                                          // 2

    //  SDP record Serial Port
    SDP_ATTR_SEQUENCE_1(69),                                            // 2 bytes
        SDP_ATTR_RECORD_HANDLE(0x10002),                                // 8 bytes
        SDP_ATTR_CLASS_ID(UUID_SERVCLASS_SERIAL_PORT),                  // 8
        SDP_ATTR_RFCOMM_PROTOCOL_DESC_LIST( 1 ),                        // 17 bytes
        SDP_ATTR_BROWSE_LIST,                                           // 8
        SDP_ATTR_PROFILE_DESC_LIST(UUID_SERVCLASS_SERIAL_PORT, 0x0102), // 13 byte
        SDP_ATTR_TEXT_1(ATTR_ID_SERVICE_NAME, 10),                      // 15
        'S', 'P', 'P', ' ', 'S', 'E', 'R', 'V', 'E', 'R',

    /************************************************************************/
    // second SDP record Device ID (total = 69 + 2 = 71)
    SDP_ATTR_SEQUENCE_1(69),                                            // 2 bytes, length of the record
    SDP_ATTR_RECORD_HANDLE(0x10003),                                    // 8 byte
    SDP_ATTR_CLASS_ID(UUID_SERVCLASS_PNP_INFORMATION),                  // 8
    SDP_ATTR_PROTOCOL_DESC_LIST(1),                                     // 18
    SDP_ATTR_UINT2(ATTR_ID_SPECIFICATION_ID, 0x103),                    // 6
    SDP_ATTR_UINT2(ATTR_ID_VENDOR_ID, 0x0f),                            // 6
    SDP_ATTR_UINT2(ATTR_ID_PRODUCT_ID, 0x0201),                         // 6
    SDP_ATTR_UINT2(ATTR_ID_PRODUCT_VERSION, 0x0001),                    // 6
    SDP_ATTR_BOOLEAN(ATTR_ID_PRIMARY_RECORD, 0x01),                     // 5
    SDP_ATTR_UINT2(ATTR_ID_VENDOR_ID_SOURCE, DI_VENDOR_ID_SOURCE_BTSIG) // 6

};
const uint16_t wiced_bt_sdp_db_size = (sizeof(wiced_bt_sdp_db));

/*
 * Copyright 2021, Cypress Semiconductor Corporation or a subsidiary of 
 * Cypress Semiconductor Corporation. All Rights Reserved.
 * 
 * This software, associated documentation and materials ("Software"),
 * is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
 *  WICED AMQP constants, data types and APIs.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************
 *                      Macros
 ******************************************************/
#define AMQP_CONNECTION_FRAME_MAX       (4*1024)        /* Maximum frame size for a connection          */
#define AMQP_CONNECTION_CHANNEL_MAX     (1)             /* Maximum number of channels per connection    */


#define AMQP_OPEN_FRAME_LIST_COUNT          (4)
#define AMQP_CLOSE_FRAME_LIST_COUNT         (1)
#define AMQP_BEGIN_FRAME_LIST_COUNT         (5)
#define AMQP_END_FRAME_LIST_COUNT           (1)
#define AMQP_ATTACH_FRAME_LIST_COUNT        (11)
#define AMQP_DETACH_FRAME_LIST_COUNT        (3)
#define AMQP_FLOW_FRAME_LIST_COUNT          (11)
#define AMQP_TRANSFER_FRAME_LIST_COUNT      (11)
#define AMQP_DISPOSTION_FRAME_LIST_COUNT    (5)



/******************************************************
 *                    Constants
 ******************************************************/
#define AMQP_CONNECTION_DEFAULT_PORT    (5672)
#define AMQP_CONNECTION_SECURITY_PORT   (5671)
#define AMQP_CONNECTION_DATA_SIZE_MAX   (AMQP_CONNECTION_FRAME_MAX - 8) /* Maximum size to put in a frame */


#define WICED_AMQP_PROTOCOL_VERSION_MAJOR          (1)
#define WICED_AMQP_PROTOCOL_VERSION_MINOR          (0)
#define WICED_AMQP_PROTOCOL_VERSION_REVISION       (0)

#define WICED_AMQP_PROTOCOL_ID_OPEN                (0)
#define WICED_AMQP_PROTOCOL_ID_TLS                 (2)
#define WICED_AMQP_PROTOCOL_ID_SASL                (3)
#define WICED_AMQP_MAJOR_NUMBER                    (1)
#define WICED_AMQP_MINOR_NUMBER                    (0)
#define WICED_AMQP_REVISION_NUMBER                 (0)

#define WICED_AMQP_LINK_CREDIT                     (10)

#define ELEMENT_ERROR                               0x1d
#define ELEMENT_SOURCE                              0x28
#define ELEMENT_TARGET                              0x29

#define HEADER_SECTION                              0x70
#define DELIVERY_ANNOTATIONS_SECTION                0x71
#define MESSAGE_ANNOTATIONS_SECTION                 0x72
#define PROPERTIES_SECTION                          0x73
#define APPLICATION_PROPERTIES_SECTION              0x74
#define DATA_SECTION                                0x75
#define AMQP_SEQUENCE_SECTION                       0x76
#define AMQP_VALUE_SECTION                          0x77
#define FOOTER_SECTION                              0x78

//#define FIXED_TWO_BYTE                               0x60
//#define FIXED_FOUR_BYTE                              0x70
//#define FIXED_EIGHT_BYTE                             0x80
//#define FIXED_SIXTEEN_BYTE                           0x90
//#define VARIABLE_ONE_BYTE                            0xA0
//#define VARIABLE_FOUR_BYTE                           0xB0
//#define COMPOUND_ONE_BYTE                            0xC0
//#define COMPOUND_FOUR_BYTE                           0xD0
//#define ARRAY_ONE_BYTE                               0xE0
//#define ARRAY_FOUR_BYTE                              0xF0


/* FIXED DATA TYPES */

#define NULL_BYTE                                    0x40



/* Boolean encoding code and values */
#define BOOLEAN_TYPE_ENCODING_CODE                        0x56
#define BOOLEAN_TYPE_TRUE                                 0x41
#define BOOLEAN_TYPE_FALSE                                0x42

/* un-signed datatypes */

/* Ubyte encoding code and values */
#define FIXED_TYPE_ONE_UBYTE                              0x50

/* Ushort encoding code and values */
#define FIXED_TYPE_ONE_USHORT                             0x60


/* Uint encoding code and values */
#define FIXED_TYPE_FOUR_BYTE_UINT                         0x70
#define FIXED_TYPE_ONE_BYTE_UINT                          0x52
#define FIXED_TYPE_EMPTY_BYTE_UINT                        0x43

/* ULONG encoding code and values */
#define FIXED_TYPE_EIGHT_BYTE_ULONG                       0x80
#define FIXED_TYPE_ONE_BYTE_ULONG                         0x53
#define FIXED_TYPE_EMPTY_BYTE_ULONG                       0x44

/* signed datatypes */

/* Byte encoding code and values */
#define FIXED_TYPE_ONE_BYTE                               0x51

/* short encoding code and values */
#define FIXED_TYPE_ONE_SHORT                             0x61


/* int encoding code and values */
#define FIXED_TYPE_FOUR_BYTE_INT                         0x71
#define FIXED_TYPE_ONE_BYTE_INT                          0x54

/* LONG encoding code and values */
#define FIXED_TYPE_EIGHT_BYTE_LONG                       0x81
#define FIXED_TYPE_ONE_BYTE_LONG                         0x55

/* Float encoding code and values */
#define FIXED_TYPE_FOUR_BYTE_FLOAT                       0x72
#define FIXED_TYPE_EIGHT_BYTE_DOUBLE                     0x82

/* Decimal encoding code and values */
#define FIXED_TYPE_FOUR_BYTE_DECIMAL                     0x74
#define FIXED_TYPE_EIGHT_BYTE_DECIMAL                    0x84
#define FIXED_TYPE_EIGHT_EIGHT_BYTE_DECIMAL              0x94

/* Uni-code char  encoding code and values  */
#define FIXED_TYPE_FOUR_BYTE_UNICODE_CHAR                0x73

/* Time stamp encoding code and values  */
#define FIXED_TYPE_EIGHT_BYTE_TIME_STAMP                 0x83

/* UUID encoding code and values  */
#define FIXED_TYPE_EIGHT_EIGHT_BYTE_UUID                 0x98


/* fixed, Variable & compound data types */

/* Binary encoding code and values */
#define VARIABLE_TYPE_ONE_BYTE_BINARY                    0xA0
#define VARIABLE_TYPE_FOUR_BYTE_BINARY                   0xB0

/* Uni-code string encoding code and values */
#define VARIABLE_TYPE_ONE_BYTE_UNICODE_STRING            0xA1
#define VARIABLE_TYPE_FOUR_BYTE_UNICODE_STRING           0xB1

/* Symbols encoding code and values */
#define VARIABLE_TYPE_ONE_BYTE_SYMBOL                   0xA3
#define VARIABLE_TYPE_FOUR_BYTE_SYMBOL                  0xB3

/* list encoding code and values */
#define FIXED_TYPE_NULL_BYTE_LIST                      0x45
#define COMPOUND_TYPE_ONE_BYTE_LIST                    0xC0
#define COMPOUND_TYPE_FOUR_BYTE_LIST                   0xD0

/* map encoding code and values */
#define COMPOUND_TYPE_ONE_BYTE_MAP                     0xC1
#define COMPOUND_TYPE_FOUR_BYTE_MAP                    0xD1

/* array encoding code and values */
#define ARRAY_TYPE_ONE_BYTE_ARRAY                     0xC1
#define ARRAY_TYPE_FOUR_BYTE_ARRAY                    0xD1

#define TRANSFER_FRAME_ELEMENTS_COUNT              (11)
#define UPPER_NIBBLE                               (0xF0)


/******************************************************
 *                   Enumerations
 ******************************************************/
/******************************************************
 *                 Type Definitions
 ******************************************************/
/* TODO : this will be used in future */
typedef struct amqp_map_s
{
        uint8_t *key;
        uint32_t key_length;
        void *value;
        uint32_t value_length;
} amqp_map;

typedef struct amqp_error_s
{
        uint8_t* condition;
        uint32_t condition_length;
        uint8_t* descrption;
        uint32_t description_length;
        amqp_map info;
} amqp_error_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Connection API Definitions
 ******************************************************/

#ifdef __cplusplus
} /* extern "C" */
#endif

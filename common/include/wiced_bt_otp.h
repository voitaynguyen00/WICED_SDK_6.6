/***************************************************************************//**
* \file <wiced_bt_otp.h_>
*
* \brief
* 	Contains Object Transfer Profile - Object Transfer Profile APIs and definitions.
*
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

#ifndef WICED_BT_OTP_H
#define WICED_BT_OTP_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
*          Type Definitions
******************************************************************************/

/* Object Transfer Service PSM Value */
#define OTS_PSM                         0x0025

/* Maximum length Object Transfer Service Characteristic - Object Name */
#define OTS_OBJECT_NAME_MAX_LENGTH      120

/* Attribute Protocol Application Error codes defined by the Object Transfer Service */
typedef enum
{
    OTS_WRITE_REQUEST_REJECTED = 0x80,
    OTS_OBJECT_NOT_SELECTED,
    OTS_CONCURRENCY_LIMIT_EXCEEDED,
    OTS_OBJECT_NAME_ALREADY_EXISTS
}wiced_bt_ots_att_app_error_codes_t;

/* OTS Feature - Object Action Control Point Features Supported Bit Mask */
typedef enum
{
    OTS_OACP_CREATE_SUPPORT                =  ( 1 << 0 ),
    OTS_OACP_DELETE_SUPPORT                =  ( 1 << 1 ),
    OTS_OACP_CALC_CHECKSUM_SUPPORT         =  ( 1 << 2 ),
    OTS_OACP_EXECUTE_SUPPORT               =  ( 1 << 3 ),
    OTS_OACP_READ_SUPPORT                  =  ( 1 << 4 ),
    OTS_OACP_WRITE_SUPPORT                 =  ( 1 << 5 ),
    OTS_OACP_APPEND_OBJECT_DATA_SUPPORT    =  ( 1 << 6 ),
    OTS_OACP_TRUNCATE_OBJECT_SUPPORT       =  ( 1 << 7 ),
    OTS_OACP_PATCH_OBJECT_SUPPORT          =  ( 1 << 8 ),
    OTS_OACP_ABORT_SUPPORT                 =  ( 1 << 9 ),
}wiced_bt_ots_oacp_feature_support_t;

/* Object Metadata - Object Properties Bitmask */
typedef enum
{
    OTS_OBJECT_PROP_DELETE                  =  ( 1 << 0 ),
    OTS_OBJECT_PROP_EXECUTE                 =  ( 1 << 1 ),
    OTS_OBJECT_PROP_READ                    =  ( 1 << 2 ),
    OTS_OBJECT_PROP_WRITE                   =  ( 1 << 3 ),
    OTS_OBJECT_PROP_APPEND                  =  ( 1 << 4 ),
    OTS_OBJECT_PROP_TRUNCATE                =  ( 1 << 5 ),
    OTS_OBJECT_PROP_PATCH                   =  ( 1 << 6 ),
    OTS_OBJECT_PROP_MARK                    =  ( 1 << 7 )
}wiced_bt_ots_object_property_bitmask_t;

/* OTS Object Action Control Point Procedures */
typedef enum
{
    OTS_OACP_RFU              = 0x00,
    OTS_OACP_CREATE,
    OTS_OACP_DELETE,
    OTS_OACP_CALCULATE_CHECKSUM,
    OTS_OACP_EXECUTE,
    OTS_OACP_READ,
    OTS_OACP_WRITE,
    OTS_OACP_ABORT,
    OTS_OACP_RESPONSE         = 0x60
}wiced_bt_ots_oacp_procedure_t;

/* OACP Response Value result codes */
typedef enum
{
    OTS_OACP_RESP_RFU         = 0x00,
    OTS_OACP_RESP_SUCCESS,
    OTS_OACP_RESP_OPCODE_NOT_SUPPORTED,
    OTS_OACP_RESP_INVALID_PARAM,
    OTS_OACP_RESP_INSUFFICIENT_RESOURCES,
    OTS_OACP_RESP_INVALID_OBJECT,
    OTS_OACP_RESP_CHANNEL_UNAVAILABLE,
    OTS_OACP_RESP_UNSUPPORTED_TYPE,
    OTS_OACP_RESP_PROCEDURE_NOT_PERMITTED,
    OTS_OACP_RESP_OBJECT_LOCKED,
    OTS_OACP_RESP_OPERATION_FAILED
}wiced_bt_ots_oacp_resp_result_t;

/* OTS Feature - Object List Control Point Features Supported Bit Mask */
typedef enum
{
    OTS_OLCP_GO_TO_SUPPORT                 =  ( 1 << 0 ),
    OTS_OLCP_ORDER_SUPPORT                 =  ( 1 << 1 ),
    OTS_OLCP_REQ_NUM_OBJECTS_SUPPORT       =  ( 1 << 2 ),
    OTS_OLCP_CLEAR_MARKING_SUPPORT         =  ( 1 << 3 )
}wiced_bt_ots_olcp_feature_support_t;

/* OTS Object List Control Point Procedures */
typedef enum
{
    OTS_OLCP_RFU              = 0x00,
    OTS_OLCP_FIRST,
    OTS_OLCP_LAST,
    OTS_OLCP_PREVIOUS,
    OTS_OLCP_NEXT,
    OTS_OLCP_GOTO,
    OTS_OLCP_ORDER,
    OTS_OLCP_REQ_NUM_OBJECTS,
    OTS_OLCP_CLEAR_MARKING,
    OTS_OLCP_RESPONSE         = 0x70
}wiced_bt_ots_olcp_procedure_t;

/* OTS Object List Control Point List Sort Order Values */
typedef enum
{
    OTS_OLCP_SORT_RFU,
    OTS_OLCP_SORT_OBJECT_NAME_ASCENDING,
    OTS_OLCP_SORT_OBJECT_TYPE_ASCENDING,
    OTS_OLCP_SORT_OBJECT_CURRENT_SIZE_ASCENDING,
    OTS_OLCP_SORT_OBJECT_FIRST_CREATE_ASCENDING,
    OTS_OLCP_SORT_OBJECT_LAST_MODIFY_ASCENDING,
    OTS_OLCP_SORT_OBJECT_NAME_DESCENDING = 0x11,
    OTS_OLCP_SORT_OBJECT_TYPE_DESCENDING,
    OTS_OLCP_SORT_OBJECT_CURRENT_SIZE_DESCENDING,
    OTS_OLCP_SORT_OBJECT_FIRST_CREATE_DESCENDING,
    OTS_OLCP_SORT_OBJECT_LAST_MODIFY_DESCENDING,
}wiced_bt_ots_olcp_list_sort_order_t;

/* OLCP Response Value result codes */
typedef enum
{
    OTS_OLCP_RESP_RFU         = 0x00,
    OTS_OLCP_RESP_SUCCESS,
    OTS_OLCP_OPCODE_NOT_SUPPORTED,
    OTS_OLCP_INVALID_PARAM,
    OTS_OLCP_RESP_OPERATION_FAILED,
    OTS_OLCP_RESP_OUT_OF_BOUNDS,
    OTS_OLCP_RESP_TOO_MANY_OBJECTS,
    OTS_OLCP_RESP_NO_OBJECT,
    OTS_OLCP_RESP_OBJECT_ID_NOT_FOUND,
}wiced_bt_ots_olcp_resp_result_t;

/* Mode Parameter for OACP Write Op Code */
typedef enum
{
    OTS_OACP_WRITE_MODE_TRUNCATE = 1 << 1,
}wiced_bt_ots_oacp_write_mode_t;

/* Object List Filter Characteristic Values */
typedef enum
{
    OTS_NO_FILTER = 0x00,
    OTS_NAME_STARTS_WITH,
    OTS_NAME_ENDS_WITH,
    OTS_NAME_CONTAINS,
    OTS_NAME_IS_EXACTLY,
    OTS_OBJECT_TYPE,
    OTS_CREATED_BETWEEN,
    OTS_MODIFIED_BETWEEN,
    OTS_CURRENT_SIZE_BETWEEN,
    OTS_ALLOCATED_SIZE_BETWEEN,
    OTS_MARKED_OBJECTS
}wiced_bt_ots_object_list_filter_value_t;

/* Object Transfer Service Characteristics name */
typedef enum
{
    OTS_FEATURE,
    OBJECT_NAME,
    OBJECT_TYPE,
    OBJECT_SIZE,
    OBJECT_FIRST_CREATED,
    OBJECT_LAST_MODIFIED,
    OBJECT_ID,
    OBJECT_PROPERTIES,
    OBJECT_ACTION_CONTROL_POINT,
    OBJECT_LIST_CONTROL_POINT,
    OBJECT_LIST_FILTER_1,
    OBJECT_LIST_FILTER_2,
    OBJECT_LIST_FILTER_3,
    OBJECT_CHANGED
}wiced_bt_ots_characteristics_t;

/* Object Transfer Channel Events */
typedef enum
{
    OTP_CHANNEL_OPEN,
    OTP_CHANNEL_CLOSE,
    OTP_CHANNEL_DATA_TX_DONE,
    OTP_CHANNEL_DATA_RX,
    OTP_CHANNEL_CONGESTION
}wiced_bt_otp_channel_event_t;

/* Coordinated Universal Time (UTC) */
typedef struct
{
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hours;
    uint8_t  minutes;
    uint8_t  seconds;
}utc_t;

/* Optional features supported by the Server */
typedef struct
{
    uint32_t oacp_features;
    uint32_t olcp_features;
}ots_feature_characteristic_t;

/* Name of the current object */
typedef struct
{
    uint8_t length;
    uint8_t name[120];
}object_name_characteristic_t;

/* Type of the current object */
typedef struct
{
    uint32_t current_size;
    uint32_t allocated_size;
}object_size_characteristic_t;

/* Metadata of the current object*/
typedef struct
{
    object_name_characteristic_t object_name;
    wiced_bt_uuid_t              object_type;
    object_size_characteristic_t object_size;
    utc_t                        object_first_created;
    utc_t                        object_last_modified;
    uint32_t                     object_id;
    uint32_t                     object_properties;
}object_metadata_characteristics_t;

/* OACP characteristic */
typedef struct
{
    uint8_t op_code;
    uint8_t params[20];
}oacp_characteristics_t;

/* OLCP characteristic */
typedef struct
{
    uint8_t op_code;
    uint8_t params[6];
}olcp_characteristics_t;

/* Object List Filter characteristic */
typedef struct
{
    uint8_t filter_value; /* wiced_bt_ots_object_list_filter_value_t */
    uint8_t param_size;
    uint8_t params[120];
}object_list_filter_characteristics_t;

/* Object Changed characteristic */
typedef struct
{
    uint8_t flags;
    uint8_t object_id[6];
}object_changed_characteristic_t;

/* Object Transfer Service characteristics */
typedef union
{
    ots_feature_characteristic_t         ots_feature;
    object_name_characteristic_t         object_name;
    wiced_bt_uuid_t                      object_type;
    object_size_characteristic_t         object_size;
    utc_t                                object_first_created;
    utc_t                                object_last_modified;
    uint32_t                             object_id;
    uint32_t                             object_properties;
    oacp_characteristics_t               oacp;
    olcp_characteristics_t               olcp;
    object_list_filter_characteristics_t filter[3];
    object_changed_characteristic_t      object_changed;
}object_characteristics_t;

/* OTS OACP create procedure parameters */
typedef struct
{
    uint32_t        object_size;
    wiced_bt_uuid_t object_type;
}ots_oacp_create_procedure_data_t;

/* OTS OACP calculate checksum procedure parameters */
typedef struct
{
    uint32_t        offset;
    uint32_t        length;
}ots_oacp_calc_checksum_procedure_data_t;

/* OTS OACP read procedure parameters */
typedef struct
{
    uint32_t        offset;
    uint32_t        length;
}ots_oacp_read_procedure_data_t;

/* OTS OACP write procedure parameters */
typedef struct
{
    uint32_t        offset;
    uint32_t        length;
    uint8_t         mode;
}ots_oacp_write_procedure_data_t;

/* OTS OACP execute procedure parameters. This parameter is defined by the application */
typedef struct
{
    uint8_t        param_len;
    uint8_t        param[1];
}ots_oacp_execute_procedure_data_t;

/* OTS OACP procedure parameters */
typedef union
{
    ots_oacp_create_procedure_data_t          oacp_create;
    ots_oacp_calc_checksum_procedure_data_t   oacp_calc_checksum;
    ots_oacp_execute_procedure_data_t         oacp_execute;
    ots_oacp_read_procedure_data_t            oacp_read;
    ots_oacp_write_procedure_data_t           oacp_write;
}wiced_bt_ots_oacp_procedure_data_t;

/* OTS OACP response parameters */
typedef struct
{
    uint8_t  param_len;
    uint8_t  p_param[1];
}wiced_bt_ots_oacp_response_t;

/* Object transfer channel open event parameters */
typedef struct
{
    wiced_bt_device_address_t bd_addr;
    uint16_t                  cid;
    uint16_t                  peer_mtu;
    uint16_t                  result;
}wiced_bt_otp_channel_open_t;

/* Object transfer channel close event parameters */
typedef struct
{
    uint16_t                  cid;
}wiced_bt_otp_channel_close_t;

/* Object transfer channel data indication event parameters */
typedef struct
{
    uint16_t                  cid;
    uint32_t                  data_len;
    uint8_t*                  p_data;
}wiced_bt_otp_channel_data_ind_t;

/* Object transfer channel congestion event parameters */
typedef struct
{
    uint16_t                  cid;
    wiced_bool_t              congested;
}wiced_bt_otp_channel_congestion_t;

/* Object transfer channel data tx complete event parameters */
typedef struct
{
    uint16_t                  cid;
    uint16_t                  num_sdu;
}wiced_bt_otp_channel_tx_complete_t;

/* Object transfer channel event data */
typedef union
{
    wiced_bt_otp_channel_open_t        channel_open;
    wiced_bt_otp_channel_close_t       channel_close;
    wiced_bt_otp_channel_data_ind_t    data_ind;
    wiced_bt_otp_channel_congestion_t  congestion;
    wiced_bt_otp_channel_tx_complete_t tx_complete;
}wiced_bt_otp_channel_event_data_t;

/******************************************************************************
*
* Function Name: wiced_bt_otp_channel_callback_t
*
* \brief This callback is invoked to notify the application of Object Transfer Channel events.
*
* \param[in]  event        : Object Transfer Channel event
* \param[in]  p_event_data : Object Transfer Channel event data
*
* \return None.
*
******************************************************************************/
typedef void (*wiced_bt_otp_channel_callback_t)( wiced_bt_otp_channel_event_t event, wiced_bt_otp_channel_event_data_t* p_event_data );

/******************************************************************************
*
* Function Name: wiced_bt_otp_channel_open
*
* \brief This API is used by the Object client to open the Object Transfer
*        Channel with the intended Object Server. Channel open indication is received
*        using a registered channel callback.
*
* \param[in]  bd_addr              : Object Server bluetooth address
* \param[in]  bd_addr_type         : Object Server bluetooth address type
* \param[in]  mtu                  : MTU to be used for data transfer over the
*                                      Object Transfer Channel
* \param[in]  req_security         : Security required. Refer wiced_bt_ble_sec_flags_e
* \param[in]  req_encr_key_size    : Encryption key size, 1<= Key Size <= 16.
*
* \return wiced_result_t
*
******************************************************************************/
wiced_result_t wiced_bt_otp_channel_open( wiced_bt_device_address_t bd_addr, wiced_bt_ble_address_type_t bd_addr_type, uint16_t rx_mtu, uint8_t req_security, uint8_t req_encr_key_size );

/******************************************************************************
*
* Function Name: wiced_bt_otp_channel_close
*
*
* \brief This API is used by the Object Server and/or Object client to close
*        the Object Transfer Channel. Channel close indication is received
*        using a registered channel callback.
*
* \param[in]  lcid      : Channel Identifier
*
* \return wiced_result_t
*
******************************************************************************/
wiced_result_t wiced_bt_otp_channel_close( uint16_t lcid );

/******************************************************************************
*
* Function Name: wiced_bt_otp_channel_send_data
*
* \brief This API is used by the Object Server and/or Object client to send data
*        over the Object Transfer Channel.
*
* \param[in]  lcid      : Channel Identifier
* \param[in]  p_data    : Data to be sent
* \param[in]  data_len  : Data length
*
* \return L2CAP_DATAWRITE_SUCCESS, if data accepted, else FALSE
*         L2CAP_DATAWRITE_CONGESTED, if data accepted and the channel is congested
*         L2CAP_DATAWRITE_FAILED, if an error.
*
******************************************************************************/
uint8_t wiced_bt_otp_channel_send_data( uint16_t lcid, uint8_t* p_data, uint32_t data_len );

/******************************************************************************
*
* Function Name: wiced_bt_otp_channel_get_peer_mtu
*
* \brief This API is used by the Object Server and/or Object client to get the peer MTU.
*
* \param[in]  lcid      : Channel Identifier
*
* \return Peer MTU.
*
******************************************************************************/
uint16_t wiced_bt_otp_channel_get_peer_mtu( uint16_t lcid );

/******************************************************************************
*
* Function Name: wiced_bt_otp_calculate_checksum
*
* \brief This API is used by the Object Server and/or Object client to calculate the
*        checksum of the object data received/sent over the Object Transfer Channel.
*
* \param[in]  p_data    : Data
* \param[in]  data_len  : Data length
*
* \return 32-bit checksum.
*
******************************************************************************/
uint32_t wiced_bt_otp_calculate_checksum( uint8_t* p_data, uint32_t data_len );

#ifdef __cplusplus
}
#endif

#endif /* WICED_BT_OTP_H_ */

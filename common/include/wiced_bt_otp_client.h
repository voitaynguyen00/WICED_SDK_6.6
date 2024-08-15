/***************************************************************************//**
* \file <wiced_bt_otp_client.h>
*
* \brief
* 	Object Transfer Profile Client.
*
* \details
* 		Object Client Performs the feature discovery as part of service and characteristic discovery.
*    	Application should read the object property characteristic before performing any oacp/olcp
*       procedures.
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

#ifndef WICED_BT_OTP_CLIENT_H
#define WICED_BT_OTP_CLIENT_H

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************************************
*          Type Definitions
******************************************************************************/

/* OACP timeout in seconds */
#define OTP_OACP_TIMEOUT                30

/* OLCP timeout in seconds */
#define OTP_OLCP_TIMEOUT                30

/* OTP Client result codes */
typedef enum
{
    SUCCESS = WICED_SUCCESS,   /* Success */
    MANDATORY_CHAR_NOT_FOUND,  /* Mandatory service characteristics not found in the Object Server */
    CHAR_DESCRIPTORS_NOT_FOUND,/* Object Transfer Service characteristic descriptor not found */
    GATT_OPERATION_FAILED,     /* GATT operation failed */
    CHANNEL_OPEN_FAILED,       /* Object transfer channel open failed */
    OPCODE_NOT_SUPPORTED,      /* OACP/OLCP procedure not supported in the Object Server */
    PROCEDURE_NOT_PERMITTED,   /* Procedure not permitted by the object in the Object Server */
    OBJECT_DATA_TX_FAILED,     /* Object Data Transfer to server failed */
    INVALID_PARAMETER,         /* Invalid parameter in the request */
    OACP_RESPONSE_FAILURE,     /* OACP response failure received from the Object Server */
    OTS_FEATURE_READ_FAILED,   /* OTS Feature characteristic read failed */
    OTP_CLIENT_BUSY,           /* OTP Client in busy state */
}wiced_bt_otp_client_result_code_t;

/* OTP Client result passed to the application */
typedef enum
{
    OTP_CLIENT_DISCOVERY,             /* OTS characteristic and descriptor discovery result */
    OTP_CLIENT_CHARACTERISTIC_READ,   /* OTS characteristic read result */
    OTP_CLIENT_CHARACTERISTIC_WRITE,  /* OTS characteristic write result */
    OTP_CLIENT_READ_API_RESULT,       /* OTS characteristic read object API result */
    OTP_CLIENT_WRITE_API_RESULT,      /* OTS characteristic write object API result*/
    OTP_CLIENT_OACP_WRITE_RESPONSE,   /* OACP write command response */
    OTP_CLIENT_OACP_PROCEDURE_TIMEOUT /* OTP Client OACP Procedure Timeout */
}wiced_bt_otp_client_event_t;

/* Configurations to be specified by the application on initialization */
typedef struct
{
    uint16_t mtu;               /* MTU supported for data transfer over object transfer channel.L2CAP minimum MTU is 48 octets */
    uint8_t  req_security;      /* Security required. Refer wiced_bt_ble_sec_flags_e */
    uint8_t  req_encr_key_size; /* Encryption key size, 1<= Key Size <= 16 */
}object_client_config_t;

/* Service and Feature Discovery result */
typedef struct
{
    uint16_t                        conn_id;                /* Connection Identifier */
    ots_feature_characteristic_t    ots_feature;            /* Server OTS Feature characteristic is read as part of discovery procedure */
    wiced_result_t                  result;                 /* Discovery Result */
}wiced_bt_otp_client_discovery_result_t;

/* Object Characteristic read result */
typedef struct
{
    uint16_t                       conn_id;    /* Connection Identifier */
    wiced_bt_gatt_status_t         result;     /* GATT operation status */
    wiced_bt_ots_characteristics_t char_name;  /* Object Characteristic name */
    object_characteristics_t       char_value; /* Object Characteristic value */
}wiced_bt_otp_client_char_read_result_t;

/* Object Characteristic write result */
typedef struct
{
    uint16_t                       conn_id;    /* Connection Identifier */
    wiced_bt_gatt_status_t         result;     /* GATT operation status */
}wiced_bt_otp_client_char_write_result_t;

/* Object Server read object content result */
typedef struct
{
    uint16_t                          conn_id;       /* Connection Identifier */
    wiced_bt_otp_client_result_code_t result;        /* Result code of the read operation */
    wiced_bt_ots_oacp_resp_result_t   oacp_response; /* OACP response value. This will be updated when the
                                                          wiced_bt_otp_client_result_code_t is OACP_RESPONSE_FAILURE */
}wiced_bt_otp_client_read_result_t;

/* Object Server write object content result */
typedef struct
{
    uint16_t                          conn_id;  /* Connection Identifier */
    wiced_bt_otp_client_result_code_t result;   /* Result code of the write operation */
}wiced_bt_otp_client_write_result_t;

/* Object Server OACP write procedure result */
typedef struct
{
    uint16_t                        conn_id;       /* Connection Identifier */
    wiced_bt_gatt_status_t          status;        /* GATT operation status */
    wiced_bt_ots_oacp_procedure_t   procedure;     /* OACP procedure performed */
    wiced_bt_ots_oacp_resp_result_t response;      /* OACP procedure response result code */
    wiced_bt_ots_oacp_response_t    response_param;/* OACP procedure response parameter */
}wiced_bt_otp_client_oacp_result_t;

/* OTP Client OACP Procedure Timeout */
typedef struct
{
    wiced_bt_ots_oacp_procedure_t   procedure;     /* OACP procedure performed */
}wiced_bt_otp_client_oacp_proc_timeout_t;

/* Object Client events passed to the application */
typedef union
{
    wiced_bt_otp_client_discovery_result_t  discovery;    /* Discovery Result */
    wiced_bt_otp_client_char_read_result_t  char_read;    /* Object Characteristic Read Result */
    wiced_bt_otp_client_char_write_result_t char_write;   /* Object Characteristic Write Result except OACP/OLCP */
    wiced_bt_otp_client_read_result_t       read;         /* Object Read Result */
    wiced_bt_otp_client_write_result_t      write;        /* Object Write Result */
    wiced_bt_otp_client_oacp_result_t       oacp;         /* OACP Write procedure result */
    wiced_bt_otp_client_oacp_proc_timeout_t oacp_timeout; /* OACP procedure timeout */
}wiced_bt_otp_client_event_data_t;

typedef void (*wiced_bt_otp_client_event)(wiced_bt_otp_client_event_t event, wiced_bt_otp_client_event_data_t* p_event_data );

typedef struct
{
    wiced_bt_otp_client_event       p_event;             /* Object Client event callback */
    wiced_bt_otp_channel_callback_t p_otp_channel_cback; /* Object Transfer Channel event callback */
}wiced_bt_otp_client_callback_t;

/******************************************************************************
*
* \name wiced_bt_otp_client_init
* \details This API shall be used by the higher layer application to initialize the
* Object Client
*
* @param[in]  p_cfg        : Configurations that the object client should use
* @param[in]  p_cback      : Callbacks registered by the application to
*                            receive the events from the Object Client
*
* @return wiced_result_t
*
******************************************************************************/
wiced_result_t wiced_bt_otp_client_init( object_client_config_t* p_cfg, wiced_bt_otp_client_callback_t* p_cback );

/******************************************************************************
*
* \name wiced_bt_otp_client_discover
* \details This API is used by the higher layer application to perform OTS
*          characteristics discovery and characteristic descriptor discovery.
*          On discovery complete, registered application callback is invoked with the result of operation.
*
* @param[in]  conn_id      :  GATT connection ID
* @param[in]  start_handle :  Start GATT handle of the Object Transfer Service
* @param[in]  end_handle   :  End GATT handle of the Object Transfer Service
*
* @return wiced_bt_gatt_status_t
*
******************************************************************************/
wiced_bt_gatt_status_t wiced_bt_otp_client_discover( uint16_t conn_id, uint16_t start_handle, uint16_t end_handle );

/******************************************************************************
*
* \name wiced_bt_otp_client_discovery_result
* \details This API is used by the higher layer application to pass the discovery results of OTS Service
*          to the OTP Client library. OTP Client Library uses this to find the handles of OTS Service
*          characteristics and characteristic descriptors.
*
* @param           p_data   : Discovery result data received from the stack
*
* @return          None.
*
******************************************************************************/
void wiced_bt_otp_client_discovery_result( wiced_bt_gatt_discovery_result_t *p_data );

/******************************************************************************
*
* \name wiced_bt_otp_client_discovery_complete
* \details This API is used by the higher layer application to pass the discovery complete
*          callbacks for OTS Service to the OTP Client library.
*
* @param           p_data   : Discovery complete data received from the stack
*
* @return          None.
*
******************************************************************************/
void wiced_bt_otp_client_discovery_complete( wiced_bt_gatt_discovery_complete_t *p_data );

/******************************************************************************
*
* \name wiced_bt_otp_client_configure_oacp_indication
* \details This API is used by the higher layer application above the object client to
*          configure the OACP indications in the Object Server.
*
* @param[in]  conn_id            : GATT connection ID
* @param[in]  client_char_config : Client characteristic config value
*
* @return  wiced_bt_gatt_status_t
*
******************************************************************************/
wiced_bt_gatt_status_t wiced_bt_otp_client_configure_oacp_indication(  uint16_t conn_id, uint16_t client_char_config );

/******************************************************************************
*
* \name wiced_bt_otp_client_configure_oacp_indication
* \details This API is used by the higher layer application above the object client to
*          perform various OACP procedures in the object server. This function takes care of
*          enabling the OACP indications in the server if not enabled already.
*
* @param[in]  conn_id        :  GATT connection ID
* @param[in]  oacp_procedure :  OACP procedure to be performed
* @param[in]  p_data         :  OACP Procedure parameters
*
* @return wiced_bt_gatt_status_t
*
******************************************************************************/
wiced_bt_gatt_status_t wiced_bt_otp_client_configure_oacp_indication( uint16_t conn_id, wiced_bt_ots_oacp_procedure_t oacp_procedure, wiced_bt_ots_oacp_procedure_data_t* p_data );

/******************************************************************************
*
* \name wiced_bt_otp_client_configure_olcp_indication
* \details This API is used by the higher layer application above the object client to
*          configure the OLCP indications in the Object Server.
*
* @param[in]  conn_id            : GATT connection ID
* @param[in]  client_char_config : Client characteristic config value
*
* @return  wiced_bt_gatt_status_t
*
******************************************************************************/
wiced_bt_gatt_status_t wiced_bt_otp_client_configure_olcp_indication(  uint16_t conn_id, uint16_t client_char_config );

/******************************************************************************
*
* \name wiced_bt_otp_client_configure_object_changed_indication
* \details This API is used by the higher layer application above the object client to
*          configure the Object Changed indications in the Object Server.
*
* @param[in]  conn_id            : GATT connection ID
* @param[in]  client_char_config : Client characteristic config value
*
* @return  wiced_bt_gatt_status_t
*
******************************************************************************/
wiced_bt_gatt_status_t wiced_bt_otp_client_configure_object_changed_indication(  uint16_t conn_id, uint16_t client_char_config );

/******************************************************************************
*
* \name wiced_bt_otp_client_read_characteristic
* \details This API is used by the higher layer application above the object client to read
*          the characteristics of the Current Object from the Object Server.
*
* @param[in]  conn_id                  :  GATT connection ID
* @param[in]  characteristic           :  Characteristic to be read
*
* @return wiced_bt_gatt_status_t
*
******************************************************************************/
wiced_bt_gatt_status_t wiced_bt_otp_client_read_characteristic( uint16_t conn_id, uint8_t characteristic );

/******************************************************************************
*
* \name wiced_bt_otp_client_read_metadata
* \details This API is used by the higher layer application above the object client to
*          read the metadata of the Current Object from the Object Server.
*
* @param[in]  conn_id                  :  GATT connection ID
* @param[in]  meta_data_characteristic :  Object Metadata Characteristic to be read
*
* @return wiced_bt_gatt_status_t
*
******************************************************************************/
wiced_bt_gatt_status_t wiced_bt_otp_client_read_metadata( uint16_t conn_id, uint8_t meta_data_characteristic );

/******************************************************************************
*
* \name wiced_bt_otp_client_write_metadata
* \details This API is used by the higher layer application above the object client to
*          write values to the metadata of the  Current Object on the Object Server.
*
* @param[in]  conn_id                  :  GATT connection ID
* @param[in]  meta_data_characteristic :  Object Metadata Characteristic to be written
* @param[in]  p_data                   :  Object Metadata Characteristic value
*
* @return wiced_bt_gatt_status_t
*
******************************************************************************/
wiced_bt_gatt_status_t wiced_bt_otp_client_write_metadata( uint16_t conn_id, uint8_t meta_data_characteristic, void* p_data );

/******************************************************************************
*
* \name wiced_bt_otp_client_read
* \details This API is used by the higher layer application above the object client to
*          read part or the whole of the contents of the Current Object from the Object Server.
*          Application reads the object size characteristic of the current object before
*          using this API.
*
* @param[in]  lcid   :  L2CAP COC channel identifier
* @param[in]  offset :  Offset from where the read starts
* @param[in]  length :  Object data length to be read
*
* @return wiced_result_t
*
******************************************************************************/
void wiced_bt_otp_client_read( uint16_t conn_id, uint32_t offset, uint32_t length );

/******************************************************************************
*
* \name wiced_bt_otp_client_prepare_for_write
* \details This API is used by the higher layer application above the object client to
*          perform the sub procedures required to be done before writing to the object.
*          Application reads the object size characteristic of the current object and
*          execute wiced_bt_otp_client_prepare_write() before using this API.
*
* @param[in]  conn_id : GATT connection ID
* @param[in]  offset  : Offset from where the write starts
* @param[in]  length  : Object data length
* @param[in]  length  : Object Write mode
*
* @return wiced_result_t
*
******************************************************************************/
void wiced_bt_otp_client_prepare_for_write( uint16_t conn_id,  uint32_t offset,uint32_t length, uint8_t mode );

/******************************************************************************
*
* \name wiced_bt_otp_client_write
* \details This API is used by the higher layer application above the object client to
*          write or overwrite a part or the whole of the contents of the Current Object.
*          Application reads the object size characteristic of the current object and
*          execute wiced_bt_otp_client_prepare_write() before using this API.
*
* @param[in]  lcid   :  L2CAP COC channel identifier
* @param[in]  length :  Object data length
* @param[in]  p_data :  Pointer to the object data
*
* @return wiced_result_t
*
******************************************************************************/
wiced_result_t wiced_bt_otp_client_write( uint16_t lcid, uint32_t length, uint8_t* p_data );

/******************************************************************************
*
* \name wiced_bt_otp_client_connection_up
* \details The application calls this function when BLE connection with a peer
*          device is established.
*
* @param[in]  p_conn_status  : pointer to a wiced_bt_gatt_connection_status_t
*                              which includes the peer bda and connection ID
*
* @return None.
*
******************************************************************************/
void wiced_bt_otp_client_connection_up( wiced_bt_gatt_connection_status_t *p_conn_status );

/******************************************************************************
*
* \name wiced_bt_otp_client_connection_down
* \details The application calls this function when BLE connection with a peer
*          device is disconnected.
*
* @param[in]  p_conn_status  : pointer to a wiced_bt_gatt_connection_status_t
*                                   which includes the peer bda and connection ID.
* @return None.
*
******************************************************************************/
void wiced_bt_otp_client_connection_down( wiced_bt_gatt_connection_status_t *p_conn_status );

/******************************************************************************
*
* \name wiced_bt_otp_client_read_response
* \details The application calls this function when it receives GATT Write Response
*          for the attribute handle which belongs to the Object Transfer service.
*
* @param[in]  p_data  : The pointer to a GATT operation complete data structure.
* @return   None.
*
******************************************************************************/
void wiced_bt_otp_client_read_response( wiced_bt_gatt_operation_complete_t *p_data );

/******************************************************************************
*
* \name wiced_bt_otp_client_write_response
* \details The application calls this function when it receives GATT Write Response
*          for the attribute handle which belongs to the Object Transfer service.
*
* @param[in]  p_data  : The pointer to a GATT operation complete data structure.
*
* @return  None.
*
******************************************************************************/
void wiced_bt_otp_client_write_response( wiced_bt_gatt_operation_complete_t *p_data );

/******************************************************************************
*
* \name wiced_bt_otp_client_indication
* \details The application calls this function when it receives GATT Indication
*          for the attribute handle which belongs to the Object Transfer service.
*
* @param[in]  p_data  : The pointer to a GATT operation complete data structure.
*
* @return  None.
*
******************************************************************************/
void wiced_bt_otp_client_indication( wiced_bt_gatt_operation_complete_t *p_data );

#ifdef __cplusplus
}
#endif

#endif /* _WICED_BT_OTP_CLIENT_H_ */


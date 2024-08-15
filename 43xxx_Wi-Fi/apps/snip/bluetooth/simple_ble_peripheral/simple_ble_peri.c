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
 *
 * This is a very simple BLE Peripheral
 * The Peripheral advertises until it is connected
 * Then it responds to read and write requests to the GATT Database
 *
 * You should add to the GATT Database
 * - defining your custom handles in simple_ble_peri_db_tags (simple_ble_peri.h)
 * - creating a place to store the data in host_info_t
 * - adding the fields to the GATT Database by updating gatt_user_attributes
 * - adding the fields to the GATT Database by updating simple_ble_peri_gatt_database
 *
 *  You should update the advertising packet
 *  You should deal with writes in simple_ble_peri_gatt_server_write_request_handler
 *  You can optionally deal with read in simple_ble_peri_gatt_server_read_request_handler
 */


#include <string.h>

#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced.h"

#include "bt_target.h"
#include "wiced_bt_stack.h"
#include "gattdefs.h"
#include "sdpdefs.h"
#include "simple_ble_peri.h"
#include "wwd_debug.h"

#include "ble_event_map.h"

/******************************************************************************
 *                                Constants
******************************************************************************/

/******************************************************************************
 *                           Function Prototypes
 ******************************************************************************/
static void                     simple_ble_peri_application_init                   ( void );
static wiced_bt_gatt_status_t   simple_ble_peri_gatts_connection_status_handler    ( wiced_bt_gatt_connection_status_t *p_status );
static wiced_bt_gatt_status_t   simple_ble_peri_gatts_connection_up                ( wiced_bt_gatt_connection_status_t *p_status );
static wiced_bt_gatt_status_t   simple_ble_peri_gatts_connection_down              ( wiced_bt_gatt_connection_status_t *p_status );
static wiced_result_t           simple_ble_peri_management_callback                ( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
static wiced_bt_gatt_status_t   simple_ble_peri_gatts_callback                     ( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data);
static wiced_bt_gatt_status_t   simple_ble_peri_gatt_server_read_request_handler   ( uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data );
static wiced_bt_gatt_status_t   simple_ble_peri_gatt_server_write_request_handler  ( uint16_t conn_id, wiced_bt_gatt_write_t * p_data );
static void                     simple_ble_peri_set_advertisement_data             ( void );



/******************************************************************************
 *                                Structures
 ******************************************************************************/

typedef struct
{
    BD_ADDR         remote_addr;                            /* remote peer device address */
    uint16_t        conn_id;                                /* connection ID referenced by the stack */
} simple_ble_peri_state_t;

typedef PACKED struct
{
    /* Todo: put in your local variables for the GATT Database */

} host_info_t;

typedef struct
{
    uint16_t        handle;
    uint16_t        attr_len;
    void*           p_attr;
} attribute_t;

/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/

static uint8_t simple_ble_peri_device_name[]          = "Hello";
static uint8_t simple_ble_peri_appearance_name[2]     = { BIT16_TO_8(APPEARANCE_GENERIC_TAG) };

static simple_ble_peri_state_t     simple_ble_peri_state;

attribute_t gatt_user_attributes[] =
{
    { HANDLE_BASICPERI_GAP_SERVICE_CHAR_DEV_NAME_VAL,       sizeof( simple_ble_peri_device_name ),         simple_ble_peri_device_name },
    { HANDLE_BASICPERI_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL, sizeof(simple_ble_peri_appearance_name),       simple_ble_peri_appearance_name },

    // Todo: Add your attributes to the GATT User Attributes


};

/******************************************************************************
 *                                GATT DATABASE
 ******************************************************************************/
/*
 * This is the GATT database for the Hello Sensor application.  It defines
 * services, characteristics and descriptors supported by the sensor.  Each
 * attribute in the database has a handle, (characteristic has two, one for
 * characteristic itself, another for the value).  The handles are used by
 * the peer to access attributes, and can be used locally by application for
 * example to retrieve data written by the peer.  Definition of characteristics
 * and descriptors has GATT Properties (read, write, notify...) but also has
 * permissions which identify if and how peer is allowed to read or write
 * into it.  All handles do not need to be sequential, but need to be in
 * ascending order.
 */
const uint8_t simple_ble_peri_gatt_database[]=
{
        /* Declare mandatory GATT service */
        PRIMARY_SERVICE_UUID16( HANDLE_BASICPERI_GATT_SERVICE, UUID_SERVCLASS_GATT_SERVER ),

        /* Declare mandatory GAP service. Device Name and Appearance are mandatory
         * characteristics of GAP service                                        */
        PRIMARY_SERVICE_UUID16( HANDLE_BASICPERI_GAP_SERVICE, UUID_SERVCLASS_GAP_SERVER ),

        /* Declare mandatory GAP service characteristic: Dev Name */
        CHARACTERISTIC_UUID16( HANDLE_BASICPERI_GAP_SERVICE_CHAR_DEV_NAME, HANDLE_BASICPERI_GAP_SERVICE_CHAR_DEV_NAME_VAL,
                GATT_UUID_GAP_DEVICE_NAME, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),

                /* Declare mandatory GAP service characteristic: Appearance */
        CHARACTERISTIC_UUID16( HANDLE_BASICPERI_GAP_SERVICE_CHAR_DEV_APPEARANCE, HANDLE_BASICPERI_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,
                GATT_UUID_GAP_ICON, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE ),


                // Todo: Add your custom services to the GATT Database
};
/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

/*
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that BT device is ready.
 */
void application_start( void )
{
    wiced_core_init();
    WPRINT_APP_INFO( ( "---------FCN: application_start\n" ) );

    /* Register call back and configuration with stack */
    wiced_bt_stack_init( simple_ble_peri_management_callback ,  &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools );
}

/*
 * This function is executed in the BTM_ENABLED_EVT management callback.
 */
static void simple_ble_peri_application_init( void )
{
    wiced_bt_gatt_status_t gatt_status;
    wiced_result_t         result;

    WPRINT_APP_INFO( ( "---------FCN: simple_ble_peri_application_init\n" ) );

    /* Register with stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register( simple_ble_peri_gatts_callback );

    WPRINT_APP_INFO(( "wiced_bt_gatt_register: %d\n", gatt_status ));

    /*  Tell stack to use our GATT database */
    gatt_status =  wiced_bt_gatt_db_init( simple_ble_peri_gatt_database, sizeof( simple_ble_peri_gatt_database ) );

    WPRINT_APP_INFO( ( "wiced_bt_gatt_db_init %d\n", gatt_status ) );

    /* Set the advertising parameters and make the device discoverable */
    simple_ble_peri_set_advertisement_data();

    result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );

    WPRINT_APP_INFO( ("called wiced_bt_start_advertisements %d\n", result ) );

}

/*
 * Setup advertisement data with 16 byte UUID and device name
 */
void simple_ble_peri_set_advertisement_data(void)
{
    WPRINT_APP_INFO(("---------FCN: simple_ble_peri_set_advertisement_data\n"));
    wiced_result_t              result;
    wiced_bt_ble_advert_elem_t  adv_elem[3];
    uint8_t ble_advertisement_flag_value        = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t num_elem                            = 0;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = 1;
    adv_elem[num_elem].p_data       = &ble_advertisement_flag_value;
    num_elem ++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = strlen((const char *)wiced_bt_cfg_settings.device_name);
    WPRINT_APP_INFO( ("wiced_bt_cfg_settings.device_name:%s\n", wiced_bt_cfg_settings.device_name));
    adv_elem[num_elem].p_data       = (uint8_t *)wiced_bt_cfg_settings.device_name;
    num_elem++;

    // Todo: Add your information to the Advertising Packet

    result = wiced_bt_ble_set_raw_advertisement_data( num_elem, adv_elem );

    WPRINT_APP_INFO( ( "wiced_bt_ble_set_advertisement_data %d\n", result ) );
}


/*
 * simple_ble_peri bt/ble link management callback
 */
static wiced_result_t simple_ble_peri_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t                   result = WICED_BT_SUCCESS;
    wiced_bt_ble_advert_mode_t*      p_mode;

    WPRINT_APP_INFO(("---------FCN: simple_ble_peri_management_callback: 0x%x=%s\n", event,getEventEnumBTM(event) ));

    switch( event )
    {
    /* Bluetooth  stack enabled */
    case BTM_ENABLED_EVT:
        simple_ble_peri_application_init( );
        break;

    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
        p_mode = &p_event_data->ble_advert_state_changed;
        WPRINT_APP_INFO(( "Advertisement State Change: %s = %d \n", getEventEnumBTM_BLE_ADVERT(*p_mode),*p_mode));
        if ( *p_mode == BTM_BLE_ADVERT_OFF )
        {
            if (simple_ble_peri_state.conn_id == 0)
                wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL );
        }
        break;

    default:
        break;
    }

    return result;
}

/*
 * Find attribute description by handle
 */
static attribute_t * simple_ble_peri_get_attribute( uint16_t handle )
{
    WPRINT_APP_INFO(("---------FCN: simple_ble_peri_get_attribute Handle=%X\n",handle ));

    int i;
    for ( i = 0; i <  sizeof( gatt_user_attributes ) / sizeof( gatt_user_attributes[0] ); i++ )
    {
        if ( gatt_user_attributes[i].handle == handle )
        {
            return ( &gatt_user_attributes[i] );
        }
    }
    WPRINT_APP_INFO(( "attribute not found:%x\n", handle ));
    return NULL;
}

/*
 * Process Read request or command from peer device
 */
static wiced_bt_gatt_status_t simple_ble_peri_gatt_server_read_request_handler( uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data )
{
    WPRINT_APP_INFO(("---------FCN: simple_ble_peri_gatt_server_read_request_handler Read Handle=0x%X\n", p_read_data->handle ));
    attribute_t *puAttribute;
    int          attr_len_to_copy;

    if ( ( puAttribute = simple_ble_peri_get_attribute(p_read_data->handle) ) == NULL)
    {
        WPRINT_APP_INFO(("read_hndlr attr not found hdl:%x\n", p_read_data->handle ));
        return WICED_BT_GATT_INVALID_HANDLE;
    }


    attr_len_to_copy = puAttribute->attr_len;


    if ( p_read_data->offset >= puAttribute->attr_len )
    {
        attr_len_to_copy = 0;
    }

    if ( attr_len_to_copy != 0 )
    {
        uint8_t *from;
        int      to_copy = attr_len_to_copy - p_read_data->offset;


        if ( to_copy > *p_read_data->p_val_len )
        {
            to_copy = *p_read_data->p_val_len;
        }

        from = ((uint8_t *)puAttribute->p_attr) + p_read_data->offset;
        *p_read_data->p_val_len = to_copy;

        memcpy( p_read_data->p_val, from, to_copy);
    }

    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process write request or write command from peer device
 */
static wiced_bt_gatt_status_t simple_ble_peri_gatt_server_write_request_handler( uint16_t conn_id, wiced_bt_gatt_write_t * p_data )
{
    wiced_bt_gatt_status_t result    = WICED_BT_GATT_SUCCESS;
//  uint8_t                *p_attr   = p_data->p_val;
//    uint8_t attribute_value = *(uint8_t *) p_data->p_val;

    WPRINT_APP_INFO(("---------FCN: write_handler: conn_id:%d hdl:0x%x prep:%d offset:%d len:%d\n ", conn_id, p_data->handle, p_data->is_prep, p_data->offset, p_data->val_len
    ));

    switch ( p_data->handle )
    {

    // Todo: Do something when your characteristics are being written


    default:
        result = WICED_BT_GATT_INVALID_HANDLE;
        break;
    }
    return result;
}

/* This function is invoked when connection is established */
static wiced_bt_gatt_status_t simple_ble_peri_gatts_connection_up( wiced_bt_gatt_connection_status_t *p_status )
{

    WPRINT_APP_INFO( ( "---------FCN: simple_ble_peri_gatts_connection_up  id:%d\n", p_status->conn_id) );

    /* Update the connection handler.  Save address of the connected device. */
    simple_ble_peri_state.conn_id = p_status->conn_id;
    memcpy(simple_ble_peri_state.remote_addr, p_status->bd_addr, sizeof(BD_ADDR));

    /* Stop advertising */
    wiced_bt_start_advertisements( BTM_BLE_ADVERT_OFF, 0, NULL );


    return WICED_BT_GATT_SUCCESS;
}

/*
 * This function is invoked when connection is lost
 */
static wiced_bt_gatt_status_t simple_ble_peri_gatts_connection_down( wiced_bt_gatt_connection_status_t *p_status )
{
    wiced_result_t result;

    WPRINT_APP_INFO( ( "---------FCN: simple_ble_peri_gatts_connection_down  conn_id:%d reason:%d\n", p_status->conn_id, p_status->reason ) );

    /* Resetting the device info */
    memset( simple_ble_peri_state.remote_addr, 0, 6 );
    simple_ble_peri_state.conn_id = 0;

    result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL );
    WPRINT_APP_INFO( ( "wiced_bt_start_advertisements %d\n", result ) );


    return WICED_BT_SUCCESS;
}

/*
 * Connection up/down event
 */
static wiced_bt_gatt_status_t simple_ble_peri_gatts_connection_status_handler( wiced_bt_gatt_connection_status_t *p_status )
{
    WPRINT_APP_INFO(("---------FCN: simple_ble_peri_gatts_connection_status_handler\n"));

 //   is_connected = p_status->connected;
    if ( p_status->connected )
    {
        return simple_ble_peri_gatts_connection_up( p_status );
    }

    return simple_ble_peri_gatts_connection_down( p_status );
}

/*
 * Process GATT request from the peer
 */
static wiced_bt_gatt_status_t simple_ble_peri_gatt_server_request_handler( wiced_bt_gatt_attribute_request_t *p_data )
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    WPRINT_APP_INFO(( "---------FCN: simple_ble_peri_gatt_server_request_handler. conn %d, type %d %s\n", p_data->conn_id, p_data->request_type,getEventEnumGATTS_REQ(p_data->request_type) ));

    switch ( p_data->request_type )
    {
    case GATTS_REQ_TYPE_READ:
        result = simple_ble_peri_gatt_server_read_request_handler( p_data->conn_id, &(p_data->data.read_req) );
        break;

    case GATTS_REQ_TYPE_WRITE:
        result = simple_ble_peri_gatt_server_write_request_handler( p_data->conn_id, &(p_data->data.write_req) );
        break;

    case GATTS_REQ_TYPE_MTU:
        WPRINT_APP_INFO(("Requested MTU: %d\n", p_data->data.mtu));
        break;

    default:

        break;
    }
    return result;
}

/*
 * Callback for various GATT events.  As this application performs only as a GATT server, some of
 * the events are ommitted.
 */
static wiced_bt_gatt_status_t simple_ble_peri_gatts_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data)
{

    WPRINT_APP_INFO(("---------FCN: simple_ble_peri_gatts_callback EVT=%s\n",getEventEnumGATT(event)));

    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    switch(event)
    {
    case GATT_CONNECTION_STATUS_EVT:
        result = simple_ble_peri_gatts_connection_status_handler( &p_data->connection_status );
        break;

    case GATT_ATTRIBUTE_REQUEST_EVT:
        result = simple_ble_peri_gatt_server_request_handler( &p_data->attribute_request );
        break;
    default:

        break;
    }

    return result;
}



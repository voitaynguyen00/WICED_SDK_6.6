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
 *  Connection functions
 *
 *  Provides connection methods for use by applications
 */

#include "wiced.h"
#include "amqp.h"
#include "amqp_internal.h"
#include "amqp_frame.h"
#include "amqp_open.h"
#include "amqp_manager.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t wiced_amqp_open( wiced_amqp_connection_instance *connection_instance)
{
    wiced_result_t ret;
    connection_instance->current_connection_state = AMQP_CONNECTION_START;
    if ( connection_instance->header_args.protocol_id == WICED_AMQP_PROTOCOL_ID_SASL )
    {
        connection_instance->is_sasl = 1;
    }
    ret = amqp_network_connect( &connection_instance->conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }

    connection_instance->current_connection_state = AMQP_CONNECTION_START;
    linked_list_init( &connection_instance->channel_endpoint );

    /* Make sure the static is valid to send */
    if ( connection_instance->is_sasl == WICED_TRUE )
    {
        return amqp_manager( WICED_AMQP_EVENT_SASL_HDR_SENT, connection_instance, 0, connection_instance );
    }
    else
    {
        return amqp_manager( WICED_AMQP_EVENT_HDR_SENT, connection_instance, 0, connection_instance );
    }

}

wiced_result_t wiced_amqp_close( wiced_amqp_connection_instance *connection_instance)
{
    /* Make sure the static is valid to send */
    return amqp_manager( WICED_AMQP_EVENT_CLOSE_SENT, connection_instance, 0, connection_instance );
}

/******************************************************
 *               Protocol Header Frames
 ******************************************************/

wiced_result_t amqp_frame_get_protocol_header( wiced_amqp_frame_t *frame, wiced_amqp_protocol_header_arg_t *args )
{
    WPRINT_LIB_DEBUG( ( "[ AMQP LIB ] : Parsing Amqp protocol header\r\n" ) );
    AMQP_BUFFER_GET_OCTET( &frame->buffer, args->protocol_id );
    AMQP_BUFFER_GET_OCTET( &frame->buffer, args->major );
    AMQP_BUFFER_GET_OCTET( &frame->buffer, args->minor );
    AMQP_BUFFER_GET_OCTET( &frame->buffer, args->revision );

    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_put_protocol_header( wiced_amqp_frame_t *frame, const wiced_amqp_protocol_header_arg_t *args )
{
    /* A long value carrying string AMQP */
    uint32_t amqp_str = AMQP_FRAME_HEADER_PROTOCOL_STR;
    AMQP_BUFFER_PUT_LONG( &frame->buffer, amqp_str );
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, args->protocol_id );
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, args->major );
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, args->minor );
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, args->revision );

    return WICED_SUCCESS;
}


wiced_result_t amqp_frame_get_open_performative( wiced_amqp_frame_t *frame, void *arg )
{
    wiced_amqp_packet_content *args = (wiced_amqp_packet_content *) arg;
    uint64_t temp = 0;
    uint32_t count = 0;

    get_fixed_frame( frame, &args->fixed_frame );
    AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );

    if ( temp == COMPOUND_TYPE_ONE_BYTE_LIST )
    {
        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );

        AMQP_BUFFER_GET_OCTET( &frame->buffer, count );
    }
    else if ( temp == COMPOUND_TYPE_FOUR_BYTE_LIST )
    {
        AMQP_BUFFER_GET_LONG( &frame->buffer, temp );

        AMQP_BUFFER_GET_LONG( &frame->buffer, count );
    }
    /*parse open frame contents*/
    if ( count )
    {
        uint8_t temp_count = 1;
        do
        {
            switch ( temp_count )
            {
                case 1:

                    AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
                    if ( temp == NULL_BYTE )
                    {
                        args->args.open_args.container_id = NULL;
                    }
                    else
                    {
                        ( ( &frame->buffer )->data )--;
                        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
                        AMQP_BUFFER_GET_SHORT_STRING( &frame->buffer, args->args.open_args.container_id, args->args.open_args.container_id_size );
                    }
                    break;
                case 2:

                    AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
                    if ( temp == NULL_BYTE )
                    {
                        args->args.open_args.host_name = NULL;
                    }
                    else
                    {
                        ( ( &frame->buffer )->data )--;
                        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
                        AMQP_BUFFER_GET_SHORT_STRING( &frame->buffer, args->args.open_args.host_name, args->args.open_args.host_name_size );
                    }
                    break;
                case 3:
                    AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
                    if ( temp == NULL_BYTE )
                    {
                        args->args.open_args.max_frame_size = 0;
                    }
                    else
                    {
                        AMQP_BUFFER_GET_LONG( &frame->buffer, args->args.open_args.max_frame_size );
                    }
                    break;
                case 4:
                    AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
                    if ( temp == NULL_BYTE )
                    {
                        args->args.open_args.channel_max = 0;
                    }
                    else
                    {
                        AMQP_BUFFER_GET_SHORT( &frame->buffer, args->args.open_args.channel_max );
                    }
                    break;
                case 5:
                    AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
                    if ( temp == NULL_BYTE )
                    {
                        args->args.open_args.idle_timeout = 0;
                    }
                    else
                    {
                        AMQP_BUFFER_GET_LONG( &frame->buffer, args->args.open_args.idle_timeout );
                    }
                    break;
                default:
                    break;
            }
            temp_count++;
        } while ( temp_count <= count );
    }

    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_put_open( wiced_amqp_frame_t *frame,  void* arg )
{
    uint8_t count = AMQP_OPEN_FRAME_LIST_COUNT;
    wiced_amqp_connection_instance *args = arg;
    uint8_t* temp1 = frame->buffer.data;

    amqp_fixed_frame fixed_frame;

    fixed_frame.channel = 0x00;
    fixed_frame.doff = 0x02;
    fixed_frame.performative_type = AMQP_OPEN;
    fixed_frame.size = 0;
    fixed_frame.type = WICED_AMQP_FRAME_TYPE;
    fixed_frame.count = count;

    put_fixed_frame( frame, &fixed_frame );

    /* Encode container id */
    if ( args->open_args.container_id == NULL )
    {
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, NULL_BYTE );
    }
    else
    {
        /* As it is container id string-8 type  code : VARIABLE_TYPE_ONE_BYTE_UNICODE_STRING*/
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, VARIABLE_TYPE_ONE_BYTE_UNICODE_STRING );
        AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->open_args.container_id, args->open_args.container_id_size );
    }

    /* Encode host name */
    if ( args->open_args.host_name == NULL )
    {
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, NULL_BYTE );
    }
    else
    {
        /* As it is host_name  string-8 type  code : VARIABLE_TYPE_ONE_BYTE_UNICODE_STRING*/
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, VARIABLE_TYPE_ONE_BYTE_UNICODE_STRING );
        AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->open_args.host_name, args->open_args.host_name_size );
    }

    /* Encode max frame size as it is uint code : FIXED_TYPE_FOUR_BYTE_UINT */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, FIXED_TYPE_FOUR_BYTE_UINT );
    AMQP_BUFFER_PUT_LONG( &frame->buffer, args->open_args.max_frame_size );

    /* Encode channel max as it is ushort code : FIXED_TYPE_ONE_USHORT */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, FIXED_TYPE_ONE_USHORT );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, args->open_args.channel_max );

    //    /* Encode idle time out as it is miliseconds (uint) code : FIXED_TYPE_ONE_USHORT */
    //    AMQP_BUFFER_PUT_OCTET( &frame->buffer, FIXED_TYPE_FOUR_BYTE_UINT );
    //    AMQP_BUFFER_PUT_LONG( &frame->buffer, args->open_args.idle_timeout );
    //    size = size + 1 + 4;
    //    count++;
    //    size++;

    /*calculate the size of open frame*/
        calculate_update_size_fields( temp1, temp1 + 12, frame );
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_get_close_performative( wiced_amqp_frame_t *frame, void *arg )
{
    wiced_amqp_packet_content *args = (wiced_amqp_packet_content *) arg;
    uint32_t temp = 0;

    get_fixed_frame( frame, &args->fixed_frame );

    AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );

    if ( temp == COMPOUND_TYPE_ONE_BYTE_LIST )
    {
        AMQP_BUFFER_GET_SHORT( &frame->buffer, temp );/*size and count*/

        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );

        if ( temp == NULL_BYTE )
        {
            args->args.close_args.error.condition_length = 0;
            args->args.close_args.error.description_length = 0;
        }

        frame->buffer.data--;
        AMQP_BUFFER_GET_SHORT( &frame->buffer, temp );/*descriptor*/
        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp ); /*error*/

        if ( temp == ELEMENT_ERROR )
        {
            get_element_error( &args->args.close_args.error, &frame->buffer );
        }

    }

    return WICED_SUCCESS;
}


wiced_result_t amqp_frame_put_close_performative( wiced_amqp_frame_t *frame,  void* arg )
{
    uint8_t count = AMQP_CLOSE_FRAME_LIST_COUNT;
    uint8_t* temp1 = frame->buffer.data;

    amqp_fixed_frame fixed_frame;
    (void) arg;

    fixed_frame.channel = 0x00;
    fixed_frame.doff = 0x02;
    fixed_frame.performative_type = AMQP_CLOSE;
    fixed_frame.size = 0;
    fixed_frame.type = WICED_AMQP_FRAME_TYPE;
    fixed_frame.count = count;

    put_fixed_frame( frame, &fixed_frame );

    put_element_bool( 0x00, WICED_FALSE, &frame->buffer );

    /*calculate the size of link frame*/
    calculate_update_size_fields( temp1, temp1 + 12, frame );

    return WICED_SUCCESS;

}

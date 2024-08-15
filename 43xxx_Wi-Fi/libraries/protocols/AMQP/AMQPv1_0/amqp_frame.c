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
 *  Frame packing and unpacking functions
 */

#include "wiced.h"
#include "amqp.h"
#include "amqp_connection.h"
#include "amqp_frame.h"
#include "amqp_session.h"
#include "amqp_link.h"
#include "amqp_open.h"
#include "amqp_transfer.h"
#include "amqp_flow.h"
#include "amqp_sasl.h"

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
#define LENGTH_OF_STRING(X) ((X).len + sizeof((X).len))
/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/
uint8_t *buffer_pointer=NULL;

/******************************************************
 *               Function Definitions
 ******************************************************/
wiced_result_t  amqp_frame_recv( wiced_amqp_buffer_t *buffer, void *p_user, uint32_t *size,uint16_t remaining_size,wiced_partial_frame_details_t *partial_frame_details )
{
    wiced_amqp_connection_instance *connection_instance = p_user;
    uint32_t amqp_str;
    wiced_amqp_frame_t frame;
    wiced_result_t result = WICED_SUCCESS;
    uint32_t size_to_be_copied=0;
    amqp_fixed_frame fixed_frame;
    wiced_amqp_packet_content args;
    wiced_amqp_protocol_header_arg_t header_args;

    memset(&frame,0,sizeof(wiced_amqp_frame_t));

    if ( NULL == buffer )
    {
        WPRINT_LIB_DEBUG( ( "[ AMQP LIB ERROR ] : Nothing received empty buffer\r\n" ) );
        return WICED_BADARG;
    }
    /* Check valid type type */

    /*set to buffer start only if there is no partial left to be be processed*/


    if(partial_frame_details->partial_frame_to_be_processed==WICED_FALSE)
    {
        frame.start = buffer->data;
        frame.buffer = *buffer;
        frame.buffer.data = frame.start;
    }
    else if(partial_frame_details->partial_frame_to_be_processed==WICED_TRUE)
    {
        if(buffer_pointer!=NULL)
        {
            size_to_be_copied=partial_frame_details->total_size-partial_frame_details->size_received;
            if(size_to_be_copied<=remaining_size)
            {
                memcpy(&buffer_pointer[partial_frame_details->size_received],buffer->data,size_to_be_copied);
                partial_frame_details->size_received=partial_frame_details->total_size;
            }

            frame.start = buffer_pointer;
            frame.buffer = *buffer;
            frame.buffer.data = frame.start;
        }
    }

    if ( connection_instance->current_connection_state == AMQP_CONNECTION_HDR_SENT || connection_instance->current_connection_state == AMQP_CONNECTION_SASL_HDR_SENT   )
    {
        if ( frame.size != 7 )
        {
            WPRINT_LIB_ERROR( ( "[ AMQP LIB ERROR ] : Received protocol header size is not matching and size = %d\r\n",frame.size ) );
        }
        memset( &header_args, 0, sizeof(wiced_amqp_protocol_header_arg_t) );
        AMQP_BUFFER_GET_LONG( &frame.buffer, amqp_str );
        amqp_frame_get_protocol_header( &frame, &header_args );
        result = amqp_connection_backend_get_protocol_header( &header_args, connection_instance );
        *size = 8;
    }
    else
    {
        uint8_t performative_type  = get_fixed_frame( &frame , &fixed_frame);

        ((&frame.buffer)->data) -= 11;

         if(partial_frame_details->partial_frame_to_be_processed==WICED_FALSE)
         {
            if(fixed_frame.size>remaining_size)
            {
                partial_frame_details->partial_frame_to_be_processed=WICED_TRUE;
                /*calloc to store the partial frame*/
                buffer_pointer=(uint8_t*)calloc(fixed_frame.size,sizeof(char));
                partial_frame_details->total_size=fixed_frame.size;
                memcpy(buffer_pointer,frame.buffer.data,remaining_size);
                partial_frame_details->size_received=remaining_size;
                performative_type=AMQP_UNKNOWN_TYPE;
            }
         }

        switch (performative_type)
        {
            case AMQP_SASL_SRV_MECHS:
            {
                memset( &args, 0, sizeof(wiced_amqp_packet_content) );
                amqp_frame_get_sasl_mechansims( &frame, &args );
                result = amqp_connection_backend_get_sasl_mechanisms( &args, connection_instance );
                *size = args.fixed_frame.size;
                break;
            }
            case AMQP_SASL_INIT:
            {
                break;
            }
            case AMQP_SASL_OUTCOME:
            {
                memset( &args, 0, sizeof(wiced_amqp_packet_content) );
                amqp_frame_get_sasl_outcome( &frame, &args );
                result = amqp_connection_backend_get_sasl_outcome( &args, connection_instance );
                *size = args.fixed_frame.size;
                break;
            }
            case AMQP_OPEN:
            {
                memset( &args, 0, sizeof(wiced_amqp_packet_content) );
                amqp_frame_get_open_performative( &frame, &args );
                result = amqp_connection_backend_get_open_performative( &args, connection_instance );
                *size = args.fixed_frame.size;
                break;
            }
            case AMQP_BEGIN:
            {
                memset( &args, 0, sizeof(wiced_amqp_packet_content) );
                amqp_frame_get_begin_performative( &frame, &args );
                result = amqp_connection_backend_get_begin_performative( &args, connection_instance );
                *size = args.fixed_frame.size;
                break;
            }
            case AMQP_ATTACH:
            {
                memset( &args, 0, sizeof(wiced_amqp_packet_content) );
                amqp_frame_get_attach_performative( &frame, &args );
                result = amqp_connection_backend_get_attach_performative( &args, connection_instance );
                *size = args.fixed_frame.size;
                break;
            }
            case AMQP_FLOW:
            {
                memset( &args, 0, sizeof(wiced_amqp_packet_content) );
                amqp_frame_get_flow_performative( &frame, &args );
                result = amqp_connection_backend_get_flow_performative( &args, connection_instance );
                *size = args.fixed_frame.size;
                break;
            }
            case AMQP_TRANSFER:
            {
                memset( &args, 0, sizeof(wiced_amqp_packet_content) );
                amqp_frame_get_transfer_performative( &frame, &args );
                result = amqp_connection_backend_get_transfer_performative( &args, connection_instance );
                *size = args.fixed_frame.size;
                break;
            }
            case AMQP_DISPOSITON:
            {
                memset( &args, 0, sizeof(wiced_amqp_packet_content) );
                amqp_frame_get_disposition_performative( &frame, &args );
                result = amqp_connection_backend_get_disposition_performative( &args, connection_instance );
                *size = args.fixed_frame.size;
                break;
            }
            case AMQP_DETACH:
            {
                memset( &args, 0, sizeof(wiced_amqp_packet_content) );
                amqp_frame_get_detach_performative( &frame, &args );
                result = amqp_connection_backend_get_detach_performative( &args, connection_instance );
                *size = args.fixed_frame.size;
                break;
            }
            case AMQP_END:
            {
                memset( &args, 0, sizeof(wiced_amqp_packet_content) );
                amqp_frame_get_end_performative( &frame, &args );
                result = amqp_connection_backend_get_end_performative( &args, connection_instance );
                *size = args.fixed_frame.size;
                break;
            }
            case AMQP_CLOSE:
            {
                memset( &args, 0, sizeof(wiced_amqp_packet_content) );
                amqp_frame_get_close_performative( &frame, &args );
                result = amqp_connection_backend_get_close_performative( &args, connection_instance );
                *size = args.fixed_frame.size;
                break;
            }
            case AMQP_UNKNOWN_TYPE:
            {
                break;
            }
            default:
            {
                break;
            }
        }
   }

    if((partial_frame_details->partial_frame_to_be_processed==WICED_TRUE) && partial_frame_details->total_size==partial_frame_details->size_received)
    {
        partial_frame_details->partial_frame_to_be_processed=WICED_FALSE;
        partial_frame_details->total_size=0;
        partial_frame_details->size_received=0;
        *size=size_to_be_copied;

        if(buffer_pointer!=NULL)
        {
            free(buffer_pointer);
        }
        buffer_pointer=NULL;
    }
    return result;
}

uint8_t put_fixed_frame( wiced_amqp_frame_t *frame, amqp_fixed_frame *fixed_frame )
{
    /* fill fixed AMQP header */
    AMQP_BUFFER_PUT_LONG( &frame->buffer, fixed_frame->size ); /*  4  total size    */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, fixed_frame->doff ); /*  1  doff          */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, fixed_frame->type ); /*  1  type          */

    AMQP_BUFFER_PUT_SHORT( &frame->buffer, fixed_frame->channel ); /*  2  channel       */

    /* performative frame*/
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0x0053 ); /*  2  descriptor   */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, fixed_frame->performative_type ); /*  1  performative */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0xc0 ); /*  1  constructor  */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x00 ); /*  1  size         */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, fixed_frame->count ); /*  1  count        */

    return fixed_frame->performative_type;

}

uint8_t get_fixed_frame( wiced_amqp_frame_t *frame, amqp_fixed_frame *fixed_frame )
{
    uint8_t performative_type = AMQP_UNKNOWN_TYPE;
     uint16_t descriptor;

    AMQP_BUFFER_GET_LONG( &frame->buffer,fixed_frame->size );
    AMQP_BUFFER_GET_OCTET( &frame->buffer, fixed_frame->doff );
    AMQP_BUFFER_GET_OCTET( &frame->buffer, fixed_frame->type );
    AMQP_BUFFER_GET_SHORT( &frame->buffer, fixed_frame->channel );

    AMQP_BUFFER_GET_SHORT( &frame->buffer, descriptor );
    AMQP_BUFFER_GET_OCTET( &frame->buffer, fixed_frame->performative_type );
    performative_type = fixed_frame->performative_type;
    return performative_type;
}

wiced_result_t  amqp_frame_create( wiced_amqp_frame_type_t type, uint16_t channel, uint16_t max_size, wiced_amqp_frame_t *frame, wiced_amqp_socket_t *socket )
{
    wiced_result_t ret;
    (void) type;
    (void) channel;
    ret = amqp_network_create_buffer( &frame->buffer, max_size, socket );
    if ( ret == WICED_SUCCESS )
    {
        frame->size = 0;
        frame->start = frame->buffer.data;
    }
    return ret;
}

wiced_result_t  amqp_frame_delete( wiced_amqp_frame_t *frame )
{
    return amqp_network_delete_buffer( &frame->buffer );
}


wiced_result_t  amqp_frame_send( wiced_amqp_frame_t *frame, wiced_amqp_socket_t *socket )
{
    return amqp_network_send_buffer( &frame->buffer, socket );
}


wiced_result_t amqp_get_buffer( void** buffer, uint32_t size )
{
    /* Allocate buffer object */
    *buffer = malloc_named( "amqp_buffer", size );
    wiced_assert( "failed to malloc", ( buffer != NULL ) );
    if ( *buffer == NULL )
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }

    return WICED_SUCCESS;
}

wiced_result_t get_element_pointer( uint8_t **data_pointer, uint32_t *data_length, wiced_amqp_buffer_t* pointer )
{
    uint32_t temp_data = 0;
    AMQP_BUFFER_GET_OCTET( pointer, temp_data );
    if ( ( temp_data & UPPER_NIBBLE ) == VARIABLE_TYPE_FOUR_BYTE_BINARY )
    {
        AMQP_BUFFER_GET_LONG( pointer, temp_data );
        *data_length = temp_data;
        *data_pointer = (uint8_t*) pointer->data;
    }
    else if ( ( temp_data & UPPER_NIBBLE ) == VARIABLE_TYPE_ONE_BYTE_BINARY )
    {
        AMQP_BUFFER_GET_OCTET( pointer, temp_data );
        *data_length = temp_data;
        *data_pointer = (uint8_t*) pointer->data;
    }
    else if ( ( temp_data & UPPER_NIBBLE ) == FIXED_TYPE_ONE_UBYTE )
    {
        *data_length = 1;
        *data_pointer = (uint8_t*) pointer->data;
    }
    else if ( ( temp_data & UPPER_NIBBLE ) == FIXED_TYPE_ONE_USHORT )
    {
        *data_length = 2;
        *data_pointer = (uint8_t*) pointer->data;
    }
    else if ( ( temp_data & UPPER_NIBBLE ) == FIXED_TYPE_FOUR_BYTE_UINT )
    {
        *data_length = 4;
        *data_pointer = (uint8_t*) pointer->data;
    }
    else if ( ( temp_data & UPPER_NIBBLE ) == NULL_BYTE )
    {
        *data_length = 0;
        *data_pointer = NULL;
    }
    pointer->data = &pointer->data[ *data_length ];
    return WICED_SUCCESS;
}

wiced_result_t get_element_value( uint64_t *value, wiced_bool_t * included, wiced_amqp_buffer_t* pointer )
{
    uint64_t temp_data = 0;

    AMQP_BUFFER_GET_OCTET( pointer, temp_data );
    *included = WICED_TRUE;
    if ( ( temp_data & UPPER_NIBBLE ) == NULL_BYTE )
    {
        if ( temp_data == 0x43 )
        {
            *value = 0;
        }
        else
        {
            *included = WICED_FALSE;
        }
    }
    else if ( ( temp_data & UPPER_NIBBLE ) == FIXED_TYPE_ONE_UBYTE )
    {
        AMQP_BUFFER_GET_OCTET( pointer, *value );/*length*/
    }
    else if ( ( temp_data & UPPER_NIBBLE ) == FIXED_TYPE_ONE_USHORT )
    {
        AMQP_BUFFER_GET_SHORT( pointer, *value );/*length*/
    }
    else if ( ( temp_data & UPPER_NIBBLE ) == FIXED_TYPE_FOUR_BYTE_UINT )
    {
        AMQP_BUFFER_GET_LONG( pointer, *value );/*length*/
    }
    else if ( ( temp_data & UPPER_NIBBLE ) == FIXED_TYPE_EIGHT_BYTE_ULONG )
    {
        AMQP_BUFFER_GET_LONG_LONG( pointer, *value );/*length*/
    }

    return WICED_SUCCESS;
}

wiced_result_t get_element_bool( wiced_bool_t *value, wiced_bool_t *included, wiced_amqp_buffer_t *pointer )
{
    uint32_t temp_data;

    *included = WICED_TRUE;
    AMQP_BUFFER_GET_OCTET( pointer, temp_data );
    if ( temp_data == 0x40 )
    {
        *included = WICED_FALSE;
    }
    else if ( temp_data == 0x41 ) /*true*/
    {
        *value = WICED_TRUE;
    }
    else if ( temp_data == 0x42 )/*false*/
    {
        *value = WICED_FALSE;
    }
    return WICED_SUCCESS;
}

wiced_result_t put_element_pointer( uint8_t* data, uint32_t data_length, wiced_amqp_buffer_t* pointer )
{
    if ( data_length > 0 && data_length <= 255 )
    {
        AMQP_BUFFER_PUT_OCTET( pointer, VARIABLE_TYPE_ONE_BYTE_BINARY );
        AMQP_BUFFER_PUT_OCTET( pointer, data_length );
        memcpy( pointer->data, data, data_length );
        pointer->data = &pointer->data[ data_length ];
    }
    else if ( data_length > 255 && data_length <= 4294967294 ) /*2^32-1*/
    {
        AMQP_BUFFER_PUT_OCTET( pointer, VARIABLE_TYPE_FOUR_BYTE_BINARY );
        AMQP_BUFFER_PUT_LONG( pointer, data_length );
        memcpy( pointer->data, data, data_length );
        pointer->data = &pointer->data[ data_length ];
    }
    else
    {
        AMQP_BUFFER_PUT_OCTET( pointer, NULL_BYTE );
    }
    return WICED_SUCCESS;
}

wiced_result_t put_element_value( uint64_t value, wiced_bool_t included, wiced_amqp_buffer_t *pointer )
{
    if ( included )
    {
        if ( value == 0 )
        {
            AMQP_BUFFER_PUT_OCTET( pointer, 0x43 );
        }
        else if ( value <= 255 )
        {
            AMQP_BUFFER_PUT_OCTET( pointer, 0x52 );
            AMQP_BUFFER_PUT_OCTET( pointer, value );
        }
        else if ( value <= ( 4294967295 ) ) /*2^32-1*/
        {
            AMQP_BUFFER_PUT_OCTET( pointer, FIXED_TYPE_FOUR_BYTE_UINT );
            AMQP_BUFFER_PUT_LONG( pointer, value );
        }
        else
        {
            AMQP_BUFFER_PUT_OCTET( pointer, FIXED_TYPE_EIGHT_BYTE_ULONG );
            AMQP_BUFFER_PUT_LONG_LONG( pointer, value );
        }
    }
    else
    {
        AMQP_BUFFER_PUT_OCTET( pointer, NULL_BYTE );
    }
    return WICED_SUCCESS;

}

wiced_result_t put_element_bool( wiced_bool_t value, wiced_bool_t included, wiced_amqp_buffer_t *pointer )
{
    if ( included )
    {
        if ( value == WICED_TRUE )
        {
            AMQP_BUFFER_PUT_OCTET( pointer, 0x41 );
        }
        else
        {
            AMQP_BUFFER_PUT_OCTET( pointer, 0x42 );
        }
    }
    else
    {
        AMQP_BUFFER_PUT_OCTET( pointer, NULL_BYTE );
    }
    return WICED_SUCCESS;
}

void calculate_update_size_fields( uint8_t* temp1, uint8_t* temp2, wiced_amqp_frame_t *frame )
{
    uint32_t total_size = 8;/*for header*/
    /*calculate the size of transfer frame*/
    *temp2 = (uint8_t) ( frame->buffer.data - temp2 - 1 ); /*update the size in size field*/
    total_size += ( (uint32_t) ( frame->buffer.data - temp2 - 1 ) ) + 3 + 2;

    /*update total frame size*/
    *temp1 = (uint8_t) ( total_size >> 24 );
    temp1++;
    *temp1 = (uint8_t) ( total_size >> 16 );
    temp1++;
    *temp1 = (uint8_t) ( total_size >> 8 );
    temp1++;
    *temp1 = (uint8_t) ( total_size );
}

wiced_result_t get_element_error( amqp_error_t* error, wiced_amqp_buffer_t *pointer )
{
    uint32_t temp = 0;
    AMQP_BUFFER_GET_OCTET( pointer, temp );
    if ( ( temp & UPPER_NIBBLE ) == COMPOUND_TYPE_ONE_BYTE_LIST )
    {
        AMQP_BUFFER_GET_SHORT( pointer, temp ); /*size and count*/
        AMQP_BUFFER_GET_OCTET( pointer, temp );
        if ( ( temp & UPPER_NIBBLE ) == VARIABLE_TYPE_ONE_BYTE_BINARY )
        {
            AMQP_BUFFER_GET_OCTET( pointer, error->condition_length );
            error->condition = (uint8_t*) pointer->data;
        }
        else if ( ( temp & UPPER_NIBBLE ) == VARIABLE_TYPE_FOUR_BYTE_BINARY )
        {
            AMQP_BUFFER_GET_LONG( pointer, error->condition_length );
            error->condition = (uint8_t*) pointer->data;
        }
    }
    else if ( ( temp & UPPER_NIBBLE ) == COMPOUND_TYPE_FOUR_BYTE_LIST )
    {
        AMQP_BUFFER_GET_LONG( pointer, temp ); /*size*/
        AMQP_BUFFER_GET_LONG( pointer, temp ); /*count*/

        AMQP_BUFFER_GET_OCTET( pointer, temp );

        if ( ( temp & UPPER_NIBBLE ) == VARIABLE_TYPE_ONE_BYTE_BINARY )
        {
            AMQP_BUFFER_GET_OCTET( pointer, error->condition_length );
            error->condition = (uint8_t*) pointer->data;
        }
        else if ( ( temp & UPPER_NIBBLE ) == VARIABLE_TYPE_FOUR_BYTE_BINARY )
        {
            AMQP_BUFFER_GET_LONG( pointer, error->condition_length );
            error->condition = (uint8_t*) pointer->data;
        }
    }

    pointer->data = &pointer->data[ error->condition_length ];

    AMQP_BUFFER_GET_OCTET( pointer, temp );
    if ( ( temp & UPPER_NIBBLE ) == VARIABLE_TYPE_ONE_BYTE_BINARY )
    {
        AMQP_BUFFER_GET_OCTET( pointer, error->description_length );
        error->descrption = (uint8_t*) pointer->data;
    }
    else if ( ( temp & UPPER_NIBBLE ) == VARIABLE_TYPE_FOUR_BYTE_BINARY )
    {
        AMQP_BUFFER_GET_LONG( pointer, error->description_length );
        error->descrption = (uint8_t*) pointer->data;
    }

    return WICED_SUCCESS;

}

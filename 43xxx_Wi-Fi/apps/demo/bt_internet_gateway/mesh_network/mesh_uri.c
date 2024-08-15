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
#include "http_server.h"
#include "mesh.h"
#include "mesh_uri.h"
#include "wiced_hci_bt_mesh.h"
#include "wwd_debug.h"

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
 *               Function Declarations
 ******************************************************/

static char* get_payload( const char* url_path );
mesh_value_handle_t* reverse_mesh_byte_stream( mesh_value_handle_t* val );
mesh_value_handle_t* create_mesh_value( const char* payload );
extern void process_connect_network();
extern void process_disconnect_network();



/******************************************************
 *               Variables Definitions
 ******************************************************/

const char json_object_start         [] = "{";
const char json_data_actuator_status [] = "\"data \": \"";
const char json_data_end4            [] = "\"}\n";
wiced_http_response_stream_t* http_event_stream = NULL;
/******************************************************
 *               Function Definitions
 ******************************************************/

int32_t process_send_mesh_data( const char* url_path, const char* url_query_string, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body )
{
    char*                payload        = get_payload( url_path );
    mesh_value_handle_t* value          = create_mesh_value( payload );
    mesh_value_handle_t* reversed_value = reverse_mesh_byte_stream( value );

    WPRINT_APP_INFO(("[%s] len =%lu\n",__func__,reversed_value->length));
    wiced_bt_mesh_send_proxy_packet(reversed_value->value, reversed_value->length);

    wiced_http_response_stream_enable_chunked_transfer( stream );
    wiced_http_response_stream_disable_chunked_transfer( stream );
    wiced_http_response_stream_flush( stream );
    wiced_http_response_stream_disconnect( stream );

    //mesh_send_status(payload, 4);

    free( value );
    free( reversed_value );
    return WICED_SUCCESS;
}

void hex_bytes_to_chars( char* cptr, const uint8_t* bptr, uint32_t blen )
{
    int i,j;
    uint8_t temp;

    i = 0;
    j = 0;
    while( i < blen )
    {
        // Convert first nibble of byte to a hex character
        temp = bptr[i] / 16;
        if ( temp < 10 )
        {
            cptr[j] = temp + '0';
        }
        else
        {
            cptr[j] = (temp - 10) + 'A';
        }
        // Convert second nibble of byte to a hex character
        temp = bptr[i] % 16;
        if ( temp < 10 )
        {
            cptr[j+1] = temp + '0';
        }
        else
        {
            cptr[j+1] = (temp - 10) + 'A';
        }
        i++;
        j+=2;
    }
}

wiced_result_t mesh_send_status( uint8_t* value , int len)
{
      char*    adv_data = NULL;
      wiced_result_t res = WICED_SUCCESS;

    if ( http_event_stream == NULL )
    {
        WPRINT_APP_INFO(("[%s] error\n",__func__));
        return WICED_ERROR;
    }
    adv_data = (char*) malloc( sizeof(char) * ((len*2) + 1) );
    if(http_event_stream != NULL)
    {
        hex_bytes_to_chars(adv_data, value, len);
        adv_data[len*2] = 0;
        res = wiced_http_response_stream_write( http_event_stream, json_object_start, sizeof( json_object_start ) - 1 );
        if (res != WICED_SUCCESS)
        {
            goto exit;
        }
        res = wiced_http_response_stream_write( http_event_stream, json_data_actuator_status, sizeof( json_data_actuator_status ) - 1 );
        if (res != WICED_SUCCESS)
        {
            goto exit;
        }
        res = wiced_http_response_stream_write( http_event_stream, adv_data, strlen(adv_data) );
        if (res != WICED_SUCCESS)
        {
            goto exit;
        }
        res = wiced_http_response_stream_write( http_event_stream, json_data_end4, sizeof( json_data_end4 ) - 1 );
        if (res != WICED_SUCCESS)
        {
            goto exit;
        }
        res = wiced_http_response_stream_flush( http_event_stream );
        if (res != WICED_SUCCESS)
        {
            goto exit;
        }
    }
    if (value[0] == 0x00 && value[1] == 0x00 )
    {
        res = wiced_http_response_stream_disconnect(http_event_stream);
        http_event_stream = NULL;
    }
    exit :
    free(adv_data);
    return res;
}


int32_t connect_network( const char* url_path, const char* url_query_string, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body )
{
    process_connect_network();
    return WICED_SUCCESS;
}

int32_t disconnect_network( const char* url_path, const char* url_query_string, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body )
{
    process_disconnect_network();
    return WICED_SUCCESS;
}

static char* get_payload( const char* url_path )
{
    /* /mesh/<command>/values/<payload> */
    char* current_path = (char*) url_path;
    uint32_t a = 0;

    while ( a < 4 )
    {
        if ( *current_path == '\0' )
        {
            /* <service> not found */
            return NULL;
        }
        else if ( *current_path == '/' )
        {
            a++;
        }

        current_path++;
    }

    return current_path;

}

mesh_value_handle_t* reverse_mesh_byte_stream( mesh_value_handle_t* val )
{
    mesh_value_handle_t* ret_val = (mesh_value_handle_t*) malloc( sizeof(mesh_value_handle_t) + val->length );
    WPRINT_APP_INFO(("Received mesh proxy packet \n"));
    for ( int i = 0; i < val->length; i++ )
    {
        ret_val->value[ i ] = val->value[ val->length - 1 - i ];
        WPRINT_APP_INFO((" %02X " , ret_val->value[ i ]));
    }
    ret_val->length = val->length;
    printf( "\n" );
    return ret_val;
}

mesh_value_handle_t* create_mesh_value( const char* payload )
{
    uint32_t temp  = strlen( payload ) / 2;
    mesh_value_handle_t* value = (mesh_value_handle_t*)malloc_named( "value", sizeof( mesh_value_handle_t ) + temp );
    if ( value != NULL )
    {
        uint32_t a;
        uint32_t b;
        memset( value, 0, sizeof( mesh_value_handle_t ) + temp );
        value->length = temp;
        for ( a = 0, b = ( value->length - 1 ) * 2 ; a < value->length; a++, b-=2 )
        {
            string_to_unsigned( &payload[b], 2, &temp, 1 );
            value->value[a] = (uint8_t)( temp & 0xff );
        }
    }

    return value;
}

int32_t subscribe_sse_events( const char* url_path, const char* url_query_string, wiced_http_response_stream_t* stream, void* arg, wiced_http_message_body_t* http_message_body )
{
    http_event_stream = stream;
    wiced_result_t res;
    wiced_http_response_stream_enable_chunked_transfer( http_event_stream );
   // VERIFY_METHOD( WICED_HTTP_GET_REQUEST );
    res = wiced_http_response_stream_write_header( http_event_stream, HTTP_200_TYPE, CHUNKED_CONTENT_LENGTH, HTTP_CACHE_DISABLED, MIME_TYPE_TEXT_EVENT_STREAM );
    WPRINT_APP_INFO(("[%s] subscribe_sse_events =%s res:%d\n",__func__,url_path, res));
    return WICED_SUCCESS;
}


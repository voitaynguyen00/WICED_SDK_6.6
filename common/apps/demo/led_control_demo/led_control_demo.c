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
 * WICED LED Control demo application.
 *
 * This application demonstrates TURN ON/OFF led and setting brightness level of LED using Client Control
 *
 * Features demonstrated
 * - LED turning on/off and brightness configuration Demo
 *
 * Application Instructions
 * - Connect a PC terminal to the serial port of the WICED Eval board,
 *   then build and download the application as described in the WICED
 *   Quick Start Guide.
 * - Send Turn on/off LED command from the Client Control
 * - Use slider to set LED brightness level
 *
 */

#include "sparcommon.h"

#include "wiced_hal_pwm.h"
#include "wiced_hal_gpio.h"
#include "wiced_hal_aclk.h"

#if ( defined(CYW20706A2) || defined(CYW20719B1) || defined(CYW20719B0) || defined(CYW20721B1) || defined(CYW20735B0) || defined(CYW43012C0) )
#include "wiced_bt_app_common.h"
#include "wiced_bt_app_hal_common.h"
#endif

#include "wiced_platform.h"
#include "wiced_timer.h"
#include "wiced_hal_puart.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_stack.h"
#include "wiced_transport.h"
#include "hci_control_api.h"

/******************************************************************************
 *                                Constants
 ******************************************************************************/
#define PWM_CHANNEL         PWM0

#define PWM_INP_CLK_IN_HZ   (512*1000)
#define PWM_FREQ_IN_HZ      (10000)
#define PWM_MAX_BRIGHTNESS_LEVEL  99
/******************************************************************************
 *                          Function Declarations
 ******************************************************************************/
void led_control_demo_app_init(void);
static void led_control_demo_transport_status( wiced_transport_type_t type );
static uint32_t led_control_demo_proc_rx_cmd( uint8_t *p_buffer, uint32_t length );
wiced_bool_t is_led_on = WICED_FALSE;

/******************************************************************************
 *                                Structures
 ******************************************************************************/
const wiced_transport_cfg_t transport_cfg =
{
    .type = WICED_TRANSPORT_UART,
    .cfg =
    {
        .uart_cfg =
        {
            .mode = WICED_TRANSPORT_UART_HCI_MODE,
            .baud_rate =  HCI_UART_DEFAULT_BAUD
        },
    },
    .rx_buff_pool_cfg =
    {
        .buffer_size  = 1024,
        .buffer_count = 1
    },
    .p_status_handler    = led_control_demo_transport_status,
    .p_data_handler      = led_control_demo_proc_rx_cmd,
    .p_tx_complete_cback = NULL
};

/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/
#ifdef CYW20706A2
#define PWM_INVERT         0
#else

#define PWM_INVERT         1
#if defined(CYW20719B0) || defined(CYW20735B0) || defined(CYW43012C0)
#define PWM_LED_GPIO       WICED_GPIO_PIN_LED1
#endif

#if defined(CYW20719B1) || defined(CYW20735B1) || defined(CYW20721B1)
extern wiced_platform_led_config_t platform_led[];
#define PWM_LED_GPIO        (uint32_t)*platform_led[WICED_PLATFORM_LED_2].gpio
#endif
#endif

wiced_bt_gpio_numbers_t  led_pin;

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/
void led_set_brighness_level (uint8_t brightness_level)
{
    pwm_config_t pwm_config;

    wiced_hal_pwm_get_params(PWM_INP_CLK_IN_HZ, brightness_level, PWM_FREQ_IN_HZ, &pwm_config);
    wiced_hal_pwm_change_values(PWM_CHANNEL, pwm_config.toggle_count, pwm_config.init_count);
}

void led_control_demo_app_init(void)
{
    pwm_config_t pwm_config;

    /* configure PWM */
#ifndef CYW20706A2
    led_pin = PWM_LED_GPIO;
    wiced_hal_gpio_select_function(led_pin, WICED_PWM0);
#else
    /* Initialize wiced app */
    wiced_bt_app_init();
    wiced_bt_app_led_init();
#endif
    wiced_hal_aclk_enable(PWM_INP_CLK_IN_HZ, ACLK1, ACLK_FREQ_24_MHZ);
    wiced_hal_pwm_get_params(PWM_INP_CLK_IN_HZ, 0, PWM_FREQ_IN_HZ, &pwm_config);
    wiced_hal_pwm_start(PWM_CHANNEL, PMU_CLK, pwm_config.toggle_count, pwm_config.init_count, PWM_INVERT);
}

/*
 *  Management callback receives various notifications from the stack
 */
wiced_result_t led_control_demo_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{

    wiced_result_t result = WICED_BT_SUCCESS;

    WICED_BT_TRACE("app_management_callback %d\n", event);

    switch (event)
    {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:
            led_control_demo_app_init();
        break;

        default:
            result = WICED_BT_USE_DEFAULT_SECURITY;
        break;
    }
    return result;
}

/*
 *  Entry point to the application.
 */
APPLICATION_START( )
{

    wiced_transport_init( &transport_cfg );
#ifdef WICED_BT_TRACE_ENABLE
     wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_WICED_UART );
#endif

     WICED_BT_TRACE( "\n--------------------------------------------------------- \n"
                     "              PWM Sample Application \n"
                     "---------------------------------------------------------\n");

     wiced_bt_stack_init(led_control_demo_management_callback, NULL, NULL);
}

static void led_control_demo_transport_status( wiced_transport_type_t type )
{
    WICED_BT_TRACE( " led_control_demo_transport_status %x \n", type );
    wiced_transport_send_data( HCI_CONTROL_EVENT_DEVICE_STARTED, NULL, 0 );
}

void led_control_handle_get_version(void)
{
    uint8_t   tx_buf[20];
    uint8_t   cmd = 0;
    uint32_t  chip = CHIP;

    tx_buf[cmd++] = WICED_SDK_MAJOR_VER;
    tx_buf[cmd++] = WICED_SDK_MINOR_VER;
    tx_buf[cmd++] = WICED_SDK_REV_NUMBER;
    tx_buf[cmd++] = WICED_SDK_BUILD_NUMBER & 0xFF;
    tx_buf[cmd++] = (WICED_SDK_BUILD_NUMBER>>8) & 0xFF;
    tx_buf[cmd++] = chip & 0xFF;
    tx_buf[cmd++] = (chip>>8) & 0xFF;
    tx_buf[cmd++] = (chip>>24) & 0xFF;
    tx_buf[cmd++] = POWER_CLASS;

    /* Send MCU app the supported features */
    tx_buf[cmd++] = HCI_CONTROL_GROUP_LED_DEMO;

    wiced_transport_send_data(HCI_CONTROL_MISC_EVENT_VERSION, tx_buf, cmd);

}


/*
 * Handle received command over UART. Please refer to the WICED Smart Ready
 * Software User Manual (WICED-Smart-Ready-SWUM100-R) for details on the
 * HCI UART control protocol.
*/
static uint32_t led_control_demo_proc_rx_cmd( uint8_t *p_buffer, uint32_t length )
{
    uint16_t opcode;
    uint16_t payload_len;
    uint8_t *p_data = p_buffer;
    uint8_t  buffer_processed = WICED_TRUE;
    uint8_t  brightness_level;

    if ( !p_buffer )
    {
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    //Expected minimum 4 byte as the wiced header
    if(( length < 4 ) || (p_data == NULL))
    {
        WICED_BT_TRACE("invalid params\n");
        wiced_transport_free_buffer( p_buffer );
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    STREAM_TO_UINT16(opcode, p_data);       // Get OpCode
    STREAM_TO_UINT16(payload_len, p_data);  // Gen Payload Length

    WICED_BT_TRACE("cmd_opcode 0x%02x payload_len %d \n", opcode, payload_len);

    switch(opcode)
    {
        case HCI_CONTROL_MISC_COMMAND_GET_VERSION:
            WICED_BT_TRACE("HCI_CONTROL_MISC_COMMAND_GET_VERSION\n");
            led_control_handle_get_version();
            break;

        case HCI_CONTROL_LED_COMMAND_TURN_ON:
            if(!is_led_on)
            {
                is_led_on = WICED_TRUE;
                WICED_BT_TRACE( "Turning ON LED \n");
                led_set_brighness_level(PWM_MAX_BRIGHTNESS_LEVEL);
            }
            break;

        case HCI_CONTROL_LED_COMMAND_TURN_OFF:
            if(is_led_on)
            {
                is_led_on = WICED_FALSE;
                WICED_BT_TRACE( "Turning OFF LED \n");
                led_set_brighness_level(0);
            }
            break;

        case HCI_CONTROL_LED_COMMAND_SET_BRIGHTNESS:
            brightness_level = (uint8_t)*p_data;
            WICED_BT_TRACE( "Setting LED brightness level : %d \n", brightness_level);

            if (brightness_level >= PWM_MAX_BRIGHTNESS_LEVEL)
            {
                brightness_level = PWM_MAX_BRIGHTNESS_LEVEL;
                is_led_on = WICED_TRUE;
            }
            else if (brightness_level == 0)
            {
                is_led_on = WICED_FALSE;
            }
            led_set_brighness_level(brightness_level);
            break;

        default:
            WICED_BT_TRACE( "unknown (opcode:%x)\n", opcode);
            break;
    }

    // Freeing the buffer in which data is received
    wiced_transport_free_buffer( p_buffer );

    return HCI_CONTROL_STATUS_SUCCESS;
}

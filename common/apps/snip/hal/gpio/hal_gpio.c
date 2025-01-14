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
 * WICED sample application for GPIO
 *
 * This application demonstrates how to use WICED GPIO APIs to,
 *
 *  a) configure GPIO to be an input pin (and receive interrupt upon change of pin state)
 *  b) configure GPIO to be an output pin (by toggling LED's available on reference board)
 *
 * The GPIO's in the array output_pin_list[] are configured as an output
 * pin and toggled at predefined frequency(APP_TIMEOUT_IN_SECONDS_A).
 * The GPIO's in the array input_pin_list[] are configured as an interrupt
 * enabled input pin, upon pressing this button the blink rate
 * of LED's is toggled between LED_BLINK_FREQ_A_IN_SECONDS and
 * LED_BLINK_FREQ_B_IN_SECONDS
 *
 * To demonstrate the snip, work through the following steps.
 * 1. Plug the WICED evaluation board to your computer.
 * 2. Build and download the application.
 * 3. Use Terminal emulation tools like Teraterm or Putty to view the trace messages(See Kit User Guide).
 *
 */

#include "sparcommon.h"
#include "wiced_bt_dev.h"

#include "wiced_hal_gpio.h"
#include "wiced_timer.h"

#include "wiced_bt_trace.h"
#include "wiced_platform.h"
#include "wiced_bt_stack.h"

/******************************************************************************
 *                                Constants
 ******************************************************************************/

/* Delay timer for LED blinking (frequency of blinking is toggled between
 * the below two values upon button press ) */
#define LED_BLINK_FREQ_A_IN_SECONDS 2 /* Seconds timer */
#define LED_BLINK_FREQ_B_IN_SECONDS 1 /* Seconds timer */

/******************************************************************************
 *                                Structures
 ******************************************************************************/

/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/

/* NOTE: when populating the below lists care should be taken to avoid pins already
 *       in use for other functionalities such as puart or other peripherals. */

//list of all the pins to be configured as input
static uint8_t input_pin_list[] =
{
    WICED_GPIO_PIN_BUTTON_1,
};

//list of all the pins to be configured as output -Please see the Hardware User Manual(Kit User Guide) of the particular Eval board for more details on Pin mapping
static uint8_t output_pin_list[] =
{
    WICED_GPIO_PIN_LED_1,
    WICED_GPIO_PIN_LED_2,
};

static wiced_timer_t hal_gpio_app_timer;

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/
static wiced_result_t hal_gpio_app_management_cback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);

static void hal_gpio_app_test_input(void);
static void hal_gpio_app_test_output(void);

void hal_gpio_app_timer_cb(uint32_t arg);

void hal_gpio_app_interrrupt_handler(void *data, uint8_t port_pin);

/*
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that BT device is ready.
 */
APPLICATION_START()
{
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_PUART);

    WICED_BT_TRACE("**** hal_gpio_app **** \n\r");

    // Register call back with stack
    wiced_bt_stack_init(hal_gpio_app_management_cback, NULL, NULL);
}

wiced_result_t hal_gpio_app_management_cback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result = WICED_SUCCESS;

    WICED_BT_TRACE("hal_gpio_app_management_cback %d\n\r", event);

    switch (event)
    {
    // Bluetooth  stack enabled
    case BTM_ENABLED_EVT:

        // Initialize timer to control the pin toggle frequency
        if (wiced_init_timer(&hal_gpio_app_timer, &hal_gpio_app_timer_cb, 0, WICED_SECONDS_PERIODIC_TIMER) == WICED_SUCCESS)
        {
            if (wiced_start_timer(&hal_gpio_app_timer, LED_BLINK_FREQ_A_IN_SECONDS) != WICED_SUCCESS)
            {
                WICED_BT_TRACE("Seconds Timer Error\n");
            }
        }

        // initialize all the output pins selected in output_pin_list
        hal_gpio_app_test_output();

        // initialize all the input pins selected in input_pin_list
        hal_gpio_app_test_input();

        break;

    default:
        break;
    }

    return result;
}

/*
 * initialize all the input pins selected in input_pin_list to be input, enable interrupts and register
 * a interrupt handler
 */
void hal_gpio_app_test_input(void)
{
    uint8_t index = 0;

    // Configure all the selected pins to be input
    for (index = 0; index < sizeof(input_pin_list); index++)
    {
        wiced_hal_gpio_register_pin_for_interrupt(input_pin_list[index], hal_gpio_app_interrrupt_handler, NULL);
        wiced_hal_gpio_configure_pin(input_pin_list[index], WICED_GPIO_BUTTON_SETTINGS(GPIO_EN_INT_RISING_EDGE), GPIO_PIN_OUTPUT_LOW);
    }
}

/*
 * initialize all the output pins selected in output_pin_list to be output pins
 */
void hal_gpio_app_test_output(void)
{
    uint8_t index = 0;

    // Configure all the selected pins to be output
    for (index = 0; index < sizeof(output_pin_list); index++)
    {
        wiced_hal_gpio_configure_pin(output_pin_list[index], GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_HIGH);
    }
}

/*
 * The function invoked on timeout of app. seconds timer.
 */
void hal_gpio_app_timer_cb(uint32_t arg)
{
    static uint32_t wiced_seconds = 0; /* number of seconds elapsed */
    uint8_t index = 0;

    wiced_seconds++;

    if (wiced_seconds & 1)
    {
        for (index = 0; index < sizeof(output_pin_list); index++)
        {
            wiced_hal_gpio_set_pin_output(output_pin_list[index], GPIO_PIN_OUTPUT_LOW);
        }
    }
    else
    {
        for (index = 0; index < sizeof(output_pin_list); index++)
        {
            wiced_hal_gpio_set_pin_output(output_pin_list[index], GPIO_PIN_OUTPUT_HIGH);
        }
    }
}

/*
 * Handle interrupt generated due to change in the GPIO state
 */
void hal_gpio_app_interrrupt_handler(void *data, uint8_t pin)
{
    static uint32_t blink_freq = LED_BLINK_FREQ_A_IN_SECONDS;

    // toggle LED blink rate upon each button press
    if (blink_freq == LED_BLINK_FREQ_A_IN_SECONDS)
    {
        blink_freq = LED_BLINK_FREQ_B_IN_SECONDS;
    }
    else
    {
        blink_freq = LED_BLINK_FREQ_A_IN_SECONDS;
    }

    if (wiced_stop_timer(&hal_gpio_app_timer) == WICED_SUCCESS)
    {
        wiced_start_timer(&hal_gpio_app_timer, blink_freq);
    }

    // clear the interrupt status
    wiced_hal_gpio_clear_pin_interrupt_status(pin);
}

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
 * GPIO API Application
 *
 * This application demonstrates how to use the WICED GPIO API
 * to toggle LEDs and read button GPIO values (not pressed/open state)
 *
 * This application uses GPIO values defined in platforms/<platform_name>/platform.h
 *
 * NOTE: On some platforms, some buttons may be "Active High".
 *       In that case, the LED behavior will be opposite of what is expected.
 *       See platforms/<platform_name>/platform.c data structure platform_gpio_buttons[]
 *
 * Features demonstrated
 *  - GPIO API
 *  - LED API
 *
 */

#include "wiced.h"
#include "platform_peripheral.h"
#include "platform_init.h"

#if !defined(WICED_PLATFORM_BUTTON_COUNT) && !defined(PLATFORM_LED_COUNT)
#error Platform must have Buttons and LEDs for this application to work. Check platforms/<platform_name>/platform.h.
#endif

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
void print_gpio_value( int index, wiced_gpio_t gpio )
{
    wiced_bool_t    value = 0;
    if (wiced_gpio_input_get_bool( gpio, &value ) != WICED_SUCCESS)
    {
        printf(" WICED_BUTTON%d ( GPIO %d):Error", index, gpio);
    }
    else
    {
        printf(" WICED_BUTTON%d (GPIO %d):%d", index, gpio, value);
    }
}

void print_gpio_values( void )
{
#if (WICED_PLATFORM_BUTTON_COUNT < 1)
    static int printed_no_button_info = 0;
    if (printed_no_button_info == 0)
    {
        printf("\n No Button GPIOs Defined.");
        printed_no_button_info = 1;
    }
#endif

#if (WICED_PLATFORM_BUTTON_COUNT > 0)
    printf(" Button GPIOs: ");
    print_gpio_value( 1, WICED_BUTTON1 );
#endif
#if (WICED_PLATFORM_BUTTON_COUNT > 1)
    print_gpio_value( 2, WICED_BUTTON2 );
#endif
#if (WICED_PLATFORM_BUTTON_COUNT > 2)
    print_gpio_value( 3, WICED_BUTTON3 );
#endif
#if (WICED_PLATFORM_BUTTON_COUNT > 3)
    print_gpio_value( 4, WICED_BUTTON4 );
#endif
#if (WICED_PLATFORM_BUTTON_COUNT > 4)
    print_gpio_value( 5, WICED_BUTTON5 );
#endif
#if (WICED_PLATFORM_BUTTON_COUNT > 5)
    print_gpio_value( 6, WICED_BUTTON6 );
#endif

    printf("\n");
}

void application_start( )
{
    wiced_bool_t gpio1_level = WICED_FALSE;
    wiced_bool_t led1 = WICED_FALSE;
#if (WICED_PLATFORM_BUTTON_COUNT > 1) && (PLATFORM_LED_COUNT > 1)
    wiced_bool_t gpio2_level = WICED_FALSE;
    wiced_bool_t led2 = WICED_FALSE;
#endif
    wiced_result_t  result;

    /* Initialise the WICED device */
    wiced_init();

    WPRINT_APP_INFO( ( "The LEDs are flashing. Holding a button will force the corresponding LED on\n" ) );

    while ( 1 )
    {
        /* print GPIO pins and values to console */
        print_gpio_values();

        /* Read the value of WICED_BUTTON1 GPIO */
        result = wiced_gpio_input_get_bool( WICED_BUTTON1, &gpio1_level );
        if (result != WICED_SUCCESS)
        {
            printf(" Button 1 Not Initialized. Check platforms/<platform_name>/platform.c\n");
            gpio1_level = WICED_FALSE;  /* so we continue flashing the LED */
        }

        /* If GPIO level is 0 (WICED_FALSE), set LED solid on
         * NOTE: GPIO level 0 may not be "Button Pressed", as this depends on button polarity.
         *       see platforms/<platform name>/platform.c   structure platform_gpio_buttons[].polarity
         */
        if ( gpio1_level == WICED_FALSE )
        {   /* Turn LED1 on */
            wiced_led_set_state(WICED_LED_INDEX_1, WICED_LED_ON );
        }
        else
        {   /* Toggle LED1 */
            if ( led1 == WICED_TRUE )
            {
                wiced_led_set_state(WICED_LED_INDEX_1, WICED_LED_OFF );
                led1 = WICED_FALSE;
            }
            else
            {
                wiced_led_set_state(WICED_LED_INDEX_1, WICED_LED_ON );
                led1 = WICED_TRUE;
            }
        }

#if (WICED_PLATFORM_BUTTON_COUNT > 1) && (PLATFORM_LED_COUNT > 1)
        /* Read the value of WICED_BUTTON2 GPIO */
        result = wiced_gpio_input_get_bool( WICED_BUTTON2, &gpio2_level );
        if (result != WICED_SUCCESS)
        {
            printf(" Button 2 Not Initialized. Check platforms/<platform_name>/platform.c\n");
            gpio2_level = WICED_FALSE;  /* so we continue flashing the LED */
        }

        /* If GPIO level is 0 (WICED_FALSE), set LED solid on
         * NOTE: GPIO level 0 may not be "Button Pressed", as this depends on button polarity.
         *       see platforms/<platform name>/platform.c   structure platform_gpio_buttons[].polarity
         */
        if ( gpio2_level == WICED_FALSE )
        {   /* Turn LED2 on */
            wiced_led_set_state(WICED_LED_INDEX_2, WICED_LED_ON );
        }
        else
        {   /* Toggle LED2 */
            if ( led2 == WICED_TRUE )
            {
                wiced_led_set_state(WICED_LED_INDEX_2, WICED_LED_OFF );
                led2 = WICED_FALSE;
            }
            else
            {
                wiced_led_set_state(WICED_LED_INDEX_2, WICED_LED_ON );
                led2 = WICED_TRUE;
            }
        }
#endif
        wiced_rtos_delay_milliseconds( 250 );
    }
}

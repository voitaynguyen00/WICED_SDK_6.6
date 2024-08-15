/*
 * Copyright Cypress Semiconductor
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Cypress Semiconductor;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Cypress Semiconductor.
 */
/** @file
 *
 * WICED sample application for SPI Master usage
 *
 * This application demonstrates how to use SPI driver interface
 * to send and receive bytes or a stream of bytes over the SPI hardware as a master.
 *
 * Features demonstrated:
 * - SPI WICED APIs
 *
 * Requirements and Usage:
 *
 * Program 1 kit with the spi_master app and another kit with spi_slave app. Connect the
 * SPI lines and ground on the 2 kits. Power on the kits. You can see the logs through PUART
 * The power supply on both the kits should be 3.3V
 *
 * Hardware Connections:
 *
 * This snip example configures the following SPI functionalities in CYW920719Q40EVB-01 Evaluation board as follows:
 * CLK     WICED_P38    J3.5
 * MISO    WICED_P01    J3.6
 * MOSI    WICED_P28    J3.7
 * CS      WICED_P07    J3.8
 * GND
 *
 * This snip example configures the following SPI functionalities in CYW920721EVK_01 Evaluation board as follows:
 * CLK     WICED_P38    J3.5
 * MISO    WICED_P01    J3.6
 * MOSI    WICED_P28    J3.7
 * CS      WICED_P07    J3.8
 * GND
 *
 * This snip example configures the following SPI functionalities in CYW920735Q60EVB-01 Evaluation board as follows:
 * CLK     WICED_P07    J3.5
 * MISO    WICED_P16    J3.6
 * MOSI    WICED_P06    J3.7
 * CS      WICED_P38    J3.8
 * GND
 */

/******************************************************************************
 *                                Includes
 ******************************************************************************/
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_gpio.h"
#include "wiced_platform.h"
#include "wiced_hal_pspi.h"
#include "wiced_hal_puart.h"
#include "wiced_hal_rand.h"
#include "wiced_timer.h"


/******************************************************************************
 *                                Constants
 ******************************************************************************/
#define START_ID                            (0xAA)
#define ACK_ID                              (0x55)
#define PACKET_HEADER                       (0x66)             /* Header to send data */
#define START_TIMER                         (100u)             /* 100 ms timer for start and ack */
#define RAND_TIMER                          (1u)               /* 1 s timer for Random numbers */
#define DELAY_TIME                          (20u)              /* 20ms delay */
#define DEFAULT_FREQUENCY                   (1000000u)         /* 1 MHz */
#define TIMEOUT                             (4u)

#ifdef CYW20719B1

#define CLK                                 WICED_P38
#define MISO                                WICED_P01
#define MOSI                                WICED_P28
#define CS                                  WICED_P07

#elif CYW20721B1

#define CLK                                 WICED_P38
#define MISO                                WICED_P01
#define MOSI                                WICED_P28
#define CS                                  WICED_P07

#elif CYW20735B1

#define CLK                                 WICED_P07
#define MISO                                WICED_P16
#define MOSI                                WICED_P06
#define CS                                  WICED_P38

#endif


/******************************************************************************
 *                                Structures
 ******************************************************************************/
typedef struct
{
    uint8_t  header;
    uint32_t data;
} __attribute__((packed)) data_packet;


/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/
static uint8_t              wait_count          = 0;
static uint8_t              timeout             = 0;
static uint32_t             spi_master_gpio_cfg = 0;
static uint32_t             prev_rand;
static wiced_timer_t        seconds_rand_timer;
static wiced_timer_t        milli_seconds_start_timer;
static wiced_timer_t        milli_seconds_delay_timer;


/******************************************************************************
 *                                Function Definitions
 ******************************************************************************/
static void             initialize_app(void);
static void             start_and_ack (uint32_t arg );
static void             spi_master_driver(uint32_t arg);
static void             delay_cb(uint32_t arg);
static uint32_t         spi_master_get_gpio_cfg(uint8_t clk, uint8_t mosi, uint8_t miso);


/******************************************************************************
* Entry point to the application. Initialize the app
 *****************************************************************************/
APPLICATION_START( )
{

    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );

    WICED_BT_TRACE("\r\nSample SPI Master Application\r\n");

    initialize_app();
}



/******************************************************************************
 * This functions initializes the SPI and GPIO as well as timer
******************************************************************************/
void initialize_app( void )
{

    spi_master_gpio_cfg = spi_master_get_gpio_cfg(CLK, MOSI, MISO);

    WICED_BT_TRACE("\r\nSPI Master GPIO Config Value: \t %x \r\n",spi_master_gpio_cfg);

    /*Initialize CS GPIO and SPI*/
    wiced_hal_gpio_configure_pin(CS, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_HIGH);

    wiced_hal_pspi_init(SPI1, SPI_MASTER, INPUT_PIN_PULL_UP, spi_master_gpio_cfg, DEFAULT_FREQUENCY, SPI_LSB_FIRST, SPI_SS_ACTIVE_LOW, SPI_MODE_0, CS);

    /*Start a timer for 100 ms*/
    if ( WICED_SUCCESS == wiced_init_timer(&milli_seconds_start_timer, &start_and_ack, 0, WICED_MILLI_SECONDS_PERIODIC_TIMER) )
    {
        if ( WICED_SUCCESS != wiced_start_timer(&milli_seconds_start_timer, START_TIMER) )
        {
            WICED_BT_TRACE( "Milli Seconds start Timer Error\n\r" );
        }
    }

}



/******************************************************************************
 * This function sends a start byte to the slave and checks for an ACK byte
 * and a new frequency.
 * This function is called every 100 ms by the timer. After receiving the frequency
 * and ACK byte, it resets the SPI to the new frequency, stops the current timer
 * and starts a new timer for 1 second to exchange data with the slave
 *****************************************************************************/
void start_and_ack (uint32_t arg)
{
    uint8_t start_byte = START_ID;
    uint8_t rec_byte[2];

    /*Send a start byte every 500 ms*/
    if(0 == wait_count)
    {
        WICED_BT_TRACE("Sent start byte\n");
        wiced_hal_gpio_set_pin_output(CS, GPIO_PIN_OUTPUT_LOW);
        wiced_hal_pspi_tx_data(SPI1, sizeof(start_byte), &start_byte);
        wiced_hal_gpio_set_pin_output(CS, GPIO_PIN_OUTPUT_HIGH);
        wait_count++;
        return;
    }

    /*Check for ack byte every 100 ms. If not received for 500 ms send a start byte again*/
    else if(6 > wait_count)
    {
        //Delay of 20 milliseconds
        if (WICED_SUCCESS == wiced_init_timer(&milli_seconds_delay_timer, delay_cb, 0, WICED_MILLI_SECONDS_TIMER))
        {
            if (WICED_SUCCESS != wiced_start_timer(&milli_seconds_delay_timer, DELAY_TIME))
            {
                WICED_BT_TRACE( "milli_seconds_delay_timer Error\n\r" );
            }
        }

        wiced_hal_gpio_set_pin_output(CS, GPIO_PIN_OUTPUT_LOW);
        wiced_hal_pspi_rx_data(SPI1, sizeof(rec_byte), rec_byte);
        wiced_hal_gpio_set_pin_output(CS, GPIO_PIN_OUTPUT_HIGH);

        /*Stop current time. Start new timer for 1 second*/
        if(ACK_ID == rec_byte[0])
        {

            if(WICED_SUCCESS != wiced_stop_timer(&milli_seconds_start_timer))
            {
                WICED_BT_TRACE("milli_seconds_start_timer Error while stopping\n\r");
            }

            WICED_BT_TRACE("Received ACK byte\n");
            WICED_BT_TRACE("Frequency: %d\n", rec_byte[1]);
            wiced_hal_pspi_reset(SPI1);
            wiced_hal_pspi_init(SPI1, SPI_MASTER, INPUT_PIN_PULL_UP, spi_master_gpio_cfg, (rec_byte[1] * 1000000), SPI_LSB_FIRST, SPI_SS_ACTIVE_LOW, SPI_MODE_0, CS);

            if(WICED_SUCCESS == wiced_init_timer(&seconds_rand_timer, &spi_master_driver, 0, WICED_SECONDS_PERIODIC_TIMER))
            {
                if(WICED_SUCCESS != wiced_start_timer(&seconds_rand_timer, RAND_TIMER))
                {
                    WICED_BT_TRACE( "Seconds Random Timer Error\n\r" );
                }
            }

            return;
        }

        wait_count++;
    }
    else
    {
        wait_count = 0;
    }
}



/****************************************************************************************
 * This function generates random numbers and exchanges it with slave
 ***************************************************************************************/
void spi_master_driver( uint32_t arg )
{

    data_packet send_data;
    data_packet rec_data;
    uint8_t flush_data_rx[10];
    uint8_t flush_data_tx[10] = {0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55};                /* Dummy bytes */

    if(0 == timeout)
    {
        /*Generate random number*/
        send_data.data = wiced_hal_rand_gen_num();
        send_data.header = PACKET_HEADER;

        /*Exchange random numbers*/
        wiced_hal_gpio_set_pin_output(CS, GPIO_PIN_OUTPUT_LOW);
        wiced_hal_pspi_exchange_data(SPI1, sizeof(send_data), (uint8_t*)&send_data, (uint8_t*)&rec_data);
        wiced_hal_gpio_set_pin_output(CS, GPIO_PIN_OUTPUT_HIGH);
        WICED_BT_TRACE("Sent Random Number: %d\n", send_data.data);
        WICED_BT_TRACE("Received Number: %d\n", rec_data.data);

        /* Check if the data matches the sent data. If not, send flush bytes */
        if(rec_data.data == prev_rand)
        {
            WICED_BT_TRACE("Matching\n\n");
            prev_rand = send_data.data;
        }
        else
        {
            WICED_BT_TRACE("Not Matching\n");
            WICED_BT_TRACE("%d Sec Timeout\n\n", TIMEOUT);
            timeout = TIMEOUT;
            wiced_hal_pspi_tx_data(SPI1, sizeof(flush_data_tx), (uint8_t*)flush_data_tx);
            prev_rand = 0;
        }
    }
    else
    {
        timeout--;
    }
}



/*****************************************************************************************
*  Delay Function
****************************************************************************************/
void delay_cb(uint32_t arg)
{
    /* No Operation */
}


/****************************************************************************************
 * This function gives SPI master gpio config
 ***************************************************************************************/
uint32_t spi_master_get_gpio_cfg(uint8_t clk, uint8_t mosi, uint8_t miso)
{
    uint32_t spiGpioCfg = 0;

    spiGpioCfg |= (clk << 16);
    spiGpioCfg |= (mosi << 8);
    spiGpioCfg |= (miso);

    return (spiGpioCfg);
}

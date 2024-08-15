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
 * WICED sample application for SPI slave usage
 *
 * This application demonstrates how to use SPI driver interface
 * to send and receive bytes or a stream of bytes over the SPI hardware as a slave.
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
#include "wiced_timer.h"


/******************************************************************************
*                                Constants
******************************************************************************/
/*start and ack state machine states*/
#define WAIT_FOR_START                      (0u)
#define SEND_ACK                            (1u)
#define START_ID                            (0xAA)
#define ACK_ID                              (0x55)
#define PACKET_HEADER                       (0x66)            /* Data Header */
#define START_TIMER                         (100u)             /*100 ms timer for start and ack*/
#define RAND_TIMER                          (1u)               /*1 s timer for Random numbers*/

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
static uint8_t              start_and_ack_state     = WAIT_FOR_START;          /*State variable*/
static uint32_t             prev_rand               = 0;
static uint32_t             spi_slave_gpio_cfg      = 0;
static wiced_timer_t        milli_seconds_start_timer;
static wiced_timer_t        seconds_rand_timer;


/******************************************************************************
 *                                Function Definitions
******************************************************************************/
static void         initialize_app( void );
static void         start_and_ack (uint32_t arg);
static void         spi_slave_driver( uint32_t arg );
static uint32_t     spi_slave_get_gpio_cfg(uint8_t cs, uint8_t clk, uint8_t mosi, uint8_t miso);


/******************************************************************************
*  Entry point to the application. Initialize the app
*****************************************************************************/
APPLICATION_START( )
{
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );

    WICED_BT_TRACE("\r\nSample SPI Slave Application\r\n");

    initialize_app();
}


/******************************************************************************
 * This functions initializes the SPI and timer
*******************************************************************************/
void initialize_app( void )
{

    spi_slave_gpio_cfg = spi_slave_get_gpio_cfg(CS, CLK, MOSI, MISO);

    WICED_BT_TRACE("\r\nSPI Slave GPIO Config Value: \t %x \r\n",spi_slave_gpio_cfg);

    wiced_hal_pspi_reset(SPI1);

    /*Initialize SPI slave*/
    wiced_hal_pspi_init(SPI1, SPI_SLAVE, INPUT_PIN_PULL_UP, spi_slave_gpio_cfg, 0, SPI_LSB_FIRST, SPI_SS_ACTIVE_LOW, SPI_MODE_0, CS);

    /*Enable Tx and Rx buffers*/
    wiced_hal_pspi_slave_enable_rx(SPI1);
    wiced_hal_pspi_slave_enable_tx(SPI1);

    /*Start a timer for 100 ms*/
    if ( WICED_SUCCESS == wiced_init_timer( &milli_seconds_start_timer, &start_and_ack, 0, WICED_MILLI_SECONDS_PERIODIC_TIMER ))
    {
        if ( WICED_SUCCESS != wiced_start_timer( &milli_seconds_start_timer, START_TIMER ))
        {
            WICED_BT_TRACE( "Milli Seconds start Timer Error\n\r" );
        }
    }

}

/****************************************************************************************
 * This function waits for a start byte from the master and sends an ACK byte
 * and a frequency byte
 * This function is called every 100 ms by the timer. After sending the
 * frequency and ACK bytes, it stops the current timer and starts a new timer
 * for 1 second to exchange data with the master
 *****************************************************************************************/
void start_and_ack (uint32_t arg)
{

    uint32_t rx_fifo_count  = 0;
    uint8_t rec_byte        = 0;
    uint8_t start_byte      = START_ID;
    uint8_t ack_byte        = ACK_ID;
    uint8_t freq            = 3;
    uint8_t send_byte[2];

    send_byte[0] = ack_byte;
    send_byte[1] = freq;

    switch (start_and_ack_state)
    {
    case WAIT_FOR_START:
        /*Check for start byte and put ACK byte in buffer*/
        rx_fifo_count = wiced_hal_pspi_slave_get_rx_fifo_count(SPI1);

        if (rx_fifo_count >= sizeof(start_byte))
        {
            wiced_hal_pspi_slave_rx_data(SPI1, sizeof(rec_byte), &rec_byte);

            if (START_ID == rec_byte)
            {
                /*Disable Rx buffer so that it doesn't receive data from master while sending ACK*/
                wiced_hal_pspi_slave_disable_rx(SPI1);
                wiced_hal_pspi_slave_tx_data(SPI1, sizeof(send_byte), send_byte);
                start_and_ack_state = SEND_ACK;
            }

        }
        break;

    case SEND_ACK:
        /*Wait while master reads the buffer*/
        if (0 != wiced_hal_pspi_slave_get_tx_fifo_count(SPI1))
        {
            break;
        }
        else
        {
            if (WICED_SUCCESS != wiced_stop_timer(&milli_seconds_start_timer))
            {
                WICED_BT_TRACE("Timer Error while stopping\n\r");
            }

            wiced_hal_pspi_slave_enable_rx(SPI1);

            if (WICED_SUCCESS == wiced_init_timer(&seconds_rand_timer, &spi_slave_driver, 0, WICED_SECONDS_PERIODIC_TIMER))
            {
                if (WICED_SUCCESS != wiced_start_timer(&seconds_rand_timer, RAND_TIMER))
                {
                    WICED_BT_TRACE("Seconds Random Timer Error\n\r");
                }

            }
            WICED_BT_TRACE("TxFIFO empty\n");
            break;
        }

    default:
        break;
    }
}

/****************************************************************************************
 * This function generates random numbers and exchanges it with master
 ****************************************************************************************/
void spi_slave_driver( uint32_t arg )
{
    data_packet send_data;
    data_packet rec_data;
    uint32_t rx_fifo_count = 0;
    uint32_t tx_fifo_count = 20;

    /*Check for number of bytes received*/
    rx_fifo_count = wiced_hal_pspi_slave_get_rx_fifo_count(SPI1);
    if(sizeof(rec_data) <= rx_fifo_count)
    {
        /*Exchange random numbers*/
        wiced_hal_pspi_slave_rx_data(SPI1, sizeof(rec_data), (uint8_t*)&rec_data);

        /* Check if data header received. If yes the proceed with normal exchange */
        if (PACKET_HEADER == rec_data.header)
        {
            send_data = rec_data;
            wiced_hal_pspi_slave_tx_data(SPI1, sizeof(send_data), (uint8_t*)&send_data);
            WICED_BT_TRACE("Sent Number: %d\n", prev_rand);
            WICED_BT_TRACE("Received Random Number: %d\n\n", rec_data.data);
            prev_rand = rec_data.data;
        }
        else
        {
            wiced_hal_pspi_reset(SPI1);
            wiced_hal_pspi_slave_enable_rx(SPI1);
            wiced_hal_pspi_slave_enable_tx(SPI1);
        }
    }
}


/****************************************************************************************
 * This function gives SPI slave gpio config
 ***************************************************************************************/
uint32_t spi_slave_get_gpio_cfg(uint8_t cs, uint8_t clk, uint8_t mosi, uint8_t miso)
{
    uint32_t spiGpioCfg = 0;

    spiGpioCfg |= (cs << 24);
    spiGpioCfg |= (clk << 16);
    spiGpioCfg |= (mosi << 8);
    spiGpioCfg |= (miso);

    return (spiGpioCfg);
}

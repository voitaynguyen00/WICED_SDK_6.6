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
 * Defines board support package for CYW943012EVB_04 board
 */
#include "platform_config.h"
#include "platform_init.h"
#include "platform_isr.h"
#include "platform_peripheral.h"
#include "platform_bluetooth.h"
#include "wwd_platform_common.h"
#include "wwd_rtos_isr.h"
#include "wiced_defaults.h"
#include "wiced_platform.h"
#include "platform_mfi.h"
#include "platform_button.h"
#include "gpio_button.h"
#include "wiced_low_power.h"

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

platform_result_t platform_bluetooth_powersave_wakeup_handler( void );
platform_result_t platform_bluetooth_powersave_sleep_handler( void );

/******************************************************
 *               Variable Definitions
 ******************************************************/

/* GPIO pin table. Used by WICED/platform/MCU/wiced_platform_common.c */
const platform_gpio_t platform_gpio_pins[] =
{
    [WICED_GPIO_1]   = { GPIOA,  0 },
    [WICED_GPIO_2]   = { GPIOA,  1 },
    [WICED_GPIO_3]   = { GPIOA,  2 },
    [WICED_GPIO_4]   = { GPIOA,  3 },
    [WICED_GPIO_5]   = { GPIOA,  4 },
    [WICED_GPIO_6]   = { GPIOA,  5 },
    [WICED_GPIO_7]   = { GPIOA,  6 },
    [WICED_GPIO_8]   = { GPIOA,  7 },
    [WICED_GPIO_9]   = { GPIOA,  8 },
    [WICED_GPIO_10]  = { GPIOA,  9 },
    [WICED_GPIO_11]  = { GPIOA, 10 },
    [WICED_GPIO_12]  = { GPIOA, 11 },
    [WICED_GPIO_13]  = { GPIOA, 12 },
    [WICED_GPIO_14]  = { GPIOA, 13 },
    [WICED_GPIO_15]  = { GPIOA, 14 },
    [WICED_GPIO_16]  = { GPIOA, 15 },
    [WICED_GPIO_17]  = { GPIOB,  0 },
    [WICED_GPIO_18]  = { GPIOB,  1 },
    [WICED_GPIO_19]  = { GPIOB,  2 },
    [WICED_GPIO_20]  = { GPIOB,  3 },
    [WICED_GPIO_21]  = { GPIOB,  4 },
    [WICED_GPIO_22]  = { GPIOB,  5 },
    [WICED_GPIO_23]  = { GPIOB,  6 },
    [WICED_GPIO_24]  = { GPIOB,  7 },
    [WICED_GPIO_25]  = { GPIOB,  8 },
    [WICED_GPIO_26]  = { GPIOB,  9 },
    [WICED_GPIO_27]  = { GPIOB, 10 },
    [WICED_GPIO_28]  = { GPIOB, 11 },
    [WICED_GPIO_29]  = { GPIOB, 12 },
    [WICED_GPIO_30]  = { GPIOB, 13 },
    [WICED_GPIO_31]  = { GPIOB, 14 },
    [WICED_GPIO_32]  = { GPIOB, 15 },
    [WICED_GPIO_33]  = { GPIOC,  0 },
    [WICED_GPIO_34]  = { GPIOC,  1 },
    [WICED_GPIO_35]  = { GPIOC,  2 },
    [WICED_GPIO_36]  = { GPIOC,  3 },
    [WICED_GPIO_37]  = { GPIOC,  4 },
    [WICED_GPIO_38]  = { GPIOC,  5 },
    [WICED_GPIO_39]  = { GPIOC,  6 },
    [WICED_GPIO_40]  = { GPIOC,  7 },
    [WICED_GPIO_41]  = { GPIOC,  8 },
    [WICED_GPIO_42]  = { GPIOC,  9 },
    [WICED_GPIO_43]  = { GPIOC, 10 },
    [WICED_GPIO_44]  = { GPIOC, 11 },
    [WICED_GPIO_45]  = { GPIOC, 12 },
    [WICED_GPIO_46]  = { GPIOC, 13 },
    [WICED_GPIO_47]  = { GPIOC, 14 },
    [WICED_GPIO_48]  = { GPIOC, 15 },
    [WICED_GPIO_49]  = { GPIOD,  0 },
    [WICED_GPIO_50]  = { GPIOD,  1 },
    [WICED_GPIO_51]  = { GPIOD,  2 },
    [WICED_GPIO_52]  = { GPIOD,  3 },
    [WICED_GPIO_53]  = { GPIOD,  4 },
    [WICED_GPIO_54]  = { GPIOD,  5 },
    [WICED_GPIO_55]  = { GPIOD,  6 },
    [WICED_GPIO_56]  = { GPIOD,  7 },
    [WICED_GPIO_57]  = { GPIOD,  8 },
    [WICED_GPIO_58]  = { GPIOD,  9 },
    [WICED_GPIO_59]  = { GPIOD, 10 },
    [WICED_GPIO_60]  = { GPIOD, 11 },
    [WICED_GPIO_61]  = { GPIOD, 12 },
    [WICED_GPIO_62]  = { GPIOD, 13 },
    [WICED_GPIO_63]  = { GPIOD, 14 },
    [WICED_GPIO_64]  = { GPIOD, 15 },
    [WICED_GPIO_65]  = { GPIOE,  0 },
    [WICED_GPIO_66]  = { GPIOE,  1 },
    [WICED_GPIO_67]  = { GPIOE,  2 },
    [WICED_GPIO_68]  = { GPIOE,  3 },
    [WICED_GPIO_69]  = { GPIOE,  4 },
    [WICED_GPIO_70]  = { GPIOE,  5 },
    [WICED_GPIO_71]  = { GPIOE,  6 },
    [WICED_GPIO_72]  = { GPIOE,  7 },
    [WICED_GPIO_73]  = { GPIOE,  8 },
    [WICED_GPIO_74]  = { GPIOE,  9 },
    [WICED_GPIO_75]  = { GPIOE, 10 },
    [WICED_GPIO_76]  = { GPIOE, 11 },
    [WICED_GPIO_77]  = { GPIOE, 12 },
    [WICED_GPIO_78]  = { GPIOE, 13 },
    [WICED_GPIO_79]  = { GPIOE, 14 },
    [WICED_GPIO_80]  = { GPIOE, 15 },
    [WICED_GPIO_81]  = { GPIOF,  0 },
    [WICED_GPIO_82]  = { GPIOF,  1 },
    [WICED_GPIO_83]  = { GPIOF,  2 },
    [WICED_GPIO_84]  = { GPIOF,  3 },
    [WICED_GPIO_85]  = { GPIOF,  4 },
    [WICED_GPIO_86]  = { GPIOF,  5 },
    [WICED_GPIO_87]  = { GPIOF,  6 },
    [WICED_GPIO_88]  = { GPIOF,  7 },
    [WICED_GPIO_89]  = { GPIOF,  8 },
    [WICED_GPIO_90]  = { GPIOF,  9 },
    [WICED_GPIO_91]  = { GPIOF, 10 },
    [WICED_GPIO_92]  = { GPIOF, 11 },
    [WICED_GPIO_93]  = { GPIOF, 12 },
    [WICED_GPIO_94]  = { GPIOF, 13 },
    [WICED_GPIO_95]  = { GPIOF, 14 },
    [WICED_GPIO_96]  = { GPIOF, 15 },
    [WICED_GPIO_97]  = { GPIOG,  0 },
    [WICED_GPIO_98]  = { GPIOG,  1 },
    [WICED_GPIO_99]  = { GPIOG,  2 },
    [WICED_GPIO_100] = { GPIOG,  3 },
    [WICED_GPIO_101] = { GPIOG,  4 },
    [WICED_GPIO_102] = { GPIOG,  5 },
    [WICED_GPIO_103] = { GPIOG,  6 },
    [WICED_GPIO_104] = { GPIOG,  7 },
    [WICED_GPIO_105] = { GPIOG,  8 },
    [WICED_GPIO_106] = { GPIOG,  9 },
    [WICED_GPIO_107] = { GPIOG, 10 },
    [WICED_GPIO_108] = { GPIOG, 11 },
    [WICED_GPIO_109] = { GPIOG, 12 },
    [WICED_GPIO_110] = { GPIOG, 13 },
    [WICED_GPIO_111] = { GPIOG, 14 },
    [WICED_GPIO_112] = { GPIOG, 15 },
    [WICED_GPIO_113] = { GPIOH,  0 },
    [WICED_GPIO_114] = { GPIOH,  1 },
    [WICED_GPIO_115] = { GPIOH,  2 },
    [WICED_GPIO_116] = { GPIOH,  3 },
    [WICED_GPIO_117] = { GPIOH,  4 },
    [WICED_GPIO_118] = { GPIOH,  5 },
    [WICED_GPIO_119] = { GPIOH,  6 },
    [WICED_GPIO_120] = { GPIOH,  7 },
    [WICED_GPIO_121] = { GPIOH,  8 },
    [WICED_GPIO_122] = { GPIOH,  9 },
    [WICED_GPIO_123] = { GPIOH, 10 },
    [WICED_GPIO_124] = { GPIOH, 11 },
    [WICED_GPIO_125] = { GPIOH, 12 },
    [WICED_GPIO_126] = { GPIOH, 13 },
    [WICED_GPIO_127] = { GPIOH, 14 },
    [WICED_GPIO_128] = { GPIOH, 15 },
    [WICED_GPIO_129] = { GPIOI,  0 },
    [WICED_GPIO_130] = { GPIOI,  1 },
    [WICED_GPIO_131] = { GPIOI,  2 },
    [WICED_GPIO_132] = { GPIOI,  3 },
    [WICED_GPIO_133] = { GPIOI,  4 },
    [WICED_GPIO_134] = { GPIOI,  5 },
    [WICED_GPIO_135] = { GPIOI,  6 },
    [WICED_GPIO_136] = { GPIOI,  7 },
    [WICED_GPIO_137] = { GPIOI,  8 },
    [WICED_GPIO_138] = { GPIOI,  9 },
    [WICED_GPIO_139] = { GPIOI, 10 },
    [WICED_GPIO_140] = { GPIOI, 11 },
};

/* ADC peripherals. Used WICED/platform/MCU/wiced_platform_common.c */
const platform_adc_t platform_adc_peripherals[] =
{
    [WICED_ADC_1] = { 0 },
};

/* PWM peripherals. Used by WICED/platform/MCU/wiced_platform_common.c */
const platform_pwm_t platform_pwm_peripherals[] =
{
    [WICED_PWM_1]  = {TIM4,  TIM_CHANNEL_2, &platform_gpio_pins[WICED_GPIO_24]}, //TODO - Modify this struct
    [WICED_PWM_2]  = {TIM1,  TIM_CHANNEL_1, NULL},
    [WICED_PWM_3]  = {TIM2,  TIM_CHANNEL_1, NULL},
    [WICED_PWM_4]  = {TIM3,  TIM_CHANNEL_1, NULL},
    [WICED_PWM_5]  = {TIM5,  TIM_CHANNEL_1, NULL},
    [WICED_PWM_6]  = {TIM8,  TIM_CHANNEL_1, NULL},
    [WICED_PWM_7]  = {TIM15, TIM_CHANNEL_1, NULL},
    [WICED_PWM_8]  = {TIM16, TIM_CHANNEL_1, NULL},
    [WICED_PWM_9]  = {TIM17, TIM_CHANNEL_1, NULL},
};


/* PWM peripherals. Used by WICED/platform/MCU/wiced_platform_common.c */
const platform_spi_t platform_spi_peripherals[] =
{
    [WICED_SPI_1] = { 0 },
};

const platform_i2c_t platform_i2c_peripherals[] =
{
    [WICED_I2C_1] =
    {
        .port           = I2C1,
        .pin_scl        = &platform_gpio_pins[WICED_GPIO_25],
        .pin_sda        = &platform_gpio_pins[WICED_GPIO_24],
        .tx_dma_config  = { NULL },
        .rx_dma_config  = { NULL },
    },
    [WICED_I2C_2] =
    {
        .port           = I2C2,
        .pin_scl        = &platform_gpio_pins[WICED_GPIO_30],
        .pin_sda        = &platform_gpio_pins[WICED_GPIO_31],
        .tx_dma_config  = { NULL },
        .rx_dma_config  = { NULL },
    },
    [WICED_I2C_3] =
    {
        .port           = I2C3,
        .pin_scl        = NULL,
        .pin_sda        = NULL,
        .tx_dma_config  = { NULL },
        .rx_dma_config  = { NULL },
    },
    [WICED_I2C_4] =
    {
        .port           = I2C4,
        .pin_scl        = &platform_gpio_pins[WICED_GPIO_61],
        .pin_sda        = &platform_gpio_pins[WICED_GPIO_62],
        .tx_dma_config      =
        {
            .controller     = DMA2,
            .stream         = DMA2_Channel2,
            .channel        = DMA_REQUEST_0,
            .irq_vector     = DMA2_Channel2_IRQn,
        },
        .rx_dma_config =
        {
            .controller     = DMA2,
            .stream         = DMA2_Channel1,
            .channel        = DMA_REQUEST_0,
            .irq_vector     = DMA2_Channel1_IRQn,
        },
    },
};
platform_i2c_driver_t platform_i2c_driver[WICED_I2C_MAX];

/* Uncomment following if Auth IC is populated in the boards */
#if 0
/* MFI-related variables */
const wiced_i2c_device_t auth_chip_i2c_device =
{
    .port          = AUTH_IC_I2C_PORT,
    .address       = 0x11,
    .address_width = I2C_ADDRESS_WIDTH_7BIT,
    .speed_mode    = I2C_STANDARD_SPEED_MODE,
};

const platform_mfi_auth_chip_t platform_auth_chip =
{
    .i2c_device = &auth_chip_i2c_device,
    .reset_pin  = WICED_GPIO_AUTH_RST
};
#endif

/* UART peripherals and runtime drivers. Used by WICED/platform/MCU/wiced_platform_common.c */
const platform_uart_t platform_uart_peripherals[] =
{
    [WICED_UART_1] =
    {
        .port               = USART1,
        .tx_pin             = &platform_gpio_pins[WICED_GPIO_10],
        .rx_pin             = &platform_gpio_pins[WICED_GPIO_11],
        .cts_pin            = NULL,
        .rts_pin            = NULL,
        .tx_dma_config      =
        {
            .controller     = DMA2,
            .stream         = DMA2_Channel6,
            .channel        = DMA_REQUEST_2,
            .irq_vector     = DMA2_Channel6_IRQn,
        },
        .rx_dma_config =
        {
            .controller     = DMA2,
            .stream         = DMA2_Channel7,
            .channel        = DMA_REQUEST_2,
            .irq_vector     = DMA2_Channel7_IRQn,
        },
    },
    [WICED_UART_2] =
    {
        .port               = USART2,
        .tx_pin             = &platform_gpio_pins[WICED_GPIO_54],
        .rx_pin             = &platform_gpio_pins[WICED_GPIO_4],
        .cts_pin            = &platform_gpio_pins[WICED_GPIO_1],
        .rts_pin            = &platform_gpio_pins[WICED_GPIO_53],
        .tx_dma_config      =
        {
            .controller     = DMA1,
            .stream         = DMA1_Channel7,
            .channel        = DMA_REQUEST_2,
            .irq_vector     = DMA1_Channel7_IRQn,
        },
        .rx_dma_config =
        {
            .controller     = DMA1,
            .stream         = DMA1_Channel6,
            .channel        = DMA_REQUEST_2,
            .irq_vector     = DMA1_Channel6_IRQn,
        },
    },
    [WICED_UART_3] =
    {
        .port               = USART3,
        .tx_pin             = NULL,
        .rx_pin             = NULL,
        .cts_pin            = NULL,
        .rts_pin            = NULL,
    },
    [WICED_UART_4] =
    {
        .port               = UART4,
        .tx_pin             = NULL,
        .rx_pin             = NULL,
        .cts_pin            = NULL,
        .rts_pin            = NULL,
    },
    [WICED_UART_5] =
    {
        .port               = UART5,
        .tx_pin             = NULL,
        .rx_pin             = NULL,
        .cts_pin            = NULL,
        .rts_pin            = NULL,
    },
};
platform_uart_driver_t platform_uart_drivers[WICED_UART_MAX];

/* SPI flash. Exposed to the applications through include/wiced_platform.h */
#if defined ( WICED_PLATFORM_INCLUDES_SPI_FLASH )
const wiced_spi_device_t wiced_spi_flash =
{

};
#endif

/* UART standard I/O configuration */
#ifndef WICED_DISABLE_STDIO
static const platform_uart_config_t stdio_config =
{
    .baud_rate    = 115200,
    .data_width   = DATA_WIDTH_8BIT,
    .parity       = NO_PARITY,
    .stop_bits    = STOP_BITS_1,
    .flow_control = FLOW_CONTROL_DISABLED,
};
#endif

/* Wi-Fi control pins. Used by WICED/platform/MCU/wwd_platform_common.c
 * SDIO: WWD_PIN_BOOTSTRAP[1:0] = b'00
 * gSPI: WWD_PIN_BOOTSTRAP[1:0] = b'01
 */
const platform_gpio_t wifi_control_pins[] =
{
    [WWD_PIN_POWER      ] = { GPIOD,  6 },
    [WWD_PIN_32K_CLK    ] = { 0 },
    [WWD_PIN_BOOTSTRAP_0] = { 0 },
    [WWD_PIN_BOOTSTRAP_1] = { 0 },
};

/* Wi-Fi SDIO bus pins. Used by WICED/platform/STM32F2xx/WWD/wwd_SDIO.c */
const platform_gpio_t wifi_sdio_pins[] =
{
    [WWD_PIN_SDIO_OOB_IRQ] = { GPIOD, 14  },
    [WWD_PIN_SDIO_CLK    ] = { GPIOC, 12  },
    [WWD_PIN_SDIO_CMD    ] = { GPIOD, 2  },
    [WWD_PIN_SDIO_D0     ] = { GPIOC, 8  },
    [WWD_PIN_SDIO_D1     ] = { GPIOC, 9  },
    [WWD_PIN_SDIO_D2     ] = { GPIOC, 10  },
    [WWD_PIN_SDIO_D3     ] = { GPIOC, 11  },
};

/* Wi-Fi gSPI bus pins. Used by WICED/platform/STM32F2xx/WWD/wwd_SPI.c */
const platform_gpio_t wifi_spi_pins[] =
{
    [WWD_PIN_SPI_IRQ ] = { 0 },
    [WWD_PIN_SPI_CS  ] = { 0 },
    [WWD_PIN_SPI_CLK ] = { 0 },
    [WWD_PIN_SPI_MOSI] = { 0 },
    [WWD_PIN_SPI_MISO] = { 0 },
};

const platform_gpio_t wiced_qspi_flash[] =
{
    [WICED_QSPI_PIN_CS]  = { GPIOE, 11 },
    [WICED_QSPI_PIN_CLK] = { GPIOE, 10 },
    [WICED_QSPI_PIN_D0]  = { GPIOB, 1 },
    [WICED_QSPI_PIN_D1]  = { GPIOE, 13 },
    [WICED_QSPI_PIN_D2]  = { GPIOE, 14},
    [WICED_QSPI_PIN_D3]  = { GPIOE, 15 },
};


/* Bluetooth control pins. Used by libraries/bluetooth/internal/bus/UART/bt_bus.c */
static const platform_gpio_t internal_bt_control_pins[] =
{
    /* Reset pin unavailable */
    [WICED_BT_PIN_POWER      ] = { GPIOC, 6 },
    [WICED_BT_PIN_HOST_WAKE  ] = { GPIOD, 0 },
    [WICED_BT_PIN_DEVICE_WAKE] = { GPIOC, 7 }
};
const platform_gpio_t* wiced_bt_control_pins[] =
{
    /* Reset pin unavailable */
    [WICED_BT_PIN_POWER      ] = &internal_bt_control_pins[WICED_BT_PIN_POWER      ],
    [WICED_BT_PIN_HOST_WAKE  ] = &internal_bt_control_pins[WICED_BT_PIN_HOST_WAKE  ],
    [WICED_BT_PIN_DEVICE_WAKE] = &internal_bt_control_pins[WICED_BT_PIN_DEVICE_WAKE],
    [WICED_BT_PIN_RESET      ] = NULL,
};

/* Bluetooth UART pins. Used by libraries/bluetooth/internal/bus/UART/bt_bus.c */
static const platform_gpio_t internal_bt_uart_pins[] =
{
    [WICED_BT_PIN_UART_TX ] = { GPIOD, 5 },
    [WICED_BT_PIN_UART_RX ] = { GPIOA, 3 },
    [WICED_BT_PIN_UART_CTS] = { GPIOA, 0 },
    [WICED_BT_PIN_UART_RTS] = { GPIOD, 4 },
};
const platform_gpio_t* wiced_bt_uart_pins[] =
{
    [WICED_BT_PIN_UART_TX ] = &internal_bt_uart_pins[WICED_BT_PIN_UART_TX ],
    [WICED_BT_PIN_UART_RX ] = &internal_bt_uart_pins[WICED_BT_PIN_UART_RX ],
    [WICED_BT_PIN_UART_CTS] = &internal_bt_uart_pins[WICED_BT_PIN_UART_CTS],
    [WICED_BT_PIN_UART_RTS] = &internal_bt_uart_pins[WICED_BT_PIN_UART_RTS],
};

const platform_uart_t*        wiced_bt_uart_peripheral = &platform_uart_peripherals[WICED_UART_2];
platform_uart_driver_t*       wiced_bt_uart_driver     = &platform_uart_drivers    [WICED_UART_2];

/* Bluetooth UART configuration. Used by libraries/bluetooth/internal/bus/UART/bt_bus.c */
const platform_uart_config_t wiced_bt_uart_config =
{
    .baud_rate    = 115200,
    .data_width   = DATA_WIDTH_8BIT,
    .parity       = NO_PARITY,
    .stop_bits    = STOP_BITS_1,
    .flow_control = FLOW_CONTROL_CTS_RTS,
};

/*BT chip specific configuration information*/
const platform_bluetooth_config_t wiced_bt_config =
{
    .patchram_download_mode      = PATCHRAM_DOWNLOAD_MODE_MINIDRV_CMD,
    .patchram_download_baud_rate = 115200,
    .featured_baud_rate          = 115200
};

const gpio_button_t platform_gpio_buttons[PLATFORM_BUTTON_MAX] =
{
    [PLATFORM_BUTTON_1] =
    {
        .gpio       = WICED_BUTTON1,
        .config     = INPUT_PULL_UP,    /* No External pull ups on CYW9WLWCDEVAL2 evaluation board */
        .polarity   = WICED_ACTIVE_LOW,
        .trigger    = IRQ_TRIGGER_BOTH_EDGES,
    },

    [PLATFORM_BUTTON_2] =
    {
        .gpio       = WICED_BUTTON2,
        .config     = INPUT_PULL_UP,    /* No External pull ups on CYW9WLWCDEVAL2 evaluation board */
        .polarity   = WICED_ACTIVE_LOW,
        .trigger    = IRQ_TRIGGER_BOTH_EDGES,
    },

    [PLATFORM_BUTTON_3] =
    {
        .gpio       = WICED_BUTTON3,
        .config     = INPUT_PULL_UP,    /* No External pull ups on CYW9WLWCDEVAL2 evaluation board */
        .polarity   = WICED_ACTIVE_LOW,
        .trigger    = IRQ_TRIGGER_BOTH_EDGES,
    },
};
const wiced_gpio_t platform_gpio_leds[PLATFORM_LED_COUNT] =
{
     [WICED_LED_INDEX_1] = WICED_LED1,
     [WICED_LED_INDEX_2] = WICED_LED2,
     [WICED_LED_INDEX_3] = WICED_LED3,
};

/******************************************************
 *               Function Definitions
 ******************************************************/

void platform_init_peripheral_irq_priorities( void )
{
    /* Interrupt priority setup. Called by WICED/platform/MCU/STM32L4xx/platform_init.c */
    NVIC_SetPriority( SDMMC1_IRQn       ,  3 ); /* WLAN SDIO           */
    NVIC_SetPriority( DMA2_Channel4_IRQn,  2 ); /* WLAN SDIO DMA       */
    NVIC_SetPriority( DMA2_Channel6_IRQn,  6 ); /* WICED_UART_1 TX DMA */
    NVIC_SetPriority( DMA1_Channel7_IRQn,  6 ); /* WICED_UART_2 TX DMA */
    NVIC_SetPriority( USART1_IRQn       ,  6 ); /* WICED_UART_1        */
    NVIC_SetPriority( USART2_IRQn       ,  6 ); /* WICED_UART_2        */
    NVIC_SetPriority( USART3_IRQn       ,  6 ); /* WICED_UART_3        */
    NVIC_SetPriority( UART4_IRQn        ,  6 ); /* WICED_UART_4        */
    NVIC_SetPriority( UART5_IRQn        ,  6 ); /* WICED_UART_5        */
    NVIC_SetPriority( EXTI0_IRQn        , 14 ); /* GPIO                */
    NVIC_SetPriority( EXTI1_IRQn        , 14 ); /* GPIO                */
    NVIC_SetPriority( EXTI2_IRQn        , 14 ); /* GPIO                */
    NVIC_SetPriority( EXTI3_IRQn        , 14 ); /* GPIO                */
    NVIC_SetPriority( EXTI4_IRQn        , 14 ); /* GPIO                */
    NVIC_SetPriority( EXTI9_5_IRQn      , 14 ); /* GPIO                */
    NVIC_SetPriority( EXTI15_10_IRQn    , 14 ); /* GPIO                */
    NVIC_SetPriority( DMA2_Channel1_IRQn,  6 ); /* WICED_I2C_4 DMA RX  */
    NVIC_SetPriority( DMA2_Channel2_IRQn,  6 ); /* WICED_I2C_4 DMA TX  */
    NVIC_SetPriority( I2C1_EV_IRQn,        6 ); /* WICED_I2C_1 EVENT   */
    NVIC_SetPriority( I2C2_EV_IRQn,        6 ); /* WICED_I2C_2 EVENT   */
    NVIC_SetPriority( I2C3_EV_IRQn,        6 ); /* WICED_I2C_3 EVENT   */
    NVIC_SetPriority( I2C4_EV_IRQn,        6 ); /* WICED_I2C_4 EVENT   */
    NVIC_SetPriority( I2C1_ER_IRQn,        6 ); /* WICED_I2C_1 ERROR   */
    NVIC_SetPriority( I2C2_ER_IRQn,        6 ); /* WICED_I2C_2 ERROR   */
    NVIC_SetPriority( I2C3_ER_IRQn,        6 ); /* WICED_I2C_3 ERROR   */
    NVIC_SetPriority( I2C4_ER_IRQn,        6 ); /* WICED_I2C_4 ERROR   */
}

/* LEDs on this platform are active HIGH */
platform_result_t platform_led_set_state(int led_index, int off_on )
{
    if ((led_index >= 0) && (led_index < PLATFORM_LED_COUNT))
    {
        switch (off_on)
        {
            case WICED_LED_OFF:
                platform_gpio_output_high( &platform_gpio_pins[platform_gpio_leds[led_index]] );
                break;
            case WICED_LED_ON:
                platform_gpio_output_low( &platform_gpio_pins[platform_gpio_leds[led_index]] );
                break;
        }
        return PLATFORM_SUCCESS;
    }
    return PLATFORM_BADARG;
}

void platform_led_init( void )
{
    /* Initialise LEDs and turn off by default */
    platform_gpio_init( &platform_gpio_pins[WICED_LED1], OUTPUT_PUSH_PULL );
    platform_gpio_init( &platform_gpio_pins[WICED_LED2], OUTPUT_PUSH_PULL );
    platform_gpio_init( &platform_gpio_pins[WICED_LED3], OUTPUT_PUSH_PULL );

    platform_led_set_state(WICED_LED_INDEX_1, WICED_LED_OFF);
    platform_led_set_state(WICED_LED_INDEX_2, WICED_LED_OFF);
    platform_led_set_state(WICED_LED_INDEX_3, WICED_LED_OFF);

 }

platform_result_t platform_powersave_sleep_handler( void )
{
    return PLATFORM_SUCCESS;
}

void platform_init_external_devices( void )
{
    /* Initialise LEDs and turn off by default */
    platform_led_init();

    /* Initialize buttons */
    platform_gpio_init( &platform_gpio_pins[ platform_gpio_buttons[PLATFORM_BUTTON_1].gpio ], platform_gpio_buttons[PLATFORM_BUTTON_1].config );
    platform_gpio_init( &platform_gpio_pins[ platform_gpio_buttons[PLATFORM_BUTTON_2].gpio ], platform_gpio_buttons[PLATFORM_BUTTON_2].config );
    platform_gpio_init( &platform_gpio_pins[ platform_gpio_buttons[PLATFORM_BUTTON_3].gpio ], platform_gpio_buttons[PLATFORM_BUTTON_3].config );

#ifndef WICED_DISABLE_STDIO
    /* Initialise UART standard I/O */
    platform_stdio_init( &platform_uart_drivers[STDIO_UART], &platform_uart_peripherals[STDIO_UART], &stdio_config );
#endif
}

uint32_t  platform_get_button_press_time ( int button_index, int led_index, uint32_t max_time )
{
    int             button_gpio;
    uint32_t        button_press_timer = 0;
    int             led_state = 0;

    /* Initialize input */
    button_gpio     = platform_gpio_buttons[button_index].gpio;
    platform_gpio_init( &platform_gpio_pins[ button_gpio ], platform_gpio_buttons[button_index].config );

    while ( (PLATFORM_BUTTON_PRESSED_STATE == platform_gpio_input_get(&platform_gpio_pins[ button_gpio ])) )
    {
        /* wait a bit */
        host_rtos_delay_milliseconds( PLATFORM_BUTTON_PRESS_CHECK_PERIOD );

        /* Toggle LED */
        platform_led_set_state(led_index, (led_state == 0) ? WICED_LED_OFF : WICED_LED_ON);
        led_state ^= 0x01;

        /* keep track of time */
        button_press_timer += PLATFORM_BUTTON_PRESS_CHECK_PERIOD;
        if ((max_time > 0) && (button_press_timer >= max_time))
        {
            break;
        }
    }

     /* turn off the LED */
    platform_led_set_state(led_index, WICED_LED_OFF );

    /* deinit the button */
    platform_gpio_deinit( &platform_gpio_pins[ button_gpio ] );

    return button_press_timer;
}

uint32_t  platform_get_factory_reset_button_time ( uint32_t max_time )
{
    return platform_get_button_press_time ( PLATFORM_FACTORY_RESET_BUTTON_INDEX, PLATFORM_RED_LED_INDEX, max_time );
}

#if 0
platform_result_t platform_bluetooth_powersave_wakeup_handler( void )
{
    return PLATFORM_SUCCESS;
}

static void host_wake_irq_handler( void* arg )
{

}

platform_result_t platform_bluetooth_powersave_sleep_handler( void )
{
    return PLATFORM_SUCCESS;
}

static void platform_sleep_event_handler( wiced_sleep_event_type_t event, int handler )
{
    switch ( event )
    {
        case WICED_SLEEP_EVENT_ENTER:
            platform_bluetooth_powersave_sleep_handler();
            break;

        case WICED_SLEEP_EVENT_CANCEL:
            platform_bluetooth_powersave_wakeup_handler();
            break;

        case WICED_SLEEP_EVENT_LEAVE:
            platform_bluetooth_powersave_wakeup_handler();
            break;

        default:
            break;
    }
}

/* Call back , which is called before entering or after leaving sleep state */
WICED_SLEEP_EVENT_HANDLER( platform_sleep_event_handler_1 )
{
    platform_sleep_event_handler( event, 0 );
}
#endif

/******************************************************
 *           Interrupt Handler Definitions
 ******************************************************/

WWD_RTOS_DEFINE_ISR( usart1_irq )
{
    platform_uart_irq( &platform_uart_drivers[WICED_UART_1] );
}

WWD_RTOS_DEFINE_ISR( usart2_irq )
{
    platform_uart_irq( &platform_uart_drivers[WICED_UART_2] );
}

WWD_RTOS_DEFINE_ISR( usart3_irq )
{
    platform_uart_irq( &platform_uart_drivers[WICED_UART_3] );
}

WWD_RTOS_DEFINE_ISR( uart4_irq )
{
    platform_uart_irq( &platform_uart_drivers[WICED_UART_4] );
}

WWD_RTOS_DEFINE_ISR( uart5_irq )
{
    platform_uart_irq( &platform_uart_drivers[WICED_UART_5] );
}

WWD_RTOS_DEFINE_ISR( gpio_irq )
{
    platform_gpio_irq();
}

WWD_RTOS_DEFINE_ISR( i2c1_ev_irq )
{
    platform_i2c_ev_irq( &platform_i2c_peripherals[WICED_I2C_1] );
}

WWD_RTOS_DEFINE_ISR( i2c2_ev_irq )
{
    platform_i2c_ev_irq( &platform_i2c_peripherals[WICED_I2C_2] );
}

WWD_RTOS_DEFINE_ISR( i2c3_ev_irq )
{
    platform_i2c_ev_irq( &platform_i2c_peripherals[WICED_I2C_3] );
}

WWD_RTOS_DEFINE_ISR( i2c4_ev_irq )
{
    platform_i2c_ev_irq( &platform_i2c_peripherals[WICED_I2C_4] );
}

WWD_RTOS_DEFINE_ISR( i2c1_er_irq )
{
    platform_i2c_er_irq( &platform_i2c_peripherals[WICED_I2C_1] );
}

WWD_RTOS_DEFINE_ISR( i2c2_er_irq )
{
    platform_i2c_er_irq( &platform_i2c_peripherals[WICED_I2C_2] );
}

WWD_RTOS_DEFINE_ISR( i2c3_er_irq )
{
    platform_i2c_er_irq( &platform_i2c_peripherals[WICED_I2C_3] );
}

WWD_RTOS_DEFINE_ISR( i2c4_er_irq )
{
    platform_i2c_er_irq( &platform_i2c_peripherals[WICED_I2C_4] );
}

WWD_RTOS_DEFINE_ISR( dma2_channel1_irq )
{
    platform_i2c_dma_rx_irq( &platform_i2c_peripherals[WICED_I2C_4] );
}

WWD_RTOS_DEFINE_ISR( dma2_channel2_irq )
{
    platform_i2c_dma_tx_irq( &platform_i2c_peripherals[WICED_I2C_4] );
}

WWD_RTOS_DEFINE_ISR( dma2_channel7_irq )
{
    platform_uart_dma_rx_irq( &platform_uart_drivers[WICED_UART_1] );
}

WWD_RTOS_DEFINE_ISR( dma2_channel6_irq )
{
    platform_uart_dma_tx_irq( &platform_uart_drivers[WICED_UART_1] );
}

WWD_RTOS_DEFINE_ISR( dma1_channel6_irq )
{
    platform_uart_dma_rx_irq( &platform_uart_drivers[WICED_UART_2] );
}

WWD_RTOS_DEFINE_ISR( dma1_channel7_irq )
{
    platform_uart_dma_tx_irq( &platform_uart_drivers[WICED_UART_2] );
}

/******************************************************
 *            Interrupt Handlers Mapping
 ******************************************************/

WWD_RTOS_MAP_ISR( usart1_irq       , USART1_irq        )
WWD_RTOS_MAP_ISR( usart2_irq       , USART2_irq        )
WWD_RTOS_MAP_ISR( usart3_irq       , USART3_irq        )
WWD_RTOS_MAP_ISR( uart4_irq        , UART4_irq         )
WWD_RTOS_MAP_ISR( uart5_irq        , UART5_irq         )

WWD_RTOS_MAP_ISR( gpio_irq         , EXTI0_irq         )
WWD_RTOS_MAP_ISR( gpio_irq         , EXTI1_irq         )
WWD_RTOS_MAP_ISR( gpio_irq         , EXTI2_irq         )
WWD_RTOS_MAP_ISR( gpio_irq         , EXTI3_irq         )
WWD_RTOS_MAP_ISR( gpio_irq         , EXTI4_irq         )
WWD_RTOS_MAP_ISR( gpio_irq         , EXTI9_5_irq       )
WWD_RTOS_MAP_ISR( gpio_irq         , EXTI15_10_irq     )

WWD_RTOS_MAP_ISR( i2c1_ev_irq      , I2C1_EV_irq       )
WWD_RTOS_MAP_ISR( i2c2_ev_irq      , I2C2_EV_irq       )
WWD_RTOS_MAP_ISR( i2c3_ev_irq      , I2C3_EV_irq       )
WWD_RTOS_MAP_ISR( i2c4_ev_irq      , I2C4_EV_irq       )
WWD_RTOS_MAP_ISR( i2c1_er_irq      , I2C1_ER_irq       )
WWD_RTOS_MAP_ISR( i2c2_er_irq      , I2C2_ER_irq       )
WWD_RTOS_MAP_ISR( i2c3_er_irq      , I2C3_ER_irq       )
WWD_RTOS_MAP_ISR( i2c4_er_irq      , I2C4_ER_irq       )


WWD_RTOS_MAP_ISR( dma2_channel6_irq, DMA2_Channel6_irq ) /* WICED_UART_1 DMA TX */
WWD_RTOS_MAP_ISR( dma2_channel7_irq, DMA2_Channel7_irq ) /* WICED_UART_1 DMA RX */
WWD_RTOS_MAP_ISR( dma1_channel7_irq, DMA1_Channel7_irq ) /* WICED_UART_2 DMA TX */
WWD_RTOS_MAP_ISR( dma1_channel6_irq, DMA1_Channel6_irq ) /* WICED_UART_2 DMA RX */
WWD_RTOS_MAP_ISR( dma2_channel1_irq, DMA2_Channel1_irq ) /* WICED_I2C_4 DMA RX  */
WWD_RTOS_MAP_ISR( dma2_channel2_irq, DMA2_Channel2_irq ) /* WICED_I2C_4 DMA TX  */

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
 * Defines internal configuration of the CYW943012EVB_04 board
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif
/******************************************************
 *  MCU Constants and Options
 ******************************************************/

/*  CPU clock : 80 MHz */
#define CPU_CLOCK_HZ                     ( 80000000 )

#define CONFIG_RCC_OSCILLATOR_SOURCE     ( RCC_OSCILLATORTYPE_MSI | RCC_OSCILLATORTYPE_HSI48 )
#define CONFIG_RCC_HSE_STATE             ( RCC_HSE_ON )
#define CONFIG_RCC_LSI_STATE             ( RCC_LSI_ON )
#define CONFIG_RCC_MSI_STATE             ( RCC_MSI_ON )
#define CONFIG_RCC_HSI_STATE             ( RCC_HSI_DIV1 )
#define CONFIG_RCC_HSI48_STATE           ( RCC_HSI48_ON )
#define CONFIG_RCC_LSE_STATE             ( RCC_LSE_ON )
#define CONFIG_RCC_PLL_STATE             ( RCC_PLL_ON )
#define CONFIG_RCC_PLL_SOURCE            ( RCC_PLLSOURCE_MSI )
#define CONFIG_RCC_PLL_M                 ( 1 )
#define CONFIG_RCC_PLL_N                 ( 40 )
#define CONFIG_RCC_PLL_P                 ( RCC_PLLP_DIV2 )
#define CONFIG_RCC_PLL_Q                 ( RCC_PLLQ_DIV2 )
#define CONFIG_RCC_PLL_R                 ( RCC_PLLR_DIV2 )
#define CONFIG_RCC_MSI_CALIBRATION_VALUE ( 0 )
#define CONFIG_RCC_MSI_CLOCK_RANGE       ( RCC_MSIRANGE_6 )

#define CONFIG_RCC_CLOCKTYPE             ( RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2 )
#define CONFIG_RCC_SYSCLKSOURCE          ( RCC_SYSCLKSOURCE_PLLCLK )
#define CONFIG_RCC_SYSCLKDIVIDER         ( RCC_SYSCLK_DIV1 )
#define CONFIG_RCC_AHB_CLKDIVIDER        ( RCC_HCLK_DIV1 )
#define CONFIG_RCC_APB1_CLKDIVIDER       ( RCC_HCLK_DIV1 )
#define CONFIG_RCC_APB2_CLKDIVIDER       ( RCC_HCLK_DIV1 )

#define CONFIG_RCC_PERIPHCLK_SELECTION   ( RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_SDMMC1 )
#define CONFIG_USART1_CLOCK_SELECTION    ( RCC_USART1CLKSOURCE_PCLK2 )
#define CONFIG_SDMMC1_CLOCK_SELECTION    ( RCC_SDMMC1CLKSOURCE_HSI48 )

#define CONFIG_FLASH_LATENCY             ( FLASH_LATENCY_4 )

/******************************************************
 *  Wi-Fi Options
 ******************************************************/

/*  GPIO pins are used to bootstrap Wi-Fi to SDIO or gSPI mode */
/* #define WICED_WIFI_USE_GPIO_FOR_BOOTSTRAP_1 */

/*  Wi-Fi GPIO0 pin is used for out-of-band interrupt */
#define WICED_WIFI_OOB_IRQ_GPIO_PIN  ( 0 )

/*  Wi-Fi power pin is present */
#define WICED_USE_WIFI_POWER_PIN

/* Wi-Fi power pin is active high */
#define WICED_USE_WIFI_POWER_PIN_ACTIVE_HIGH

/*  WLAN Powersave Clock Source
 *  The WLAN sleep clock can be driven from one of two sources:
 *  1. MCO (MCU Clock Output) - default
 *  2. WLAN 32K internal oscillator (30% inaccuracy)
 *     - Comment the following directive : WICED_USE_WIFI_32K_CLOCK_MCO
 */
/* #define WICED_USE_WIFI_32K_CLOCK_MCO */

/*  OTA */
/* #define PLATFORM_HAS_OTA */

/*  WICED Resources uses a filesystem */
#ifndef WWD_DIRECT_RESOURCES
#define USES_RESOURCE_FILESYSTEM
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

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
 * Ethernet Testing for Console Application
 *
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************
 *                     Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define ETHERNET_COMMANDS \
    { (char*) "ethernet_up",         ethernet_up,       0, NULL, NULL, "[ip netmask gateway]", (char*) "Brings up the Ethernet. DHCP assumed if no IP address provided"}, \
    { (char*) "ethernet_down",       ethernet_down,     0, NULL, NULL, NULL, (char*) "Brings down the ethernet"}, \
    { (char*) "ethernet_ping",       ethernet_ping,     0, NULL, NULL, (char*) "<destination> [-i <interval in ms>] [-n <number>] [-l <length>]", (char*) "Pings the specified IP or Host via Ethernet."}, \
    { (char*) "network_suspend",     network_suspend,   0, NULL, NULL, NULL, (char*) "Will suspend network"}, \
    { (char*) "network_resume",      network_resume,    0, NULL, NULL, NULL, (char*) "Will resume network"},

#define PING_THREAD_STACK_SIZE (2500)
#define PING_DESCRIPTION_LEN    (200)
#define PING_HISTORY_LEN          (5)
#define PING_RESULT_LEN          (30)

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
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/
int ethernet_up( int argc, char* argv[] );
int ethernet_down( int argc, char* argv[] );
int ethernet_ping( int argc, char* argv[] );
int network_suspend( int argc, char* argv[] );
int network_resume( int argc, char* argv[] );

#ifdef __cplusplus
}   /*extern "C"    */
#endif

#
# Copyright 2021, Cypress Semiconductor Corporation or a subsidiary of 
 # Cypress Semiconductor Corporation. All Rights Reserved.
 # This software, including source code, documentation and related
 # materials ("Software"), is owned by Cypress Semiconductor Corporation
 # or one of its subsidiaries ("Cypress") and is protected by and subject to
 # worldwide patent protection (United States and foreign),
 # United States copyright laws and international treaty provisions.
 # Therefore, you may use this Software only as provided in the license
 # agreement accompanying the software package from which you
 # obtained this Software ("EULA").
 # If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 # non-transferable license to copy, modify, and compile the Software
 # source code solely for use in connection with Cypress's
 # integrated circuit products. Any reproduction, modification, translation,
 # compilation, or representation of this Software except as specified
 # above is prohibited without the express written permission of Cypress.
 #
 # Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 # EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 # WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 # reserves the right to make changes to the Software without notice. Cypress
 # does not assume any liability arising out of the application or use of the
 # Software or any product or circuit described in the Software. Cypress does
 # not authorize its products for use in any products where a malfunction or
 # failure of the Cypress product may reasonably be expected to result in
 # significant property damage, injury or death ("High Risk Product"). By
 # including Cypress's product in a High Risk Product, the manufacturer
 # of such system or application assumes all risk of such use and in doing
 # so agrees to indemnify Cypress against all liability.
#

NAME := App_WiFi_Onboarding

$(NAME)_COMPONENTS := daemons/device_onboarding

$(NAME)_SOURCES    := wifi_onboarding.c

$(NAME)_INCLUDES   := . \
                      daemons/device_onboarding/

#.WIFI_CONFIG_DCT_H  := wifi_config_dct.h

VALID_PLATFORMS += BCM943909WCD* \
                   BCM943340WCD1 \
                   BCM943362WCD4 \
                   BCM943364WCD1 \
                   BCM9WCD1AUDIO \
                   BCM94343WWCD2 \
                   BCM943438WLPTH_2 \
                   CYW9MCU7X9N364 \
                   BCM943907WAE2_1 \
                   CYW943907AEVAL1F Quicksilver_EVL \
                   BCM943907AEVAL1F \
                   CYW9WCD2REFAD2* \
                   CYW9WCD760PINSDAD2 \
                   CYW943907AEVAL1F \
                   CYW954907AEVAL1F \
                   CYW943455EVB* \
                   CYW943012EVB* \
                   CY8CKIT_062 \
                   CYW943012P6EVB_01

INVALID_PLATFORMS  += CYW9MCU7X9N364

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
NAME := Lib_WICED_Bluetooth_Firmware_Driver_for_BCM$(BT_CHIP)$(BT_CHIP_REVISION)

# By default,the 20706A2/43012C0 will use the firmware images that are part of the release.
# The firmware driver is picked depending on the BT chip, XTAL frequency and the mode it is configured with(controller/embedded).
# E.g. If 20706 A1 BT chip with XTAL 40Mhz is selected in controller mode,
# then the file path would be 20706A1/40Mhz/bt_firmware_controller.c.
# E.g. If 20706 A2 BT chip with XTAL 40Mhz is selected in embedded mode,
# then the file path would be 20706A2/40Mhz/bt_firmware_embedded_(EMBEDDED_APP_NAME).c.
# For embedded mode, the EMBEDDED_APP_NAME should be provided in the application MakeFile.
# The EMBEDDED_APP_NAME will be same as the application name used to generate firmware in the 20706-SDK.
# NOTE: Only 20706A2 and 43012 will support both controller/embedded Mode. Rest of the BT chips support controller mode only.

ifneq ($(EMBEDDED_APP_NAME),)
ifneq ( $(BT_CHIP_XTAL_FREQUENCY), )
$(NAME)_SOURCES := $(BT_CHIP)$(BT_CHIP_REVISION)/$(BT_CHIP_XTAL_FREQUENCY)/bt_firmware_embedded_$(EMBEDDED_APP_NAME).c
else
$(NAME)_SOURCES := $(BT_CHIP)$(BT_CHIP_REVISION)/40MHz/bt_firmware_embedded_$(EMBEDDED_APP_NAME).c
endif
else
ifeq ( $(BT_CHIP_XTAL_FREQUENCY), )
$(NAME)_SOURCES := $(BT_CHIP)$(BT_CHIP_REVISION)/bt_firmware_controller.c
else
$(NAME)_SOURCES := $(BT_CHIP)$(BT_CHIP_REVISION)/$(BT_CHIP_XTAL_FREQUENCY)/bt_firmware_controller.c
endif
endif

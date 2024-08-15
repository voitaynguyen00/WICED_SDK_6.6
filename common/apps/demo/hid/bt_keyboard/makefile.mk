#
# Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of Cypress Semiconductor 
#  Corporation. All rights reserved. This software, including source code, documentation and  related 
# materials ("Software"), is owned by Cypress Semiconductor  Corporation or one of its 
#  subsidiaries ("Cypress") and is protected by and subject to worldwide patent protection  
# (United States and foreign), United States copyright laws and international treaty provisions. 
# Therefore, you may use this Software only as provided in the license agreement accompanying the 
# software package from which you obtained this Software ("EULA"). If no EULA applies, Cypress 
# hereby grants you a personal, nonexclusive, non-transferable license to  copy, modify, and 
# compile the Software source code solely for use in connection with Cypress's  integrated circuit 
# products. Any reproduction, modification, translation, compilation,  or representation of this 
# Software except as specified above is prohibited without the express written permission of 
# Cypress. Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO  WARRANTY OF ANY KIND, EXPRESS 
# OR IMPLIED, INCLUDING,  BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY 
# AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the right to make changes to 
# the Software without notice. Cypress does not assume any liability arising out of the application 
# or use of the Software or any product or circuit  described in the Software. Cypress does 
# not authorize its products for use in any products where a malfunction or failure of the 
# Cypress product may reasonably be expected to result  in significant property damage, injury 
# or death ("High Risk Product"). By including Cypress's product in a High Risk Product, the 
#  manufacturer of such system or application assumes  all risk of such use and in doing so agrees 
# to indemnify Cypress against all liability.
#

########################################################################
# Add Application sources here.
########################################################################

APP_SRC = appinit_keyboard.c
APP_SRC += wiced_bt_cfg.c 
APP_SRC += keyboard.c 
APP_SRC += bt_keyboard_pin_config.c

########################################################################
# Component(s) needed
########################################################################

APP_PATCHES_AND_LIBS += wiced_hidd_lib.a

#include HIDD library w/ BT HIDD only flag
APPLIBDIR := ../../common/libraries/hidd_lib
include $(APPLIBDIR)/makefile.mk

APP_SRC += $(LIB_SRC)
INCS += $(APPLIBDIR)

C_FLAGS += -DBT_HIDD_ONLY

########################################################################
# C flags
########################################################################

C_FLAGS += -DWICED_BT_TRACE_ENABLE

# set local IO capabilities to BTM_IO_CAPABILITIES_KEYBOARD_ONLY (default is BTM_IO_CAPABILITIES_NONE)
#C_FLAGS += -DUSE_KEYBOARD_IO_CAPABILITIES

ifeq ($(TESTING_USING_HCI), 1)
#!!!only enabled for testing via HCI UART using ClientControl tool.
#It is only for the case that the WICED BOARD you get doesn't have any GPIO pins 
#exposed, you can't use any fly wire to connect to GPIOs to simulate key press 
#and key release to test out the application. ClientControl tool can help you
#simulate key press and release by sending key report via HCI to the WICED BOARD
C_FLAGS += -DTESTING_USING_HCI
endif



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

ENABLE_SCROLL = y
ENABLE_MOTION = y

APP_SRC = appinit_ble_mouse.c wiced_bt_cfg.c ble_mouse.c 

#enable application traces for debugging
C_FLAGS += -DWICED_BT_TRACE_ENABLE

APP_PATCHES_AND_LIBS += wiced_hidd_lib.a

#include HIDD library w/ LE HIDD only flag
APPLIBDIR := ../../common/libraries/hidd_lib
include $(APPLIBDIR)/makefile.mk

APP_SRC += $(LIB_SRC)
INCS += $(APPLIBDIR)

C_FLAGS += -DLE_HIDD_ONLY

#enable OTA Firmware Update
ifeq ($(OTA_SEC_FW_UPGRADE), 1)
APP_SRC += secure/ecdsa256_pub.c
C_FLAGS += -DOTA_SECURE_FIRMWARE_UPGRADE
C_FLAGS += -DOTA_FIRMWARE_UPGRADE
C_FLAGS += -DOTA_SKIP_CONN_PARAM_UPDATE
C_FLAGS += -DSFLASH_SIZE_2M_BITS
$(NAME)_COMPONENTS := fw_upgrade_lib.a
endif

ifeq ($(OTA_FW_UPGRADE), 1)
C_FLAGS += -DOTA_FIRMWARE_UPGRADE
C_FLAGS += -DOTA_SKIP_CONN_PARAM_UPDATE
C_FLAGS += -DSFLASH_SIZE_2M_BITS
$(NAME)_COMPONENTS := fw_upgrade_lib.a
endif

#enalbe assymmetric slave latency 
#C_FLAGS += -DASSYM_SLAVE_LATENCY

ifeq ($(ENABLE_SCROLL), y)
 #enable scroll wheel
 C_FLAGS += -DSUPPORT_SCROLL
endif

ifeq ($(ENABLE_MOTION), y)
 C_FLAGS += -DSUPPORT_MOTION
 INCS += $(DIR)/motion
 APP_SRC += motion/PAW3805_opticalsensor.c motion/interrupt.c
endif

ifeq ($(TESTING_USING_HCI), 1)
#!!!only enabled for testing via HCI UART using ClientControl tool.
#It is only for the case that the WICED BOARD you get doesn't have any GPIO pins 
#exposed, you can't use any fly wire to connect to GPIOs to simulate button press 
#and button release to test out the application. ClientControl tool can help you
#simulate button press and release by sending key report via HCI to the WICED BOARD
C_FLAGS += -DTESTING_USING_HCI
endif
########################################################################
################ DO NOT MODIFY FILE BELOW THIS LINE ####################
########################################################################




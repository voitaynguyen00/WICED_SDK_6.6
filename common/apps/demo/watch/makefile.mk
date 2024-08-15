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
# Application source files
########################################################################

APP_SRC  = wiced_app_cfg.c
APP_SRC += wiced_app.c
APP_SRC += watch.c
APP_SRC += hci_control_le.c
APP_SRC += hci_control_audio.c
APP_SRC += hci_control_rc_target.c
APP_SRC += hci_control_test.c
APP_SRC += hci_control_misc.c
APP_SRC += hci_control_rc_controller.c
APP_SRC += le_slave.c
APP_SRC += GeneratedSource/cycfg_gatt_db.c
APP_SRC += ams_client.c
APP_SRC += ancs_client.c
ifneq (,$(filter 20719,$(CHIP)))
APP_SRC += watch_pin_config.c
endif

########################################################################
# Component(s) needed
########################################################################
$(NAME)_COMPONENTS := avrc_controller.a
$(NAME)_COMPONENTS += avrc_target.a

ifneq (,$(filter 20719 20721,$(CHIP)))
APP_PATCHES_AND_LIBS += wiced_audio_source_lib.a
ifneq (,$(findstring SPI,$(TRANSPORT)))
APP_PATCHES_AND_LIBS += wiced_transport_spi_lib.a
endif
endif

ifeq ($(CHIP), 20703)
APP_PATCHES_AND_LIBS += wiced_audio_source.a
ifneq (,$(findstring SPI,$(TRANSPORT)))
APP_PATCHES_AND_LIBS += wiced_hal_lib.a
endif
endif

ifeq ($(CHIP), 20819)
#To save memory limit no. of BLE connection to 2.
APP_PATCHES_AND_LIBS += wiced_ble_pre_init_lib.a
endif

########################################################################
# C flags
# To use SPI transport, append TRANSPORT=SPI to the make target, 
# for example "demo.watch-CYW920706WCDEVAL TRANSPORT=SPI download"
########################################################################
ifeq ($(CHIP), 43012)
C_FLAGS += -DCOEX_SUPPORTED=0
C_FLAGS += -DSLEEP_SUPPORTED=0
else
C_FLAGS += -DSLEEP_SUPPORTED=1
C_FLAGS += -DCOEX_SUPPORTED=1
endif
C_FLAGS += -DWICED_BT_TRACE_ENABLE

C_FLAGS += -DCATEGORY_2_PASSTROUGH

C_FLAGS += -DWICED_HCI_TRANSPORT_UART=1
C_FLAGS += -DWICED_HCI_TRANSPORT_SPI=2

ifeq ($(CHIP), 20819)
C_FLAGS += -DWICED_MAX_LE_CLIENT_CONN=1
else
C_FLAGS += -DWICED_MAX_LE_CLIENT_CONN=3
endif

ifneq (,$(findstring SPI,$(TRANSPORT)))
$(info Transport=SPI)
C_FLAGS += -DWICED_HCI_TRANSPORT=2
else
$(info Transport=UART)
C_FLAGS += -DWICED_HCI_TRANSPORT=1
endif


########################################################################
################ DO NOT MODIFY FILE BELOW THIS LINE ####################
########################################################################

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

NAME := hci_hci_host

########################################################################
# Add Application sources here.
########################################################################
APP_SRC  = wiced_app_cfg.c
APP_SRC += hci_control.c
APP_SRC += hci_control_hidh.c

########################################################################
# Component(s) needed
########################################################################
$(NAME)_COMPONENTS := hidh_lib.a

########################################################################
# C flags
# To use SPI transport, append _SPI to the make target,
# for example "demo.hci_hid_host-BCM920706_P49_SPI download"
########################################################################

C_FLAGS += -DWICED_BT_TRACE_ENABLE
#C_FLAGS += -DPTS_TEST_ONLY
C_FLAGS += -DCATEGORY_2_PASSTROUGH
C_FLAGS += -DWICED_SMART_SNIFF_MODE_ENABLE

C_FLAGS += -DWICED_HCI_TRANSPORT_UART=1
C_FLAGS += -DWICED_HCI_TRANSPORT_SPI=2

ifneq (,$(findstring _SPI,$(PLATFORM_FULL)))
$(info Transport=SPI)
C_FLAGS += -DWICED_HCI_TRANSPORT=2
else
$(info Transport=UART)
C_FLAGS += -DWICED_HCI_TRANSPORT=1
endif


########################################################################
################ DO NOT MODIFY FILE BELOW THIS LINE ####################
########################################################################
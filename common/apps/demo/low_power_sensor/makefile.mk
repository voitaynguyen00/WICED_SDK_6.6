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
APP_SRC =  low_power_sensor.c
APP_SRC += low_power_sensor_pin_config.c
APP_SRC += wiced_bt_cfg.c

C_FLAGS += -DWICED_BT_TRACE_ENABLE

########################################################################
# SLEEP MODE configuration
########################################################################
C_FLAGS += -DSLEEP_MODE_NO_TRANSPORT=0
C_FLAGS += -DSLEEP_MODE_TRANSPORT=1

ifeq ($(SLEEP_MODE), SLEEP_MODE_NO_TRANSPORT)
C_FLAGS += -DWICED_SLEEP_MODE=0
else
ifeq ($(SLEEP_MODE), SLEEP_MODE_TRANSPORT)
C_FLAGS += -DWICED_SLEEP_MODE=1
endif
endif

########################################################################
# SLEEP type (SDS/PDS) configuration
########################################################################
C_FLAGS += -DSLEEP_TYPE_SHUTDOWN=1
C_FLAGS += -DSLEEP_TYPE_NOT_SHUTDOWN=2

ifeq ($(SLEEP_TYPE), SLEEP_TYPE_SHUTDOWN)
C_FLAGS += -DWICED_SLEEP_TYPE=1
else
ifeq ($(SLEEP_TYPE), SLEEP_TYPE_NOT_SHUTDOWN)
C_FLAGS += -DWICED_SLEEP_TYPE=2
endif
endif

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

NAME := mesh_level_server

# Mesh debug: enable (or pass via make target) to use debug/trace mesh libraries
# NOTE:  20706 limited to only 1 of mesh_models or mesh_core debug libraries
#        - do not enable both simultaneously for 20706
# MESH_MODELS_DEBUG := 1
# MESH_CORE_DEBUG := 1

########################################################################
# Add Application sources here.
########################################################################
APP_SRC = mesh_level_server.c

########################################################################
# Component(s) needed
########################################################################
$(NAME)_COMPONENTS := mesh_app_lib.a
$(NAME)_COMPONENTS += fw_upgrade_lib.a

ifeq ($(MESH_MODELS_DEBUG),1)
$(NAME)_COMPONENTS += mesh_models_lib-d.a
else
$(NAME)_COMPONENTS += mesh_models_lib.a
endif
ifeq ($(MESH_CORE_DEBUG),1)
$(NAME)_COMPONENTS += mesh_core_lib-d.a
else
$(NAME)_COMPONENTS += mesh_core_lib.a
endif

ifeq ($(BLD),A_20719B0)
APP_PATCHES_AND_LIBS := multi_adv_patch_lib.a
else ifeq ($(CHIP),20703)
APP_PATCHES_AND_LIBS := rtc_lib.a
APP_PATCHES_AND_LIBS += wiced_bt_mesh.a
endif

########################################################################
# C flags
########################################################################
# define this for 2 chip solution or for testing over WICED HCI
C_FLAGS += -DHCI_CONTROL

# Warning! Low Power Node will not be accessable after provisioning if network has no Friend nodes.
ifdef LOW_POWER_NODE
C_FLAGS += -DLOW_POWER_NODE=$(LOW_POWER_NODE)
else
C_FLAGS += -DLOW_POWER_NODE=0
endif

C_FLAGS += -DWICED_BT_TRACE_ENABLE

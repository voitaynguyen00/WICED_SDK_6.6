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

TOOLCHAIN = GCC

PLATFORMS_VERSION = 1.0

CONFIG = Debug

CY_EXAMPLE_NAME = BLE_Mesh_ProvisionClient

CY_EXAMPLE_DESCRIPTION = This application demonstrates how to create a BLE Mesh device that implements a Mesh Provisioner client.

CY_SHOW_NEW_PROJECT := true

CY_VALID_PLATFORMS =  \
  CYW920819EVB-02 \
  CYW920820EVB-02 \
  CYBT-213043-MESH \
  CYBT-213043-EVAL

PLATFORM = CYBT-213043-MESH

# to link to mesh libraries with tracing enabled, use the following settings:
# Note: 20706 can only have 1 enabled, more than one will exceed memory limits
MESH_MODELS_DEBUG_TRACES_DEFAULT=off
MESH_CORE_DEBUG_TRACES_DEFAULT=off
MESH_PROVISIONER_DEBUG_TRACES_DEFAULT=off

APP_FEATURES += \
	MESH_MODELS_DEBUG_TRACES,app,onoff,$(MESH_MODELS_DEBUG_TRACES_DEFAULT) \
	MESH_CORE_DEBUG_TRACES,app,onoff,$(MESH_CORE_DEBUG_TRACES_DEFAULT) \
	MESH_PROVISIONER_DEBUG_TRACES,app,onoff,$(MESH_PROVISIONER_DEBUG_TRACES_DEFAULT)

MESH_MODELS_DEBUG_TRACES ?= $(MESH_MODELS_DEBUG_TRACES_DEFAULT)
MESH_CORE_DEBUG_TRACES ?= $(MESH_CORE_DEBUG_TRACES_DEFAULT)
MESH_PROVISIONER_DEBUG_TRACES ?= $(MESH_PROVISIONER_DEBUG_TRACES_DEFAULT)

CY_MAINAPP_SWCOMP_RULES +=  \
  $(CY_WICED_LIB_COMP_BASE)/BT-SDK/common/libraries/mesh_app_lib \
  $(CY_WICED_LIB_COMP_BASE)/BT-SDK/common/libraries/mesh_core_lib \
  $(CY_WICED_LIB_COMP_BASE)/BT-SDK/common/libraries/mesh_models_lib \
  $(CY_WICED_LIB_COMP_BASE)/BT-SDK/common/libraries/mesh_provisioner_lib

# OTAFU enabled by default for all mesh apps
OTA_FW_UPGRADE = 1

# NOTE: This variable cannot be renamed or moved to a different file. It is updated by the ModusToolbox
# middleware editor.
CY_MAINAPP_SWCOMP_USED =  \
  $(CY_WICED_LIB_COMP_BASE)/BT-SDK/common/libraries/gatt_utils_lib \
  $(CY_WICED_LIB_COMP_BASE)/BT-SDK/common/libraries/fw_upgrade_lib

# NOTE: This variable cannot be renamed or moved to a different file. It is updated by the ModusToolbox
# middleware editor.
CY_MAINAPP_SWCOMP_EXT =

CY_APP_DEFINES =  \
  -DHCI_CONTROL \
  -DREMOTE_PROVISIONG_SERVER_SUPPORTED \
  -DTRANSPORT_BUFFER_SIZE=400 

# value of the LOW_POWER_NODE defines mode. It can be normal node (0), or low power node (1)
APP_FEATURES += LOW_POWER_NODE,app,enum,0,0,1
LOW_POWER_NODE ?= 0
CY_APP_DEFINES += -DLOW_POWER_NODE=$(LOW_POWER_NODE)

# If PTS is defined then device gets hardcoded BD address from make target
# Otherwise it is random
PTS ?= 0
ifeq ($(PTS),1)
CY_APP_DEFINES += -DPTS
endif # PTS

CY_APP_SOURCE =  \
  ./mesh_provision_client.c \
  ./mesh_onoff_client.c \
  ./mesh_level_client.c \
  ./mesh_light_lightness_client.c \
  ./mesh_light_hsl_client.c \
  ./mesh_light_ctl_client.c \
  ./mesh_default_transition_time_client.c \
  ./mesh_sensor_client.c \
  ./mesh_scene_client.c \
  ./mesh_vendor_server.c \
  ./mesh_vendor_server.h

# do not link from normal WICED/libraries/ - use the newly built .a from sources
MESH_INTERNAL_BUILD=1

ifeq ($(MESH_CORE_DEBUG),1)
CY_APP_SOURCE += $(CY_BUILD_LOCATION)/Debug/mainapp/prebuilt_libs-1.0/components/BT-SDK/common/libraries/mesh_core_lib/mesh_core_lib.debug.a
else
CY_APP_SOURCE += $(CY_BUILD_LOCATION)/Debug/mainapp/prebuilt_libs-1.0/components/BT-SDK/common/libraries/mesh_core_lib/mesh_core_lib.a
endif
ifeq ($(MESH_MODELS_DEBUG),1)
CY_APP_SOURCE += $(CY_BUILD_LOCATION)/Debug/mainapp/prebuilt_libs-1.0/components/BT-SDK/common/libraries/mesh_models_lib/mesh_models_lib.debug.a
else
CY_APP_SOURCE += $(CY_BUILD_LOCATION)/Debug/mainapp/prebuilt_libs-1.0/components/BT-SDK/common/libraries/mesh_models_lib/mesh_models_lib.a
endif
ifeq ($(MESH_PROVISIONER_DEBUG),1)
CY_APP_SOURCE += $(CY_BUILD_LOCATION)/Debug/mainapp/prebuilt_libs-1.0/components/BT-SDK/common/libraries/mesh_provisioner_lib/mesh_provisioner_lib.debug.a
else
CY_APP_SOURCE += $(CY_BUILD_LOCATION)/Debug/mainapp/prebuilt_libs-1.0/components/BT-SDK/common/libraries/mesh_provisioner_lib/mesh_provisioner_lib.a
endif

CY_APP_RESOURCES =

# declare local directory for include path
CY_APP_PATH := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
# handle cygwin drive/path mangling for GNU tools - replace /cygdrive/c/ with c:/
ifneq ($(findstring cygdrive,$(CY_APP_PATH)),)
CY_APP_PATH := $(subst /, ,$(patsubst /cygdrive/%,%,$(CY_APP_PATH)))
CY_APP_PATH := $(subst $(empty) $(empty),/,$(patsubst %,%:,$(firstword $(CY_APP_PATH))) $(wordlist 2,$(words $(CY_APP_PATH)),$(CY_APP_PATH)))
endif

ifndef CYSDK
$(error The SDK must be defined via the CYSDK environment variable)
endif
include $(CYSDK)/libraries/platforms-$(PLATFORMS_VERSION)/common/find_platform.mk

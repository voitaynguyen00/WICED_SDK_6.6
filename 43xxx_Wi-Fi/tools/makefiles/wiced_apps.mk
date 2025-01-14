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

SECTOR_COUNT_SCRIPT  := $(TOOLS_ROOT)/text_to_c/sector_count.pl
SECTOR_ADDRESS_SCRIPT  := $(TOOLS_ROOT)/text_to_c/sector_address.pl
SECTOR_NUMBER_SCRIPT  := $(TOOLS_ROOT)/text_to_c/sector_number.pl
CREATE_FLASH_IMAGE_TOOL := $(TOOLS_ROOT)/text_to_c/flash_image_create.pl

APPS_SECTORS_DEFAULT_COUNT	:= 1
CURRENT_SECTOR := $(APPS_START_SECTOR)

APPS_HEADER_DEFINES :=
CURRENT_DEPENDENCY :=
CURRENT_PACKAGE_DEPENDENCY :=
ifneq ($(BOARD_REVISION),)
SFLASH_APP_TARGET := waf.sflash_write-NoOS-$(PLATFORM)-$(BOARD_REVISION)-$(BUS)
else
SFLASH_APP_TARGET := waf.sflash_write-NoOS-$(PLATFORM)-$(BUS)
endif

OPENOCD_LOG_FILE ?= build/openocd_log.txt
DOWNLOAD_LOG := >> $(OPENOCD_LOG_FILE)

###############################################################################
# MACRO: BUILD_APPS_RULES
# Creates targets to build a resource file
# the first target converts the text resource file to a C file
# the second target compiles the C resource file into an object file
# $(1) is the name of a resource
# $(2) should be MEM or FILESYSTEM - indication location of resource
define BUILD_APPS_RULES
$(if $($(1)),$(eval $(1)_ENTRY_COUNT := 1),$(eval $(1)_ENTRY_COUNT := 0))
$(if $($(1)),$(eval $(1)_SECTOR_START := $(CURRENT_SECTOR)),$(eval $(1)_SECTOR_START := 0))
$(if $($(1)),$(eval $(1)_SECTOR_COUNT := $(shell $(PERL) $(SECTOR_COUNT_SCRIPT) $($(1)) 0 4096)),$(eval $(1)_SECTOR_COUNT := 0))
$(if $($(1)),$(eval $(1)_SECTOR_ADDRESS := $(shell $(PERL) $(SECTOR_ADDRESS_SCRIPT) $($(1)_SECTOR_START) 4096)),)
$(if $($(1)),$(eval CURRENT_SECTOR := $(shell $(PERL) $(SECTOR_COUNT_SCRIPT) $($(1)) $(CURRENT_SECTOR) 4096)),)
$(if $($(1)),$(eval $(1)_IS_SECURE := $(if $($(1)_SECURE), $($(1)_SECURE), 0)),	$(eval $(1)_IS_SECURE := 0))
$(eval APPS_HEADER_DEFINES += -D$(1)_ENTRY_COUNT=$($(1)_ENTRY_COUNT))
$(eval APPS_HEADER_DEFINES += -D$(1)_SECTOR_START=$($(1)_SECTOR_START))
$(eval APPS_HEADER_DEFINES += -D$(1)_SECTOR_COUNT=$($(1)_SECTOR_COUNT))
$(eval APPS_HEADER_DEFINES += -D$(1)_IS_SECURE=$($(1)_IS_SECURE))
endef
### end of BUILD_APPS_RULES

###############################################################################
# MACRO: BUILD_APP_PACKAGE_DEPENDENCY
define BUILD_APP_PACKAGE_DEPENDENCY
$(if $($(1)),$(eval $(1)_PACKAGE_DEPENDENCY := $($(1)) $(CURRENT_PACKAGE_DEPENDENCY) sflash_write_app display_map_summary $(APPS_LUT_PACKAGE_DEP) APPS_LOOKUP_TABLE_RULES  $(LINK_APPS_FILE)),)
$(if $($(1)),$(eval CURRENT_PACKAGE_DEPENDENCY += $(1)_PACKAGE),)
endef
#### end of BUILD_APP_PACKAGE_DEPENDENCY

###############################################################################
# MACRO: BUILD_APP_DOWNLOAD_DEPENDENCY
define BUILD_APP_DOWNLOAD_DEPENDENCY
$(if $($(1)),$(eval $(1)_DOWNLOAD_DEPENDENCY := $($(1)) $(CURRENT_DEPENDENCY) sflash_write_app display_map_summary $(APPS_LUT_DOWNLOAD_DEP) APPS_LOOKUP_TABLE_RULES  $(LINK_APPS_FILE)),)
$(if $($(1)),$(eval CURRENT_DEPENDENCY += $(1)_DOWNLOAD),)
endef
#### end of BUILD_APP_DOWNLOAD_DEPENDENCY

APPS_DOWNLOADS_DEPENDENCY :=  $(GENERATED_MAC_FILE)
APPS_PACKAGE_DEPENDENCY := $(GENERATED_MAC_FILE)
APPS_HEADER_DEPENDENCY := $(GENERATED_MAC_FILE)
APPS := FR_APP DCT_IMAGE OTA_APP FILESYSTEM_IMAGE WIFI_FIRMWARE APP0 APP1 APP2
$(foreach APP,$(APPS),$(eval $(if $($(APP)), APPS_HEADER_DEPENDENCY += $($(APP)))))
$(foreach APP,$(APPS),$(eval $(if $($(APP)), APPS_DOWNLOADS_DEPENDENCY += $(APP)_DOWNLOAD)))
$(foreach APP,$(APPS),$(eval $(call BUILD_APP_DOWNLOAD_DEPENDENCY,$(APP))))
$(foreach APP,$(APPS),$(eval $(if $($(APP)), APPS_PACKAGE_DEPENDENCY += $(APP)_PACKAGE)))
$(foreach APP,$(APPS),$(eval $(call BUILD_APP_PACKAGE_DEPENDENCY,$(APP))))

LINK_APPS_FILE            :=$(OUTPUT_DIR)/APPS$(LINK_OUTPUT_SUFFIX)
STRIPPED_LINK_APPS_FILE   :=$(LINK_APPS_FILE:$(LINK_OUTPUT_SUFFIX)=.stripped$(LINK_OUTPUT_SUFFIX))
FINAL_APPS_FILE           :=$(LINK_APPS_FILE:$(LINK_OUTPUT_SUFFIX)=$(FINAL_OUTPUT_SUFFIX))
MAP_APPS_FILE             :=$(LINK_APPS_FILE:$(LINK_OUTPUT_SUFFIX)=.map)

# include the correct FLASH positioning
#$(info wiced_apps.mk ota2_support=$(OTA2_SUPPORT))
ifeq (1,$(OTA2_SUPPORT))
# OTA Image Support
include platforms/$(subst .,/,$(PLATFORM))/ota2_image_defines.mk
# If Defines from the platform make file is also needed, add
# OTA2_SUPPORT_PLATFORM_MK_NEEDED :=1 to the platforms/$(PLATFORM)/ota2_image_defined.mk file
ifeq (1,$(OTA2_SUPPORT_PLATFORM_MK_NEEDED))
include platforms/$(subst .,/,$(PLATFORM))/$(subst .,/,$(PLATFORM)).mk
endif

OTA2_IMAGE_FILE                       :=$(OUTPUT_DIR)/OTA2_image_file
OTA2_IMAGE_CONFIG_FILE                :=$(OTA2_IMAGE_FILE).cfg
OTA2_IMAGE_BIN_FILE                   :=$(OTA2_IMAGE_FILE).bin
OTA2_IMAGE_FACTORY_RESET_FILE         :=$(OUTPUT_DIR)/OTA2_factory_reset_file
OTA2_IMAGE_FACTORY_RESET_CONFIG_FILE  :=$(OTA2_IMAGE_FACTORY_RESET_FILE).cfg
OTA2_IMAGE_FACTORY_RESET_BIN_FILE     :=$(OTA2_IMAGE_FACTORY_RESET_FILE).bin
OTA2_IMAGE_MANUFACTURING_FILE         :=$(OUTPUT_DIR)/OTA2_manuf_image_file
OTA2_IMAGE_MANUFACTURING_CONFIG_FILE  :=$(OTA2_IMAGE_MANUFACTURING_FILE).cfg
OTA2_IMAGE_MANUFACTURING_BIN_FILE     :=$(OTA2_IMAGE_MANUFACTURING_FILE).bin

endif


###############################################################################
# MACRO: BUILD_OTA2_APPS_RULES
#
# $(FILE)_ENTRY_COUNT :=0		when not used
# $(FILE)_ENTRY_COUNT :=1		when used - this is the # of "sector/count" entries in the LUT
#         (see app_header_t in WICED/platform/MCU/wiced_apps_common.h and WICED/platform/MCU/wiced_apps_lut.c)
#
define BUILD_OTA2_APPS_RULES
	$(eval APPS_HEADER_DEFINES := )

	$(eval OTA2_FAILSAFE_APP_ADDRESS    := $(OTA2_IMAGE_FAILSAFE_APP_AREA_BASE) )
	$(eval OTA2_FAILSAFE_SECTOR_START   := $(shell $(PERL) $(SECTOR_NUMBER_SCRIPT) $(OTA2_FAILSAFE_APP_ADDRESS) 4096) )
	$(eval APPS_HEADER_DEFINES += -DOTA2_FAILSAFE_APP_ADDRESS=$(OTA2_FAILSAFE_APP_ADDRESS) )

	# we currently don't use a Factory Reset App (that is for the old-style OTA)
	# Set $(NAME)_ENTRY_COUNT to 1 and fill in SECTOR info (see below) if you add FR_APP
	$(eval FR_APP_ENTRY_COUNT       := 0 )
	$(eval FR_APP_IS_SECURE	 	    := 0 )
	$(eval FR_APP_SECTOR_ADDRESS    := 0 )
	$(eval FR_APP_SECTOR_START      := 0 )
	$(eval FR_APP_SECTOR_COUNT      := 0 )

	$(eval APPS_HEADER_DEFINES += -DFR_APP_ENTRY_COUNT=$(FR_APP_ENTRY_COUNT) )
	$(eval APPS_HEADER_DEFINES += -DFR_APP_IS_SECURE=$(FR_APP_IS_SECURE) )
	$(eval APPS_HEADER_DEFINES += -DFR_APP_SECTOR_ADDRESS=$(FR_APP_SECTOR_ADDRESS) )
	$(eval APPS_HEADER_DEFINES += -DFR_APP_SECTOR_START=$(FR_APP_SECTOR_START) )
	$(eval APPS_HEADER_DEFINES += -DFR_APP_SECTOR_COUNT=$(FR_APP_SECTOR_COUNT) )

	$(eval DCT_IMAGE_ENTRY_COUNT    := 1 )
	$(eval DCT_IMAGE_IS_SECURE	 	:= $(if $(DCT_IMAGE_IS_SECURE), $(DCT_IMAGE_IS_SECURE), 0) )
	$(eval DCT_IMAGE_SECTOR_ADDRESS := $(OTA2_IMAGE_CURR_DCT_1_AREA_BASE) )
	$(eval DCT_IMAGE_SECTOR_START   := $(shell $(PERL) $(SECTOR_NUMBER_SCRIPT) $(DCT_IMAGE_SECTOR_ADDRESS) 4096) )
	$(eval DCT_IMAGE_SECTOR_COUNT   := $(shell $(PERL) $(SECTOR_COUNT_SCRIPT) $(subst \,/,$(FINAL_DCT_FILE)) 0 4096) )

	$(eval APPS_HEADER_DEFINES += -DDCT_IMAGE_ENTRY_COUNT=$(DCT_IMAGE_ENTRY_COUNT) )
	$(eval APPS_HEADER_DEFINES += -DDCT_IMAGE_IS_SECURE=$(DCT_IMAGE_IS_SECURE) )
	$(eval APPS_HEADER_DEFINES += -DDCT_IMAGE_SECTOR_ADDRESS=$(DCT_IMAGE_SECTOR_ADDRESS) )
	$(eval APPS_HEADER_DEFINES += -DDCT_IMAGE_SECTOR_START=$(DCT_IMAGE_SECTOR_START) )
	$(eval APPS_HEADER_DEFINES += -DDCT_IMAGE_SECTOR_COUNT=$(DCT_IMAGE_SECTOR_COUNT) )

	$(eval OTA_APP_ENTRY_COUNT      := 1 )
	$(eval OTA_APP_IS_SECURE		:= $(if $(OTA_APP_SECURE), $(OTA_APP_SECURE), 0) )
	$(eval OTA_APP_SECTOR_ADDRESS   := $(OTA2_IMAGE_CURR_OTA_APP_AREA_BASE) )
	$(eval OTA_APP_SECTOR_START     := $(shell $(PERL) $(SECTOR_NUMBER_SCRIPT) $(OTA_APP_SECTOR_ADDRESS) 4096) )
	$(eval OTA_APP_SECTOR_COUNT     := $(shell $(PERL) $(SECTOR_COUNT_SCRIPT) $(subst \,/,$(OTA_APP)) 0 4096) )

	$(eval APPS_HEADER_DEFINES += -DOTA_APP_ENTRY_COUNT=$(OTA_APP_ENTRY_COUNT) )
	$(eval APPS_HEADER_DEFINES += -DOTA_APP_IS_SECURE=$(OTA_APP_IS_SECURE) )
	$(eval APPS_HEADER_DEFINES += -DOTA_APP_SECTOR_ADDRESS=$(OTA_APP_SECTOR_ADDRESS) )
	$(eval APPS_HEADER_DEFINES += -DOTA_APP_SECTOR_START=$(OTA_APP_SECTOR_START) )
	$(eval APPS_HEADER_DEFINES += -DOTA_APP_SECTOR_COUNT=$(OTA_APP_SECTOR_COUNT) )

	$(eval FILESYSTEM_IMAGE_ENTRY_COUNT    := 1 )
	$(eval FILESYSTEM_IMAGE_IS_SECURE	 	:= $(if $(FILESYSTEM_IMAGE_SECURE), $(FILESYSTEM_IMAGE_SECURE), 0) )
	$(eval FILESYSTEM_IMAGE_SECTOR_ADDRESS := $(OTA2_IMAGE_CURR_FS_AREA_BASE) )
	$(eval FILESYSTEM_IMAGE_SECTOR_START   := $(shell $(PERL) $(SECTOR_NUMBER_SCRIPT) $(FILESYSTEM_IMAGE_SECTOR_ADDRESS) 4096) )
	$(eval FILESYSTEM_IMAGE_SECTOR_COUNT   := $(shell $(PERL) $(SECTOR_COUNT_SCRIPT) $(subst \,/,$(FILESYSTEM_IMAGE)) 0 4096) )

	$(eval APPS_HEADER_DEFINES += -DFILESYSTEM_IMAGE_ENTRY_COUNT=$(FILESYSTEM_IMAGE_ENTRY_COUNT) )
	$(eval APPS_HEADER_DEFINES += -DFILESYSTEM_IMAGE_IS_SECURE=$(FILESYSTEM_IMAGE_IS_SECURE) )
	$(eval APPS_HEADER_DEFINES += -DFILESYSTEM_IMAGE_SECTOR_ADDRESS=$(FILESYSTEM_IMAGE_SECTOR_ADDRESS) )
	$(eval APPS_HEADER_DEFINES += -DFILESYSTEM_IMAGE_SECTOR_START=$(FILESYSTEM_IMAGE_SECTOR_START) )
	$(eval APPS_HEADER_DEFINES += -DFILESYSTEM_IMAGE_SECTOR_COUNT=$(FILESYSTEM_IMAGE_SECTOR_COUNT) )

	# Set $(NAME)_ENTRY_COUNT to 1 and fill in SECTOR info (see above) if you add separate FIRMWARE (not in filesystem)
	$(eval WIFI_FIRMWARE_ENTRY_COUNT     := 0 )
	$(eval WIFI_FIRMWARE_IS_SECURE       := 0 )
	$(eval WIFI_FIRMWARE_SECTOR_ADDRESS  := 0 )
	$(eval WIFI_FIRMWARE_SECTOR_START    := 0 )
	$(eval WIFI_FIRMWARE_SECTOR_COUNT    := 0 )

	$(eval APPS_HEADER_DEFINES += -DWIFI_FIRMWARE_ENTRY_COUNT=$(WIFI_FIRMWARE_ENTRY_COUNT) )
	$(eval APPS_HEADER_DEFINES += -DWIFI_FIRMWARE_IS_SECURE=$(WIFI_FIRMWARE_IS_SECURE) )
	$(eval APPS_HEADER_DEFINES += -DWIFI_FIRMWARE_SECTOR_ADDRESS=$(WIFI_FIRMWARE_SECTOR_ADDRESS) )
	$(eval APPS_HEADER_DEFINES += -DWIFI_FIRMWARE_SECTOR_START=$(WIFI_FIRMWARE_SECTOR_START) )
	$(eval APPS_HEADER_DEFINES += -DWIFI_FIRMWARE_SECTOR_COUNT=$(WIFI_FIRMWARE_SECTOR_COUNT) )

	$(eval APP0_ENTRY_COUNT	    := 1 )
	$(eval APP0_IS_SECURE	 	:= $(if $(APP0_SECURE), $(APP0_SECURE), 0) )
	$(eval APP0_SECTOR_ADDRESS  := $(OTA2_IMAGE_CURR_APP0_AREA_BASE) )
	$(eval APP0_SECTOR_START	:= $(shell $(PERL) $(SECTOR_NUMBER_SCRIPT) $(APP0_SECTOR_ADDRESS) 4096) )
	$(eval APP0_SECTOR_COUNT	:= $(shell $(PERL) $(SECTOR_COUNT_SCRIPT) $(subst \,/,$(APP0)) 0 4096) )

	$(eval APPS_HEADER_DEFINES += -DAPP0_ENTRY_COUNT=$(APP0_ENTRY_COUNT) )
	$(eval APPS_HEADER_DEFINES += -DAPP0_IS_SECURE=$(APP0_IS_SECURE) )
	$(eval APPS_HEADER_DEFINES += -DAPP0_SECTOR_ADDRESS=$(APP0_SECTOR_ADDRESS) )
	$(eval APPS_HEADER_DEFINES += -DAPP0_SECTOR_START=$(APP0_SECTOR_START) )
	$(eval APPS_HEADER_DEFINES += -DAPP0_SECTOR_COUNT=$(APP0_SECTOR_COUNT) )

	# Set $(NAME)_ENTRY_COUNT to 1 and fill in SECTOR info (see above) if you add APP1
	$(eval APP1_ENTRY_COUNT	    := 0 )
	$(eval APP1_IS_SECURE	   	:= 0 )
	$(eval APP1_SECTOR_ADDRESS  := 0 )
	$(eval APP1_SECTOR_START	:= 0 )
	$(eval APP1_SECTOR_COUNT	:= 0 )

	$(eval APPS_HEADER_DEFINES += -DAPP1_ENTRY_COUNT=$(APP1_ENTRY_COUNT) )
	$(eval APPS_HEADER_DEFINES += -DAPP1_IS_SECURE=$(APP1_IS_SECURE) )
	$(eval APPS_HEADER_DEFINES += -DAPP1_SECTOR_ADDRESS=$(APP1_SECTOR_ADDRESS) )
	$(eval APPS_HEADER_DEFINES += -DAPP1_SECTOR_START=$(APP1_SECTOR_START) )
	$(eval APPS_HEADER_DEFINES += -DAPP1_SECTOR_COUNT=$(APP1_SECTOR_COUNT) )

	# Set $(NAME)_ENTRY_COUNT to 1 and fill in SECTOR info (see above) if you add APP2
	$(eval APP2_ENTRY_COUNT	    := 0 )
	$(eval APP2_IS_SECURE	   	:= 0 )
	$(eval APP2_SECTOR_ADDRESS  := 0 )
	$(eval APP2_SECTOR_START	:= 0 )
	$(eval APP2_SECTOR_COUNT	:= 0 )

	$(eval APPS_HEADER_DEFINES += -DAPP2_ENTRY_COUNT=$(APP2_ENTRY_COUNT) )
	$(eval APPS_HEADER_DEFINES += -DAPP2_IS_SECURE=$(APP2_IS_SECURE) )
	$(eval APPS_HEADER_DEFINES += -DAPP2_SECTOR_ADDRESS=$(APP2_SECTOR_ADDRESS) )
	$(eval APPS_HEADER_DEFINES += -DAPP2_SECTOR_START=$(APP2_SECTOR_START) )
	$(eval APPS_HEADER_DEFINES += -DAPP2_SECTOR_COUNT=$(APP2_SECTOR_COUNT) )

	$(eval APP_LUT_SECTOR_COUNT = $(shell $(PERL) $(SECTOR_COUNT_SCRIPT) $(subst \,/,$(FINAL_APPS_FILE)) 0 4096) )

	$(eval APPS_HEADER_DEFINES += -DAPP_LUT_SECTOR_COUNT=$(APP_LUT_SECTOR_COUNT) )
	$(eval APPS_HEADER_DEFINES += -DOTA2_SUPPORT=1 )

endef	#### End of BUILD_OTA2_APPS_RULES


.PHONY: DOWNLOAD_APPS_HEADER FILESYSTEM_IMAGE_DOWNLOAD APP0_DOWNLOAD APPS_LOOKUP_TABLE_RULES
.PHONY: FILESYSTEM_IMAGE_PACKAGE APP0_PACKAGE

APPS_LOOKUP_TABLE_RULES: display_map_summary $(APPS_HEADER_DEPENDENCY)
ifneq (1,$(OTA2_SUPPORT))
	$(foreach APP,$(APPS),$(eval $(call BUILD_APPS_RULES,$(APP))))
else
	$(eval $(call BUILD_OTA2_APPS_RULES))
endif	# OTA2_SUPPORT

$(LINK_APPS_FILE): display_map_summary $(SOURCE_ROOT)WICED/platform/MCU/wiced_apps_lut.c APPS_LOOKUP_TABLE_RULES
	$(QUIET)$(ECHO) Building apps lookup table
	$(QUIET)$(CC) $(CPU_CFLAGS) $(COMPILER_SPECIFIC_COMP_ONLY_FLAG)  $(SOURCE_ROOT)WICED/platform/MCU/wiced_apps_lut.c $(APPS_HEADER_DEFINES) $(WICED_SDK_DEFINES) $(WICED_SDK_INCLUDES) $(COMPILER_SPECIFIC_DEBUG_CFLAGS)  $(COMPILER_SPECIFIC_STANDARD_CFLAGS) -I$(OUTPUT_DIR) -I$(SOURCE_ROOT). -o $(OUTPUT_DIR)/apps_header.o $(COMPILER_SPECIFIC_STDOUT_REDIRECT)
	$(QUIET)$(LINKER) $(WICED_SDK_LDFLAGS) $(WICED_SDK_DCT_LINK_CMD) $(call COMPILER_SPECIFIC_LINK_MAP,$(MAP_APPS_FILE)) -o $@  $(OUTPUT_DIR)/apps_header.o $(COMPILER_SPECIFIC_STDOUT_REDIRECT)

$(STRIPPED_LINK_APPS_FILE): $(LINK_APPS_FILE)
	$(QUIET)$(STRIP) $(call STRIP_OPTIONS,$<,$@)

$(FINAL_APPS_FILE): $(STRIPPED_LINK_APPS_FILE)
	$(QUIET)$(OBJCOPY) $(call FINAL_OUTPUT_OPTIONS,$<,$@)
#	$(EVAL APP_LUT_SECTOR_COUNT = $(shell $(PERL) $(SECTOR_COUNT_SCRIPT) $(subst \,/,$(FINAL_APPS_FILE)) 0 4096))

# THIS IS A DODGY HACK AS THE DEFINE MACRO IS NOT WORKING!!!
ifneq ($(FR_APP),)
ifneq (1,$(OTA2_SUPPORT))
FR_APP_DOWNLOAD:  $(FR_APP_DOWNLOAD_DEPENDENCY)
	$(QUIET)$(ECHO) Downloading FR_APP $(FR_APP) at sector $(FR_APP_SECTOR_START) address $(FR_APP_SECTOR_ADDRESS)...
ifeq ($(JLINK_NATIVE), 1)
	$(QUIET)$(call JLINK_MAKE_SCRIPT_ALL,$(FR_APP),$(FR_APP_SECTOR_ADDRESS))
	$(QUIET)$(call JLINK_EXE_SCRIPT)
else
	$(call CONV_SLASHES,$(OPENOCD_FULL_NAME)) -f $(OPENOCD_PATH)$(JTAG).cfg -f $(OPENOCD_PATH)$(HOST_OPENOCD).cfg -f apps/waf/sflash_write/sflash_write.tcl -c "sflash_write_file $(FR_APP) $(FR_APP_SECTOR_ADDRESS) $(SFLASH_APP_PLATFROM_BUS) 0 $(SFLASH_APP_BCM4390)" -c shutdown $(DOWNLOAD_LOG) 2>&1
endif
else
FR_APP_DOWNLOAD:
	@:
endif
endif

ifneq ($(OTA_APP),)
OTA_APP_DOWNLOAD:  $(OTA_APP_DOWNLOAD_DEPENDENCY) $(SECURE_OTA_APP)
ifeq (1, $(OTA_APP_SECURE))
	$(QUIET)$(ECHO) Downloading OTA_APP $(SECURE_OTA_APP) at sector $(OTA_APP_SECTOR_START) address $(OTA_APP_SECTOR_ADDRESS)...
ifeq ($(JLINK_NATIVE), 1)
	$(QUIET)$(call JLINK_MAKE_SCRIPT_ALL,$(SECURE_OTA_APP),$(OTA_APP_SECTOR_ADDRESS))
	$(QUIET)$(call JLINK_EXE_SCRIPT)
else
	$(call CONV_SLASHES,$(OPENOCD_FULL_NAME)) -f $(OPENOCD_PATH)$(JTAG).cfg -f $(OPENOCD_PATH)$(HOST_OPENOCD).cfg -f apps/waf/sflash_write/sflash_write.tcl -c "sflash_write_file $(SECURE_OTA_APP) $(OTA_APP_SECTOR_ADDRESS) $(SFLASH_APP_PLATFROM_BUS) 0 $(SFLASH_APP_BCM4390)" -c shutdown $(DOWNLOAD_LOG) 2>&1
endif
else
	$(QUIET)$(ECHO) Downloading OTA_APP $(OTA_APP) at sector $(OTA_APP_SECTOR_START) address $(OTA_APP_SECTOR_ADDRESS)...
ifeq ($(JLINK_NATIVE), 1)
	$(QUIET)$(call JLINK_MAKE_SCRIPT_ALL,$(OTA_APP),$(OTA_APP_SECTOR_ADDRESS))
	$(QUIET)$(call JLINK_EXE_SCRIPT)
else
	$(call CONV_SLASHES,$(OPENOCD_FULL_NAME)) -f $(OPENOCD_PATH)$(JTAG).cfg -f $(OPENOCD_PATH)$(HOST_OPENOCD).cfg -f apps/waf/sflash_write/sflash_write.tcl -c "sflash_write_file $(OTA_APP) $(OTA_APP_SECTOR_ADDRESS) $(SFLASH_APP_PLATFROM_BUS) 0 $(SFLASH_APP_BCM4390)" -c shutdown $(DOWNLOAD_LOG) 2>&1
endif
endif
endif

ifneq ($(DCT_IMAGE),)
DCT_IMAGE_DOWNLOAD:  $(DCT_IMAGE_DOWNLOAD_DEPENDENCY)
	$(QUIET)$(ECHO) Downloading DCT_IMAGE $(DCT_IMAGE) at sector $(DCT_IMAGE_SECTOR_START) offset $(DCT_IMAGE_SECTOR_ADDRESS) size $(DCT_IMAGE_SECTOR_COUNT)...
ifeq ($(JLINK_NATIVE), 1)
	$(QUIET)$(call JLINK_MAKE_SCRIPT_ALL,$(DCT_IMAGE),$(DCT_IMAGE_SECTOR_ADDRESS))
	$(QUIET)$(call JLINK_EXE_SCRIPT)
else
	$(call CONV_SLASHES,$(OPENOCD_FULL_NAME)) -f $(OPENOCD_PATH)$(JTAG).cfg -f $(OPENOCD_PATH)$(HOST_OPENOCD).cfg -f apps/waf/sflash_write/sflash_write.tcl -c "sflash_write_file $(DCT_IMAGE) $(DCT_IMAGE_SECTOR_ADDRESS) $(SFLASH_APP_PLATFROM_BUS) 0 $(SFLASH_APP_BCM4390)" -c shutdown $(DOWNLOAD_LOG) 2>&1
endif
endif

ifneq ($(FILESYSTEM_IMAGE),)
FILESYSTEM_IMAGE_DOWNLOAD: $(FILESYSTEM_IMAGE_DOWNLOAD_DEPENDENCY) $(SECURE_FS_IMAGE)
ifeq (1,$(FILESYSTEM_IMAGE_SECURE))
	$(QUIET)$(ECHO) Downloading resources secure filesystem ... $(FILESYSTEM_IMAGE).enc at sector $(FILESYSTEM_IMAGE_SECTOR_START) size $(FILESYSTEM_IMAGE_SECTOR_COUNT)...
ifeq ($(JLINK_NATIVE), 1)
	$(QUIET)$(shell cp $(FILESYSTEM_IMAGE).enc $(FILESYSTEM_IMAGE).enc.bin)
	$(QUIET)$(call JLINK_MAKE_SCRIPT_ALL,$(FILESYSTEM_IMAGE).enc.bin,$(FILESYSTEM_IMAGE_SECTOR_ADDRESS))
	$(QUIET)$(call JLINK_EXE_SCRIPT)
else
	$(call CONV_SLASHES,$(OPENOCD_FULL_NAME)) -f $(OPENOCD_PATH)$(JTAG).cfg -f $(OPENOCD_PATH)$(HOST_OPENOCD).cfg -f apps/waf/sflash_write/sflash_write.tcl -c "sflash_write_file $(FILESYSTEM_IMAGE).enc $(FILESYSTEM_IMAGE_SECTOR_ADDRESS) $(SFLASH_APP_PLATFROM_BUS) 0 $(SFLASH_APP_BCM4390)" -c shutdown $(DOWNLOAD_LOG) 2>&1
endif
else
	$(QUIET)$(ECHO) Downloading resources filesystem ... $(FILESYSTEM_IMAGE) at sector $(FILESYSTEM_IMAGE_SECTOR_START) size $(FILESYSTEM_IMAGE_SECTOR_COUNT)...
ifeq ($(JLINK_NATIVE), 1)
	$(QUIET)$(call JLINK_MAKE_SCRIPT_ALL,$(FILESYSTEM_IMAGE),$(FILESYSTEM_IMAGE_SECTOR_ADDRESS))
	$(QUIET)$(call JLINK_EXE_SCRIPT)
else
	$(call CONV_SLASHES,$(OPENOCD_FULL_NAME)) -f $(OPENOCD_PATH)$(JTAG).cfg -f $(OPENOCD_PATH)$(HOST_OPENOCD).cfg -f apps/waf/sflash_write/sflash_write.tcl -c "sflash_write_file $(FILESYSTEM_IMAGE) $(FILESYSTEM_IMAGE_SECTOR_ADDRESS) $(SFLASH_APP_PLATFROM_BUS) 0 $(SFLASH_APP_BCM4390)" -c shutdown $(DOWNLOAD_LOG) 2>&1
endif
endif
FILESYSTEM_IMAGE_PACKAGE: $(FILESYSTEM_IMAGE_PACKAGE_DEPENDENCY)
	$(QUIET)$(ECHO) Packaging resources filesystem ... $(FILESYSTEM_IMAGE) at sector $(FILESYSTEM_IMAGE_SECTOR_START) size $(FILESYSTEM_IMAGE_SECTOR_COUNT)...
	$(call ADD_TO_PACKAGE, $(FILESYSTEM_IMAGE), sflash, $(FILESYSTEM_IMAGE_SECTOR_ADDRESS))

endif

ifneq ($(WIFI_FIRMWARE),)
WIFI_FIRMWARE_DOWNLOAD: $(WIFI_FIRMWARE_DOWNLOAD_DEPENDENCY)
	$(QUIET)$(ECHO) Downloading WIFI_FIRMWARE ... at sector $(WIFI_FIRMWARE_SECTOR_START) size $(WIFI_FIRMWARE_SECTOR_COUNT)...
ifeq ($(JLINK_NATIVE), 1)
	$(QUIET)$(call JLINK_MAKE_SCRIPT_ALL,$(WIFI_FIRMWARE),$(WIFI_FIRMWARE_SECTOR_ADDRESS))
	$(QUIET)$(call JLINK_EXE_SCRIPT)
else
	$(call CONV_SLASHES,$(OPENOCD_FULL_NAME)) -f $(OPENOCD_PATH)$(JTAG).cfg -f $(OPENOCD_PATH)$(HOST_OPENOCD).cfg -f apps/waf/sflash_write/sflash_write.tcl -c "sflash_write_file $(WIFI_FIRMWARE) $(WIFI_FIRMWARE_SECTOR_ADDRESS) $(SFLASH_APP_PLATFROM_BUS) 0 $(SFLASH_APP_BCM4390)" -c shutdown $(DOWNLOAD_LOG) 2>&1
endif
endif

ifneq ($(APP0),)
ifeq (1,$(CURRENT_APPLICATION_USES_INTERNAL_FLASH))
APP0_DOWNLOAD:
	$(QUIET)$(ECHO) Skipping downloading of APP0 to sflash since it's stored internally...
else
APP0_DOWNLOAD:  $(APP0_DOWNLOAD_DEPENDENCY) $(SECURE_APP0)
ifeq (1,$(APP0_SECURE))
	$(QUIET)$(ECHO) Downloading secure APP0 $(SECURE_APP0) @ sector $(APP0_SECTOR_START) address $(APP0_SECTOR_ADDRESS) size $(APP0_SECTOR_COUNT)...
ifeq ($(JLINK_NATIVE), 1)
	$(QUIET)$(call JLINK_MAKE_SCRIPT_ALL,$(SECURE_APP0),$(APP0_SECTOR_ADDRESS))
	$(QUIET)$(call JLINK_EXE_SCRIPT)
else
	$(call CONV_SLASHES,$(OPENOCD_FULL_NAME)) -f $(OPENOCD_PATH)$(JTAG).cfg -f $(OPENOCD_PATH)$(HOST_OPENOCD).cfg -f apps/waf/sflash_write/sflash_write.tcl -c "sflash_write_file $(SECURE_APP0) $(APP0_SECTOR_ADDRESS) $(SFLASH_APP_PLATFROM_BUS) 0 $(SFLASH_APP_BCM4390)" -c shutdown $(DOWNLOAD_LOG) 2>&1
endif
else
	$(QUIET)$(ECHO) Downloading APP0 $(APP0) @ sector $(APP0_SECTOR_START) address $(APP0_SECTOR_ADDRESS) size $(APP0_SECTOR_COUNT)...
ifeq ($(JLINK_NATIVE), 1)
	$(QUIET)$(shell cp $(APP0) $(APP0).bin)
	$(QUIET)$(call JLINK_MAKE_SCRIPT_ALL,$(APP0).bin,$(APP0_SECTOR_ADDRESS))
	$(QUIET)$(call JLINK_EXE_SCRIPT)
else
	$(call CONV_SLASHES,$(OPENOCD_FULL_NAME)) -f $(OPENOCD_PATH)$(JTAG).cfg -f $(OPENOCD_PATH)$(HOST_OPENOCD).cfg -f apps/waf/sflash_write/sflash_write.tcl -c "sflash_write_file $(APP0) $(APP0_SECTOR_ADDRESS) $(SFLASH_APP_PLATFROM_BUS) 0 $(SFLASH_APP_BCM4390)" -c shutdown $(DOWNLOAD_LOG) 2>&1
endif
endif
endif
APP0_PACKAGE: $(APP0_PACKAGE_DEPENDENCY)
	$(call ADD_TO_PACKAGE, $(APP0), sflash, $(APP0_SECTOR_ADDRESS))
endif

ifneq ($(APP1),)
APP1_DOWNLOAD:  $(APP1_DOWNLOAD_DEPENDENCY)
	$(QUIET)$(ECHO) Downloading APP1 $(APP1) at sector $(APP1_SECTOR_START) address $(APP1_SECTOR_ADDRESS) size: $(APP1_SECTOR_COUNT)...
ifeq ($(JLINK_NATIVE), 1)
	$(QUIET)$(call JLINK_MAKE_SCRIPT_ALL,$(APP1),$(APP1_SECTOR_ADDRESS))
	$(QUIET)$(call JLINK_EXE_SCRIPT)
else
	$(call CONV_SLASHES,$(OPENOCD_FULL_NAME)) -f $(OPENOCD_PATH)$(JTAG).cfg -f $(OPENOCD_PATH)$(HOST_OPENOCD).cfg -f apps/waf/sflash_write/sflash_write.tcl -c "sflash_write_file $(APP1) $(APP1_SECTOR_ADDRESS) $(SFLASH_APP_PLATFROM_BUS) 0 $(SFLASH_APP_BCM4390)" -c shutdown $(DOWNLOAD_LOG) 2>&1
endif
endif

ifneq ($(APP2),)
APP2_DOWNLOAD:  $(APP2_DOWNLOAD_DEPENDENCY)
	$(QUIET)$(ECHO) Downloading APP2 $(APP2) at sector $(APP2_SECTOR_START) address $(APP2_SECTOR_ADDRESS) size: $(APP1_SECTOR_COUNT)...
ifeq ($(JLINK_NATIVE), 1)
	$(QUIET)$(call JLINK_MAKE_SCRIPT_ALL,$(APP2),$(APP2_SECTOR_ADDRESS))
	$(QUIET)$(call JLINK_EXE_SCRIPT)
else
	$(call CONV_SLASHES,$(OPENOCD_FULL_NAME)) -f $(OPENOCD_PATH)$(JTAG).cfg -f $(OPENOCD_PATH)$(HOST_OPENOCD).cfg -f apps/waf/sflash_write/sflash_write.tcl -c "sflash_write_file $(APP2) $(APP2_SECTOR_ADDRESS) $(SFLASH_APP_PLATFROM_BUS) 0 $(SFLASH_APP_BCM4390)" -c shutdown $(DOWNLOAD_LOG) 2>&1
endif
endif

# If Downloading is required, then the Serial Flash app need to be built
sflash_write_app:
ifeq ($(JLINK_NATIVE), 1)
	$(QUIET)$(ECHO) Using jlink_native and ignore sflash_write_app ...
else
	$(QUIET)$(ECHO) Building Serial Flash Loader App
	$(QUIET)$(MAKE) -r -f $(SOURCE_ROOT)Makefile $(SFLASH_APP_TARGET) -I$(OUTPUT_DIR)  SFLASH= EXTERNAL_WICED_GLOBAL_DEFINES=$(EXTERNAL_WICED_GLOBAL_DEFINES) SUB_BUILD=sflash_app $(SFLASH_REDIRECT)
	$(QUIET)$(ECHO) Finished Building Serial Flash Loader App
	$(QUIET)$(ECHO_BLANK_LINE)
endif

APPS_LUT_DOWNLOAD: sflash_write_app $(FINAL_APPS_FILE) $(APPS_DOWNLOADS_DEPENDENCY) $(APPS_LUT_DOWNLOAD_DEP) APPS_LOOKUP_TABLE_RULES
	$(QUIET)$(ECHO) Downloading apps lookup table in wiced_apps.mk ... $(FINAL_APPS_FILE) @ $(APPS_LUT_HEADER_LOC) size $(APP_LUT_SECTOR_COUNT)
ifeq ($(JLINK_NATIVE), 1)
	$(QUIET)$(call JLINK_MAKE_SCRIPT_ALL,$(FINAL_APPS_FILE),$(APPS_LUT_HEADER_LOC))
	$(QUIET)$(call JLINK_EXE_SCRIPT)
else
	$(call CONV_SLASHES,$(OPENOCD_FULL_NAME)) -f $(OPENOCD_PATH)$(JTAG).cfg -f $(OPENOCD_PATH)$(HOST_OPENOCD).cfg -f apps/waf/sflash_write/sflash_write.tcl -c "sflash_write_file $(FINAL_APPS_FILE) $(APPS_LUT_HEADER_LOC) $(SFLASH_APP_PLATFROM_BUS) 0 $(SFLASH_APP_BCM4390)" -c shutdown $(DOWNLOAD_LOG) 2>&1
endif

APPS_LUT_PACKAGE: sflash_write_app $(FINAL_APPS_FILE) $(APPS_PACKAGE_DEPENDENCY) $(APPS_LUT_PACKAGE_DEP) APPS_LOOKUP_TABLE_RULES
	$(QUIET)$(ECHO) Packaging apps lookup table in wiced_apps.mk ... $(FINAL_APPS_FILE) @ $(APPS_LUT_HEADER_LOC) size $(APP_LUT_SECTOR_COUNT)
	$(call ADD_TO_PACKAGE, $(FINAL_APPS_FILE), sflash, $(APPS_LUT_HEADER_LOC))

# TODO: add compression to individual components before building OTA Image - here or in the builder?
# If OTA Image is required, then build the OTA Image maker

ifeq ($(APP_VERSION_FOR_OTA2_MAJOR),)
APP_VERSION_FOR_OTA2_MAJOR := 0
endif

ifeq ($(APP_VERSION_FOR_OTA2_MINOR),)
APP_VERSION_FOR_OTA2_MINOR := 0
endif

# Dependencies for building the OTA2 image or factory image files
OTA2_IMAGE_DEPENDENCIES	:= $(DCT_IMAGE) $(FILESYSTEM_IMAGE) $(FINAL_APPS_FILE) $(OTA_APP) $(WIFI_FIRMWARE) $(APP0) $(APP1) $(APP2) $(SECURE_APP0) $(SECURE_FS_IMAGE) $(DCT_IMAGE_PLATFORM) $(SECURE_OTA_APP)
# If ota2_download specified, wait for other components to be downloaded
OTA2_DOWNLOAD_DEPENDENCY := $(filter download download_apps, $(MAKECMDGOALS))
# If ota2_factory_download specified, wait for ota2_download to finish before ota2_factory_download starts
OTA2_FACTORY_DOWNLOAD_DEPENDENCY := $(OTA2_DOWNLOAD_DEPENDENCY) $(filter ota2_download, $(MAKECMDGOALS))

ota2_image: $(OTA2_IMAGE_DEPENDENCIES)
ifeq ($(OTA_APP),)
	$(QUIET)$(ECHO) ""
	$(QUIET)$(ECHO) "   The OTA2 extraction application ***MUST*** be included in your OTA2 update"
	$(QUIET)$(ECHO) "   Please build apps/snip/ota2_extract (or your ota2 extraction application)"
	$(QUIET)$(ECHO) "   And add these lines to your <application>.mk file (see apps/snip/ota2_example/ota2_example.mk):"
	$(QUIET)$(ECHO) ""
	$(QUIET)$(ECHO) "   OTA_APPLICATION	:= snip.ota2_extract-$$(PLATFORM)"
	$(QUIET)$(ECHO) "   OTA_APP    := build/$$(OTA_APPLICATION)/binary/$$(OTA_APPLICATION).stripped.elf"
	$(QUIET)$(ECHO) ""
	$(error        OTA2 Factory Reset Image Info File Not built!)
else
	$(QUIET)$(ECHO) Building OTA2 Image Info File $(OTA2_IMAGE_CONFIG_FILE)
	$(QUIET)$(call WRITE_FILE_CREATE, $(OTA2_IMAGE_CONFIG_FILE) ,FACTORY_RESET=0x00)
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_CONFIG_FILE) ,MAJOR_VERSION=$(APP_VERSION_FOR_OTA2_MAJOR))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_CONFIG_FILE) ,MINOR_VERSION=$(APP_VERSION_FOR_OTA2_MINOR))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_CONFIG_FILE) ,PLATFORM_NAME=$(PLATFORM).$(APPS_CHIP_REVISION))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_CONFIG_FILE) ,APPS_LUT_LOC=$(APPS_LUT_HEADER_LOC))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_CONFIG_FILE) ,APPS_LUT_FILE=$(call CONV_SLASHES,$(FINAL_APPS_FILE)))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_CONFIG_FILE) ,FR_APP_LOC=$(FR_APP_SECTOR_ADDRESS))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_CONFIG_FILE) ,FR_APP_FILE=$(call CONV_SLASHES,$(FR_APP)))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_CONFIG_FILE) ,DCT_LOC=$(SFLASH_DCT_LOC))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_CONFIG_FILE) ,DCT_FILE=$(call CONV_SLASHES,$(DCT_IMAGE_PLATFORM)))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_CONFIG_FILE) ,OTA_APP_LOC=$(OTA_APP_SECTOR_ADDRESS))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_CONFIG_FILE) ,OTA_APP_FILE=$(if $(SECURE_OTA_APP),$(call CONV_SLASHES,$(SECURE_OTA_APP)),$(call CONV_SLASHES,$(OTA_APP))))
ifneq (1,$(PLATFORM_SUPPORTS_OC_FLASH))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_CONFIG_FILE) ,FILESYSTEM_LOC=$(FILESYSTEM_IMAGE_SECTOR_ADDRESS))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_CONFIG_FILE) ,FILESYSTEM_FILE=$(if $(SECURE_FS_IMAGE),$(call CONV_SLASHES,$(SECURE_FS_IMAGE)),$(call CONV_SLASHES,$(FS_IMAGE))))
endif
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_CONFIG_FILE) ,WIFI_FIRMWARE_LOC=$(WIFI_FIRMWARE_SECTOR_ADDRESS))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_CONFIG_FILE) ,WIFI_FIRMWARE_FILE=$(call CONV_SLASHES,$(WIFI_FIRMWARE)))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_CONFIG_FILE) ,APPLICATION_0_LOC=$(APP0_SECTOR_ADDRESS))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_CONFIG_FILE) ,APPLICATION_0_FILE=$(if $(APP0_SECURE),$(call CONV_SLASHES,$(SECURE_APP0)),$(call CONV_SLASHES,$(APP0))))
ifeq (1, $(XIP_SUPPORT))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_CONFIG_FILE) ,APPLICATION_0_XIP_LOC=$(XIP_LOAD_ADDRESS))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_CONFIG_FILE) ,APPLICATION_0_XIP_FILE=$(call CONV_SLASHES,$(XIP_OUTPUT_FILE)))
endif #ifeq (1, $(XIP_SUPPORT))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_CONFIG_FILE) ,APPLICATION_1_LOC=$(APP1_SECTOR_ADDRESS))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_CONFIG_FILE) ,APPLICATION_1_FILE=$(call CONV_SLASHES,$(APP1)))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_CONFIG_FILE) ,APPLICATION_2_LOC=$(APP2_SECTOR_ADDRESS))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_CONFIG_FILE) ,APPLICATION_2_FILE=$(call CONV_SLASHES,$(APP2)))
	$(QUIET)$(ECHO) Building OTA2 Image File $(OTA2_IMAGE_BIN_FILE)
	$(COMMON_TOOLS_PATH)mk_wiced_ota2_image32 $(OTA2_IMAGE_CONFIG_FILE) $(OTA2_IMAGE_BIN_FILE) -v $(VERBOSE)
	$(QUIET)$(ECHO) OTA2 Image File Done
endif

ota2_download: ota2_image $(OTA2_DOWNLOAD_DEPENDENCY)
	$(QUIET)$(ECHO) Downloading OTA2 Image $(OTA2_IMAGE_BIN_FILE) at $(OTA2_IMAGE_STAGING_AREA_BASE) ...
ifeq ($(JLINK_NATIVE), 1)
	$(QUIET)$(call JLINK_MAKE_SCRIPT_ALL,$(OTA2_IMAGE_BIN_FILE),$(OTA2_IMAGE_STAGING_AREA_BASE))
	$(QUIET)$(call JLINK_EXE_SCRIPT)
else
	$(call CONV_SLASHES,$(OPENOCD_FULL_NAME)) -f $(OPENOCD_PATH)$(JTAG).cfg -f $(OPENOCD_PATH)$(HOST_OPENOCD).cfg -f apps/waf/sflash_write/sflash_write.tcl -c "sflash_write_file $(OTA2_IMAGE_BIN_FILE) $(OTA2_IMAGE_STAGING_AREA_BASE) $(SFLASH_APP_PLATFROM_BUS) 0 $(SFLASH_APP_BCM4390)" -c shutdown $(DOWNLOAD_LOG) 2>&1
endif
	$(QUIET)$(ECHO) Downloading OTA2 Image Done

ota2_factory_image: $(OTA2_IMAGE_DEPENDENCIES)
ifeq ($(OTA_APP),)
	$(QUIET)$(ECHO) ""
	$(QUIET)$(ECHO) "   The OTA2 extraction application ***MUST*** be included in your OTA2 update"
	$(QUIET)$(ECHO) "   Please build apps/snip/ota2_extract (or your ota2 extraction application)"
	$(QUIET)$(ECHO) "   And add these lines to your <application>.mk file (see apps/snip/ota2_example/ota2_example.mk):"
	$(QUIET)$(ECHO) ""
	$(QUIET)$(ECHO) "   OTA_APPLICATION	:= snip.ota2_extract-$$(PLATFORM)"
	$(QUIET)$(ECHO) "   OTA_APP    := build/$$(OTA_APPLICATION)/binary/$$(OTA_APPLICATION).stripped.elf"
	$(QUIET)$(ECHO) ""
	$(error        OTA2 Factory Reset Image Info File Not built!)
else
	$(QUIET)$(ECHO) Building OTA2 Factory Reset Image Info File $(OTA2_IMAGE_FACTORY_RESET_CONFIG_FILE)
	$(QUIET)$(call WRITE_FILE_CREATE, $(OTA2_IMAGE_FACTORY_RESET_CONFIG_FILE) ,FACTORY_RESET=0x01)
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_FACTORY_RESET_CONFIG_FILE) ,MAJOR_VERSION=$(APP_VERSION_FOR_OTA2_MAJOR))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_FACTORY_RESET_CONFIG_FILE) ,MINOR_VERSION=$(APP_VERSION_FOR_OTA2_MINOR))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_FACTORY_RESET_CONFIG_FILE) ,PLATFORM_NAME=$(PLATFORM).$(APPS_CHIP_REVISION))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_FACTORY_RESET_CONFIG_FILE) ,APPS_LUT_LOC=$(APPS_LUT_HEADER_LOC))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_FACTORY_RESET_CONFIG_FILE) ,APPS_LUT_FILE=$(call CONV_SLASHES,$(FINAL_APPS_FILE)))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_FACTORY_RESET_CONFIG_FILE) ,FR_APP_LOC=$(FR_APP_SECTOR_ADDRESS))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_FACTORY_RESET_CONFIG_FILE) ,FR_APP_FILE=$(call CONV_SLASHES,$(FR_APP)))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_FACTORY_RESET_CONFIG_FILE) ,DCT_LOC=$(SFLASH_DCT_LOC))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_FACTORY_RESET_CONFIG_FILE) ,DCT_FILE=$(call CONV_SLASHES,$(DCT_IMAGE_PLATFORM)))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_FACTORY_RESET_CONFIG_FILE) ,OTA_APP_LOC=$(OTA_APP_SECTOR_ADDRESS))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_FACTORY_RESET_CONFIG_FILE) ,OTA_APP_FILE=$(if $(SECURE_OTA_APP),$(call CONV_SLASHES,$(SECURE_OTA_APP)),$(call CONV_SLASHES,$(OTA_APP))))
ifneq (1,$(PLATFORM_SUPPORTS_OC_FLASH))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_FACTORY_RESET_CONFIG_FILE) ,FILESYSTEM_LOC=$(FILESYSTEM_IMAGE_SECTOR_ADDRESS))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_FACTORY_RESET_CONFIG_FILE) ,FILESYSTEM_FILE=$(if $(SECURE_FS_IMAGE),$(call CONV_SLASHES,$(SECURE_FS_IMAGE)),$(call CONV_SLASHES,$(FS_IMAGE))))
endif
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_FACTORY_RESET_CONFIG_FILE) ,WIFI_FIRMWARE_LOC=$(WIFI_FIRMWARE_SECTOR_ADDRESS))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_FACTORY_RESET_CONFIG_FILE) ,WIFI_FIRMWARE_FILE=$(call CONV_SLASHES,$(WIFI_FIRMWARE)))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_FACTORY_RESET_CONFIG_FILE) ,APPLICATION_0_LOC=$(APP0_SECTOR_ADDRESS))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_FACTORY_RESET_CONFIG_FILE) ,APPLICATION_0_FILE=$(if $(APP0_SECURE),$(call CONV_SLASHES,$(SECURE_APP0)),$(call CONV_SLASHES,$(APP0))))
ifeq (1, $(XIP_SUPPORT))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_FACTORY_RESET_CONFIG_FILE) ,APPLICATION_0_XIP_LOC=$(XIP_LOAD_ADDRESS))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_FACTORY_RESET_CONFIG_FILE) ,APPLICATION_0_XIP_FILE=$(call CONV_SLASHES,$(XIP_OUTPUT_FILE)))
endif #ifeq (1, $(XIP_SUPPORT))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_FACTORY_RESET_CONFIG_FILE) ,APPLICATION_1_LOC=$(APP1_SECTOR_ADDRESS))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_FACTORY_RESET_CONFIG_FILE) ,APPLICATION_1_FILE=$(call CONV_SLASHES,$(APP1)))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_FACTORY_RESET_CONFIG_FILE) ,APPLICATION_2_LOC=$(APP2_SECTOR_ADDRESS))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_FACTORY_RESET_CONFIG_FILE) ,APPLICATION_2_FILE=$(call CONV_SLASHES,$(APP2)))
	$(QUIET)$(ECHO) Building OTA2 Factory Reset Image $(OTA2_IMAGE_FACTORY_RESET_BIN_FILE)
	$(COMMON_TOOLS_PATH)mk_wiced_ota2_image32 $(OTA2_IMAGE_FACTORY_RESET_CONFIG_FILE) $(OTA2_IMAGE_FACTORY_RESET_BIN_FILE) -v $(VERBOSE)
	$(QUIET)$(ECHO) OTA2 Factory Reset Image Done
endif

ota2_factory_download: ota2_factory_image $(OTA2_FACTORY_DOWNLOAD_DEPENDENCY) sflash_write_app
	$(QUIET)$(ECHO) Downloading OTA2 Factory Reset Image $(OTA2_IMAGE_FACTORY_RESET_BIN_FILE) at $(OTA2_IMAGE_FACTORY_RESET_AREA_BASE) ...
ifeq ($(JLINK_NATIVE), 1)
	$(QUIET)$(call JLINK_MAKE_SCRIPT_ALL,$(OTA2_IMAGE_FACTORY_RESET_BIN_FILE),$(OTA2_IMAGE_FACTORY_RESET_AREA_BASE))
	$(QUIET)$(call JLINK_EXE_SCRIPT)
else
	$(call CONV_SLASHES,$(OPENOCD_FULL_NAME)) -f $(OPENOCD_PATH)$(JTAG).cfg -f $(OPENOCD_PATH)$(HOST_OPENOCD).cfg -f apps/waf/sflash_write/sflash_write.tcl -c "sflash_write_file $(OTA2_IMAGE_FACTORY_RESET_BIN_FILE) $(OTA2_IMAGE_FACTORY_RESET_AREA_BASE) $(SFLASH_APP_PLATFROM_BUS) 0 $(SFLASH_APP_BCM4390)" -c shutdown $(DOWNLOAD_LOG) 2>&1
endif
	$(QUIET)$(ECHO) Downloading OTA2 Factory Reset Image Done

# This will create an image suitable for directly flashing to the device
# The image will be as big as the FLASH (defined in ota2_image_defines.mk), and be filled with 0xFF for unused parts of the FLASH
ota2_manuf_image: ota2_factory_image $(OTA2_IMAGE_DEPENDENCIES) $(DCT_IMAGE) $(DCT_IMAGE_DOWNLOAD_DEPENDENCY) ota2_failsafe
	$(QUIET)$(call WRITE_FILE_CREATE, $(OTA2_IMAGE_MANUFACTURING_CONFIG_FILE) ,$(OTA2_IMAGE_BOOTLOADER_START) build/$(BOOTLOADER_TARGET)/binary/$(BOOTLOADER_TARGET).trx.bin)
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_MANUFACTURING_CONFIG_FILE) ,$(OTA2_IMAGE_FACTORY_RESET_AREA_BASE) $(OTA2_IMAGE_FACTORY_RESET_BIN_FILE))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_MANUFACTURING_CONFIG_FILE) ,$(OTA2_IMAGE_FAILSAFE_APP_AREA_BASE) $(if $(FAILSAFE_APP_SECURE),build/$(OTA2_FAILSAFE_TARGET)/binary/$(OTA2_FAILSAFE_TARGET).stripped.elf.sig.enc,build/$(OTA2_FAILSAFE_TARGET)/binary/$(OTA2_FAILSAFE_TARGET).stripped.elf))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_MANUFACTURING_CONFIG_FILE) ,$(OTA2_IMAGE_CURR_LUT_AREA_BASE) $(FINAL_APPS_FILE))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_MANUFACTURING_CONFIG_FILE) ,$(OTA2_IMAGE_CURR_DCT_1_AREA_BASE) $(DCT_IMAGE_PLATFORM))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_MANUFACTURING_CONFIG_FILE) ,$(OTA2_IMAGE_CURR_OTA_APP_AREA_BASE) $(if $(SECURE_OTA_APP),$(SECURE_OTA_APP),$(OTA_APP)))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_MANUFACTURING_CONFIG_FILE) ,$(OTA2_IMAGE_CURR_FS_AREA_BASE) $(if $(SECURE_FS_IMAGE),$(SECURE_FS_IMAGE),$(FS_IMAGE)))
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_MANUFACTURING_CONFIG_FILE) ,$(OTA2_IMAGE_CURR_APP0_AREA_BASE) $(if $(APP0_SECURE),$(SECURE_APP0),$(APP0)))
ifneq ($(APP1),)
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_MANUFACTURING_CONFIG_FILE) ,$(OTA2_IMAGE_CURR_APP1_AREA_BASE) $(APP1))
endif
ifneq ($(APP2),)
	$(QUIET)$(call WRITE_FILE_APPEND, $(OTA2_IMAGE_MANUFACTURING_CONFIG_FILE) ,$(OTA2_IMAGE_CURR_APP2_AREA_BASE) $(APP2))
endif
	$(shell $(PERL) $(CREATE_FLASH_IMAGE_TOOL) $(call CONV_SLASHES,$(OTA2_IMAGE_MANUFACTURING_CONFIG_FILE)) $(call CONV_SLASHES,$(OTA2_IMAGE_MANUFACTURING_BIN_FILE)))
	$(QUIET)$(ECHO) OTA2 Manufacturing Image Done! $(OTA2_IMAGE_MANUFACTURING_BIN_FILE)

# Currently this only works for FLASH images < 4MB - there is an issue in sflash_write, or OpenOCD...
ota2_manuf_download: ota2_manuf_image sflash_write_app
	$(QUIET)$(ECHO) Downloading OTA2 Manuf Image $(OTA2_IMAGE_MANUFACTURING_BIN_FILE) at 0x00 ...
ifeq ($(JLINK_NATIVE), 1)
	$(QUIET)$(call JLINK_MAKE_SCRIPT_ALL,$(OTA2_IMAGE_MANUFACTURING_BIN_FILE),0x00)
	$(QUIET)$(call JLINK_EXE_SCRIPT)
else
	$(call CONV_SLASHES,$(OPENOCD_FULL_NAME)) -f $(OPENOCD_PATH)$(JTAG).cfg -f $(OPENOCD_PATH)$(HOST_OPENOCD).cfg -f apps/waf/sflash_write/sflash_write.tcl -c "sflash_write_file $(OTA2_IMAGE_MANUFACTURING_BIN_FILE) 0x00 $(SFLASH_APP_PLATFROM_BUS) 0 $(SFLASH_APP_BCM4390)" -c shutdown $(DOWNLOAD_LOG) 2>&1
endif
	$(QUIET)$(ECHO) Downloading OTA2 Manuf Image Done

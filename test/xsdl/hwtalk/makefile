
HWTALK_ROOT=.
BUILD_DIR=$(HWTALK_ROOT)/build/$(OS_LC).$(BUILD_ARCH)
OBJ_DIR=$(BUILD_DIR)/obj

include make.common

INCLUDES += -I. -I../include -I../include/common -I../include/linux

LDFLAGS += -L$(BOOST_LIB) -lboost_filesystem -lboost_system -lboost_program_options

PROG := $(OBJ_DIR)/../hwtalk

SOURCE := \
	hwtalk.cpp \
	logmanager.cpp

OBJ := $(addprefix $(OBJ_DIR)/,$(subst .cpp,.o,$(notdir $(SOURCE))))

VPATH := .:./common ./linux

BINS += $(PROG)

all vdebug debug profile: .bindir $(OBJ) $(PROG)

ifeq ($(BUILD_SYSNAME),Darwin)
$(PROG) : .bindir $(OBJ)
	@echo " "
	$(GXX) $(LDFLAGS) -L$(HWTALK_ROOT)/../osx -lxsdl -o $@ $(OBJ)
	@cp -f $(HWTALK_ROOT)/../osx/lib* $(BUILD_DIR)
	@install_name_tool -change ../../pifunctionblocks/xsdl/build/darwin.x86_64/libxsdl.so @executable_path/libxsdl.so $(BUILD_DIR)/hwtalk
else
$(PROG) : .bindir $(OBJ)
	@echo " "
	$(GXX) $(LDFLAGS) '-Wl,-rpath,$$ORIGIN' -L$(HWTALK_ROOT)/../bin/$(OS_LC) -o $@ $(OBJ) -lxsdl -llowlevel -lpthread
	@cp -f $(HWTALK_ROOT)/../bin/$(OS_LC)/lib*.so $(BUILD_DIR)
endif

clean:
	rm -f $(OBJ)
	rm -f $(PROG)

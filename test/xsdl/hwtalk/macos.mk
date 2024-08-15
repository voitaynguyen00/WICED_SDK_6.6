

## -----------------------------------------------------------------------------------------------
## COMPILER DEFINITIONS
## -----------------------------------------------------------------------------------------------
GCC = gcc
GXX = g++


## -----------------------------------------------------------------------------------------------
## GENERIC DEFINES
## -----------------------------------------------------------------------------------------------
DEFINES += -DLINUX -D_GNU_SOURCE -D__APPLE__
.bindir:
	mkdir -p $(OBJ_DIR)

## -----------------------------------------------------------------------------------------------
## COMPILER SPECIFIC VARIABLES AND RULES
## -----------------------------------------------------------------------------------------------

WARN 	= -Wall -Wno-strict-aliasing
OPTS 	= -Os -ftemplate-depth-300
DBG 	= $(if $(PROFILE_ON), -g -pg, -g)
CCOPTS += -fPIC
CXXOPTS += -fPIC -std=c++0x
LDLINKS = $(LIBEXT) #-pthread
CFLAGS 	= $(ARCH) $(CCOPTS) $(WARN) $(DEFINES) $(INCLUDES) $(ADDL_CFLAGS) -m64
CXXFLAGS= $(ARCH) $(CXXOPTS) $(WARN) $(DEFINES) $(INCLUDES) $(ADDL_CXXFLAGS) -m64

OBJ_DIR = build/macos/obj

LL = ../../../../Tools/source/pcswrepo
INCLUDES = -I$(LL)/lowlevel/linux -I$(LL)/lowlevel/common -I$(LL)/pifunctionblocks/xsdl/include -I$(LL)/piutils/inc -I/usr/local/include

LIBS = -L/usr/local/lib -lboost_filesystem -lboost_system -lboost_program_options 

OBJ = $(OBJ_DIR)/logmanager.o $(OBJ_DIR)/circque.o $(OBJ_DIR)/osobjects.o $(OBJ_DIR)/hwtalk.o $(OBJ_DIR)/basetype.o $(OBJ_DIR)/highertypes.o $(OBJ_DIR)/miscfunc.o $(OBJ_DIR)/oscommon.o  $(OBJ_DIR)/trancommon.o $(OBJ_DIR)/transport.o $(OBJ_DIR)/uartrxthread.o $(OBJ_DIR)/uarttrandata.o $(OBJ_DIR)/uarttxthread.o $(OBJ_DIR)/usbrxthread.o $(OBJ_DIR)/usbtrandata.o

OBJ += $(OBJ_DIR)/DSDLCppWrapper.o $(OBJ_DIR)/const.o $(OBJ_DIR)/dso_fix.o $(OBJ_DIR)/enum.o $(OBJ_DIR)/env.o $(OBJ_DIR)/expression.o $(OBJ_DIR)/factory.o $(OBJ_DIR)/field.o $(OBJ_DIR)/ns.o $(OBJ_DIR)/nsobj.o $(OBJ_DIR)/structure.o $(OBJ_DIR)/utils.o $(OBJ_DIR)/expr.o $(OBJ_DIR)/tu.o $(OBJ_DIR)/forge.o

SRC = $(LL)/lowlevel/common/miscfunc.cpp $(LL)/lowlevel/common/highertypes.cpp logmanager.cpp $(LL)/lowlevel/common/basetype.cpp $(LL)/lowlevel/common/trancommon.cpp $(LL)/lowlevel/linux/usbtrandata.cpp $(LL)/lowlevel/linux/usbrxthread.cpp $(LL)/lowlevel/linux/uarttrandata.cpp $(LL)/lowlevel/linux/uartrxthread.cpp $(LL)/lowlevel/linux/uarttxthread.cpp $(LL)/lowlevel/common/oscommon.cpp $(LL)/lowlevel/linux/osobjects.cpp $(LL)/lowlevel/linux/transport.cpp ../../../../Tools/source/pcswrepo/piutils/datastruct/circque.cpp

.bindir:
	mkdir -p $(OBJ_DIR)

PROG := $(OBJ_DIR)/../hwtalk

	
all : .bindir $(OBJ) $(PROG)

$(PROG) : .bindir $(OBJ)
	$(GXX) -m64 -o build/macos/hwtalk $(OBJ) -L/usr/local/lib -lboost_filesystem -lboost_system -lboost_program_options 
	cp build/macos/hwtalk ../bin/osx/.

clean:
	rm build/macos/hwtalk
	rm $(OBJ_DIR)/*.o

$(OBJ_DIR)/circque.o: ../../../../Tools/source/pcswrepo/piutils/datastruct/circque.cpp
	echo "  GCC $<"
	$(GCC) -c $(CFLAGS) -o $@ $<
 
$(OBJ_DIR)/transport.o:  $(LL)/lowlevel/linux/transport.cpp
	echo "  GCC $<"
	$(GCC) -c $(CFLAGS) -o $@ $<

$(OBJ_DIR)/osobjects.o:  $(LL)/lowlevel/linux/osobjects.cpp
	echo "  GCC $<"
	$(GCC) -c $(CFLAGS) -o $@ $<

$(OBJ_DIR)/oscommon.o:  $(LL)/lowlevel/common/oscommon.cpp
	echo "  GCC $<"
	$(GCC) -c $(CFLAGS) -o $@ $<

$(OBJ_DIR)/uarttxthread.o:  $(LL)/lowlevel/linux/uarttxthread.cpp
	echo "  GCC $<"
	$(GCC) -c $(CFLAGS) -o $@ $<

$(OBJ_DIR)/uartrxthread.o:  $(LL)/lowlevel/linux/uartrxthread.cpp
	echo "  GCC $<"
	$(GCC) -c $(CFLAGS) -o $@ $<

$(OBJ_DIR)/uarttrandata.o:  $(LL)/lowlevel/linux/uarttrandata.cpp
	echo "  GCC $<"
	$(GCC) -c $(CFLAGS) -o $@ $<

$(OBJ_DIR)/usbrxthread.o:  $(LL)/lowlevel/linux/usbrxthread.cpp
	echo "  GCC $<"
	$(GCC) -c $(CFLAGS) -o $@ $<

$(OBJ_DIR)/usbtrandata.o:  $(LL)/lowlevel/linux/usbtrandata.cpp
	echo "  GCC $<"
	$(GCC) -c $(CFLAGS) -o $@ $<

$(OBJ_DIR)/trancommon.o:  $(LL)/lowlevel/common/trancommon.cpp
	echo "  GCC $<"
	$(GCC) -c $(CFLAGS) -o $@ $<

$(OBJ_DIR)/miscfunc.o:  $(LL)/lowlevel/common/miscfunc.cpp
	echo "  GCC $<"
	$(GCC) -c $(CFLAGS) -o $@ $<

$(OBJ_DIR)/highertypes.o:  $(LL)/lowlevel/common/highertypes.cpp
	echo "  GCC $<"
	$(GCC) -c $(CFLAGS) -o $@ $<

$(OBJ_DIR)/basetype.o:  $(LL)/lowlevel/common/basetype.cpp
	echo "  GCC $<"
	$(GCC) -c $(CFLAGS) -o $@ $<

$(OBJ_DIR)/logmanager.o:  logmanager.cpp
	echo "  GCC $<"
	$(GCC) -c $(CFLAGS) -o $@ $<

SRC_LIB = ../../../../Tools/source/pcswrepo/pifunctionblocks/xsdl

$(OBJ_DIR)/forge.o: $(SRC_LIB)/src/factory/forge.cpp
	echo "  GXX $<"
	$(GXX) -c $(CXXFLAGS) -o $@ $<
	
$(OBJ_DIR)/expr.o: $(SRC_LIB)/src/parser/expr.cpp
	echo "  GXX $<"
	$(GXX) -c $(CXXFLAGS) -o $@ $<

$(OBJ_DIR)/tu.o: $(SRC_LIB)/src/parser/tu.cpp
	echo "  GXX $<"
	$(GXX) -c $(CXXFLAGS) -o $@ $<
	
$(OBJ_DIR)/sdlapi.o: $(SRC_LIB)/src/swig/sdlapi.cpp
	echo "  GXX $<"
	$(GXX) -c $(CXXFLAGS) -o $@ $<

$(OBJ_DIR)/%.o: $(SRC_LIB)/src/%.c
	echo "  GXX $<"
	$(GXX) -c $(CXXFLAGS) -o $@ $<
 
$(OBJ_DIR)/%.o: $(SRC_LIB)/src/%.cpp
	echo "  GXX $<"
	$(GXX) -c $(CXXFLAGS) -o $@ $<
	
$(OBJ_DIR)/%.o: %.cpp
	echo "  GXX $<"
	$(GXX) -c $(CXXFLAGS) -o $@ $<

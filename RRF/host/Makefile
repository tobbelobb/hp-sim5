CXX := g++
CXXFLAGS := -std=gnu++17 -Wall -Wextra -pedantic -pthread \
    -I. \
    -Iinclude \
    -Irtos \
    -I../ReprapFirmware/src \
    -I../ReprapFirmware/src/Hardware/SAME70 \
    -I../RRFLibraries/src \
    -I../CANlib/src \
	-D__SAME70Q20B__ \
	-DDUET3_MB6HC=1 \
	-DDUET3=1 \
	-DSAME70=1 \
	-DRRF_HOST_BUILD=1 \
    -DSRC_MOVEMENT_STEPTIMER_H_=1 \
    -DUSE_SBC=0
LDFLAGS := -pthread
TARGET := host_rrf_bootstrap

HOST_SRC := src/main.cpp src/devices_stub.cpp rtos/freertos_shim.cpp \
	storage/HostFileStore.cpp \
	storage/HostMassStorage.cpp
RRF_SRC := ../ReprapFirmware/src/Version.cpp \
	../ReprapFirmware/src/Storage/CRC32.cpp \
	../ReprapFirmware/src/GCodes/GCodeFileInfo.cpp \
	../ReprapFirmware/src/GCodes/GCodeMachineState.cpp \
	../ReprapFirmware/src/GCodes/GCodeInput.cpp \
	../ReprapFirmware/src/GCodes/GCodeQueue.cpp \
	../ReprapFirmware/src/GCodes/GCodeException.cpp \
	../ReprapFirmware/src/GCodes/GCodes.cpp \
	../ReprapFirmware/src/GCodes/GCodes2.cpp \
	../ReprapFirmware/src/GCodes/GCodes3.cpp \
	../ReprapFirmware/src/GCodes/GCodes4.cpp \
	../ReprapFirmware/src/GCodes/GCodes5.cpp \
	../ReprapFirmware/src/GCodes/GCodes6.cpp \
	../ReprapFirmware/src/GCodes/GCodes7.cpp \
	../ReprapFirmware/src/GCodes/GCodeBuffer/GCodeBuffer.cpp \
	../ReprapFirmware/src/GCodes/GCodeBuffer/StringParser.cpp \
	../ReprapFirmware/src/GCodes/GCodeBuffer/BinaryParser.cpp \
	../ReprapFirmware/src/GCodes/GCodeBuffer/ExpressionParser.cpp

RRFLIB_SRC := ../RRFLibraries/src/General/StringRef.cpp \
	../RRFLibraries/src/General/SafeVsnprintf.cpp \
	../RRFLibraries/src/General/StringFunctions.cpp \
	../RRFLibraries/src/General/Strnlen.cpp

BUILD_DIR := build

HOST_OBJ := $(addprefix $(BUILD_DIR)/,$(HOST_SRC:.cpp=.o))
RRF_OBJ := $(addprefix $(BUILD_DIR)/rrf_,$(RRF_SRC:../ReprapFirmware/src/%.cpp=%.o))
RRFLIB_OBJ := $(addprefix $(BUILD_DIR)/rrflib_,$(RRFLIB_SRC:../RRFLibraries/src/%.cpp=%.o))
OBJ_FILES := $(HOST_OBJ) $(RRF_OBJ) $(RRFLIB_OBJ)

.PHONY: all clean

all: $(BUILD_DIR)/$(TARGET)

$(BUILD_DIR)/$(TARGET): $(OBJ_FILES)
	$(CXX) $(OBJ_FILES) $(LDFLAGS) -o $@

$(BUILD_DIR)/%.o: %.cpp | $(BUILD_DIR)
	mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/rrf_%.o: ../ReprapFirmware/src/%.cpp | $(BUILD_DIR)
	mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/rrflib_%.o: ../RRFLibraries/src/%.cpp | $(BUILD_DIR)
	mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)/src

clean:
	rm -rf $(BUILD_DIR)

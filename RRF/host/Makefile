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
    can/CanCapture.cpp \
    can/CanInterfaceHost.cpp \
    storage/HostFileStore.cpp \
    storage/HostMassStorage.cpp \
    platform/Heap.cpp \
    platform/StringHandle.cpp \
    platform/ArrayHandle.cpp \
    platform/SoftwareReset.cpp \
    platform/PlatformHost.cpp \
    platform/RepRapHost.cpp \
    platform/GCodesHost.cpp \
    gcodes/GCodeInputStubs.cpp \
    gcodes/GCodeFileInfoHost.cpp \
    networking/MacAddressHost.cpp \
    platform/GetFloatFormatStringHost.cpp \
    platform/TasksHost.cpp \
    platform/DebugPrintHost.cpp \
    object_model/ObjectModelHost.cpp \
	object_model/VariableHost.cpp \
	object_model/GlobalVariablesHost.cpp \
	object_model/ObjectExplorationContextHost.cpp \
	movement/MoveHost.cpp \
	movement/CartesianKinematicsHost.cpp \
	movement/DDAHost.cpp \
	movement/StepTimerHost.cpp \
	movement/DDARingHost.cpp
RRF_SRC := ../ReprapFirmware/src/Version.cpp \
    ../ReprapFirmware/src/Storage/CRC32.cpp \
    ../ReprapFirmware/src/Storage/CRC16.cpp \
    ../ReprapFirmware/src/GCodes/GCodeException.cpp \
	../ReprapFirmware/src/GCodes/GCodeMachineState.cpp \
	../ReprapFirmware/src/GCodes/GCodeBuffer/GCodeBuffer.cpp \
	../ReprapFirmware/src/GCodes/GCodeBuffer/StringParser.cpp \
	../ReprapFirmware/src/GCodes/GCodeBuffer/BinaryParser.cpp \
	../ReprapFirmware/src/GCodes/GCodeBuffer/ExpressionParser.cpp \
	../ReprapFirmware/src/CAN/CanMotion.cpp \
	../ReprapFirmware/src/libc/strptime.cpp \
	../ReprapFirmware/src/Movement/Kinematics/Kinematics.cpp \
	../ReprapFirmware/src/Movement/Kinematics/RoundBedKinematics.cpp \
	../ReprapFirmware/src/Movement/Kinematics/HangprinterKinematics.cpp

CANLIB_SRC := ../CANlib/src/CanMessageBuffer.cpp

RRFLIB_SRC := ../RRFLibraries/src/General/StringRef.cpp \
	../RRFLibraries/src/General/SafeVsnprintf.cpp \
	../RRFLibraries/src/General/StringFunctions.cpp \
	../RRFLibraries/src/General/Strnlen.cpp \
	../RRFLibraries/src/General/NamedEnum.cpp \
	../RRFLibraries/src/General/NumericConverter.cpp \
	../RRFLibraries/src/General/SafeStrtod.cpp \
	../RRFLibraries/src/General/IPAddress.cpp \
	../RRFLibraries/src/Math/Isqrt.cpp

BUILD_DIR := build

HOST_OBJ := $(addprefix $(BUILD_DIR)/,$(HOST_SRC:.cpp=.o))
RRF_OBJ := $(addprefix $(BUILD_DIR)/rrf_,$(RRF_SRC:../ReprapFirmware/src/%.cpp=%.o))
CANLIB_OBJ := $(addprefix $(BUILD_DIR)/canlib_,$(CANLIB_SRC:../CANlib/src/%.cpp=%.o))
RRFLIB_OBJ := $(addprefix $(BUILD_DIR)/rrflib_,$(RRFLIB_SRC:../RRFLibraries/src/%.cpp=%.o))
OBJ_FILES := $(HOST_OBJ) $(RRF_OBJ) $(CANLIB_OBJ) $(RRFLIB_OBJ)

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

$(BUILD_DIR)/canlib_%.o: ../CANlib/src/%.cpp | $(BUILD_DIR)
	mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/rrflib_%.o: ../RRFLibraries/src/%.cpp | $(BUILD_DIR)
	mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)/src

clean:
	rm -rf $(BUILD_DIR)

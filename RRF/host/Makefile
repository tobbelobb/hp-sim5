CXX := g++
CXXFLAGS := -std=gnu++17 -Wall -Wextra -pedantic \
	-Iinclude \
	-I../ReprapFirmware/src \
	-I../ReprapFirmware/src/Hardware/SAME70 \
	-I../RRFLibraries/src \
	-I../CANlib/src \
	-D__SAME70Q20B__ \
	-DDUET3_MB6HC=1 \
	-DSAME70=1 \
	-DUSE_SBC=0 \
	-DSUPPORT_CAN_EXPANSION=1 \
	-DSUPPORT_REMOTE_COMMANDS=0 \
	-DSUPPORT_PANELDUE_FLASH=0 \
	-DSUPPORT_LED_STRIPS=0 \
	-DSUPPORT_ACCELEROMETERS=0 \
	-DSUPPORT_LASER=0 \
	-DSUPPORT_IOBITS=0 \
	-DHAS_AUX_DEVICES=0 \
	-DHAS_MASS_STORAGE=0 \
	-DHAS_WIFI_NETWORKING=0 \
	-DHAS_W5500_NETWORKING=0 \
	-DHAS_SBC_INTERFACE=0 \
	-DSUPPORT_HTTP=0 \
	-DSUPPORT_FTP=0 \
	-DSUPPORT_TELNET=0
LDFLAGS :=
TARGET := host_rrf_bootstrap

HOST_SRC := src/main.cpp src/devices_stub.cpp
RRF_SRC := ../ReprapFirmware/src/Version.cpp

BUILD_DIR := build

HOST_OBJ := $(addprefix $(BUILD_DIR)/,$(HOST_SRC:.cpp=.o))
RRF_OBJ := $(BUILD_DIR)/rrf_Version.o
OBJ_FILES := $(HOST_OBJ) $(RRF_OBJ)

.PHONY: all clean

all: $(BUILD_DIR)/$(TARGET)

$(BUILD_DIR)/$(TARGET): $(OBJ_FILES)
	$(CXX) $(OBJ_FILES) $(LDFLAGS) -o $@

$(BUILD_DIR)/%.o: %.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/rrf_%.o: ../ReprapFirmware/src/%.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)/src

clean:
	rm -rf $(BUILD_DIR)

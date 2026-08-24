#!/usr/bin/env bash
# =============================================================================
# ROSCon 2025 Workshop: Pixhawk uXRCE-DDS Serial Diagnostic & Smoke Test
# =============================================================================
# Usage:
#   ./scripts/test_pixhawk_connection.sh [DEVICE_PORT] [BAUD_RATE]
# Default:
#   Device: /dev/ttyAMA0 (or /dev/ttyACM0 if auto-detected)
#   Baud:   921600
# =============================================================================

set -e

# ANSI Color Codes
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Auto-detect default port if not provided
DEFAULT_PORT="/dev/ttyAMA0"
if [ ! -e "/dev/ttyAMA0" ] && [ -e "/dev/ttyACM0" ]; then
    DEFAULT_PORT="/dev/ttyACM0"
elif [ ! -e "/dev/ttyAMA0" ] && [ -e "/dev/ttyUSB0" ]; then
    DEFAULT_PORT="/dev/ttyUSB0"
fi

SERIAL_PORT="${1:-${DEFAULT_PORT}}"
BAUD_RATE="${2:-921600}"

echo -e "${CYAN}=======================================================================${NC}"
echo -e "${CYAN}        PIXHAWK ↔ COMPANION uXRCE-DDS SERIAL DIAGNOSTIC TOOL           ${NC}"
echo -e "${CYAN}=======================================================================${NC}"
echo -e " Target Serial Port: ${YELLOW}${SERIAL_PORT}${NC}"
echo -e " Baud Rate:          ${YELLOW}${BAUD_RATE}${NC}"
echo -e "${CYAN}-----------------------------------------------------------------------${NC}"

# 1. Check if Serial Device exists
echo -n "[1/4] Checking device file existence... "
if [ -e "${SERIAL_PORT}" ]; then
    echo -e "${GREEN}FOUND (${SERIAL_PORT})${NC}"
else
    echo -e "${RED}NOT FOUND${NC}"
    echo -e "${YELLOW}[!] Warning: Device ${SERIAL_PORT} does not exist.${NC}"
    echo -e "    - On Raspberry Pi: Check GPIO UART pins & /boot/config.txt (enable_uart=1)"
    echo -e "    - On Host: Check Pixhawk USB connection (ls /dev/ttyACM*)"
    exit 1
fi

# 2. Check User Permissions (dialout group)
echo -n "[2/4] Checking serial port permissions... "
if [ -r "${SERIAL_PORT}" ] && [ -w "${SERIAL_PORT}" ]; then
    echo -e "${GREEN}OK (Read/Write accessible)${NC}"
else
    echo -e "${RED}PERMISSION DENIED${NC}"
    echo -e "${YELLOW}[!] Current user does not have permission to access ${SERIAL_PORT}.${NC}"
    echo -e "    Fix with: ${CYAN}sudo usermod -a -G dialout \$USER${NC} (and re-login)"
    exit 1
fi

# 3. Check if MicroXRCEAgent is installed
echo -n "[3/4] Checking MicroXRCEAgent binary... "
if command -v MicroXRCEAgent &> /dev/null; then
    AGENT_BIN="MicroXRCEAgent"
    echo -e "${GREEN}FOUND (${AGENT_BIN})${NC}"
elif [ -f "/usr/local/bin/MicroXRCEAgent" ]; then
    AGENT_BIN="/usr/local/bin/MicroXRCEAgent"
    echo -e "${GREEN}FOUND (${AGENT_BIN})${NC}"
elif [ -f "/home/ubuntu/px4_ros_ws/install/micro_xrce_dds_agent/bin/MicroXRCEAgent" ]; then
    AGENT_BIN="/home/ubuntu/px4_ros_ws/install/micro_xrce_dds_agent/bin/MicroXRCEAgent"
    echo -e "${GREEN}FOUND (${AGENT_BIN})${NC}"
else
    echo -e "${RED}NOT FOUND in PATH${NC}"
    echo -e "${YELLOW}[!] Sourcing ROS 2 workspaces to locate MicroXRCEAgent...${NC}"
    if [ -f "/opt/ros/humble/setup.bash" ]; then source /opt/ros/humble/setup.bash; fi
    if [ -f "/home/ubuntu/px4_ros_ws/install/setup.bash" ]; then source /home/ubuntu/px4_ros_ws/install/setup.bash; fi
    
    if command -v MicroXRCEAgent &> /dev/null; then
        AGENT_BIN="MicroXRCEAgent"
        echo -e "${GREEN}FOUND after sourcing${NC}"
    else
        echo -e "${RED}MicroXRCEAgent could not be found. Please ensure workspace is sourced.${NC}"
        exit 1
    fi
fi

# 4. Run Live Handshake Verification
echo -e "${CYAN}-----------------------------------------------------------------------${NC}"
echo -e "[4/4] Starting MicroXRCEAgent test session for 6 seconds..."
echo -e "      ${CYAN}Running: ${AGENT_BIN} serial --dev ${SERIAL_PORT} -b ${BAUD_RATE} -v 3${NC}"
echo -e "${CYAN}-----------------------------------------------------------------------${NC}"

# Source ROS 2 environment for topic inspection
if [ -f "/opt/ros/humble/setup.bash" ]; then source /opt/ros/humble/setup.bash; fi
if [ -f "/home/ubuntu/px4_ros_ws/install/setup.bash" ]; then source /home/ubuntu/px4_ros_ws/install/setup.bash; fi

# Launch Agent in background
${AGENT_BIN} serial --dev "${SERIAL_PORT}" -b "${BAUD_RATE}" -v 3 > /tmp/micro_xrce_test.log 2>&1 &
AGENT_PID=$!

cleanup() {
    if kill -0 "${AGENT_PID}" 2>/dev/null; then
        kill -SIGINT "${AGENT_PID}" 2>/dev/null || true
        wait "${AGENT_PID}" 2>/dev/null || true
    fi
}
trap cleanup EXIT

# Monitor log for Session Created / Subscribed
SUCCESS=0
for i in {1..12}; do
    sleep 0.5
    if grep -q "Session created" /tmp/micro_xrce_test.log 2>/dev/null || grep -q "Subscribed topic" /tmp/micro_xrce_test.log 2>/dev/null; then
        SUCCESS=1
        break
    fi
done

echo -e "\n${CYAN}--- AGENT LOG OUTPUT (Sample) ---${NC}"
head -n 25 /tmp/micro_xrce_test.log || true
echo -e "${CYAN}---------------------------------${NC}"

if [ "${SUCCESS}" -eq 1 ]; then
    echo -e "\n${GREEN}=======================================================================${NC}"
    echo -e "${GREEN}  [SUCCESS] PIXHAWK ↔ COMPANION uXRCE-DDS LINK VERIFIED SUCCESSFULLY!  ${NC}"
    echo -e "${GREEN}=======================================================================${NC}"
    echo -e "  ✓ Serial Link @ ${BAUD_RATE} baud is active"
    echo -e "  ✓ Pixhawk FMU uORB messages are translating to ROS 2 topics"
    exit 0
else
    echo -e "\n${RED}=======================================================================${NC}"
    echo -e "${RED}  [FAILED] NO RESPONSE RECEIVED FROM PIXHAWK                           ${NC}"
    echo -e "${RED}=======================================================================${NC}"
    echo -e "  Suggestions to troubleshoot:"
    echo -e "  1. Verify Pixhawk Parameter: ${YELLOW}UXRCE_DDS_CFG = 102${NC} (TELEM 2)"
    echo -e "  2. Verify Pixhawk Parameter: ${YELLOW}SER_TEL2_BAUD = 921600${NC}"
    echo -e "  3. Check UART Wiring: TX ↔ RX, RX ↔ TX, and ${YELLOW}GND ↔ GND${NC}"
    echo -e "  4. Ensure Pixhawk is powered and booted."
    exit 1
fi

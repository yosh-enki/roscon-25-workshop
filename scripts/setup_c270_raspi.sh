#!/usr/bin/env bash
# ==============================================================================
# 1-Click Setup Script: Logitech C270 USB Camera for Raspberry Pi 4 / ROS 2
# ==============================================================================
set -e

GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${CYAN}=======================================================================${NC}"
echo -e "${CYAN} 📷 1-Click Setup: Logitech C270 USB Camera for Raspberry Pi 4 (ROS 2) ${NC}"
echo -e "${CYAN}=======================================================================${NC}"

# 1. ติดตั้ง Dependencies
echo -e "${YELLOW}[1/3] Installing ROS 2 usb_cam & v4l-utils...${NC}"
apt-get update
apt-get install -y ros-humble-usb-cam v4l-utils

# 2. ตั้งค่าไฟล์ Calibration สำหรับ C270 ที่ 720p
echo -e "${YELLOW}[2/3] Setting up C270 720p Calibration...${NC}"
mkdir -p /root/.ros/camera_info
mkdir -p /home/ubuntu/.ros/camera_info 2>/dev/null || true

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
CALIB_SOURCE="${SCRIPT_DIR}/../full_self_driving/config/camera_calibrations/c270_720p.yaml"

if [ -f "${CALIB_SOURCE}" ]; then
    cp "${CALIB_SOURCE}" /root/.ros/camera_info/c270.yaml
    cp "${CALIB_SOURCE}" /home/ubuntu/.ros/camera_info/c270.yaml 2>/dev/null || true
else
    cat << 'EOF' | tee /root/.ros/camera_info/c270.yaml > /home/ubuntu/.ros/camera_info/c270.yaml 2>/dev/null || true
image_width: 1280
image_height: 720
camera_name: c270
camera_matrix:
  rows: 3
  cols: 3
  data: [1000.0, 0.0, 640.0, 0.0, 1000.0, 360.0, 0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.0, 0.0, 0.0, 0.0, 0.0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [1000.0, 0.0, 640.0, 0.0, 0.0, 1000.0, 360.0, 0.0, 0.0, 0.0, 1.0, 0.0]
EOF
fi

# 3. ตรวจจับกล้อง C270 บน USB
echo -e "${YELLOW}[3/3] Detecting USB Camera devices...${NC}"
if ls /dev/video* 1> /dev/null 2>&1; then
    echo -e "${GREEN}✔ Video devices found:${NC}"
    ls -la /dev/v4l/by-id/ 2>/dev/null || ls -la /dev/video*
else
    echo -e "${YELLOW}⚠ Warning: No /dev/video* devices detected yet. Make sure USB camera is plugged in.${NC}"
fi

echo ""
echo -e "${GREEN}=======================================================================${NC}"
echo -e "${GREEN} ✔ Logitech C270 Setup Complete! Ready for Autonomous Flight (FSD)     ${NC}"
echo -e "${GREEN}=======================================================================${NC}"
echo "To test C270 camera & perception in 1 command, run:"
echo "  ./scripts/test_c270_perception.sh"

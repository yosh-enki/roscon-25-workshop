#!/usr/bin/env bash
# ==============================================================================
# 1-Click Test Script: Run C270 Camera + ArUco Perception Test in Background
# ==============================================================================
set -e

DEVICE="${1:-/dev/video0}"
MARKER_ID="${2:-1}"
DICTIONARY="${3:-DICT_4X4_50}"

GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Auto-detect C270 by-id if /dev/video0 is not present
if [ ! -e "$DEVICE" ]; then
    BY_ID=$(ls /dev/v4l/by-id/usb-046d_0825*-video-index0 2>/dev/null | head -n 1 || true)
    if [ -n "$BY_ID" ]; then
        DEVICE="$BY_ID"
    fi
fi

echo -e "${CYAN}=======================================================================${NC}"
echo -e "${CYAN} 🧪 Testing Logitech C270 + ArUco Perception (Marker ID: ${MARKER_ID}) ${NC}"
echo -e "${CYAN} Device: ${DEVICE} | Dictionary: ${DICTIONARY} ${NC}"
echo -e "${CYAN}=======================================================================${NC}"

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Auto-source ROS 2 & workspace overlay
source /opt/ros/humble/setup.bash 2>/dev/null || true

for setup_cand in \
    "${SCRIPT_DIR}/../../install/setup.bash" \
    "${SCRIPT_DIR}/../install/setup.bash" \
    "/home/ubuntu/roscon-25-workshop_ws/install/setup.bash" \
    "/home/ubuntu/px4_ros_ws/install/setup.bash" \
    "$(pwd)/install/setup.bash"; do
    if [ -f "$setup_cand" ]; then
        source "$setup_cand" 2>/dev/null || true
    fi
done

# Function to clean up background processes on Ctrl+C
cleanup() {
    echo -e "\n${YELLOW}Stopping test nodes...${NC}"
    kill $(jobs -p) 2>/dev/null || true
    echo -e "${GREEN}✔ Stopped cleanly.${NC}"
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

# 1. Start usb_cam in background
echo -e "${YELLOW}[1/2] Launching usb_cam (1280x720 @ 30 FPS mjpeg2rgb)...${NC}"
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:="${DEVICE}" \
  -p image_width:=1280 \
  -p image_height:=720 \
  -p pixel_format:="mjpeg2rgb" \
  -p camera_name:="c270" \
  -p frame_id:="camera_frame" \
  -r image_raw:=/camera \
  -r camera_info:=/camera_info &

sleep 2

# 2. Start fsd_perception in foreground
echo -e "${YELLOW}[2/2] Launching fsd_perception node...${NC}"
echo -e "${GREEN}👉 Point camera at ArUco Marker ID ${MARKER_ID} (${DICTIONARY}) to test 3D lock!${NC}"
echo -e "${CYAN}Press Ctrl+C to stop test.${NC}\n"

ros2 run full_self_driving fsd_perception --ros-args \
  -p dictionary:="${DICTIONARY}" \
  -p marker_size:=0.20 \
  -p selected_marker_id:="${MARKER_ID}" \
  -p selected_namespace:="aavc2026" \
  -p autostart:=true

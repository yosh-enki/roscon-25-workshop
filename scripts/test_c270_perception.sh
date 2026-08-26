#!/usr/bin/env bash
# ==============================================================================
# Production-Grade C270 Camera & Perception Test Runner
# Supports both:
#   1. Standalone Camera FPS test (No cbuild needed): ./scripts/test_c270_perception.sh --fps
#   2. Full ArUco Perception test: ./scripts/test_c270_perception.sh [DEVICE] [ID] [DICT]
# ==============================================================================
set -e

GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BOLD='\033[1m'
NC='\033[0m'

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# 1. Source ROS 2 & workspace overlay
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

# 2. Parse arguments
FPS_ONLY=false
if [ "$1" == "--fps" ] || [ "$1" == "-f" ] || [ "$1" == "--fps-only" ]; then
    FPS_ONLY=true
    shift || true
fi

DEVICE="${1:-}"
MARKER_ID="${2:-1}"
DICTIONARY="${3:-DICT_4X4_50}"

# 3. Auto-detect Logitech C270 if DEVICE is not provided or invalid
if [ -z "$DEVICE" ] || [ ! -e "$DEVICE" ]; then
    # Try finding via /dev/v4l/by-id/
    BY_ID=$(ls /dev/v4l/by-id/*046d*video-index0 2>/dev/null | head -n 1 || true)
    if [ -n "$BY_ID" ] && [ -e "$BY_ID" ]; then
        DEVICE=$(readlink -f "$BY_ID")
    fi
fi

if [ -z "$DEVICE" ] || [ ! -e "$DEVICE" ]; then
    # Scan /dev/video0..9 for uvcvideo driver
    for dev in /dev/video0 /dev/video1 /dev/video2 /dev/video3 /dev/video4; do
        if [ -e "$dev" ]; then
            DRIVER_NAME=$(v4l2-ctl -d "$dev" --info 2>/dev/null | grep -i "Driver name" || true)
            if [[ "$DRIVER_NAME" == *"uvcvideo"* ]]; then
                DEVICE="$dev"
                break
            fi
        fi
    done
fi

# Fallback default
if [ -z "$DEVICE" ]; then
    DEVICE="/dev/video0"
fi

# 4. Strict Hardware Presence Check
if [ ! -e "$DEVICE" ]; then
    echo -e "${RED}${BOLD}=======================================================================${NC}"
    echo -e "${RED}${BOLD} ❌ ERROR: Camera device '${DEVICE}' not found in container!         ${NC}"
    echo -e "${RED}${BOLD}=======================================================================${NC}"
    echo -e "${YELLOW}Hardware Diagnostic:${NC}"
    if command -v lsusb &>/dev/null; then
        USB_DEV=$(lsusb | grep -i "046d:0825\|Logitech" || true)
        if [ -n "$USB_DEV" ]; then
            echo -e "${GREEN}  ✔ USB Bus detects camera:${NC} ${USB_DEV}"
            echo -e "${YELLOW}  ⚠ But /dev/video* is not mapped in this container session.${NC}"
            echo -e "  👉 ${BOLD}Fix:${NC} Exit container and run: ${CYAN}./run_raspi.sh reset && ./run_raspi.sh${NC}"
        else
            echo -e "${RED}  ✗ No Logitech USB device detected on USB bus.${NC}"
            echo -e "  👉 ${BOLD}Fix:${NC} Check physical USB cable connection on Raspberry Pi 4 (Blue port)."
        fi
    fi
    echo ""
    exit 1
fi

echo -e "${CYAN}=======================================================================${NC}"
echo -e "${CYAN} 📷 Logitech C270 Test Runner | Target Device: ${BOLD}${DEVICE}${NC}"
if [ "$FPS_ONLY" = true ]; then
    echo -e "${CYAN} Mode: Standalone 30 FPS Stream Test (Zero Compilation Needed)        ${NC}"
else
    echo -e "${CYAN} Mode: Camera Stream + ArUco Perception (Marker ID: ${MARKER_ID})             ${NC}"
fi
echo -e "${CYAN}=======================================================================${NC}"

# 5. Tune Camera Hardware Settings for Rock-Solid 30 FPS
if command -v v4l2-ctl &>/dev/null; then
    echo -e "${YELLOW}[1/3] Locking 30 FPS UVC Shutter & Hardware Gain...${NC}"
    # Disable 50Hz Anti-flicker throttle and dynamic framerate drop
    v4l2-ctl -d "$DEVICE" -c power_line_frequency=0 2>/dev/null || true
    v4l2-ctl -d "$DEVICE" -c exposure_dynamic_framerate=0 2>/dev/null || true
    # Manual exposure mode with fast shutter (60 = 6ms, well within 33ms budget for 30 FPS)
    v4l2-ctl -d "$DEVICE" -c auto_exposure=1 2>/dev/null || true
    v4l2-ctl -d "$DEVICE" -c exposure_time_absolute=60 2>/dev/null || true
    # Boost hardware sensor gain & brightness for clear image in normal indoor lighting
    v4l2-ctl -d "$DEVICE" -c gain=128 2>/dev/null || true
    v4l2-ctl -d "$DEVICE" -c brightness=135 2>/dev/null || true
    v4l2-ctl -d "$DEVICE" -c contrast=32 2>/dev/null || true
    v4l2-ctl -d "$DEVICE" -c white_balance_temperature_auto=1 2>/dev/null || true
fi

# 6. Cleanup handler
cleanup() {
    echo -e "\n${YELLOW}Stopping test processes...${NC}"
    killall -9 usb_cam_node_exe 2>/dev/null || true
    kill $(jobs -p) 2>/dev/null || true
    echo -e "${GREEN}✔ Stopped cleanly.${NC}"
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

# Kill stale instances
killall -9 usb_cam_node_exe 2>/dev/null || true
sleep 1

# 7. Launch usb_cam node
echo -e "${YELLOW}[2/3] Launching usb_cam (1280x720 @ 30 FPS, mjpeg2rgb, mmap)...${NC}"
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:="${DEVICE}" \
  -p image_width:=1280 \
  -p image_height:=720 \
  -p framerate:=30.0 \
  -p pixel_format:="mjpeg2rgb" \
  -p io_method:="mmap" \
  -p autoexposure:=false \
  -p autofocus:=false \
  -p camera_name:="c270" \
  -p frame_id:="camera_frame" \
  -r image_raw:=/camera \
  -r camera_info:=/camera_info >/dev/null 2>&1 &

CAM_PID=$!
sleep 2

if ! kill -0 "$CAM_PID" 2>/dev/null; then
    echo -e "${RED}❌ Failed to start usb_cam on ${DEVICE}.${NC}"
    exit 1
fi
echo -e "${GREEN}✔ usb_cam is running and publishing /camera @ 30 FPS.${NC}"

# 8. Execution Mode Handling
if [ "$FPS_ONLY" = true ]; then
    echo -e "\n${GREEN}⚡ Measuring live FPS on /camera (Ctrl+C to stop)...${NC}\n"
    exec ros2 topic hz /camera
fi

# Check if fsd_perception package is built
if ! ros2 pkg prefix full_self_driving &>/dev/null; then
    echo -e "\n${YELLOW}⚠ Package 'full_self_driving' is not built yet in this container.${NC}"
    echo -e "${GREEN}👉 Showing camera live FPS instead. (Run 'cbuild' if you want to test ArUco detection).${NC}\n"
    exec ros2 topic hz /camera
fi

echo -e "${YELLOW}[3/3] Launching fsd_perception node...${NC}"
echo -e "${GREEN}👉 Point camera at ArUco Marker ID ${MARKER_ID} (${DICTIONARY}) to test 3D lock!${NC}"
echo -e "${CYAN}Press Ctrl+C to stop test.${NC}\n"

ros2 run full_self_driving fsd_perception --ros-args \
  -p dictionary:="${DICTIONARY}" \
  -p marker_size:=0.40 \
  -p selected_marker_id:="${MARKER_ID}" \
  -p selected_namespace:="aavc2026" \
  -p autostart:=true

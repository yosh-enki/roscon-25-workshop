#!/usr/bin/env bash
set -e

# =============================================================================
# ROSCon 2025 Workshop: ESP32 + Servo Hardware-in-the-Loop (HITL) Runner
# =============================================================================

SERIAL_PORT="${1:-/dev/ttyACM0}"
WORLD="${2:-kmitl_airfield}"
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "======================================================================="
echo "  LAUNCHING ROSCON-25 HARDWARE-IN-THE-LOOP (HITL) DELIVERY SIMULATION  "
echo "======================================================================="
echo "  Workspace:    ${WORKSPACE_DIR}"
echo "  Serial Port:  ${SERIAL_PORT}"
echo "  Target World: ${WORLD}"
echo "======================================================================="

if [ -e "${SERIAL_PORT}" ]; then
    echo "[INFO] Physical device detected at ${SERIAL_PORT}."
    DEVICE_FLAG="--device=${SERIAL_PORT}:${SERIAL_PORT}"
else
    echo "[WARN] Physical device not detected at ${SERIAL_PORT}."
    echo "       Running with virtual/simulation fallback."
    DEVICE_FLAG=""
fi

# Run Docker container with X11 / Wayland forwarding and USB device passthrough
docker run --rm -it \
    --net=host \
    --ipc=host \
    --group-add dialout \
    ${DEVICE_FLAG} \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -e DISPLAY="${DISPLAY}" \
    -v "${WORKSPACE_DIR}:/home/ubuntu/roscon-25-workshop" \
    -w /home/ubuntu/roscon-25-workshop \
    dronecode/roscon-25-workshop:latest \
    bash -c 'source /opt/ros/humble/setup.bash && source /home/ubuntu/px4_ros_ws/install/setup.bash && source install/setup.bash && (python3 full_self_driving/scripts/esp32_gripper_bridge.py --ros-args -p port:="'"${SERIAL_PORT}"'" &) && ros2 launch full_self_driving full_self_driving.launch.py simulation:=true world:="'"${WORLD}"'" headless:=false payload_adapter:=px4_uorb_gripper_actuator'

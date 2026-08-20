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

# NVIDIA GPU Acceleration auto-detection
if command -v nvidia-smi &>/dev/null && docker run --rm --gpus all dronecode/roscon-25-workshop:latest nvidia-smi &>/dev/null; then
    echo "[INFO] NVIDIA RTX 4050 GPU detected. Enabling full GPU hardware acceleration."
    GPU_FLAGS="--gpus all -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all -e __NV_PRIME_RENDER_OFFLOAD=1 -e __GLX_VENDOR_LIBRARY_NAME=nvidia"
else
    GPU_FLAGS=""
fi

# Foxglove WebSocket Port Selection (Avoid conflict with VSCode port forward on 8765)
if ss -tulpn 2>/dev/null | grep -q ':8765 '; then
    FOXGLOVE_PORT="8766"
    echo "[INFO] Port 8765 is occupied on host (e.g. by VSCode). Auto-switching Foxglove port to ${FOXGLOVE_PORT}."
    echo "[INFO] 👉 In Foxglove Studio, connect to: ws://localhost:${FOXGLOVE_PORT}"
else
    FOXGLOVE_PORT="8765"
    echo "[INFO] 👉 In Foxglove Studio, connect to: ws://localhost:${FOXGLOVE_PORT}"
fi

# Run Docker container with X11 / Wayland forwarding, GPU passthrough, and USB device
docker run --rm -it \
    --net=host \
    --ipc=host \
    --group-add dialout \
    ${DEVICE_FLAG} \
    ${GPU_FLAGS} \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -e DISPLAY="${DISPLAY}" \
    -e QT_X11_NO_MITSHM=1 \
    -v "${WORKSPACE_DIR}:/home/ubuntu/roscon-25-workshop" \
    -w /home/ubuntu/roscon-25-workshop \
    dronecode/roscon-25-workshop:latest \
    bash -c 'source /opt/ros/humble/setup.bash && source /home/ubuntu/px4_ros_ws/install/setup.bash && source install/setup.bash && (python3 full_self_driving/scripts/esp32_gripper_bridge.py --ros-args -p port:="'"${SERIAL_PORT}"'" &) && ros2 launch full_self_driving full_self_driving.launch.py simulation:=true world:="'"${WORLD}"'" headless:=false payload_adapter:=px4_uorb_gripper_actuator foxglove_port:="'"${FOXGLOVE_PORT}"'"'

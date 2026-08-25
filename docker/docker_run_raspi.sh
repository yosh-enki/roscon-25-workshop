#!/usr/bin/env bash

# ==============================================================================
# ROSCon 2025 Workshop - Raspberry Pi 4 Runner Script
# Optimized for Pixhawk Hardware Integration (UART / USB / MicroXRCEAgent)
# ==============================================================================

set -e

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WORKSPACE_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

# Default Configurations
DEFAULT_DEVICE="/dev/ttyAMA0"
BAUDRATE="921600"
RUN_AGENT=false
DOCKER_IMAGE="dronecode/roscon-25-workshop:latest"
CONTAINER_NAME="px4-roscon-25-raspi"

# Display Help
show_help() {
    echo "Usage: ./docker_run_raspi.sh [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --dev <path>       Serial device path (Default: /dev/ttyAMA0)"
    echo "  --baud <rate>      Baud rate (Default: 921600)"
    echo "  --agent            Auto-start MicroXRCEAgent on container startup"
    echo "  --image <name>     Custom docker image name"
    echo "  --help, -h         Show this help message"
    echo ""
    echo "Examples:"
    echo "  ./docker_run_raspi.sh                       # Open interactive ROS 2 bash"
    echo "  ./docker_run_raspi.sh --agent               # Auto-launch MicroXRCEAgent"
    echo "  ./docker_run_raspi.sh --dev /dev/ttyACM0    # Use USB port instead of GPIO"
}

# Parse Arguments
SERIAL_DEV="${DEFAULT_DEVICE}"
while [[ $# -gt 0 ]]; do
    case $1 in
        --dev)
            SERIAL_DEV="$2"
            shift 2
            ;;
        --baud)
            BAUDRATE="$2"
            shift 2
            ;;
        --agent)
            RUN_AGENT=true
            shift
            ;;
        --image)
            DOCKER_IMAGE="$2"
            shift 2
            ;;
        --help|-h)
            show_help
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
done

echo "========================================================"
echo " 🚁 Launching ROSCon 2025 Environment for Raspberry Pi"
echo "========================================================"

# Check and setup permissions for serial device
if [ -e "$SERIAL_DEV" ]; then
    echo "✔ Found Serial Device: $SERIAL_DEV"
    sudo chmod 666 "$SERIAL_DEV" 2>/dev/null || true
else
    echo "⚠ Warning: $SERIAL_DEV not found on host!"
    echo "  Checking for alternative devices..."
    if [ -e "/dev/ttyACM0" ]; then
        echo "  -> Found /dev/ttyACM0 (USB Pixhawk), switching to it."
        SERIAL_DEV="/dev/ttyACM0"
        sudo chmod 666 "$SERIAL_DEV" 2>/dev/null || true
    elif [ -e "/dev/ttyAMA0" ]; then
        echo "  -> Found /dev/ttyAMA0 (GPIO UART), switching to it."
        SERIAL_DEV="/dev/ttyAMA0"
        sudo chmod 666 "$SERIAL_DEV" 2>/dev/null || true
    else
        echo "  ❌ No Pixhawk serial device found. Continuing without direct device map."
    fi
fi

# Build Docker Command
DOCKER_CMD="docker run -it --rm"
DOCKER_CMD="$DOCKER_CMD --name ${CONTAINER_NAME}"
DOCKER_CMD="$DOCKER_CMD --net=host"
DOCKER_CMD="$DOCKER_CMD --ipc=host"
DOCKER_CMD="$DOCKER_CMD --privileged"
DOCKER_CMD="$DOCKER_CMD --shm-size=1g"

# Mount devices if they exist
if [ -e "$SERIAL_DEV" ]; then
    DOCKER_CMD="$DOCKER_CMD --device=${SERIAL_DEV}:${SERIAL_DEV}"
fi

# Pass through video cameras if available (e.g. Raspberry Pi Camera / USB Cam)
for vdev in /dev/video*; do
    if [ -e "$vdev" ]; then
        DOCKER_CMD="$DOCKER_CMD --device=${vdev}:${vdev}"
    fi
done

# Pass through GUI if DISPLAY is set
if [ -n "$DISPLAY" ]; then
    DOCKER_CMD="$DOCKER_CMD -v /tmp/.X11-unix:/tmp/.X11-unix:ro -e DISPLAY=$DISPLAY"
fi

# Mount Workshop Workspace
DOCKER_CMD="$DOCKER_CMD -v ${WORKSPACE_ROOT}:/home/ubuntu/roscon-25-workshop_ws/src/roscon-25-workshop"
DOCKER_CMD="$DOCKER_CMD -w /home/ubuntu/roscon-25-workshop_ws"

# Set environment variables inside container (Add MicroXRCEAgent to PATH directly)
DOCKER_CMD="$DOCKER_CMD -e PATH=/home/ubuntu/px4_ros_ws/install/micro_xrce_dds_agent/bin:/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
DOCKER_CMD="$DOCKER_CMD -e RASPBERRY_PI=1"
DOCKER_CMD="$DOCKER_CMD -e PIXHAWK_DEV=${SERIAL_DEV}"
DOCKER_CMD="$DOCKER_CMD -e PIXHAWK_BAUD=${BAUDRATE}"

echo "Target Serial Device: ${SERIAL_DEV} @ ${BAUDRATE} baud"
echo "Starting container: ${DOCKER_IMAGE}..."
echo "========================================================"

if [ "$RUN_AGENT" = true ]; then
    echo "⚡ Auto-starting MicroXRCEAgent..."
    eval $DOCKER_CMD $DOCKER_IMAGE bash -c "\
        source /opt/ros/humble/setup.bash && \
        source /home/ubuntu/px4_ros_ws/install/setup.bash && \
        echo 'Starting MicroXRCEAgent on ${SERIAL_DEV} (${BAUDRATE})...' && \
        MicroXRCEAgent serial --dev ${SERIAL_DEV} -b ${BAUDRATE}"
else
    # Interactive bash session with auto-sourced environment
    eval $DOCKER_CMD $DOCKER_IMAGE bash -c "\
        if [ -f /home/ubuntu/px4_ros_ws/install/setup.bash ]; then \
            echo 'source /opt/ros/humble/setup.bash' >> /home/ubuntu/.bashrc; \
            echo 'source /home/ubuntu/px4_ros_ws/install/setup.bash' >> /home/ubuntu/.bashrc; \
        fi; \
        exec bash"
fi

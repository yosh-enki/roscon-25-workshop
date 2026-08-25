#!/usr/bin/env bash

# ==============================================================================
# ROSCon 2025 Workshop - Raspberry Pi 4 Runner Script (Enhanced Edition)
# Optimized for Pixhawk Hardware Integration (UART / USB / MicroXRCEAgent)
# ==============================================================================

set -e

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WORKSPACE_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

# Default Configurations
DEFAULT_DEVICE="/dev/ttyAMA0"
BAUDRATE="921600"
DOCKER_IMAGE="dronecode/roscon-25-workshop:latest"
CONTAINER_NAME="px4-roscon-25-raspi"

# Colors
GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Display Help
show_help() {
    echo -e "${CYAN}========================================================${NC}"
    echo -e "${CYAN} 🚁 ROSCon 2025 - Raspberry Pi 4 Container Manager${NC}"
    echo -e "${CYAN}========================================================${NC}"
    echo "Usage: ./run_raspi.sh [COMMAND / OPTIONS]"
    echo ""
    echo "Commands:"
    echo "  (default)          Run interactive container"
    echo "  start, -d, --bg    Start MicroXRCEAgent in BACKGROUND (Daemon Mode)"
    echo "  exec, shell        Open a new shell inside the running container"
    echo "  logs, -f           View real-time logs from the background Agent"
    echo "  status             Check status of the container and connection"
    echo "  stop               Stop the running container"
    echo ""
    echo "Options:"
    echo "  --agent            Run MicroXRCEAgent in FOREGROUND (Interactive)"
    echo "  --dev <path>       Serial device path (Default: /dev/ttyAMA0)"
    echo "  --baud <rate>      Baud rate (Default: 921600)"
    echo "  --image <name>     Custom docker image name"
    echo "  --help, -h         Show this help message"
    echo ""
    echo "Examples:"
    echo "  ./run_raspi.sh start      # 🚀 Start Agent in background"
    echo "  ./run_raspi.sh exec       # 💻 Open terminal to run ROS 2 commands"
    echo "  ./run_raspi.sh logs       # 📜 View Pixhawk communication logs"
    echo "  ./run_raspi.sh stop       # 🛑 Stop all"
}

# Subcommands Handling
ACTION="interactive"
SERIAL_DEV="${DEFAULT_DEVICE}"

while [[ $# -gt 0 ]]; do
    case $1 in
        start|-d|--daemon|--bg)
            ACTION="daemon"
            shift
            ;;
        exec|shell)
            ACTION="exec"
            shift
            ;;
        logs|-f)
            ACTION="logs"
            shift
            ;;
        status)
            ACTION="status"
            shift
            ;;
        stop)
            ACTION="stop"
            shift
            ;;
        --agent)
            ACTION="agent_foreground"
            shift
            ;;
        --dev)
            SERIAL_DEV="$2"
            shift 2
            ;;
        --baud)
            BAUDRATE="$2"
            shift 2
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
            echo -e "${RED}Unknown option: $1${NC}"
            show_help
            exit 1
            ;;
    esac
done

# Action: STOP
if [ "$ACTION" = "stop" ]; then
    echo -e "${YELLOW}Stopping container ${CONTAINER_NAME}...${NC}"
    docker stop "${CONTAINER_NAME}" 2>/dev/null || true
    echo -e "${GREEN}✔ Stopped.${NC}"
    exit 0
fi

# Action: STATUS
if [ "$ACTION" = "status" ]; then
    if docker ps -q -f name="^/${CONTAINER_NAME}$" | grep -q .; then
        echo -e "${GREEN}✔ Container '${CONTAINER_NAME}' is RUNNING.${NC}"
        echo "Active ROS 2 Nodes / Topics can be checked via: ./run_raspi.sh exec"
    else
        echo -e "${YELLOW}Container '${CONTAINER_NAME}' is NOT running.${NC}"
    fi
    exit 0
fi

# Action: LOGS
if [ "$ACTION" = "logs" ]; then
    echo -e "${CYAN}Following logs from ${CONTAINER_NAME} (Ctrl+C to exit)...${NC}"
    exec docker logs -f "${CONTAINER_NAME}"
fi

# Action: EXEC (Open new terminal in existing container)
if [ "$ACTION" = "exec" ]; then
    if ! docker ps -q -f name="^/${CONTAINER_NAME}$" | grep -q .; then
        echo -e "${YELLOW}Container is not running. Starting interactive container instead...${NC}"
        ACTION="interactive"
    else
        echo -e "${GREEN}Connecting to running container '${CONTAINER_NAME}'...${NC}"
        exec docker exec -it "${CONTAINER_NAME}" bash --init-file <(cat << 'INITECHO'
if [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash; fi
if [ -f /home/ubuntu/px4_ros_ws/install/setup.bash ]; then source /home/ubuntu/px4_ros_ws/install/setup.bash; fi
if [ -f /home/ubuntu/roscon-25-workshop_ws/install/setup.bash ]; then source /home/ubuntu/roscon-25-workshop_ws/install/setup.bash; fi

# Convenient Aliases
alias agent="MicroXRCEAgent serial --dev $PIXHAWK_DEV -b $PIXHAWK_BAUD"
alias status="ros2 topic echo /fmu/out/vehicle_status_v1"
alias imu="ros2 topic echo /fmu/out/sensor_combined"
alias gps="ros2 topic echo /fmu/out/vehicle_global_position"
alias odom="ros2 topic echo /fmu/out/vehicle_odometry"
alias topics="ros2 topic list"
alias build="cd /home/ubuntu/roscon-25-workshop_ws && colcon build --symlink-install && source install/setup.bash"

echo "======================================================"
echo " 🚁 Connected to ROSCon 2025 Container"
echo "======================================================"
echo " Quick Aliases Ready:"
echo "   • topics : View all ROS 2 topics"
echo "   • status : Echo /fmu/out/vehicle_status_v1"
echo "   • imu    : Echo /fmu/out/sensor_combined"
echo "   • gps    : Echo /fmu/out/vehicle_global_position"
echo "   • build  : colcon build & source workspace"
echo "======================================================"
INITECHO
)
    fi
fi

# Detect Devices & Setup Permissions cleanly
if [ -e "$SERIAL_DEV" ]; then
    echo -e "${GREEN}✔ Found Serial Device: $SERIAL_DEV${NC}"
    if [ ! -r "$SERIAL_DEV" ] || [ ! -w "$SERIAL_DEV" ]; then
        sudo chmod 666 "$SERIAL_DEV" 2>/dev/null || true
    fi
else
    echo -e "${YELLOW}⚠ Warning: $SERIAL_DEV not found. Checking alternatives...${NC}"
    if [ -e "/dev/ttyACM0" ]; then
        echo -e "${GREEN}  -> Found /dev/ttyACM0 (USB Pixhawk), switching to it.${NC}"
        SERIAL_DEV="/dev/ttyACM0"
    elif [ -e "/dev/ttyAMA0" ]; then
        echo -e "${GREEN}  -> Found /dev/ttyAMA0 (GPIO UART), switching to it.${NC}"
        SERIAL_DEV="/dev/ttyAMA0"
    fi
    if [ -e "$SERIAL_DEV" ] && { [ ! -r "$SERIAL_DEV" ] || [ ! -w "$SERIAL_DEV" ]; }; then
        sudo chmod 666 "$SERIAL_DEV" 2>/dev/null || true
    fi
fi

# Build Docker Arguments
DOCKER_ARGS=(
    --name "${CONTAINER_NAME}"
    --net=host
    --ipc=host
    --privileged
    --shm-size=1g
    -v "${WORKSPACE_ROOT}:/home/ubuntu/roscon-25-workshop_ws/src/roscon-25-workshop"
    -w "/home/ubuntu/roscon-25-workshop_ws"
    -e "RASPBERRY_PI=1"
    -e "PIXHAWK_DEV=${SERIAL_DEV}"
    -e "PIXHAWK_BAUD=${BAUDRATE}"
    -e "PATH=/home/ubuntu/px4_ros_ws/install/micro_xrce_dds_agent/bin:/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
)

if [ -e "$SERIAL_DEV" ]; then
    DOCKER_ARGS+=(--device="${SERIAL_DEV}:${SERIAL_DEV}")
fi

for vdev in /dev/video*; do
    if [ -e "$vdev" ]; then
        DOCKER_ARGS+=(--device="${vdev}:${vdev}")
    fi
done

if [ -n "$DISPLAY" ]; then
    DOCKER_ARGS+=(-v /tmp/.X11-unix:/tmp/.X11-unix:ro -e DISPLAY="$DISPLAY")
fi

ENTRY_SETUP="source /opt/ros/humble/setup.bash 2>/dev/null || true; source /home/ubuntu/px4_ros_ws/install/setup.bash 2>/dev/null || true; source /home/ubuntu/roscon-25-workshop_ws/install/setup.bash 2>/dev/null || true"

# Clean up any leftover stopped container with same name
docker rm -f "${CONTAINER_NAME}" 2>/dev/null || true

echo -e "${CYAN}========================================================${NC}"
echo -e "${CYAN} 🚁 Starting ROSCon 2025 Container${NC}"
echo -e "${CYAN} Serial Device: ${SERIAL_DEV} @ ${BAUDRATE} baud${NC}"
echo -e "${CYAN}========================================================${NC}"

# Action: DAEMON (Background MicroXRCEAgent)
if [ "$ACTION" = "daemon" ]; then
    echo -e "${GREEN}Starting MicroXRCEAgent in BACKGROUND daemon mode...${NC}"
    docker run -d "${DOCKER_ARGS[@]}" "$DOCKER_IMAGE" bash -c "${ENTRY_SETUP}; echo 'Agent running in background...'; exec MicroXRCEAgent serial --dev ${SERIAL_DEV} -b ${BAUDRATE}"
    echo ""
    echo -e "${GREEN}✔ MicroXRCEAgent is now running in background!${NC}"
    echo "Useful commands:"
    echo "  • Open Terminal:  ./run_raspi.sh exec"
    echo "  • View Live Logs: ./run_raspi.sh logs"
    echo "  • Stop Service:   ./run_raspi.sh stop"
    exit 0
fi

# Action: FOREGROUND AGENT
if [ "$ACTION" = "agent_foreground" ]; then
    echo -e "${GREEN}⚡ Starting MicroXRCEAgent in foreground...${NC}"
    exec docker run -it --rm "${DOCKER_ARGS[@]}" "$DOCKER_IMAGE" bash -c "${ENTRY_SETUP}; MicroXRCEAgent serial --dev ${SERIAL_DEV} -b ${BAUDRATE}"
fi

# Action: INTERACTIVE SHELL (Default)
exec docker run -it --rm "${DOCKER_ARGS[@]}" "$DOCKER_IMAGE" bash --init-file <(cat << 'INITECHO'
if [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash; fi
if [ -f /home/ubuntu/px4_ros_ws/install/setup.bash ]; then source /home/ubuntu/px4_ros_ws/install/setup.bash; fi
if [ -f /home/ubuntu/roscon-25-workshop_ws/install/setup.bash ]; then source /home/ubuntu/roscon-25-workshop_ws/install/setup.bash; fi

alias agent="MicroXRCEAgent serial --dev $PIXHAWK_DEV -b $PIXHAWK_BAUD"
alias status="ros2 topic echo /fmu/out/vehicle_status_v1"
alias imu="ros2 topic echo /fmu/out/sensor_combined"
alias gps="ros2 topic echo /fmu/out/vehicle_global_position"
alias odom="ros2 topic echo /fmu/out/vehicle_odometry"
alias topics="ros2 topic list"
alias build="cd /home/ubuntu/roscon-25-workshop_ws && colcon build --symlink-install && source install/setup.bash"

echo "======================================================"
echo "  🚁 ROSCon 2025 Workshop Container Ready (Raspberry Pi)"
echo "======================================================"
echo " Quick Aliases Ready:"
echo "   • agent  : Start MicroXRCEAgent on $PIXHAWK_DEV"
echo "   • topics : View all ROS 2 topics"
echo "   • status : Echo /fmu/out/vehicle_status_v1"
echo "   • imu    : Echo /fmu/out/sensor_combined"
echo "   • build  : colcon build & source workspace"
echo "======================================================"
INITECHO
)

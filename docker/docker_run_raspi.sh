#!/usr/bin/env bash

# ==============================================================================
# ROSCon 2025 Workshop - Raspberry Pi 4 Runner Script (Enhanced Edition)
# Optimized for Pixhawk Hardware Integration (UART / USB / MicroXRCEAgent)
# ==============================================================================

set -e

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
if [ -d "${SCRIPT_DIR}/full_self_driving" ]; then
    WORKSPACE_ROOT="${SCRIPT_DIR}"
else
    WORKSPACE_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)
fi

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
    echo "  (default)          Connect to persistent container (interactive shell)"
    echo "  start, -d, --bg    Start MicroXRCEAgent in BACKGROUND (Daemon Mode)"
    echo "  exec, shell        Open a new shell inside the running container"
    echo "  logs, -f           View real-time logs from the background Agent"
    echo "  status             Check status of the container and connection"
    echo "  stop               Stop the running container (preserves build files)"
    echo "  reset, clean       Remove existing container completely to start fresh"
    echo ""
    echo "Options:"
    echo "  --agent            Run MicroXRCEAgent in FOREGROUND"
    echo "  --dev <path>       Serial device path (Default: /dev/ttyAMA0)"
    echo "  --baud <rate>      Baud rate (Default: 921600)"
    echo "  --image <name>     Custom docker image name"
    echo "  --help, -h         Show this help message"
    echo ""
    echo "Build Helper inside container:"
    echo "  cbuild             # Fast incremental build of full_self_driving (2 cores)"
    echo "  cbuild-all         # Build all packages in workspace (2 cores)"
    echo ""
    echo "Examples:"
    echo "  ./run_raspi.sh start      # 🚀 Start Agent in background"
    echo "  ./run_raspi.sh            # 💻 Open terminal (or ./run_raspi.sh exec)"
    echo "  ./run_raspi.sh logs       # 📜 View Pixhawk communication logs"
    echo "  ./run_raspi.sh stop       # 🛑 Pause container (builds preserved)"
    echo "  ./run_raspi.sh reset      # 🧹 Remove container to start fresh"
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
        reset|clean|--recreate)
            ACTION="reset"
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

# Action: RESET / CLEAN
if [ "$ACTION" = "reset" ]; then
    echo -e "${YELLOW}Removing container ${CONTAINER_NAME}...${NC}"
    docker rm -f "${CONTAINER_NAME}" 2>/dev/null || true
    echo -e "${GREEN}✔ Container removed. Next run will create a fresh container.${NC}"
    exit 0
fi

# Action: STOP
if [ "$ACTION" = "stop" ]; then
    echo -e "${YELLOW}Stopping container ${CONTAINER_NAME}...${NC}"
    docker stop "${CONTAINER_NAME}" 2>/dev/null || true
    echo -e "${GREEN}✔ Stopped. (Container and build files are preserved)${NC}"
    exit 0
fi

# Action: STATUS
if [ "$ACTION" = "status" ]; then
    if docker ps -q -f name="^/${CONTAINER_NAME}$" | grep -q .; then
        echo -e "${GREEN}✔ Container '${CONTAINER_NAME}' is RUNNING.${NC}"
        echo "Active ROS 2 Nodes / Topics can be checked via: ./run_raspi.sh exec"
    elif docker ps -a -q -f name="^/${CONTAINER_NAME}$" | grep -q .; then
        echo -e "${YELLOW}Container '${CONTAINER_NAME}' exists but is STOPPED.${NC}"
        echo "Resume it with: ./run_raspi.sh start  or  ./run_raspi.sh"
    else
        echo -e "${YELLOW}Container '${CONTAINER_NAME}' does not exist.${NC}"
    fi
    exit 0
fi

# Action: LOGS
if [ "$ACTION" = "logs" ]; then
    echo -e "${CYAN}Following logs from ${CONTAINER_NAME} (Ctrl+C to exit)...${NC}"
    exec docker logs -f "${CONTAINER_NAME}"
fi

# Detect Devices & Setup Permissions cleanly
if [ -e "$SERIAL_DEV" ]; then
    echo -e "${GREEN}✔ Found Serial Device: $SERIAL_DEV${NC}"
    sudo chmod 666 "$SERIAL_DEV" 2>/dev/null || true
    # Reset TTY flags to raw mode and disable hardware flow control
    sudo stty -F "$SERIAL_DEV" raw -echo -crtscts -ixon -ixoff "${BAUDRATE}" 2>/dev/null || true
    # Enable Linux Kernel low-latency serial mode (reduces 16ms buffer delay to <1ms)
    sudo setserial "$SERIAL_DEV" low_latency 2>/dev/null || true
else
    echo -e "${YELLOW}⚠ Warning: $SERIAL_DEV not found. Checking alternatives...${NC}"
    if [ -e "/dev/ttyACM0" ]; then
        echo -e "${GREEN}  -> Found /dev/ttyACM0 (USB Pixhawk), switching to it.${NC}"
        SERIAL_DEV="/dev/ttyACM0"
    elif [ -e "/dev/ttyAMA0" ]; then
        echo -e "${GREEN}  -> Found /dev/ttyAMA0 (GPIO UART), switching to it.${NC}"
        SERIAL_DEV="/dev/ttyAMA0"
    fi
    if [ -e "$SERIAL_DEV" ]; then
        sudo chmod 666 "$SERIAL_DEV" 2>/dev/null || true
        sudo stty -F "$SERIAL_DEV" raw -echo -crtscts -ixon -ixoff "${BAUDRATE}" 2>/dev/null || true
    fi
fi

# Optimization: Set CPU governor to performance (prevents throttling during flight)
if [ -d "/sys/devices/system/cpu/cpu0/cpufreq" ]; then
    echo -e "${GREEN}⚡ Optimizing CPU Governor to performance...${NC}"
    for gov in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
        if [ -w "$gov" ]; then
            echo performance > "$gov" 2>/dev/null || true
        else
            echo performance | sudo tee "$gov" >/dev/null 2>&1 || true
        fi
    done
fi

# Ensure high-speed RAM-based volatile storage exists for evidence & journaling (preserves SD card life)
mkdir -p /tmp/fsd_evidence /tmp/fsd_state /tmp/fsd_plans /tmp/fsd_backups 2>/dev/null || true
chmod 777 /tmp/fsd_evidence /tmp/fsd_state /tmp/fsd_plans /tmp/fsd_backups 2>/dev/null || true

# Build Docker Arguments (Always add dialout and video group permissions, and run as root)
DOCKER_ARGS=(
    --name "${CONTAINER_NAME}"
    --user root
    --net=host
    --ipc=host
    --privileged
    --group-add dialout
    --group-add video
    --shm-size=1g
    -v /dev:/dev
    -v "${WORKSPACE_ROOT}:/home/ubuntu/roscon-25-workshop_ws/src/roscon-25-workshop"
    -v "/tmp/fsd_evidence:/tmp/fsd_evidence"
    -v "/tmp/fsd_state:/tmp/fsd_state"
    -v "/tmp/fsd_plans:/tmp/fsd_plans"
    -v "/tmp/fsd_backups:/tmp/fsd_backups"
    -w "/home/ubuntu/roscon-25-workshop_ws"
    -e "RASPBERRY_PI=1"
    -e "PIXHAWK_DEV=${SERIAL_DEV}"
    -e "PIXHAWK_BAUD=${BAUDRATE}"
    -e "MAKEFLAGS=-j2"
    -e "CMAKE_BUILD_PARALLEL_LEVEL=2"
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

# Background daemon supervisor command that keeps container alive and auto-restarts Agent if needed
DAEMON_BOOTSTRAP="\
    source /opt/ros/humble/setup.bash 2>/dev/null || true; \
    source /home/ubuntu/px4_ros_ws/install/setup.bash 2>/dev/null || true; \
    source /home/ubuntu/roscon-25-workshop_ws/install/setup.bash 2>/dev/null || true; \
    export PATH=/home/ubuntu/px4_ros_ws/install/micro_xrce_dds_agent/bin:\$PATH; \
    echo '🚁 Starting MicroXRCEAgent Supervisor in background...'; \
    while true; do \
        if [ -e \"${SERIAL_DEV}\" ]; then \
            echo \"Connecting MicroXRCEAgent to ${SERIAL_DEV} @ ${BAUDRATE} baud...\"; \
            nice -n -10 MicroXRCEAgent serial --dev \"${SERIAL_DEV}\" -b \"${BAUDRATE}\" -v 3 || true; \
            echo 'MicroXRCEAgent stopped. Reconnecting in 2 seconds...'; \
        else \
            echo \"Waiting for serial device ${SERIAL_DEV}...\"; \
        fi; \
        sleep 2; \
    done \
"

# Inject executable helper commands and environment into container
ensure_container_setup() {
    docker exec --user root "${CONTAINER_NAME}" bash -c "
        cat << 'EOF' > /usr/local/bin/cbuild
#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash 2>/dev/null || true
source /home/ubuntu/px4_ros_ws/install/setup.bash 2>/dev/null || true
source /home/ubuntu/roscon-25-workshop_ws/install/setup.bash 2>/dev/null || true
export PATH=/home/ubuntu/px4_ros_ws/install/micro_xrce_dds_agent/bin:\$PATH
cd /home/ubuntu/roscon-25-workshop_ws
echo -e '\033[0;32m🔨 Building full_self_driving (2 workers, symlink-install)...\033[0m'
exec colcon build --packages-select full_self_driving --symlink-install --parallel-workers 2 \"\$@\"
EOF
        chmod +x /usr/local/bin/cbuild

        cat << 'EOF' > /usr/local/bin/cbuild-all
#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash 2>/dev/null || true
source /home/ubuntu/px4_ros_ws/install/setup.bash 2>/dev/null || true
source /home/ubuntu/roscon-25-workshop_ws/install/setup.bash 2>/dev/null || true
export PATH=/home/ubuntu/px4_ros_ws/install/micro_xrce_dds_agent/bin:\$PATH
cd /home/ubuntu/roscon-25-workshop_ws
echo -e '\033[0;32m🔨 Building all packages (2 workers, symlink-install)...\033[0m'
exec colcon build --symlink-install --parallel-workers 2 \"\$@\"
EOF
        chmod +x /usr/local/bin/cbuild-all

        cat << 'EOF' > /usr/local/bin/run-agent
#!/usr/bin/env bash
source /opt/ros/humble/setup.bash 2>/dev/null || true
source /home/ubuntu/px4_ros_ws/install/setup.bash 2>/dev/null || true
export PATH=/home/ubuntu/px4_ros_ws/install/micro_xrce_dds_agent/bin:\$PATH
exec nice -n -10 MicroXRCEAgent serial --dev \"\${PIXHAWK_DEV:-${SERIAL_DEV}}\" -b \"\${PIXHAWK_BAUD:-${BAUDRATE}}\" -v 3 \"\$@\"
EOF
        chmod +x /usr/local/bin/run-agent

        cat << 'EOF' > /etc/profile.d/roscon_setup.sh
source /opt/ros/humble/setup.bash 2>/dev/null || true
source /home/ubuntu/px4_ros_ws/install/setup.bash 2>/dev/null || true
source /home/ubuntu/roscon-25-workshop_ws/install/setup.bash 2>/dev/null || true
export PATH=/home/ubuntu/px4_ros_ws/install/micro_xrce_dds_agent/bin:\$PATH
EOF

        for rc in /root/.bashrc /home/ubuntu/.bashrc; do
            grep -q 'source /etc/profile.d/roscon_setup.sh' \$rc 2>/dev/null || echo 'source /etc/profile.d/roscon_setup.sh' >> \$rc
            grep -q 'source /opt/ros/humble/setup.bash' \$rc 2>/dev/null || echo 'source /opt/ros/humble/setup.bash' >> \$rc
            grep -q 'px4_ros_ws/install/setup.bash' \$rc 2>/dev/null || echo 'source /home/ubuntu/px4_ros_ws/install/setup.bash 2>/dev/null' >> \$rc
            grep -q 'roscon-25-workshop_ws/install/setup.bash' \$rc 2>/dev/null || echo 'source /home/ubuntu/roscon-25-workshop_ws/install/setup.bash 2>/dev/null' >> \$rc
        done

        # Ensure volatile storage and camera calibration are available inside container
        mkdir -p /tmp/fsd_evidence /tmp/fsd_state /tmp/fsd_plans /tmp/fsd_backups /root/.ros/camera_info /home/ubuntu/.ros/camera_info 2>/dev/null || true
        chmod 777 /tmp/fsd_evidence /tmp/fsd_state /tmp/fsd_plans /tmp/fsd_backups 2>/dev/null || true
        if [ -f "/home/ubuntu/roscon-25-workshop_ws/src/roscon-25-workshop/full_self_driving/config/camera_calibrations/c270_720p.yaml" ]; then
            cp "/home/ubuntu/roscon-25-workshop_ws/src/roscon-25-workshop/full_self_driving/config/camera_calibrations/c270_720p.yaml" /root/.ros/camera_info/c270.yaml 2>/dev/null || true
            cp "/home/ubuntu/roscon-25-workshop_ws/src/roscon-25-workshop/full_self_driving/config/camera_calibrations/c270_720p.yaml" /home/ubuntu/.ros/camera_info/c270.yaml 2>/dev/null || true
        fi
    " 2>/dev/null || true
}

echo -e "${CYAN}========================================================${NC}"
echo -e "${CYAN} 🚁 ROSCon 2025 Container Manager${NC}"
echo -e "${CYAN} Serial Device: ${SERIAL_DEV} @ ${BAUDRATE} baud${NC}"
echo -e "${CYAN}========================================================${NC}"

# Action: DAEMON (Background MicroXRCEAgent)
if [ "$ACTION" = "daemon" ]; then
    if docker ps -q -f name="^/${CONTAINER_NAME}$" | grep -q .; then
        echo -e "${GREEN}✔ Container '${CONTAINER_NAME}' is already running in background.${NC}"
        ensure_container_setup
    elif docker ps -a -q -f name="^/${CONTAINER_NAME}$" | grep -q .; then
        echo -e "${GREEN}Starting existing container '${CONTAINER_NAME}' (preserving build files)...${NC}"
        docker start "${CONTAINER_NAME}"
        ensure_container_setup
    else
        echo -e "${GREEN}Starting MicroXRCEAgent in BACKGROUND daemon mode (High Priority)...${NC}"
        docker run -d "${DOCKER_ARGS[@]}" "$DOCKER_IMAGE" /ros_entrypoint.sh bash -c "${DAEMON_BOOTSTRAP}"
        ensure_container_setup
    fi
    echo ""
    echo -e "${GREEN}✔ MicroXRCEAgent is active!${NC}"
    echo "Useful commands:"
    echo "  • Open Terminal:  ./run_raspi.sh exec  (or simply ./run_raspi.sh)"
    echo "  • View Live Logs: ./run_raspi.sh logs"
    echo "  • Stop Service:   ./run_raspi.sh stop"
    exit 0
fi

# Action: FOREGROUND AGENT
if [ "$ACTION" = "agent_foreground" ]; then
    echo -e "${GREEN}⚡ Starting MicroXRCEAgent in foreground (High Priority)...${NC}"
    if docker ps -q -f name="^/${CONTAINER_NAME}$" | grep -q .; then
        ensure_container_setup
        echo -e "${CYAN}Attaching MicroXRCEAgent to running container...${NC}"
        exec docker exec -it --user root "${CONTAINER_NAME}" /ros_entrypoint.sh bash -c "\
            source /home/ubuntu/px4_ros_ws/install/setup.bash 2>/dev/null || true && \
            export PATH=/home/ubuntu/px4_ros_ws/install/micro_xrce_dds_agent/bin:\$PATH && \
            nice -n -10 MicroXRCEAgent serial --dev ${SERIAL_DEV} -b ${BAUDRATE} -v 3"
    else
        # If stopped or not created, start persistent container
        docker rm -f "${CONTAINER_NAME}" 2>/dev/null || true
        docker run -d "${DOCKER_ARGS[@]}" "$DOCKER_IMAGE" /ros_entrypoint.sh bash -c "${DAEMON_BOOTSTRAP}"
        ensure_container_setup
        exec docker exec -it --user root "${CONTAINER_NAME}" /ros_entrypoint.sh bash -c "\
            source /home/ubuntu/px4_ros_ws/install/setup.bash 2>/dev/null || true && \
            export PATH=/home/ubuntu/px4_ros_ws/install/micro_xrce_dds_agent/bin:\$PATH && \
            nice -n -10 MicroXRCEAgent serial --dev ${SERIAL_DEV} -b ${BAUDRATE} -v 3"
    fi
fi

# Action: EXEC / INTERACTIVE SHELL
# 1. If container is already running -> connect directly
if docker ps -q -f name="^/${CONTAINER_NAME}$" | grep -q .; then
    echo -e "${GREEN}✔ Connecting to running container '${CONTAINER_NAME}'...${NC}"
    ensure_container_setup
    exec docker exec -it --user root -w /home/ubuntu/roscon-25-workshop_ws "${CONTAINER_NAME}" /ros_entrypoint.sh bash -i
fi

# 2. If container exists but is stopped -> start it (keeps all build/install artifacts)
if docker ps -a -q -f name="^/${CONTAINER_NAME}$" | grep -q .; then
    echo -e "${GREEN}✔ Resuming container '${CONTAINER_NAME}' (build artifacts intact)...${NC}"
    docker start "${CONTAINER_NAME}"
    ensure_container_setup
    exec docker exec -it --user root -w /home/ubuntu/roscon-25-workshop_ws "${CONTAINER_NAME}" /ros_entrypoint.sh bash -i
fi

# 3. If container does not exist -> create persistent daemon with MicroXRCEAgent and connect
echo -e "${GREEN}Creating persistent container '${CONTAINER_NAME}' with MicroXRCEAgent in background...${NC}"
docker run -d "${DOCKER_ARGS[@]}" "$DOCKER_IMAGE" /ros_entrypoint.sh bash -c "${DAEMON_BOOTSTRAP}"

ensure_container_setup
echo -e "${GREEN}✔ Container created and MicroXRCEAgent started in background.${NC}"
echo -e "${GREEN}Connecting to interactive terminal...${NC}"
exec docker exec -it --user root -w /home/ubuntu/roscon-25-workshop_ws "${CONTAINER_NAME}" /ros_entrypoint.sh bash -i

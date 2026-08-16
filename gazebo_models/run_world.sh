#!/usr/bin/env bash
#
# Launch a Gazebo world from this repository's gazebo_models/worlds/
# using the same environment as the standard workshop setup, so that the
# PX4 drone (e.g. x500_mono_cam_down) can be spawned afterwards.
#
# Usage:
#   ./gazebo_models/run_world.sh [world_name] [--headless]
#
# Examples:
#   ./gazebo_models/run_world.sh                 # kmitl_airfield, with GUI
#   ./gazebo_models/run_world.sh kmitl_airfield  # kmitl_airfield, with GUI
#   ./gazebo_models/run_world.sh kmitl_airfield --headless
#
set -e

# Default world (the first of this repo) / fall back to provided name
WORLD_NAME="${1:-kmitl_airfield}"
if [ "$2" = "--headless" ] || [ "$1" = "--headless" ]; then
    HEADLESS="-s"
    # if the user only passed the flag, reset the default world name
    if [ "$1" = "--headless" ]; then
        WORLD_NAME="kmitl_airfield"
    fi
else
    HEADLESS=""
fi

# Resolve the location of this script -> gazebo_models dir
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOTS="${SCRIPT_DIR}"
WORLD_FILE="${SCRIPT_DIR}/worlds/${WORLD_NAME}.sdf"

if [ ! -f "${WORLD_FILE}" ]; then
    echo "ERROR: world file not found: ${WORLD_FILE}" >&2
    echo "Available worlds:" >&2
    ls -1 "${SCRIPT_DIR}/worlds"/*.sdf 2>/dev/null || true
    exit 1
fi

# PX4-gazebo-models location (where the drone models live in the container)
GZ_MODELS_DIR="/home/ubuntu/PX4-gazebo-models"

if [ ! -d "${GZ_MODELS_DIR}/models" ]; then
    echo "ERROR: PX4 gazebo models not found at ${GZ_MODELS_DIR}" >&2
    exit 1
fi

# Resource path: the PX4 models (so the drone can be spawned later) plus the
# repo worlds dir (so the relative materials/textures of the world resolve).
export GZ_SIM_RESOURCE_PATH="${GZ_MODELS_DIR}/models:${SCRIPT_DIR}/worlds"

# Same server configuration used by the standard simulation-gazebo helper.
export GZ_SIM_SERVER_CONFIG_PATH="${GZ_MODELS_DIR}/server.config"

echo "> Launching gazebo simulation (world: ${WORLD_NAME})..."
echo "  GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}"
echo "  GZ_SIM_SERVER_CONFIG_PATH=${GZ_SIM_SERVER_CONFIG_PATH}"

# shellcheck disable=SC2086
exec gz sim -r "${WORLD_FILE}" ${HEADLESS}
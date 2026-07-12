#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FSD_WS="${ROOT_DIR}/WUTA-FSD/ros2_ws"
SIM_WS="${ROOT_DIR}/WUTA-SIM"
FSD_BUILD_SCRIPT="${FSD_WS}/build_ws.sh"

CLEAN=0
SKIP_BUILD=0
BUILD_ONLY=0
LAUNCH_ARGS=()

usage() {
  cat <<'EOF'
Usage: ./start_simulator.sh [options] [--] [ROS launch arguments]

Build WUTA-FSD, build the simulator overlay, then launch simulator_bringup.

Options:
  --clean       Clean both workspaces before building.
  --skip-build  Skip all builds and use the existing install spaces.
  --build-only  Build both workspaces without starting ROS nodes.
  --rviz        Start RViz2 with the default simulator visualization config.
  -h, --help    Show this help.

Examples:
  ./start_simulator.sh
  ./start_simulator.sh --rviz
  ./start_simulator.sh launch_fsd:=false
  ./start_simulator.sh track_file:=skidpad mission_mode:=skidpad
  ./start_simulator.sh --clean --build-only
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --clean)
      CLEAN=1
      shift
      ;;
    --skip-build)
      SKIP_BUILD=1
      shift
      ;;
    --build-only)
      BUILD_ONLY=1
      shift
      ;;
    --rviz)
      LAUNCH_ARGS+=("launch_rviz:=true")
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      LAUNCH_ARGS+=("$@")
      break
      ;;
    *)
      LAUNCH_ARGS+=("$1")
      shift
      ;;
  esac
done

if [[ ! -x "${FSD_BUILD_SCRIPT}" ]]; then
  echo "WUTA-FSD build script is missing or not executable:" >&2
  echo "  ${FSD_BUILD_SCRIPT}" >&2
  exit 1
fi

if [[ ! -f "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" ]]; then
  echo "ROS 2 setup file not found for ROS_DISTRO=${ROS_DISTRO:-humble}." >&2
  exit 1
fi

if [[ "${SKIP_BUILD}" -eq 0 ]]; then
  echo "[1/2] Building the complete WUTA-FSD workspace..."
  if [[ "${CLEAN}" -eq 1 ]]; then
    (cd "${FSD_WS}" && ./build_ws.sh --clean)
    rm -rf "${SIM_WS}/build" "${SIM_WS}/install" "${SIM_WS}/log"
    find "${SIM_WS}" -mindepth 2 -maxdepth 5 -type d \
      \( -name "build" -o -name "install" -o -name "log" -o -name "__pycache__" \) \
      -exec rm -rf {} + 2>/dev/null || true
    find "${FSD_WS}" -mindepth 3 -maxdepth 6 -type d -name "__pycache__" \
      -exec rm -rf {} + 2>/dev/null || true
  else
    (cd "${FSD_WS}" && ./build_ws.sh)
  fi

  set +u
  source "${FSD_WS}/install/setup.bash"
  set -u

  echo "[2/2] Building the simulator overlay..."
  (
    cd "${SIM_WS}"
    colcon build \
      --base-paths . \
      --symlink-install \
      --packages-up-to simulator_bringup
  )
fi

if [[ ! -f "${FSD_WS}/install/setup.bash" ]]; then
  echo "WUTA-FSD is not built: ${FSD_WS}/install/setup.bash is missing." >&2
  exit 1
fi

if [[ ! -f "${SIM_WS}/install/setup.bash" ]]; then
  echo "Simulator overlay is not built: ${SIM_WS}/install/setup.bash is missing." >&2
  exit 1
fi

set +u
source "${FSD_WS}/install/setup.bash"
source "${SIM_WS}/install/setup.bash"
set -u

if [[ "${BUILD_ONLY}" -eq 1 ]]; then
  echo "Build complete. Simulator launch skipped (--build-only)."
  exit 0
fi

echo "Starting simulator_bringup..."
exec ros2 launch simulator_bringup simulator.launch.py "${LAUNCH_ARGS[@]}"

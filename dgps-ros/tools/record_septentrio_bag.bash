#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Record a ROS 2 bag from the Septentrio GPS topics.

Usage:
  record_septentrio_bag.bash [options]

Options:
  --nmea-dev DEV       NMEA serial device passed to septentrio.launch.py
                       default: /dev/ttyACM2
  --rtcm-dev DEV       RTCM serial device passed to septentrio.launch.py
                       default: /dev/ttyACM3
  --rtcm-disabled      Launch with RTCM forwarding disabled
  --bag-dir DIR        Directory where bags are written
                       default: /home/dtc/bags
  --bag-prefix PREFIX  Output bag name prefix
                       default: septentrio
  --duration SEC       Stop recording after SEC seconds
  --no-launch          Only record topics; do not start septentrio_node
  --help               Show this help

Environment overrides:
  NMEA_DEV, RTCM_DEV, BAG_DIR, BAG_PREFIX, DURATION, NO_LAUNCH
EOF
}

nmea_dev="${NMEA_DEV:-/dev/ttyACM2}"
rtcm_dev="${RTCM_DEV:-/dev/ttyACM3}"
bag_dir="${BAG_DIR:-/home/dtc/bags}"
bag_prefix="${BAG_PREFIX:-septentrio}"
duration="${DURATION:-}"
no_launch="${NO_LAUNCH:-0}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --nmea-dev)
      nmea_dev="$2"
      shift 2
      ;;
    --rtcm-dev)
      rtcm_dev="$2"
      shift 2
      ;;
    --rtcm-disabled)
      rtcm_dev=""
      shift
      ;;
    --bag-dir)
      bag_dir="$2"
      shift 2
      ;;
    --bag-prefix)
      bag_prefix="$2"
      shift 2
      ;;
    --duration)
      duration="$2"
      shift 2
      ;;
    --no-launch)
      no_launch=1
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

source_if_present() {
  local setup_file="$1"
  if [[ -f "$setup_file" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "$setup_file"
    set -u
  fi
}

source_if_present /opt/ros/jazzy/setup.bash
source_if_present /home/dtc/ws/install/setup.bash
source_if_present install/setup.bash

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 was not found. Source ROS 2 before running this script." >&2
  exit 1
fi

mkdir -p "$bag_dir"
timestamp="$(date +%Y%m%d_%H%M%S)"
bag_out="${bag_dir}/${bag_prefix}_${timestamp}"

topics=(
  /sept/antenna1/fix
  /sept/antenna2/fix
  /sept/center/fix
  /sept/dfix
  /sept/heading
  /sept/orientation
  /sept/baseline_velocity
)

launch_pid=""
cleanup() {
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" >/dev/null 2>&1; then
    echo "Stopping Septentrio launch process..."
    kill -INT "$launch_pid" >/dev/null 2>&1 || true
    wait "$launch_pid" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

if [[ "$no_launch" != "1" ]]; then
  if [[ ! -e "$nmea_dev" ]]; then
    echo "NMEA device does not exist: $nmea_dev" >&2
    echo "Available ttyACM devices:" >&2
    ls -l /dev/ttyACM* >&2 2>/dev/null || true
    exit 1
  fi

  if [[ -n "$rtcm_dev" && ! -e "$rtcm_dev" ]]; then
    echo "RTCM device does not exist: $rtcm_dev" >&2
    echo "Use --rtcm-disabled if you only want to record NMEA-derived topics." >&2
    exit 1
  fi

  echo "Launching Septentrio node with nmea_dev=${nmea_dev}, rtcm_dev=${rtcm_dev:-<disabled>}"
  ros2 launch dgps septentrio.launch.py "nmea_dev:=${nmea_dev}" "rtcm_dev:=${rtcm_dev}" &
  launch_pid="$!"
  sleep 3
fi

echo "Recording bag: $bag_out"
echo "Topics:"
printf '  %s\n' "${topics[@]}"

if [[ -n "$duration" ]]; then
  timeout --signal=INT "$duration" ros2 bag record -o "$bag_out" "${topics[@]}"
else
  ros2 bag record -o "$bag_out" "${topics[@]}"
fi

#!/usr/bin/env bash
set -eo pipefail
package_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_dir="$(cd "$package_dir/../.." && pwd)"
source /opt/ros/humble/setup.bash
underlay="${BAI06_UNDERLAY:-/home/admin1/turtlebot5_github_ws/install/setup.bash}"
if [[ -f "$underlay" ]]; then
  source "$underlay"
fi

if [[ "${BAI06_SKIP_BUILD:-0}" != 1 ]]; then
  cd "$workspace_dir"
  colcon build --symlink-install --base-paths "$package_dir" \
    --packages-select bai06_turtlebot4
fi
source "$workspace_dir/install/setup.bash"
deps_dir="${BAI06_PYTHON_DEPS:-$workspace_dir/python_deps}"
if [[ ! -d "$deps_dir" && -d "$workspace_dir/../python_deps" ]]; then
  deps_dir="$workspace_dir/../python_deps"
fi
export PYTHONPATH="$deps_dir${PYTHONPATH:+:$PYTHONPATH}"
export MPLCONFIGDIR="$workspace_dir/../.matplotlib"
mkdir -p "$MPLCONFIGDIR"

export ROS_DOMAIN_ID="${BAI06_ROS_DOMAIN_ID:-$((110 + $$ % 100))}"
export ROS_LOCALHOST_ONLY=1
export IGN_PARTITION="${BAI06_IGN_PARTITION:-bai06_$(date +%Y%m%d_%H%M%S)_$$}"
export DISPLAY="${DISPLAY:-:0}"
run_dir="${BAI06_OUTPUT_DIR:-$workspace_dir/../runs/$(date +%Y%m%d_%H%M%S)}"
model="${BAI06_MODEL:-$package_dir/models/gesture_recognizer.task}"
mkdir -p "$run_dir"
if [[ ! -f "$model" ]]; then
  printf 'Missing MediaPipe model: %s\n' "$model" >&2
  exit 1
fi
printf 'Run: %s\nROS_DOMAIN_ID=%s IGN_PARTITION=%s\n' "$run_dir" "$ROS_DOMAIN_ID" "$IGN_PARTITION"
printf 'The launch starts webcam/behavior nodes and sends /undock after Gazebo is ready.\n'
printf 'Ctrl+C closes this demo session.\n'
python3 "$package_dir/scripts/arrange_windows.py" &
arranger_pid=$!
cleanup() {
  kill "$arranger_pid" 2>/dev/null || true
  wait "$arranger_pid" 2>/dev/null || true
}
trap cleanup EXIT
ros2 launch bai06_turtlebot4 demo.launch.py \
  output_dir:="$run_dir" model_path:="$model" "$@" 2>&1 | tee "$run_dir/launch.log"

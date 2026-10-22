#!/usr/bin/env bash
set -eo pipefail
package_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_dir="$(cd "$package_dir/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$workspace_dir/install/setup.bash"
deps_dir="${BAI06_PYTHON_DEPS:-$workspace_dir/../python_deps}"
export PYTHONPATH="$deps_dir${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID="${BAI06_CAMERA_DOMAIN:-229}"
export ROS_LOCALHOST_ONLY=1
run_dir="$workspace_dir/../runs/wave_camera_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$run_dir"
printf 'Camera test: no Gazebo or behavior node; /detected_gesture is disabled.\n'
printf 'Open five fingers, then wiggle at least three fingers twice. Ctrl+C closes.\n'
ros2 run bai06_turtlebot4 gesture_recognition_node --ros-args \
  --params-file "$package_dir/config/gestures.yaml" \
  -p camera_test_only:=true -p model_path:="$package_dir/models/gesture_recognizer.task" \
  -p output_dir:="$run_dir" "$@"

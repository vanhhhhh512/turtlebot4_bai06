#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
underlay="${BAI06_UNDERLAY:-/home/admin1/turtlebot5_github_ws/install/setup.bash}"
if [[ -f "$underlay" ]]; then
  source "$underlay"
fi
package_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_dir="$(cd "$package_dir/../.." && pwd)"
source "$workspace_dir/install/setup.bash"
: "${ROS_DOMAIN_ID:?Set the ROS_DOMAIN_ID shown by run_demo.sh}"
ros2 action send_goal /undock irobot_create_msgs/action/Undock '{}'

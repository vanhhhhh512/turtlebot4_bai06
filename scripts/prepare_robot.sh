#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
underlay="${BAI06_UNDERLAY:-/home/admin1/turtlebot5_github_ws/install/setup.bash}"
if [[ -f "$underlay" ]]; then
  source "$underlay"
fi
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)/install/setup.bash"
: "${ROS_DOMAIN_ID:?Set the ROS_DOMAIN_ID shown by run_demo.sh}"
ros2 action send_goal /undock irobot_create_msgs/action/Undock '{}'

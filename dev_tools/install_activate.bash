#!/bin/bash
SCRIPT_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
# set -e -o pipefail
# . /opt/ros/jazzy/setup.bash
# . /opt/ros/rolling/setup.bash
# . ~/rclpy_ws/install/setup.bash
cd $SCRIPT_DIR/..
uv venv --system-site-packages --clear
uv pip install -e .[dev,zenoh]
. .venv/bin/activate
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

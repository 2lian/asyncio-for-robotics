#!/bin/bash
SCRIPT_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
set -e -o pipefail
. /opt/ros/jazzy/setup.bash
cd $SCRIPT_DIR/..
uv venv --system-site-packages # --clear
uv pip install -e .[jazzy,dev,zenoh]
. .venv/bin/activate
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp


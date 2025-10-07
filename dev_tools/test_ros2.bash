#!/bin/bash
SCRIPT_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
set -e -o pipefail
source ~/rclpy_ws/install/setup.bash
cd $SCRIPT_DIR/..
uv venv --system-site-packages --clear
uv pip install -e .[dev,zenoh]
. .venv/bin/activate

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 -m pytest ./tests \
    -vv \
    -x \
    # -m only \
    # -s \


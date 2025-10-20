#!/bin/bash
SCRIPT_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
set -e -o pipefail
. /opt/ros/jazzy/setup.bash
cd $SCRIPT_DIR/..
uv venv --system-site-packages --clear
uv pip install -e .[dev,zenoh] # we have install everything because pytest skip is broken in ros
. .venv/bin/activate
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
uv run pytest ./tests \
    --continue-on-collection-errors \
    -v \
    -x \
    # -m only \
    # -s \


#!/bin/bash
SCRIPT_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
set -e -o pipefail

cd $SCRIPT_DIR/..
uv venv --clear
uv pip install -e .[dev,zenoh]
. .venv/bin/activate
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 -m pytest ./tests \
    --continue-on-collection-errors \
    -v \
    -x \
    # -m only \
    # -s \


#!/bin/bash
SCRIPT_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
set -e -o pipefail
cd $SCRIPT_DIR/..
uv sync --group dev --extra zenoh
cd $SCRIPT_DIR
. ../.venv/bin/activate
cd $SCRIPT_DIR/..
python3 -m pytest ./tests \
    -x \
    -v \
    # -m only \
    # -s \

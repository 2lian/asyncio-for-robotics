#!/bin/bash
SCRIPT_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
set -e -o pipefail

cd $SCRIPT_DIR/..
pixi run -e dev python3 -m pytest ./tests/test_textio.py \
    -v \
    -s \
    -x \
    # -m only \

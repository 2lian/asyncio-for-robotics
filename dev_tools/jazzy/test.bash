#!/bin/bash
SCRIPT_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
set -e -o pipefail
cd $SCRIPT_DIR/../..
sudo docker build -t afor:jazzy -f $SCRIPT_DIR/Dockerfile .
sudo docker run -it --network host afor:jazzy uv run pytest ./tests -x

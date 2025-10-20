#!/bin/bash
SCRIPT_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
set -e -o pipefail
cd $SCRIPT_DIR/../..
sudo docker build -t afor:humble -f $SCRIPT_DIR/Dockerfile .
sudo docker run -it --network host afor:humble uv run pytest ./tests -x

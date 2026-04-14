#!/usr/bin/env bash

set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Prefer the system Python over Conda so ament/CMake resolves the ROS Python stack correctly.
export PATH="/usr/bin:/bin:/usr/sbin:/sbin:${PATH}"
unset PYTHONHOME
unset PYTHONPATH

source /opt/ros/humble/setup.bash

cd "${workspace_dir}"
colcon build "$@" --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

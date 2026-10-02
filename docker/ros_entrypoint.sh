#!/bin/bash
set -e

source "/opt/ros/humble/setup.bash"

if [ -f "/mnt/work_space/install/setup.bash" ]; then
    source "/mnt/work_space/install/setup.bash"
fi

exec "$@"

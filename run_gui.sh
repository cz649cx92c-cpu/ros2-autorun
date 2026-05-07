#!/usr/bin/env bash
set -euo pipefail

cd /root/ugv/autorun_final
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash
exec /usr/bin/python3 ./app.py "$@"

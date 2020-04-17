#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
# source "/root/python3_ws/install/setup.bash"

source "/root/catkin_ws/devel/setup.bash"

# start avahi daemon
/etc/init.d/dbus start &>/dev/null
service avahi-daemon start &>/dev/null

# setup ros environment
# source "/root/msl_raptor_ws/devel/setup.bash"

exec "$@"
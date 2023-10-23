#!/bin/bash
# Basic entrypoint for ROS / Colcon Docker containers

UNDERLAY_WS=$UNDERLAY_WS

# Source ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash
echo "Sourced ROS 2 ${ROS_DISTRO}"

# Source the base workspace, if built
if [ -f ${UNDERLAY_WS}/install/setup.bash ]
then
  source ${UNDERLAY_WS}/install/setup.bash
  vcs pull ${UNDERLAY_WS}/src
  echo "Sourced base workspace"
fi

# Source the overlay workspace, if built
if [ -f /overlay_ws/install/setup.bash ]
then
  source /overlay_ws/install/setup.bash
  vcs pull /overlay_ws/src
  echo "Sourced overlay workspace"
fi

# Implement updating all repositories at launch
if [ -f ${UNDERLAY_WS}/]
then
  cd ${UNDERLAY_WS}
  vcs pull src
  echo "Updated base workspace"
fi

if [ -f /overlay_ws/]
then
  ldconfig -v
  cd /overlay_ws
  vcs pull src
  echo "Updated overlay workspace"
fi

# Execute the command passed into this entrypoint
exec "$@"

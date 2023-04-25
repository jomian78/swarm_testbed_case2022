#!/bin/bash

workspace=${1-swarm_testbed_ws}
roshost=${2-illini76}

pushd $HOME/$workspace
source devel/setup.bash
export ROS_MASTER_URI=http://$roshost:11311

popd
exec "$@"

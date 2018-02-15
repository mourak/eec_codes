#!/bin/bash 

echo "Starting deployer"
source ../../devel/setup.bash

rosrun rtt_ros deployer -s deploy/deploy.ops

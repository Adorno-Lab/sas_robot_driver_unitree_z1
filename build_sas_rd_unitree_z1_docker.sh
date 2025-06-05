#!/bin/bash
echo $1
# Argument validation check
if [ $# -eq 0 ]; then
    echo "Error: No arguments provided."
    echo "Usage: sh build_sas_rd_unitree_z1_docker.sh ROS_DOMAIN_ID"
    echo "Example: sh build_sas_rd_unitree_z1_docker.sh 1"
    exit 1
fi
docker build -t sas_robot_driver_unitree_z1 docker/sas_robot_driver_unitree_z1/ --build-arg ROS_DOMAIN_ID=$1


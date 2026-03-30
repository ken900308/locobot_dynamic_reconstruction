#!/bin/bash

MIN_DOMAIN_ID=0
MAX_DOMAIN_ID=232

ROS_DOMAIN_ID=${1:-0}

if [[ "$ROS_DOMAIN_ID" -ge "$MIN_DOMAIN_ID" && "$ROS_DOMAIN_ID" -le "$MAX_DOMAIN_ID" ]]; then
    export ROS_DOMAIN_ID
    echo "ROS_DOMAIN_ID set to $ROS_DOMAIN_ID"
else
    echo "Invalid ROS_DOMAIN_ID. Please provide a value between $MIN_DOMAIN_ID and $MAX_DOMAIN_ID."
    return 1
fi
#!/usr/bin/env bash

if [ "$#" -ne 5 ]; then
    echo "Usage: ${0} data_dir p_id object_name"
    echo "Got ${#} arguments"
    exit -1
fi

DATA_DIR=$1
P_ID=$2
OBJECT_NAME=$3

BAG_FILENAME=$(<$DATA_DIR/$P_ID/$OBJECT_NAME/recording.txt)

rosbag play --clock -d 2 $DATA_DIR/$P_ID/$OBJECT_NAME/$BAG_FILENAME \
/deepgrasp/boson/camera_info:=/deepgrasp/boson/camera_info_old /tf:=/tf_old

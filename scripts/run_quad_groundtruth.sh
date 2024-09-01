#!/usr/bin/env bash

source /home/patrick/workspace/catkin_ws_ov/devel/setup.bash


DATASET="/media/patrick/RPNG FLASH 3/quad/raw_data/"
BASEDIR="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"



bagnames=(
    "mav_imu_joints_GRF/mavSystemID_1"
    "mav_imu_joints_GRF/mavSystemID_2"
    "mav_imu_joints_GRF/mavSystemID_3"
    "mav_imu_joints_GRF/mavSystemID_4"
)
bagnames_gt=(
    "vicon_data/session1"
    "vicon_data/session2"
    "vicon_data/session3"
    "vicon_data/session4"
)

bagdurr=(
    "-1"
    "-1"
    "-1"
    "-1"
)


rm -rf "$DATASET/groundtruth/"
rm -rf "$DATASET/groundtruth_info/"
mkdir -p "$DATASET/groundtruth/"
mkdir -p "$DATASET/groundtruth_info/"


# Loop through all datasets
for i in "${!bagnames[@]}"; do

roslaunch "$BASEDIR/../launch/exp_quad.launch" \
    path_bag:="$DATASET/${bagnames[i]}.bag" \
    path_bag_gt:="$DATASET/${bagnames_gt[i]}.bag" \
    bag_durr:="${bagdurr[i]}" \
    stats_path_states:="$DATASET/groundtruth/${bagnames[i]}.csv" \
    stats_path_info:="$DATASET/groundtruth_info/${bagnames[i]}.txt" \
    topic_imu:="/mav_imu" \
    topic_vicon:="/ghostgt/world"

done


# convert to the right format
rosrun ov_eval format_converter "$DATASET/groundtruth/" &> /dev/null





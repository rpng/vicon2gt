#!/usr/bin/env bash

source /home/cmb/singularity/workspaces_ros1/vicon_ws/devel/setup.bash


DATASET="/media/cmb/T7/singularity_data/dataset/ghost/sysID/data_sept10/raw_data"
BASEDIR="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

bagnames=(
    "imu/sample"
)

bagnames_gt=(
    "vicon_data/sample"
)

bagdurr=(
    "-1"
)

#bagnames=(
#    "imu/sample"
#    "imu/sample2"
#    "imu/session1"
#    "imu/session2"
#    "imu/session3"
#    "imu/session4"
#    "imu/session5"
#)
#
#bagnames_gt=(
#    "vicon_data/sample"
#    "vicon_data/sample2"
#    "vicon_data/session1"
#    "vicon_data/session2"
#    "vicon_data/session3"
#    "vicon_data/session4"
#    "vicon_data/session5"
#)
#
#bagdurr=(
#    "-1"
#    "-1"
#    "-1"
#    "-1"
#    "-1"
#    "-1"
#    "-1"
#)

rm -rf "$DATASET/groundtruth/"
rm -rf "$DATASET/groundtruth_info/"
mkdir -p "$DATASET/groundtruth/"
mkdir -p "$DATASET/groundtruth_info/"


# Loop through all datasets
for i in "${!bagnames[@]}"; do

echo "path_bag:=$DATASET/${bagnames[i]}.bag"
echo "path_bag_gt:=$DATASET/${bagnames_gt[i]}.bag"

roslaunch "$BASEDIR/../launch/exp_quad.launch" \
    path_bag:="$DATASET/${bagnames[i]}.bag" \
    path_bag_gt:="$DATASET/${bagnames_gt[i]}.bag" \
    bag_durr:="${bagdurr[i]}" \
    stats_path_states:="$DATASET/groundtruth/${bagnames[i]}.csv" \
    stats_path_info:="$DATASET/groundtruth_info/${bagnames[i]}.txt" \
    topic_imu:="/mcu/state/d_imu" \
    topic_vicon:="/ghostgt/world"

done

# convert to the right format
#rosrun ov_eval format_converter "$DATASET/groundtruth/" #&> /dev/null
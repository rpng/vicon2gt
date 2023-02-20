#!/usr/bin/env bash

# Source our workspace directory to load ENV variables
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
source ${SCRIPT_DIR}/../../../devel/setup.bash

#=============================================================
#=============================================================
#=============================================================

# dataset locations
bagnames=(
  "V1_01_easy"
  "V1_02_medium"
  "V1_03_difficult"
  "V2_01_easy"
  "V2_02_medium"
  "V2_03_difficult"
)
topics=(
 "/vicon/firefly_sbx/firefly_sbx "
 "/vicon/firefly_sbx/firefly_sbx "
 "/vicon/firefly_sbx/firefly_sbx "
 "vicon/firefly_sbx/firefly_sbx"
 "vicon/firefly_sbx/firefly_sbx"
 "vicon/firefly_sbx/firefly_sbx"
)


# location to save log files into
save_path1="$SCRIPT_DIR/../results/euroc_mav/algorithms"
log_path="$SCRIPT_DIR/../results/euroc_mav/temp"
bag_path="/home/patrick/datasets/euroc_mav/"



#=============================================================
#=============================================================
#=============================================================

# Loop through all datasets
for i in "${!bagnames[@]}"; do


mkdir -p "$log_path"
mkdir -p "$save_path1/vicon2gt_100hz/${bagnames[i]}/"

# start timing
start_time="$(date -u +%s)"
filename1="$log_path/vicon2gt_states.csv"
filename1_converted="$log_path/vicon2gt_states.txt"
filename1_final="$save_path1/vicon2gt_100hz/${bagnames[i]}/states.txt"


# run our ROS launch file (note we send console output to terminator)
roslaunch vicon2gt exp_eth.launch \
  dataset:="${bagnames[i]}" \
  topic_vicon:="${topics[i]}" \
  topic_imu:="/imu0" \
  folder:="$bag_path" \
  stats_path_info:="/tmp/vicon2gt_info.txt" \
  stats_path_states:="$filename1" &> /dev/null

# print out the time elapsed
end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"
echo "BASH: ${bagnames[i]} - took $elapsed seconds";

# convert to the right format and move
rosrun ov_eval format_converter "$filename1" &> /dev/null
mv "$filename1_converted" "$filename1_final"

# delete the temp folder
rm -rf "$log_path"

done


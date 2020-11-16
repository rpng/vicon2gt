#!/usr/bin/env bash

# Source our workspace directory to load ENV variables
source /home/patrick/workspace/catkin_ws_ov/devel/setup.bash

# varying level of noises
noises=(
    "0.001"
    "0.005"
    "0.01"
    "0.05"
    "0.10"
)


# location to save log files into
save_path1="/home/patrick/github/pubs_data/pgeneva/2020_vicon2gt/algorithms"
save_path2="/home/patrick/github/pubs_data/pgeneva/2020_vicon2gt/truths"
log_path="/home/patrick/github/pubs_data/pgeneva/2020_vicon2gt/temp"


#=============================================================
#=============================================================
#=============================================================

# loop through all pairwise combo of the noises!
for h in "${!noises[@]}"; do
for i in "${!noises[@]}"; do

# ensure our paths exists
mkdir -p "$log_path"
run_name="ori${noises[h]}_pos${noises[i]}"
mkdir -p "$save_path1/$run_name/simulation/"
mkdir -p "$save_path2"


# Monte Carlo runs for this dataset
for j in {10..29}; do

# start timing
start_time="$(date -u +%s)"
filename1="$log_path/vicon2gt_states.csv"
filename2="$log_path/vicon2gt_states_gt.csv"
filename1_converted="$log_path/vicon2gt_states.txt"
filename2_converted="$log_path/vicon2gt_states_gt.txt"
filename3="$save_path1/$run_name/simulation/${j}_states.txt"
filename4="$save_path2/simulation.txt"

# run our ROS launch file (note we send console output to terminator)
roslaunch vicon2gt sim.launch save2file:=true stats_path_states:=$filename1 stats_path_gt:=$filename2 seed:=$((10#$j)) vicon_sigmas:="[${noises[h]},${noises[h]},${noises[h]},${noises[i]},${noises[i]},${noises[i]}]" &> /dev/null

# print out the time elapsed
end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"
echo "BASH: $run_name - run $j took $elapsed seconds";

# convert to the right format
rosrun ov_eval format_converter "$filename1" &> /dev/null
rosrun ov_eval format_converter "$filename2" &> /dev/null

# move to the final folder
mv "$filename1_converted" "$filename3"
mv "$filename2_converted" "$filename4"

done
done
done

# delete the temp folder
rm -rf "$log_path"




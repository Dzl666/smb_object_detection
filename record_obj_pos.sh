#! /bin/bash

# Other variables
now="$(date +"%F-%H-%M")"
today="$(date +"%F")"

# List of useful colors
COLOR_RESET="\033[0m"
COLOR_WARN="\033[0;33m"

# Check outputpath
outpath="$HOME/bags/${today}"
if [ ! -d "${outpath}" ]; then
  mkdir -p "${outpath}"
fi

# Record
rosparam dump ${outpath}/${now}.yaml
rosbag record --output-name=${outpath}/${now}"_smb_obj_detect" \
/tf \
/object_detector/object_poses

#!/bin/bash
#
# Loop through all the packages in ROS_PACKAGE_PATH 
# If we find our package and the amazon directory,
# run the ngrok program inside there.
# Original author: Matthew Wilson, 2017
# Adapted by: Brandon Trabucco, 2018

IFS=":" 
path_found=false

for path in $ROS_PACKAGE_PATH
do
    ngrok_path="${path}/image_caption_machine/scripts"
    #printf "\n$ngrok_path\n" 
    if [ -d "${ngrok_path}" ]; then
        path_found=true
        $ngrok_path/ngrok http 5000 --subdomain=image-caption-machine
    fi
done


if [ ! $path_found ] ; then
    echo "Path to ngrok not found. Set your package path or add the image_caption_machine repo to your workspace"
fi

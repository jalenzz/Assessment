#!/bin/bash
path=/.snapshots
time=$(date "+%Y-%m-%d")
snapshot=${path}/${time}
files_in_path=$(sudo ls $path)

for filename in $files_in_path
do
    if [ $filename == $time ]
    then
        echo "${snapshot} exist"
        exit 0
    fi
done
sudo btrfs subvolume snapshot / ${snapshot}
#!/usr/bin/bash

trap exit INT


for bag in *_fixed.bag; do
  #evo_traj bag ${bag} /lsd_slam/pose -v --full_check --save_as_tum
  # LSD output with translated coordinates to match the groundtruth
  evo_traj bag ${bag} /changed_system/pose -v --full_check --save_as_tum
  num=$(echo ${bag//[^0-9]/})
  #mv -v -- lsd_slam_pose.tum "measurements/lsd_mh_${num}_poses.tum"
  mv -v -- changed_system_pose.tum "tums/lsd_mh_${num}_poses_fixed.tum"
done

for bag in *.bag; do
  evo_traj bag ${bag} /lsd_slam/pose -v --full_check --save_as_tum
  num=$(echo ${bag//[^0-9]/})
  mv -v -- lsd_slam_pose.tum "tums/lsd_mh_${num}_poses.tum"
done

#!/usr/bin/env bash

trap exit INT

# Extract poses for ROVIOLI
for bag in bags/*_ROV.bag; do
    evo_traj bag ${bag} /maplab_rovio/T_M_I -v --full_check --save_as_tum
    num=$(echo ${bag//[^0-9]/})
    mv -v -- maplab_rovio_T_M_I.tum "measurements/rovioli_mh_${num}_poses.tum"
done

# Extract poses for LSD
for bag in bags/*_LSD.bag; do
    evo_traj bag ${bag} /lsd_slam/pose -v --full_check --save_as_tum
    # LSD output with translated coordinates to match the groundtruth
    #evo_traj bag ${bag} /changed_system/pose -v --full_check --save_as_tum
    num=$(echo ${bag//[^0-9]/})
    mv -v -- lsd_slam_pose.tum "measurements/lsd_mh_${num}_poses.tum"
    #mv -v -- changed_system_pose.tum "measurements/lsd_mh_${num}_poses.tum"
done

# Extract poses for ORB-SLAM3
for bag in bags/*_ORB.bag; do
    evo_traj bag ${bag} /orb_pose -v --full_check --save_as_tum
    num=$(echo ${bag//[^0-9]/})
    mv -v -- orb_pose.tum "measurements/orb_mh_${num}_poses.tum"
done

echo && printf '=%.0s' {1..80} && echo

RPE_UNIT='f'
RPE_DELTA=10

for i in {1..5}; do
    # Calculate Rovioli's metrics
    rov_result="measurements/rovioli_mh_0${i}_poses.tum"
    if [ -f ${rov_result} ]; then
        evo_rpe tum "groundtruths/euroc_mh_0${i}_gt.tum" ${rov_result} -v --align_origin --pose_relation trans_part \
            --no_warnings -d ${RPE_DELTA} -u ${RPE_UNIT} --save_results "accuracy/rpe_mh_0${i}_rovioli.zip"
        evo_ape tum "groundtruths/euroc_mh_0${i}_gt.tum" ${rov_result} -v --align_origin --pose_relation trans_part \
            --no_warnings --save_results "accuracy/ate_mh_0${i}_rovioli.zip"
    fi
    # Calculate LSD's metrics
    lsd_result="measurements/lsd_mh_0${i}_poses.tum"
    if [ -f ${lsd_result} ]; then
        evo_rpe tum "groundtruths/euroc_mh_0${i}_gt.tum" ${lsd_result} -v --align_origin -a -s --pose_relation trans_part \
            --no_warnings -d ${RPE_DELTA} -u ${RPE_UNIT} --save_results "accuracy/rpe_mh_0${i}_lsd.zip"
        evo_ape tum "groundtruths/euroc_mh_0${i}_gt.tum" ${lsd_result} -v --align_origin -a -s --pose_relation trans_part \
            --no_warnings --save_results "accuracy/ate_mh_0${i}_lsd.zip"
    fi
    # Calculate ORB-SLAM3's metrics
    orb_result="measurements/orb_mh_0${i}_poses.tum"
    if [ -f ${orb_result} ]; then
        evo_rpe tum "groundtruths/euroc_mh_0${i}_gt.tum" ${orb_result} -v --align_origin --pose_relation trans_part \
            --no_warnings -d ${RPE_DELTA} -u ${RPE_UNIT} --save_results "accuracy/rpe_mh_0${i}_orb.zip"
        evo_ape tum "groundtruths/euroc_mh_0${i}_gt.tum" ${orb_result} -v --align_origin --pose_relation trans_part \
            --no_warnings --save_results "accuracy/ate_mh_0${i}_orb.zip"
    fi
done

echo && printf '=%.0s' {1..80} && echo

# Prepare training data
python3 -c "from extract_training_data import assemble_training_data; assemble_training_data()"
#python3 extract_training_data.py

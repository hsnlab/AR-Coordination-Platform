# slam_accuracy

## Dependencies

Used tool: https://github.com/MichaelGrupp/evo

```bash
sudo apt update && sudo apt install python3 python3-pip
python3 -m pip install --extra-index-url https://rospypi.github.io/simple/ rospy rosbag roslz4 tf2_ros
python3 -m pip install evo
```

## Convert, extract and calculate measurements

Copy the recorded rosbag (containing the poses) under _bags/_ with the given naming convention.

Run the update script:

```bash
./update_all.sh
```

## Manual substep commands

### Convert EuRoC ground truth file

```bash
# evo_traj euroc groundtruths/mh_0?/data.csv -v --full_check --save_as_tum
./convert_gt.sh
```

### Extract pose data from test bags

```bash
evo_traj bag bags/MH_???_ROV.bag /maplab_rovio/T_M_I -v --save_as_tum
evo_traj bag bags/MH_???_LSD.bag /lsd_slam/pose -v --save_as_tum
evo_traj bag bags/MH_???_ORB3.bag /orb_pose -v --save_as_tum
```

### Check and plot all the measurements for a benchmark dataset

```bash
evo_traj tum -v -p --align_origin --full_check --ref groundtruths/euroc_mh_0?_gt.tum measurements/*_mh_0?_poses.tum
evo_traj tum -v -p -a -s --full_check --ref groundtruths/euroc_mh_0?_gt.tum measurements/lsd_mh_0?_poses.tum
```

### Compare and calculate RPE for the measurements

```bash
# Rovioli
evo_rpe tum groundtruths/euroc_mh_??_gt.tum measurements/rovioli_mh_??_poses.tum \
        -v --align_origin -p -d 1 -u f --save_results accuracy/rpe_mh_??_rovioli.zip
# LSD
evo_rpe tum groundtruths/euroc_mh_??_gt.tum measurements/lsd_mh_??_poses.tum \
        -v --align_origin -a -p -d 1 -u f --save_results accuracy/rpe_mh_??_lsd.zip
# ORB3

```

### Compare and calculate RPE for the measurements

```bash
# Rovioli
evo_ape tum groundtruths/euroc_mh_??_gt.tum measurements/rovioli_mh_??_poses.tum \
        -v --align_origin --save_results accuracy/ate_mh_??_rovioli.zip
# LSD
evo_ape tum groundtruths/euroc_mh_??_gt.tum measurements/lsd_mh_??_poses.tum \
        -v --align_origin -a --save_results accuracy/ate_mh_??_lsd.zip
# ORB3

```
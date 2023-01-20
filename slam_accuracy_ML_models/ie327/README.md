# FAQ

## Convert ROSBAG to TUM

```bash
evo_traj bag -v --full_check test_result_ie327_rovioli.bag /maplab_rovio/T_M_I --save_as_tum
mv -v -- maplab_rovio_T_M_I.tum test_result_ie327_rovioli.tum
```

## Check/plot trajectory

```bash
evo_traj bag test_result_ie327_rovioli.bag /maplab_rovio/T_M_I -v --full_check -p 
# with groundtruth
evo_traj tum --ref GT_Times_17_38_36.tum test_result_ie327_rovioli.tum -v --full_check -p --align_origin
# If type of coordinate systems (left/right) conflict is GT and test result 
evo_traj tum --ref GT_Times_17_38_36.tum test_result_ie327_rovioli.tum -v --full_check -p -a 
```

## Calculate pose error

```bash
evo_ape tum GT_Times_17_38_36.tum test_result_ie327_rovioli.tum -v -a -p --pose_relation trans_part
evo_rpe tum GT_Times_17_38_36.tum test_result_ie327_rovioli.tum -v -a -p --pose_relation trans_part -d 1 -u 'f'
```
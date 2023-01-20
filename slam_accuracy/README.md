 
 ### Capture ground truth from URSim
 `python3 main.py --capture_robot_positions`
 
 and start trajectory in URSim. At the end of the trajectory stop the capturing with Ctrl+C. 
 You can find the robot trajectory in the measurements folder (robot_ground_truth....txt)
 
 ### Capture the estimated positions by slam 
 

 ### Analyze: bring ground truth and slam positions to the same coordinate system and analyze
e.g.:
```
python3 main.py --analyze --gt ./measurements/ground_truth/robot_gt_negyzet_with_tcp.txt --slam ./measurements/slam/p30_negyzet/remote_speed30_p30_fixed.txt,./measurements/slam/p30_negyzet/remote_speed42_p30.txt,./measurements/slam/p30_negyzet/remote_speed60_p30.txt,./measurements/slam/p30_negyzet/remote_speed100_p30.txt --slam_type rovioli,rovioli,rovioli,rovioli --res ./p30_negyzet_all_remote
```

e.g.:
```
python3 main.py --analyze --gt ./measurements/ground_truth/robot_gt_negyzet_with_tcp.txt --slam ./measurements/slam/p30_negyzet/speed_30.txt,./measurements/slam/p30_negyzet/speed_42.txt,./measurements/slam/p30_negyzet/speed_60.txt,./measurements/slam/p30_negyzet/speed_100.txt --slam_type arcore,arcore,arcore,arcore --res ./p30_negyzet_all_local
```



## Calculate ATE, RPE
use evaluate_ate.py and evaluate_rpe.py

1. run the main.py on the dataset
2. delete the second column from the stamped_traj_estimate_.... (this will be the second input file)
3. run evaluate_ate.py and evaluate_rpe.py (first input file is the stamped_gt file)

e.g.:
python2 evaluate_rpe.py p30_haz_fekvo_lsd_60_2/stamped_groundtruth.txt p30_haz_fekvo_lsd_60_2/asd.txt --plot ate_plot.png --max_pairs 10000 --fixed_delta --plot asd_rpe.png


python2 evaluate_ate.py p30_haz_fekvo_lsd_60_2/stamped_groundtruth.txt p30_haz_fekvo_lsd_60_2/asd.txt --plot ate_plot.png --verbose --max_difference 0.1
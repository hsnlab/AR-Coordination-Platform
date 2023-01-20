
This is the SLAM Image Analyzer for the AR project of BME TMIT.


Installation, compilation

Requirements: ROS, catkin, Eigen

Tested on: Ubuntu 18.04.


Building
=======

1. Go to your local catkin folder `cd $CATKIN_WS/src`
1. `git clone git@netsoft.tmit.bme.hu:gslam/sia.git`
1. `catkin build slam_image_analyzer`

Running plain
=======

1. `source $CATKIN_WS/devel/setup.bash`
1. `rosrun slam_image_analyzer decision_maker`

This will run the decision_maker module of SIA.

Running with decision module testing
=======

1. `source $CATKIN_WS/devel/setup.bash`
1. `roslaunch slam_image_analyzer init.launch`

Use prediction publisher
=======
git clone <sia repo>

# update docker image
docker pull dokaj/cloudslam_orb_lsd_rovioli_pred_pub

# start docker-compose
cd sia
docker-compose up

# login container
docker exec -it <container ID or name> bash 


Start prediction publisher ROS node (in the container)
=============================
git clone <sia> (if needed)
cd sia
rm -r devel/ build/ 
source /opt/ros/melodic/setup.bash
catkin_make
source devel/setup.bash
source ~/cvbridge_build_ws/devel/setup.bash --extend
rosrun prediction_publisher rov_prediction_publisher.py


[new terminal]
source /opt/ros/melodic/setup.bash
source devel/setup.bash
source ~/cvbridge_build_ws/devel/setup.bash --extend
rosrun prediction_publisher lsd_prediction_publisher.py
=======

=======
Credits
=======

## Decision Maker:
Contact: [Zsofia Kecskes-Solymosi](mailto:solymosi.zsofia@gsuite.tmit.bme.com)

## Prediction Publisher:
Contact: [Janos Doka](mailto:doka@gsuite.tmit.bme.com)

Contributing & License
=======

License: [BSD](https://choosealicense.com/licenses/bsd-2-clause/)

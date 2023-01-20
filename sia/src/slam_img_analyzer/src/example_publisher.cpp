#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include "slam_img_analyzer/StringStamped.h"

#include <stdlib.h> 

ros::Publisher lsd_pub;
ros::Publisher rovioli_pub;

long long convertTime(long sec, long nsec){
    long long t = sec;
    t = t * 1000000000;
    t += nsec;
    return t;
}

long long getTimeFromString() {
}


void imageCallback(const sensor_msgs::ImageConstPtr& cam_img_msg) {
    ROS_INFO("Camera image recieved with timestamp: %lld\n", convertTime(cam_img_msg->header.stamp.sec, cam_img_msg->header.stamp.sec));
  
    int prediction = rand() % 10 + 1;

    slam_img_analyzer::StringStamped ss;
    ss.header = cam_img_msg->header;
    ss.prediction = std::to_string(cam_img_msg->header.stamp.sec) + "." + std::to_string(cam_img_msg->header.stamp.nsec) + ";" + std::to_string(rand() % 10 + 1);
    lsd_pub.publish(ss);
    
    ss.prediction = std::to_string(cam_img_msg->header.stamp.sec) + "." + std::to_string(cam_img_msg->header.stamp.nsec) + ";" + std::to_string(rand() % 10 + 1);
    rovioli_pub.publish(ss);
}


int main(int argc, char **argv)
{
    srand (time(NULL));

    ros::init(argc, argv, "example_publisher");
    ROS_INFO("SIA Example Publisher Started!\n");
    ros::NodeHandle nh;

    ros::Subscriber cam_img_sub = nh.subscribe("cam0/image_raw", 1000, imageCallback);
    lsd_pub = nh.advertise<slam_img_analyzer::StringStamped>("/lsd_prediction",1000);
    rovioli_pub = nh.advertise<slam_img_analyzer::StringStamped>("/rov_prediction",1000);

    ros::spin();

  return 0;
}

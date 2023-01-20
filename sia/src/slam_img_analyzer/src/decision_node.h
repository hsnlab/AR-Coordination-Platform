#ifndef DECISION_MAKER_H
#define DECISION_MAKER_H

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"

#include "geometry_msgs/PoseStamped.h"
#include "slam_img_analyzer/SlamPoseStamped.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/cache.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <slam_img_analyzer/StringStamped.h>

#include <Eigen/Geometry>
#include "tf_conversions/tf_eigen.h"
#include <opencv2/core/mat.hpp>
#include <opencv2/calib3d.hpp>

using std::string;

class DecisionNode
{
public:
  // Constructor.
  DecisionNode(ros::NodeHandle _nh);

  // Destructor.
  ~DecisionNode();

  float a_orb, b_orb;
  float a_lsd, b_lsd;
  float a_rovioli, b_rovioli;
  int orb_count, lsd_count, rovioli_count;
  cv::Mat_<cv::Point3f> first, second;
  cv::Mat_<cv::Point3f> first_rovioli, second_rovioli;
  cv::Mat T;
  
  string cam_topic = "cam0/image_raw";
  string slam_1_topic = "lsd_slam/pose";
  string slam_2_topic = "maplab_rovio/T_G_I";
  string slam_3_topic = "svo/pose_cam/0";
  string lsd_pred_topic = "/lsd_prediction";
  string rov_pred_topic = "/rov_prediction";
  string publish_topic = "sia/pose";
  string publish_slam = "sia/slam";
  string publish_slam_lsd = "sia/lsd";
  string publish_slam_rovioli = "sia/rovioli";
  string publish_path = "sia/trajectory";
  ros::Publisher _pub;
  ros::Publisher _pub_slam;
  ros::Publisher _pub_slam_lsd;
  ros::Publisher _pub_slam_rovioli;
  ros::Publisher _pub_path;

  nav_msgs::Path path;

  boost::shared_ptr<geometry_msgs::PoseStamped const> _last_lsd_pose;
  boost::shared_ptr<geometry_msgs::PoseStamped const> _last_rovioli_pose;
  boost::shared_ptr<geometry_msgs::PoseStamped const> _last_orb_pose;
  boost::shared_ptr<slam_img_analyzer::StringStamped const> _last_lsd_pred_pose;
  boost::shared_ptr<slam_img_analyzer::StringStamped const> _last_rov_pred_pose;

  boost::shared_ptr<geometry_msgs::PoseStamped const> _predict_lsd_pose;
  boost::shared_ptr<geometry_msgs::PoseStamped const> _predict_rovioli_pose;
  boost::shared_ptr<geometry_msgs::PoseStamped const> _predict_orb_pose;

  message_filters::Subscriber<sensor_msgs::Image> _sub;
  message_filters::Subscriber<geometry_msgs::PoseStamped> _sub_lsd_slam;
  message_filters::Subscriber<geometry_msgs::PoseStamped> _sub_rovioli;
  message_filters::Subscriber<geometry_msgs::PoseStamped> _sub_orb;
  message_filters::Subscriber<slam_img_analyzer::StringStamped> _sub_lsd_prediciton;
  message_filters::Subscriber<slam_img_analyzer::StringStamped> _sub_rov_prediciton;

  message_filters::Cache<geometry_msgs::PoseStamped> _lsd_cache;
  message_filters::Cache<geometry_msgs::PoseStamped> _rovioli_cache;
  message_filters::Cache<geometry_msgs::PoseStamped> _orb_cache;
  message_filters::Cache<slam_img_analyzer::StringStamped> _lsd_pred_cache;
  message_filters::Cache<slam_img_analyzer::StringStamped> _rov_pred_cache;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped, geometry_msgs::PoseStamped, geometry_msgs::PoseStamped, slam_img_analyzer::StringStamped, slam_img_analyzer::StringStamped> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> _sync;

  string getPredFromString(const string unprocessed); 

  cv::Point3f calculate_mean(const cv::Mat_<cv::Point3f> &points);
  cv::Mat calculate_transform(const cv::Mat_<cv::Point3f> &points1, const cv::Mat_<cv::Point3f> &points2);
  // Callback function for subscriber.
  void messageCallback(
    const sensor_msgs::ImageConstPtr& cam_img_msg,
    message_filters::Cache<geometry_msgs::PoseStamped> &lsd_cache,
    message_filters::Cache<geometry_msgs::PoseStamped> &rovioli_cache,
    message_filters::Cache<geometry_msgs::PoseStamped> &orb_cache,
    message_filters::Cache<slam_img_analyzer::StringStamped> &lsd_pred_cache,
    message_filters::Cache<slam_img_analyzer::StringStamped> &rov_pred_cache
  );

  void callback(geometry_msgs::PoseStampedConstPtr& ps);

  boost::shared_ptr<geometry_msgs::PoseStamped const>  correct_lsd_pose(boost::shared_ptr<geometry_msgs::PoseStamped const> );

  boost::shared_ptr<geometry_msgs::PoseStamped const> correct_rovioli_pose(boost::shared_ptr<geometry_msgs::PoseStamped const> rovioli_pose);

  boost::shared_ptr<geometry_msgs::PoseStamped const> correct_orb_pose(boost::shared_ptr<geometry_msgs::PoseStamped const> orb_pose);
  
  std::string time_to_string(int sec, int nsec);

  std::string slam_to_string(std::string time, geometry_msgs::Pose pose, std::string slam);
};

#endif // DECISION_MAKER_H
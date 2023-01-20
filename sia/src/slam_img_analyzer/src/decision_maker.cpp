#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/PoseStamped.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/cache.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sstream>

#include "decision_node.h"

DecisionNode *decision_node;



/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "decision_maker");
  ros::NodeHandle _nh;

  std::cout << "[ SIA - DECISION MAKER ] \n";
  std::cout << "Decision Maker started! \n";

  decision_node = new DecisionNode(_nh);

  ros::spin();
  return 0;
}
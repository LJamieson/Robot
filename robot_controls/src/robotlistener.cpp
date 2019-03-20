#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * This will receive the command from control.cpp and make the robot move
 */
 
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robotlistener");

  ros::NodeHandle n;


  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  /**
   * loop until Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}

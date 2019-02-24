#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

/**
 *  This will send commands to the listener node
 */

int main(int argc, char **argv)
{

  ros::init(argc, argv, "control");

  ros::NodeHandle n;


  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);


  /** 
   * int count = 0;
   */
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;
    std::string ss;
    std::cout << "Please enter w,s,a or d:  ";
    std::getline(std::cin, ss);
    msg.data = ss;

    

    //send a request to the node serving out the messages
    //print out recieved messages.
    ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);
	
    ros::spinOnce();

    loop_rate.sleep();
    /**
     * ++count;
     */
  }


  return 0;
}

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include <sstream>


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I received: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::spinOnce();

 // ros::init(argc, argv, "responder");
  ros::Publisher pub = n.advertise<std_msgs::Float64>("response", 5);
  ros::Rate loop_rate(1);
  ros::spinOnce();
  ros::Rate poll_rate(100);
  float count = 0;

  while (ros::ok())
  {
    while(pub.getNumSubscribers()==0){
      poll_rate.sleep();
    }
    std_msgs::Float64 msg;
   // msg.data = ss.str();
   // msg.data = "Hello there";
    msg.data = count;
    ROS_INFO("%f", msg.data);
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    count += 1;
  }

  return 0;
}
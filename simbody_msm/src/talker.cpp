#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <sstream>

const float rate = 10.0; // In Hz
int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::Float32>("mass", 1000/rate);

  ros::Rate loop_rate(rate);

  int count =0;
  while (ros::ok())
    {
      std_msgs::Float32 msg;
      msg.data = count;

      ROS_INFO("%f", msg.data);

      chatter_pub.publish(msg);

      ros::spinOnce();
      loop_rate.sleep();
      ++count;
    }

  return 0;
}

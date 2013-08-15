#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include "MyNodeHandle.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "inel_ros_test");
  ros::NodeHandle node;
  ros::Publisher cmdVelPub = node.advertise<geometry_msgs::Twist>("/goal_vel", 100);
  ros::Publisher adjustLinearPub = node.advertise<geometry_msgs::Vector3>("/adjust_linear", 100);
  ros::Rate loopRate(0.2);

  geometry_msgs::Twist speed;
  speed.linear.x = 0.3;
  speed.angular.z = 0;
  geometry_msgs::Vector3 adjustLinear;
  adjustLinear.x = 0.2;
  while (ros::ok())
  {
    speed.linear.x *= -1;
    speed.angular.z *= -1;
    cmdVelPub.publish(speed);
    adjustLinearPub.publish(adjustLinear);
    loopRate.sleep();
  }

  return 0;
}

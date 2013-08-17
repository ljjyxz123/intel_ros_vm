#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include "MyNodeHandle.h"

ros::Publisher cmdVelPub;
bool paused; // dynamic pause or resume this program
int minX, minY, maxX, maxY, goalZ, maxZ, minPoints;
double linearRespRate, angularRespRate;
double linearSpeed = 0;
double angularSpeed = 0;

void commandCallback(const std_msgs::String::ConstPtr& msg)
{
  if (msg->data == "follower")
  {
    paused = false;
  }
  else
  {
    paused = true;
  }
}

void pointsCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  if (paused) return;
  sensor_msgs::Image img = *msg;

  long int x, y, z;
  x = y = z = 0;
  int n = 0;
  int centerX = img.width / 2;

  uint16_t* data = (uint16_t*)&img.data[0];
  for (int i = minY; i < maxY; i++)
  {
    for (int j = minX; j < maxX; j++)
    {
      uint16_t depth = data[img.width * i + j];
      if (depth < maxZ)
      {
        x += j;
        y += i;
        z += depth;
        n++;
      }

    }
  }

  geometry_msgs::Twist moveCmd;

  if (n > minPoints)
  {
    int avX = x / n;
    int avY = y / n;
    int avZ = z / n;

    linearSpeed = (avZ - goalZ) * 0.001 * linearRespRate;
    angularSpeed = (centerX - avX) * (1.0 / 100.0) * angularRespRate;
  }

  moveCmd.linear.x = linearSpeed;
  moveCmd.angular.z = angularSpeed;
  cmdVelPub.publish(moveCmd);
}

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "intel_ros_follower2");
  MyNodeHandle node;
  ros::Subscriber depthRawSub = node.subscribe("/camera/depth/image_raw", 100, pointsCallback);
  ros::Subscriber commandSub = node.subscribe("/cmd_center/author", 100, commandCallback);
  cmdVelPub = node.advertise<geometry_msgs::Twist>("/goal_vel", 100);

  // get params
  ROS_INFO("follower2 get params:");
  paused = node.getParamEx("intel_ros_follower2/paused", true);
  minX = node.getParamEx("intel_ros_follower2/minX", 80);
  maxX = node.getParamEx("intel_ros_follower2/maxX", 240);
  minY = node.getParamEx("intel_ros_follower2/minY", 40);
  maxY = node.getParamEx("intel_ros_follower2/maxY", 200);
  goalZ = node.getParamEx("intel_ros_follower2/goalZ", 600);
  maxZ = node.getParamEx("intel_ros_follower2/maxZ", 900);
  minPoints = node.getParamEx("intel_ros_follower2/minPoints", 1000);
  linearRespRate = node.getParamEx("intel_ros_follower2/linearRespRate", 1.0);
  angularRespRate = node.getParamEx("intel_ros_follower2/angularRespRate", 1.0);

  ros::spin();

  return 0;
}

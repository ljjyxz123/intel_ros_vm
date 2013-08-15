#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>

ros::Publisher cmdVelPub;
bool paused; // dynamic pause or resume this program
int minX, minY, maxX, maxY, goalZ, maxZ, minPoints;
double maxLinearSpeed;
double maxAngularSpeed;
double slowDownRate;
bool isStop = false;
float linearSpeed = 0;
float angularSpeed = 0;

void commandCallback(const std_msgs::String::ConstPtr& msg)
{
  if (msg->data == "follow me")
  {
    paused = false;
    isStop = false;
  }
  else if (msg->data == "stop")
  {
    paused = false;
    isStop = true;
  }
  else
  {
    paused = true;
  }
}

void pointsCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  if (paused)
    return;
  ROS_INFO("sec: %d, nsec: %d", msg->header.stamp.sec, msg->header.stamp.nsec);
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

  if (n > minPoints && !isStop)
  {
    int avX = x / n;
    int avY = y / n;
    int avZ = z / n;

    linearSpeed = (avZ - goalZ) * 0.001;
    angularSpeed = -(avX - centerX) * (1.0 / 100.0);

    if (linearSpeed > maxLinearSpeed || linearSpeed < -maxLinearSpeed)
    {
      ROS_ERROR("Linear speed is too high: [%f]", linearSpeed);
      linearSpeed = linearSpeed > 0 ? maxLinearSpeed : -maxLinearSpeed;
    }
    if (angularSpeed > maxAngularSpeed || angularSpeed < -maxAngularSpeed)
    {
      ROS_ERROR("Angular speed is too high: [%f]", angularSpeed);
      angularSpeed = angularSpeed > 0 ? maxAngularSpeed : -maxAngularSpeed;
    }
  }
  else // stop slowly
  {
    linearSpeed *= slowDownRate;
    angularSpeed *= slowDownRate;
  }

  moveCmd.linear.x = linearSpeed;
  moveCmd.angular.z = angularSpeed;
  cmdVelPub.publish(moveCmd);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "inel_ros_follower");
  ros::NodeHandle node;
  ros::Subscriber depthRawSub = node.subscribe("/camera/depth/image_raw", 100, pointsCallback);
  ros::Subscriber commandSub = node.subscribe("/recognizer/output", 100, commandCallback);
  cmdVelPub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

  // Get params
  if (!node.getParam("inel_ros_follower/paused", paused))
  {
    paused = true;
  }
  if (!node.getParam("inel_ros_follower/minX", minX))
  {
    minX = 80;
  }
  if (!node.getParam("inel_ros_follower/maxX", maxX))
  {
    maxX = 240;
  }
  if (!node.getParam("inel_ros_follower/minY", minY))
  {
    minY = 40;
  }
  if (!node.getParam("inel_ros_follower/maxY", maxY))
  {
    maxY = 200;
  }
  if (!node.getParam("inel_ros_follower/goalZ", goalZ))
  {
    goalZ = 600;
  }
  if (!node.getParam("inel_ros_follower/maxZ", maxZ))
  {
    maxZ = 900;
  }
  if (!node.getParam("inel_ros_follower/minPoints", minPoints))
  {
    minPoints = 1000;
  }
  if (!node.getParam("inel_ros_follower/maxLinearSpeed", maxLinearSpeed))
  {
    maxLinearSpeed = 0.3;
  }
  if (!node.getParam("inel_ros_follower/maxAngularSpeed", maxAngularSpeed))
  {
    maxAngularSpeed = 1;
  }
  if (!node.getParam("inel_ros_follower/slowDownRate", slowDownRate))
  {
    slowDownRate = 0.8;
  }

  ROS_INFO("Set paused [%s]", paused ? "true" : "false");
  ROS_INFO("Set minX [%d]", minX);
  ROS_INFO("Set maxX [%d]", maxX);
  ROS_INFO("Set minY [%d]", minY);
  ROS_INFO("Set maxY [%d]", maxY);
  ROS_INFO("Set goalZ [%d]", goalZ);
  ROS_INFO("Set maxZ [%d]", maxZ);
  ROS_INFO("Set minPoints [%d]", minPoints);
  ROS_INFO("Set maxLinearSpeed [%f]", maxLinearSpeed);
  ROS_INFO("Set maxAngularSpeed [%f]", maxAngularSpeed);
  ROS_INFO("Set slowDownRate [%f]", slowDownRate);

  ros::spin();

  return 0;
}

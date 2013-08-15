#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include "MyNodeHandle.h"

// publishers
ros::Publisher authorPub;
ros::Publisher goalVelPub;
ros::Publisher goalLinearPub;
ros::Publisher goalAngularPub;
ros::Publisher adjustLinearPub;
ros::Publisher adjustAngularPub;
ros::Publisher increLinearPub;
ros::Publisher increAngularPub;

void pubGoalLinear(double linear)
{
  geometry_msgs::Vector3 goalLinear;
  goalLinear.x = linear;
  goalLinearPub.publish(goalLinear);
}

void pubGoalAngular(double angular)
{
  geometry_msgs::Vector3 goalAngular;
  goalAngular.z = angular;
  goalAngularPub.publish(goalAngular);
}

void pubGoalVel(double linear, double angular)
{
  geometry_msgs::Twist goalVel;
  goalVel.linear.x = linear;
  goalVel.angular.z = angular;
  goalVelPub.publish(goalVel);
}

void pubAdjustLinear(double linear)
{
  geometry_msgs::Vector3 adjustLinear;
  adjustLinear.x = linear;
  adjustLinearPub.publish(adjustLinear);
}

void pubAdjustAngular(double angular)
{
  geometry_msgs::Vector3 adjustAngular;
  adjustAngular.z = angular;
  adjustAngularPub.publish(adjustAngular);
}

void pubAdjustVel(double linear, double angular)
{
  pubAdjustLinear(linear);
  pubAdjustAngular(angular);
}

void pubIncreLinear(double linear)
{
  geometry_msgs::Vector3 increLinear;
  increLinear.x = linear;
  increLinearPub.publish(increLinear);
}

void pubIncreAngular(double angular)
{
  geometry_msgs::Vector3 increAngular;
  increAngular.z = angular;
  increAngularPub.publish(increAngular);
}

void pubIncreVel(double linear, double angular)
{
  pubIncreLinear(linear);
  pubIncreAngular(angular);
}

void pubAuthor(std::string author)
{
  std_msgs::String authorMsg;
  authorMsg.data = author;
  authorPub.publish(authorMsg);
}

// global vars
bool paused;
double linearSpeed;
double angularSpeed;
double linearStep;
double angularStep;

long msec = 0;
std::string cmd;
// skip the duplicate command in 2 secs. check success will return true
bool checkCmd(std::string tcmd)
{
  long tmsec = clock() / 1000;
  if (cmd == tcmd && tmsec - msec < 2000)
    return false;
  cmd = tcmd;
  msec = tmsec;
  return true;
}

void recogCallback(const std_msgs::String::ConstPtr& msg)
{
  std::string cmd = msg->data;

  // program setting command
  if (cmd == "continue")
  {
    if (checkCmd("continue"))
      paused = false;
  }
  else if (cmd == "pause")
  {
    if (checkCmd("pause"))
      paused = true;
  }
  if (paused)
    return;

  // command vel list
  if (cmd == "stop")
  {
    if (checkCmd("stop"))
    {
      pubGoalVel(0, 0);
      pubAdjustVel(0, 0);
      pubAuthor("center");
    }
  }
  else if (cmd == "forward")
  {
    if (checkCmd("forward"))
    {
      pubGoalVel(linearSpeed, 0);
      pubAdjustAngular(0);
      pubAuthor("center");
    }
  }
  else if (cmd == "backward")
  {
    if (checkCmd("backward"))
    {
      pubGoalVel(-linearSpeed, 0);
      pubAdjustAngular(0);
      pubAuthor("center");
    }
  }
  else if (cmd == "slower")
  {
    if (checkCmd("slower"))
    {
      pubIncreVel(-0.4, -0.8);
    }
  }
  else if (cmd == "faster")
  {
    if (checkCmd("faster"))
    {
      pubIncreVel(0.4, 0.8);
    }
  }
  else if (cmd == "turn left")
  {
    if (checkCmd("turn left"))
    {
      pubGoalAngular(0.8);
      pubAuthor("center");
    }
  }
  else if (cmd == "turn right")
  {
    if (checkCmd("turn right"))
    {
      pubGoalAngular(-0.8);
      pubAuthor("center");
    }
  }
  else if (cmd == "rotate left")
  {
    if (checkCmd("rotate left"))
    {
      pubGoalVel(0, angularSpeed);
      pubAuthor("center");
    }
  }
  else if (cmd == "rotate right")
  {
    if (checkCmd("rotate right"))
    {
      pubGoalVel(0, -angularSpeed);
      pubAuthor("center");
    }
  }
  else if (cmd == "follow me")
  {
    if (checkCmd("follow me"))
      pubAuthor("follower");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cmd_center");
  MyNodeHandle node;
  ros::Subscriber recogSub = node.subscribe("/recognizer/output", 100, recogCallback);
  authorPub = node.advertise<std_msgs::String>("/cmd_center/author", 100);
  goalVelPub = node.advertise<geometry_msgs::Twist>("/goal_vel", 100);
  goalLinearPub = node.advertise<geometry_msgs::Vector3>("/goal_linear", 100);
  goalAngularPub = node.advertise<geometry_msgs::Vector3>("/goal_angular", 100);
  adjustLinearPub = node.advertise<geometry_msgs::Vector3>("/adjust_linear", 100);
  adjustAngularPub = node.advertise<geometry_msgs::Vector3>("/adjust_angular", 100);
  increLinearPub = node.advertise<geometry_msgs::Vector3>("/incre_linear", 100);
  increAngularPub = node.advertise<geometry_msgs::Vector3>("/incre_angular", 100);

  // Get params
  paused = node.getParamEx("cmd_center/paused", false);
  linearSpeed = node.getParamEx("cmd_center/linearSpeed", 0.2);
  angularSpeed = node.getParamEx("cmd_center/angularSpeed", 0.4);
  linearStep = node.getParamEx("cmd_center/linearStep", 0.1);
  angularStep = node.getParamEx("cmd_center/linearStep", 0.2);

  ros::spin();
  return 0;
}

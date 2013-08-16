#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include "MyNodeHandle.h"
#include "csvparser.h"
#include <iostream>
#include <fstream>

// global vars
bool paused;
double linearSpeed;
double angularSpeed;
double linearStep;
double angularStep;

// publishers
ros::Publisher authorPub;
ros::Publisher goalVelPub;
ros::Publisher goalLinearPub;
ros::Publisher goalAngularPub;
ros::Publisher adjustLinearPub;
ros::Publisher adjustAngularPub;
ros::Publisher increLinearPub;
ros::Publisher increAngularPub;
ros::Publisher speaker;

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

std::string author;
void pubAuthor(std::string authorName)
{
  std_msgs::String authorMsg;
  authorMsg.data = authorName;
  authorPub.publish(authorMsg);
  author = authorName;
}

// csv commands file parser
std::map<std::string, std::string> commands;
void loadCommandsFile(std::string path)
{
  ifstream infile(path.c_str());
  if (!infile)
  {
    ROS_ERROR("Can not open the commands file, please check the path [%s]!", path.c_str());
    exit(1);
  }

  string sLine;
  CSVParser parser;
  while (!infile.eof())
  {
    getline(infile, sLine); // Get a line
    if (sLine == "")
      continue;

    parser << sLine; // Feed the line to the parser

    // Now extract the columns from the line
    int id;
    std::string command, speech, response;
    parser >> id >> command >> speech >> response;

    // save cmd into cmds
    commands.insert(map<std::string, std::string>::value_type(command, response));
  }
  infile.close();

  ROS_INFO("Loaded [%d] commands.", commands.size());
}

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
  std_msgs::String response;
  response.data = commands.find(tcmd)->second;
  speaker.publish(response);

  return true;
}

void recogCallback(const std_msgs::String::ConstPtr& msg)
{
  std::string cmd = msg->data;
  bool skipFlag = true;

  // program setting command
  if (cmd == "continue")
  {
    if (checkCmd("continue"))
      paused = false;
  }
  else if (cmd == "pause")
  {
    if (checkCmd("pause"))
    {
      paused = true;
      pubGoalVel(0, 0);
      pubAdjustVel(0, 0);
      pubAuthor("center");
    }
  }
  else
    skipFlag = false;
  if (paused)
    return;

  // common commands without "center" author privilege
  if (skipFlag)
    return;
  else
    skipFlag = true;
  if (cmd == "stop")
  {
    if (checkCmd("stop"))
    {
      pubGoalVel(0, 0);
      pubAdjustVel(0, 0);
      pubAuthor("center");
    }
  }
  else if (cmd == "follow me")
  {
    if (checkCmd("follow me"))
      pubAdjustVel(0, 0);
      pubAuthor("follower");
  }
  else if (cmd == "slower")
  {
    if (checkCmd("slower"))
    {
      pubIncreVel(-0.1, -0.8);
    }
  }
  else if (cmd == "faster")
  {
    if (checkCmd("faster"))
    {
      pubIncreVel(0.1, 0.8);
    }
  }
  else if (cmd == "turn left")
  {
    if (checkCmd("turn left"))
    {
      pubGoalAngular(0.8);
    }
  }
  else if (cmd == "turn right")
  {
    if (checkCmd("turn right"))
    {
      pubGoalAngular(-0.8);
    }
  }
  else if (cmd == "reset speed")
  {
    if (checkCmd("reset speed"))
    {
      pubAdjustVel(0, 0);
    }
  }
  else if (cmd == "hello")
  {
    if (checkCmd("hello"))
    {
      // Empty, just speak back
    }
  }
  else
    skipFlag = false;

  // check controlling author
  if (author != "center")
    return;

  // "center" authored commands
  if (skipFlag)
    return;
  else
    skipFlag = true;
  if (cmd == "forward")
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
  else
  {
    ROS_WARN("Unknown command [%s]", cmd.c_str());
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
  speaker = node.advertise<std_msgs::String>("voice_syn", 100);

  // Get params
  paused = node.getParamEx("cmd_center/paused", false);
  linearSpeed = node.getParamEx("cmd_center/linearSpeed", 0.2);
  angularSpeed = node.getParamEx("cmd_center/angularSpeed", 0.4);
  linearStep = node.getParamEx("cmd_center/linearStep", 0.1);
  angularStep = node.getParamEx("cmd_center/linearStep", 0.2);
  author = node.getParamEx("cmd_center/author", std::string("center"));
  std::string csvPath = node.getParamEx("cmd_center/cmd_csv_path", std::string("please/set/the/path.csv"));

  // load commands
  loadCommandsFile(csvPath);

  ros::spin();
  return 0;
}

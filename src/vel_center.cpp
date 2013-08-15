#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "MyNodeHandle.h"
#include <math.h>

// global speed vars
double goalLinearSpeed = 0;
double goalAngularSpeed = 0;
double adjustLinearSpeed = 0;
double adjustAngularSpeed = 0;
double increLinearSpeed = 0;
double increAngularSpeed = 0;

void goalVelCallback(const geometry_msgs::TwistConstPtr msg)
{
  goalLinearSpeed = msg->linear.x;
  goalAngularSpeed = msg->angular.z;
}

void goalLinearCallback(const geometry_msgs::Vector3ConstPtr msg)
{
  goalLinearSpeed = msg->x;
}

void goalAngularCallback(const geometry_msgs::Vector3ConstPtr msg)
{
  goalAngularSpeed = msg->z;
}

void adjustLinearCallback(const geometry_msgs::Vector3ConstPtr msg)
{
  adjustLinearSpeed = msg->x;
}

void adjustAngularCallback(const geometry_msgs::Vector3ConstPtr msg)
{
  adjustAngularSpeed = msg->z;
}

void increLinearCallback(const geometry_msgs::Vector3ConstPtr msg)
{
  increLinearSpeed = msg->x;
}

void increAngularCallback(const geometry_msgs::Vector3ConstPtr msg)
{
  increAngularSpeed = msg->z;
}

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "vel_center");
  MyNodeHandle node;
  ros::Subscriber goalVelSub = node.subscribe("/goal_vel", 100, goalVelCallback);
  ros::Subscriber goalLinearSub = node.subscribe("/goal_linear", 100, goalLinearCallback);
  ros::Subscriber goalAngularSub = node.subscribe("/goal_angular", 100, goalAngularCallback);
  ros::Subscriber adjustLinearSub = node.subscribe("/adjust_linear", 100, adjustLinearCallback);
  ros::Subscriber adjustAngularSub = node.subscribe("/adjust_angular", 100, adjustAngularCallback);
  ros::Subscriber increLinearSub = node.subscribe("/incre_linear", 100, increLinearCallback);
  ros::Subscriber increAngularSub = node.subscribe("/incre_angular", 100, increAngularCallback);
  ros::Publisher cmdVelPub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

  // get params
  double minLinearSpeed = node.getParamEx("vel_center/minLinearSpeed", 0.03);
  double minAngularSpeed = node.getParamEx("vel_center/minAngularSpeed", 0.15);
  double maxLinearSpeed = node.getParamEx("vel_center/maxLinearSpeed", 0.5);
  double maxAngularSpeed = node.getParamEx("vel_center/maxAngularSpeed", 2.5);
  double minAdjustLinear = node.getParamEx("vel_center/minAdjustLinear", 0.02);
  double minAdjustAngular = node.getParamEx("vel_center/minAdjustAngular", 0.04);
  double maxAdjustLinear = node.getParamEx("vel_center/maxAdjustLinear", 0.4);
  double maxAdjustAngular = node.getParamEx("vel_center/maxAdjustAngular", 1.5);
  double linearAccelerate = node.getParamEx("vel_center/linearAccelerate", 0.25); // num / s
  double angularAccelerate = node.getParamEx("vel_center/angularAccelerate", 1); // num / s
  int rate = node.getParamEx("vel_center/rate", 10);

  // start pub loop
  ros::Rate loopRate(rate);
  double linearSpeed = 0; // current speed
  double angularSpeed = 0; // current speed
  double calcLinearSpeed = 0; // goal speed after adjust
  double calcAngularSpeed = 0; // goal speed after adjust
  linearAccelerate /= rate; // accelerate limit per loop
  angularAccelerate /= rate; // accelerate limit per loop
  while (ros::ok())
  {
    // calc adjust speed
    adjustLinearSpeed += increLinearSpeed;
    adjustAngularSpeed += increAngularSpeed;
    increLinearSpeed = 0;
    increAngularSpeed = 0;

    // calc last goal linear speed
    if (fabs(adjustLinearSpeed) < minAdjustLinear || fabs(goalLinearSpeed) < minLinearSpeed)
    {
      calcLinearSpeed = goalLinearSpeed;
    }
    else if (adjustLinearSpeed > 0) // increase speed
    {
      if (adjustLinearSpeed > maxAdjustLinear) adjustLinearSpeed = maxAdjustLinear;
      calcLinearSpeed = goalLinearSpeed + (goalLinearSpeed > 0 ? adjustLinearSpeed : -adjustLinearSpeed);
    }
    else // decrease speed
    {
      if (-adjustLinearSpeed > maxAdjustLinear) adjustLinearSpeed = -maxAdjustLinear;
      if (fabs(goalLinearSpeed) - fabs(adjustLinearSpeed) < minLinearSpeed) calcLinearSpeed = minLinearSpeed;
      else calcLinearSpeed = goalLinearSpeed + adjustLinearSpeed;
    }

    // calc last goal angular speed
    if (fabs(adjustAngularSpeed) < minAdjustAngular || fabs(goalAngularSpeed) < minAngularSpeed)
    {
      calcAngularSpeed = goalAngularSpeed;
    }
    else if (adjustAngularSpeed > 0) // increase speed
    {
      if (adjustAngularSpeed > maxAdjustAngular) adjustAngularSpeed = maxAdjustAngular;
      calcAngularSpeed = goalAngularSpeed + (goalAngularSpeed > 0 ? adjustAngularSpeed : -adjustAngularSpeed);
    }
    else // decrease speed
    {
      if (-adjustAngularSpeed > maxAdjustAngular) adjustAngularSpeed = -maxAdjustAngular;
      if (fabs(goalAngularSpeed) - fabs(adjustAngularSpeed) < minAngularSpeed) calcAngularSpeed = minAngularSpeed;
      else calcAngularSpeed = goalAngularSpeed + adjustAngularSpeed;
    }

    // verify the max speed
    if (calcLinearSpeed > maxLinearSpeed || calcLinearSpeed < -maxLinearSpeed)
    {
      ROS_WARN("Goal linear speed is too high: [%f]", calcLinearSpeed);
      calcLinearSpeed = calcLinearSpeed > 0 ? maxLinearSpeed : -maxLinearSpeed;
    }
    if (calcAngularSpeed > maxAngularSpeed || calcAngularSpeed < -maxAngularSpeed)
    {
      ROS_WARN("Goal angular speed is too high: [%f]", calcAngularSpeed);
      calcAngularSpeed = calcAngularSpeed > 0 ? maxAngularSpeed : -maxAngularSpeed;
    }

    // smooth speed change
    if (calcLinearSpeed - linearSpeed > linearAccelerate)
      linearSpeed += linearAccelerate;
    else if (linearSpeed - calcLinearSpeed > linearAccelerate)
      linearSpeed -= linearAccelerate;
    else
      linearSpeed = calcLinearSpeed;
    if (calcAngularSpeed - angularSpeed > angularAccelerate)
      angularSpeed += angularAccelerate;
    else if (angularSpeed - calcAngularSpeed > angularAccelerate)
      angularSpeed -= angularAccelerate;
    else
      angularSpeed = calcAngularSpeed;

    // pub msg
    geometry_msgs::Twist cmdVel;
    cmdVel.linear.x = linearSpeed;
    cmdVel.angular.z = angularSpeed;
    cmdVelPub.publish(cmdVel);
    ROS_INFO("linear: [%f] angular: [%f]", linearSpeed, angularSpeed);

    ros::spinOnce();
    loopRate.sleep();
  }

  return 0;
}

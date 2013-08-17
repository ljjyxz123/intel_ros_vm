#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "MyNodeHandle.h"
#include "Utility.h"

using namespace cv;

ros::Publisher cmdVelPub;
bool paused; // dynamic pause or resume this program
int goalZ, maxZ, minPoints;
double maxLinear;
double maxAngular;
double linearRespRate, angularRespRate;
double maxWeight;
bool isViewVideo;
bool isSaveVideo;
double linearSpeed = 0;
double angularSpeed = 0;

VideoWriter histWriter;
VideoWriter depthWriter;
VideoWriter rgbWriter;
Size histSize(320, 240);
Size depthSize(640, 480);
Size rgbSize(640, 480);

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

void depthCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  // convert sensor_msgs/Image to Mat
  cv_bridge::CvImagePtr cvImgPtr;
  Mat_<uint16_t> depthImg;
  try
  {
    cvImgPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    depthImg = cvImgPtr->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  // init
  Utility util;
  int rows = depthImg.rows;
  int cols = depthImg.cols;

  // calc histgram
  float range[2] = {0, maxZ};
  vector<int> hist = util.calcHist(depthImg, histSize.width / 2, range);
  Mat histogram = util.drawHist(hist, histSize.height, 2);
  if (isViewVideo)
    imshow("hist", histogram);
  if (isSaveVideo)
  {
    cvtColor(histogram, histogram, CV_GRAY2BGR);
    histWriter << histogram;
  }

  // weight
  int maxRange = maxZ;
  Mat_<float> wMat(rows, cols); // weight mat
  wMat.setTo(0);
  float wSum = 0;
  double xSum, ySum, zSum;
  xSum = ySum = zSum = 0;
  int pointsSum = 0; // points sum which in the range
  for (int i = 0; i < rows; i++)
  {
    uint16_t* pDepthRow = depthImg.ptr<uint16_t>(i);
    float* pwMatRow = wMat.ptr<float>(i);
    for (int j = 0; j < cols; j++)
    {
      int z = pDepthRow[j]; // depth of current point
      double w = 0; // weight of current point
      if (z < maxRange) // if z in the range, calc the w
      {
        w = (1 - (double)pDepthRow[j] / maxRange) * 10;
        pwMatRow[j] = w;
        wSum += w;
        xSum += j * w;
        ySum += i * w;
        zSum += z * w;
        pointsSum++;
      }
    }
  }

  // draw depth color image
  bool channals[3] = {true, false, false};
  Mat depthColorImg = util.depth2Color(depthImg, 1200, channals);

  // draw depth color image advance
  float depthRange[2] = {0, 900};
  bool discard[2] = {false, true};
  bool depthChannals[3] = {false, false, true};
  util.depth2Color(depthImg, depthColorImg, depthRange, discard, depthChannals);

  // control
  float centerX = cols / 2;
  float avX, avY, avZ;
  if (pointsSum > minPoints)
  {
    avX = xSum / wSum;
    avY = ySum / wSum;
    avZ = zSum / wSum;
    float linearBias = avZ - goalZ;
    float angularBias = centerX - avX;

    linearSpeed = linearBias > 0 ? linearBias / (maxRange - goalZ) * maxLinear : linearBias / goalZ * maxLinear;
    angularSpeed = angularBias * 2 / cols * maxAngular;

    linearSpeed *= linearRespRate;
    angularSpeed *= angularRespRate;

    Point center((int)avX, (int)avY);
    circle(depthColorImg,center,6,Scalar(0, 255, 0), -1);
  }

  // put text info
  resize(depthColorImg, depthColorImg, depthSize);
  Point pos(20, 40);
  Scalar textColor(100, 100, 0);
  string text("linear: ");
  text.append(boost::lexical_cast<string>(linearSpeed));
  putText(depthColorImg, text, pos, FONT_HERSHEY_SIMPLEX, 0.6, textColor);
  pos.y += 20;
  text = "angular: ";
  text.append(boost::lexical_cast<string>(angularSpeed));
  putText(depthColorImg, text, pos, FONT_HERSHEY_SIMPLEX, 0.6, textColor);
  pos.y += 20;
  text = "time: ";
  text.append(boost::lexical_cast<string>(ros::Time::now()));
  putText(depthColorImg, text, pos, FONT_HERSHEY_SIMPLEX, 0.6, textColor);

  if (isViewVideo)
  {
    imshow("center", depthColorImg);
    waitKey(10);
  }
  if (isSaveVideo)
    depthWriter << depthColorImg;

  // pub message
  if (paused)
    return;
  geometry_msgs::Twist moveCmd;
  moveCmd.linear.x = linearSpeed;
  moveCmd.angular.z = angularSpeed;
  cmdVelPub.publish(moveCmd);
}

void rgbCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  cv_bridge::CvImagePtr cvImgPtr;
  Mat_<Vec3b> rgbImg;
  try
  {
    cvImgPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    rgbImg = cvImgPtr->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  // put time
  Point pos(20, 40);
  Scalar textColor(0, 0, 255);
  string text("time: ");
  text.append(boost::lexical_cast<string>(ros::Time::now()));
  putText(rgbImg, text, pos, FONT_HERSHEY_SIMPLEX, 0.6, textColor);

  // display or save video
  if (isViewVideo)
  {
    imshow("color image", rgbImg);
    waitKey(10);
  }
  if (isSaveVideo)
    rgbWriter << rgbImg;
}

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "follower3");
  MyNodeHandle node;
  ros::Subscriber depthRawSub = node.subscribe("/camera/depth/image_raw", 100, depthCallback);
  ros::Subscriber rgbRawSub = node.subscribe("/camera/rgb/image_raw", 100, rgbCallback);
  ros::Subscriber commandSub = node.subscribe("/cmd_center/author", 100, commandCallback);
  cmdVelPub = node.advertise<geometry_msgs::Twist>("/goal_vel", 100);

  // get params
  ROS_INFO("follower2 get params:");
  paused = node.getParamEx("follower3/paused", false);
  maxLinear = node.getParamEx("follower3/maxLinear", 0.5);
  maxAngular = node.getParamEx("follower3/maxAngular", 2.0);
  goalZ = node.getParamEx("follower3/goalZ", 600);
  maxZ = node.getParamEx("follower3/maxZ", 900);
  maxWeight = node.getParamEx("follower3/maxWeight", 10.0);
  minPoints = node.getParamEx("follower3/minPoints", 1000);
  linearRespRate = node.getParamEx("follower3/linearRespRate", 1.0);
  angularRespRate = node.getParamEx("follower3/angularRespRate", 1.0);
  isViewVideo = node.getParamEx("follower3/isViewVideo", true);
  isSaveVideo = node.getParamEx("follower3/isSaveVideo", true);
  string histVideoPath = node.getParamEx("follower3/histVideoPath", "follower3_histogram.avi");
  string depthVideoPath = node.getParamEx("follower3/depthVideoPath", "follower3_depth.avi");
  string rgbVideoPath = node.getParamEx("follower3/depthVideoPath", "follower3_rgb.avi");

  try
  {
    if ("" != histVideoPath)
      histWriter.open(histVideoPath, CV_FOURCC('M', 'J', 'P', 'G'), 10, histSize);
    if ("" != depthVideoPath)
      depthWriter.open(depthVideoPath, CV_FOURCC('M', 'J', 'P', 'G'), 10, depthSize);
    if ("" != rgbVideoPath)
      rgbWriter.open(rgbVideoPath, CV_FOURCC('M', 'J', 'P', 'G'), 10, rgbSize);
  }
  catch (...)
  {
    ROS_ERROR("Can not access the video saved path, save vedio failed, please check!");
    exit(1);
  }

  ros::spin();

  return 0;
}

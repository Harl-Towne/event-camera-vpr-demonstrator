// #include "ros/ros.h"
// #include "std_msgs/String.h"

// void chatterCallback(const std_msgs::String::ConstPtr& msg)
// {
//   ROS_INFO("I heard: [%s]", msg->data.c_str());
// }

// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "listener");

//   ros::NodeHandle n;

//   // ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

//   ros::spin();

//   return 0;
// }


#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <dvs_msgs/EventArray.h>
#include <demo_msgs/EventPacket.h>
// #include <sensor_msgs/Image.h>
#include <ros/console.h>
// #include <sstream>
#include <string>
// #include <iostream>
// using namespace std;

ros::Publisher pub;

void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
  int data_length = msg->events.size();

  cv_bridge::CvImage data_out;
  data_out.image = cv::Mat(data_length, 4, CV_64FC1);
  data_out.encoding = "64FC1";

  for(int i = 0; i < msg->events.size(); i++)
  {
    data_out.image.at<_Float64>(cv::Point(0, i)) = (_Float64)msg->events[i].ts.sec + (_Float64)msg->events[i].ts.nsec * 10e-10;
    data_out.image.at<_Float64>(cv::Point(1, i)) = msg->events[i].x;
    data_out.image.at<_Float64>(cv::Point(2, i)) = msg->events[i].y;
    data_out.image.at<_Float64>(cv::Point(3, i)) = msg->events[i].polarity;
  }

  demo_msgs::EventPacket msg_out = demo_msgs::EventPacket();
  msg_out.events = *(data_out.toImageMsg());
  msg_out.height = msg->height;
  msg_out.width = msg->width;
  
  pub.publish(msg_out);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "format_converter");

  ros::NodeHandle n;

  ros::Subscriber event_sub = n.subscribe("dvs/events", 1, eventsCallback);
  pub = n.advertise<demo_msgs::EventPacket>("/davis346/data/event_packets", 10);

  ros::spin();

  return 0;
}
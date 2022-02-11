#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <dvs_msgs/EventArray.h>
#include <qcr_event_msgs/EventPacket.h>
#include <ros/console.h>
#include <string>
#include <opencv2/opencv.hpp>
 #include <iostream>

ros::Publisher pub;

void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
  int data_length = msg->events.size();

  // create image for events
  cv_bridge::CvImage data_out;
  data_out.image = cv::Mat(data_length, 4, CV_64FC1);
  data_out.encoding = "64FC1";

  // fill image with events to be sent out
  for(int i = 0; i < msg->events.size(); i++)
  {
    data_out.image.at<double>(cv::Point(0, i)) = (double)(msg->events[0].ts.sec + (double)(msg->events[0].ts.nsec) * 10e-10);
    data_out.image.at<double>(cv::Point(1, i)) = msg->events[i].x;
    data_out.image.at<double>(cv::Point(2, i)) = msg->events[i].y;
    data_out.image.at<double>(cv::Point(3, i)) = msg->events[i].polarity;
  }

  // create packet for data and add data/image to it
  qcr_event_msgs::EventPacket msg_out = qcr_event_msgs::EventPacket();
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
  pub = n.advertise<qcr_event_msgs::EventPacket>("/event_camera_demo/event_packets", 10);

  ros::spin();

  return 0;
}
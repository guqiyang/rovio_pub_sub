/*!
 * FileName： rovioBase_pub.cpp
 * Description: 针对rovio小车的一个示范程序；主要工作是利用yuanbo she 封装好的rovio模块，通过pub/sub形式控制rovio小车；
 * Function List:
 * 1. ros::Publisher reportPub;
 * 2. ros::Publisher imgPub;
 *  <author>   <time>   <version >     <desc>
 *  guqiyang   16/1/6       1.0     build this moudle
 * Group: AICRobo http://aicrobo.github.io
 *-------------------------------------------------
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2016, AICRobo.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include "rovioParser.h"
#include"rovio_pub/Image.h"
#include"rovio_pub/manDrv.h"
#include"rovio_pub/report.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
#include<math.h>
using namespace cv;
using namespace std;

rovioParser parser;

ros::Publisher reportPub;
ros::Publisher imgPub;

void control(geometry_msgs::Twist twist)
{
  uint8_t drive;
  uint8_t speed;
  twist.linear.x;
  twist.linear.y;
  twist.angular.z;

  if (twist.linear.x > 0)
  {
    if (twist.linear.y > 0)
    {
      drive = 8;
      speed = 5;
    }
    else
    {
      drive = 10;
      speed = 5;
    }

  }
  else if (twist.linear.x < 0)
  {
    if (twist.linear.y > 0)
    {
      drive = 10;
      speed = 5;
    }
    else
    {
      drive = 9;
      speed = 5;
    }
  }
  rvManualDrive rv = parser.manualDrive(drive, speed);
  ROS_INFO("Rovio control message: drive=%d, speed=%d", drive, speed);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rovioBase");
  ros::NodeHandle nh;
  ros::NodeHandle ph("~");
  //
  string host, port, user, pw;
  parser.initParam(host, port, user, pw);
  ros::Subscriber crlSub = nh.subscribe("rovioControl", 10, control);
  imgPub = nh.advertise<sensor_msgs::Image>("rovioImage", 10);
  reportPub = nh.advertise<rovio_pub::report>("rovioReport", 10);
  ros::Rate loopRate(10);
  ROS_INFO("Rovio pub ON...");
  while (ros::ok())
  {
    //get the status from MCU
    rovio_pub::report report;
    rvMCUReport rv = parser.getMCUReport();
    report.length = rv.length;
    report.lDirection = rv.lDirection;
    report.lNum = rv.lNum;
    report.rDirection = rv.rDirection;
    report.rNum = rv.rNum;
    report.rearDirection = rv.rearDirection;
    report.rearNum = rv.rearNum;
    report.headPosition = rv.headPosition;
    report.isLedOn = rv.isLedOn;
    reportPub.publish(report);
    ROS_INFO("Rovio report Responsed!");
//    get the image message from rovio
    sensor_msgs::Image img;
    cv_bridge::CvImage cvImg(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, parser.getImg());
    cvImg.toImageMsg(img);  //convert the image to image_message
    imgPub.publish(img);
    ROS_INFO("Rovio image Responsed!");
    ros::spinOnce();
    loopRate.sleep();
  }
  return 0;
}

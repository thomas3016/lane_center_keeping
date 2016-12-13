/*
 * lane_centerline_detection_node.cpp
 *
 *  Created on: Sep 28, 2016
 *      Author: aicrobo
 */

#include <ros/ros.h>
#include<lane_center_keeping/lane_centerline_detection.h>



/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "lane_centerline_detection_node");

  lane_center_keeping::LaneCenterlineDetection scan(ros::NodeHandle(),ros::NodeHandle("~"));
  ros::spin();

  return 0;
}






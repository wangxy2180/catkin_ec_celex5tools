//
// Created by free on 2020/11/11.
//
/**
 * in this file, just create a detector called Fast_detector
 * all exec in constructor of Fast_detector
 *
 */
#include "corner_detector/fast_detector.h"
#include "../include/detector.h"
#include <iostream>
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "corner_detector_fast_corner");

  ROS_INFO("Now we will use Fast corner, good luck!");
  ROS_INFO("don't worry, I'm still alive, no info recv.");

  std::string feature_type;
  ros::param::param<std::string>("feature_type",feature_type,"FAST");


  corner_detector::Detector *detector;
  if(feature_type=="FAST")
  {
      detector = new corner_detector::Fast_detector();
  }
  else if(feature_type=="arc_star")
  {}
  else{
      ROS_ERROR("invaild detector, please check: %s" ,feature_type.c_str());
  }
  ros::spin();
  delete detector;
  ROS_INFO("see you next time~");
  ros::shutdown();
  return 0;
}
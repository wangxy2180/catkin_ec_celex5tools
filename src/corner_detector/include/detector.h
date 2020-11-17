//
// Created by free on 2020/11/11.
//

#ifndef SRC_DETECTOR_H
#define SRC_DETECTOR_H

//#include "celex5_msgs/eventVector.h"
#include "celex5_msgs/eventVector.h"
#include <ros/ros.h>

#include <chrono>

namespace corner_detector {
class Detector {
public:
  Detector();
  virtual ~Detector();

  virtual bool isFeature(const celex5_msgs::event &e) = 0;

private:
  ros::NodeHandle nh;
  ros::Publisher feature_pub;
  ros::Subscriber event_sub;
  void eventCallback(const celex5_msgs::eventVector::ConstPtr &msg);
};
} // namespace corner_detector

#endif // SRC_DETECTOR_H

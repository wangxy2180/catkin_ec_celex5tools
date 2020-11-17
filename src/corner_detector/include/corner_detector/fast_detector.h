//
// Created by free on 2020/11/11.
//

#ifndef SRC_CORNER_MYFAST_H
#define SRC_CORNER_MYFAST_H

#include "detector.h"
#include <Eigen/Dense>
#include <ros/ros.h>

namespace corner_detector {
class Fast_detector : public Detector {
public:
  Fast_detector();
  virtual ~Fast_detector();

  bool isFeature(const celex5_msgs::event &e);

private:
  //  SAE
  //  this is a 2-d MatrixXd matrix, 2 multi-d matrix
//  for 2 polar
  Eigen::MatrixXd SAE[2];

  //  2 circle
  int circle3[16][2];
  int circle4[20][2];

  static constexpr int sensor_width = 1280;
  static constexpr int sensor_height = 800;
};
} // namespace corner_detector
#endif // SRC_CORNER_MYFAST_H

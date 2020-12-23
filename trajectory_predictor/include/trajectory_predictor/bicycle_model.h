#ifndef __BICYCLE_MODEL_H__
#define __BICYCLE_MODEL_H__

#include <geometry_msgs/PoseArray.h>

class BicycleModel {
 public:
  BicycleModel();
  BicycleModel(double x, double y, double theta, double delta, double delta_dot, double velocity, double L);

  virtual ~BicycleModel() {}

  geometry_msgs::PoseArray predict(double t_max, double d_t);

 private:
  double x;          // x position of rear axle
  double y;          // y position of rear axle
  double theta;      // yaw of vehicle [rad]
  double delta;      // steering angle [rad]
  double delta_dot;  // rate of change of steering angle [rad/s]
  double velocity;   // velocity [m/s]
  double L;          // wheel base [m]

  BicycleModel propagateState(double d_t);
  geometry_msgs::Pose toRosMsg();
};
#endif  // __BICYCLE_MODEL_H__
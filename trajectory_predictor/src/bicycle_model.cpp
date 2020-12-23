#include "trajectory_predictor/bicycle_model.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

BicycleModel::BicycleModel(double x, double y, double theta, double delta, double delta_dot, double velocity, double L) {
  this->x = x;
  this->y = y;
  this->theta = theta;
  this->delta = delta;
  this->delta_dot = delta_dot;
  this->velocity = velocity;
  this->L = L;
}

geometry_msgs::PoseArray BicycleModel::predict(double t_max, double d_t) {
  geometry_msgs::PoseArray prediction;
  prediction.header.frame_id = "map";
  prediction.header.stamp = ros::Time::now();
  prediction.poses.push_back(this->toRosMsg());

  for (double t = 0; t < t_max; t += d_t) {
    prediction.poses.push_back(propagateState(d_t).toRosMsg());
  }

  return prediction;
}

BicycleModel BicycleModel::propagateState(double d_t) {
  double x_dot = velocity * cos(theta);
  double y_dot = velocity * sin(theta);
  double theta_dot = velocity * tan(delta) / L;

  x = x + x_dot * d_t;
  y = y + y_dot * d_t;
  theta = theta + theta_dot * d_t;
  delta = delta + delta_dot * d_t;

  return *this;
}

geometry_msgs::Pose BicycleModel::toRosMsg() {
  geometry_msgs::Pose pose_msg;

  pose_msg.position.x = x;
  pose_msg.position.y = y;

  tf2::Quaternion q;
  q.setRPY(0, 0, theta);

  pose_msg.orientation.x = q.x();
  pose_msg.orientation.y = q.y();
  pose_msg.orientation.z = q.z();
  pose_msg.orientation.w = q.w();

  return pose_msg;
}